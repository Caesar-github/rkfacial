/*
 * Copyright (C) 2019 Rockchip Electronics Co., Ltd.
 * author: Zhihua Wang, hogan.wang@rock-chips.com
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL), available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <stdio.h>
#include <memory.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <pthread.h>
#include <errno.h>

#include <list>

#include "face_common.h"
#include "database.h"
#include "rockface_control.h"
#include "play_wav.h"
#include "load_feature.h"
#include "video_common.h"
#include "rkisp_control.h"
#include "rkcif_control.h"
#include "snapshot.h"
#include "db_monitor.h"
#include "rkfacial.h"

#define DEFAULT_FACE_NUMBER 1000
#define DEFAULT_FACE_PATH "/userdata"
#define FACE_SCORE_RGB 0.55 /* range 0 - 1.0, higher score means higher expectation */
#define FACE_SCORE_IR 0.7 /* range 0 - 1.0, higher score means higher expectation */
#define FACE_SCORE_LANDMARK_RUNNING 0.9 /* range 0 - 1.0, higher score means higher expectation */

/*
 * suggest range 0.7 ~ 1.3, lower score
 * means need higer similarity to recognize
 */
#define FACE_SIMILARITY_SCORE 1.0
#define FACE_SCORE_LANDMARK_IMAGE 0.5 /* range 0 - 1.0, higher score means higher expectation */
#define FACE_SCORE_REGISTER 0.99 /* range 0 - 1.0, higher score means higher expectation */
#define FACE_REGISTER_CNT 5
#define FACE_REAL_SCORE 0.7 /* range 0 - 1.0, higher score means higher expectation */
#define LICENCE_PATH PRE_PATH "/key.lic"
#define BAK_LICENCE_PATH BAK_PATH "/key.lic"
#define FACE_DATA_PATH "/usr/lib"
#define MIN_FACE_WIDTH(w) ((w) / 5)
#define FACE_TRACK_FRAME 0
#define FACE_RETRACK_TIME 1
#define SNAP_TIME 3

#define DET_BUFFER_NUM 2
#define DET_WIDTH 360
#define DET_HEIGHT 640

struct face_buf {
    rockface_image_t img;
    rockface_det_t face;
    bo_t bo;
    int fd;
    int id;
};

static struct face_buf g_feature;
static struct face_buf g_detect[DET_BUFFER_NUM];
static pthread_mutex_t g_det_lock = PTHREAD_MUTEX_INITIALIZER;
static std::list<struct face_buf*> g_det_free;
static std::list<struct face_buf*> g_det_ready;

static void *g_face_data = NULL;
static int g_face_index = 0;
static int g_face_cnt = DEFAULT_FACE_NUMBER;

static rockface_handle_t face_handle;
static int g_total_cnt;

static pthread_t g_tid;
static bool g_run;
static char last_name[NAME_LEN];
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cond = PTHREAD_COND_INITIALIZER;
static bool g_feature_flag;
static pthread_t g_detect_tid;
static pthread_mutex_t g_detect_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_detect_cond = PTHREAD_COND_INITIALIZER;
static bool g_detect_flag;
static int g_rgb_track = -1;
static pthread_mutex_t g_rgb_track_mutex = PTHREAD_MUTEX_INITIALIZER;

static rockface_image_t g_ir_img;
static bo_t g_ir_bo;
static int g_ir_fd = -1;

static bo_t g_ir_det_bo;
static int g_ir_det_fd = -1;

enum ir_state {
    IR_STATE_CANCELED,
    IR_STATE_PREPARED,
    IR_STATE_COMPLETE,
    IR_STATE_USING,
};

enum ir_state g_ir_state;

static bool g_register = false;
static int g_register_cnt = 0;
static bool g_delete = false;

static struct snapshot g_snap;

static pthread_mutex_t g_lib_lock = PTHREAD_MUTEX_INITIALIZER;

bool g_face_en;
int g_face_width;
int g_face_height;
static int g_ratio;

void set_face_param(int width, int height, int cnt)
{
    int tmp = width < height ? width : height;
    g_face_en = true;
    g_face_width = width;
    g_face_height = height;
    g_face_cnt = cnt;
    g_ratio = tmp / DET_WIDTH;
}

static void check_pre_path(const char *pre)
{
    char cmd[128];
    FILE *fp;
    size_t size = 0;
    char buffer[128];

    snprintf(cmd, sizeof(cmd), "mount | grep %s", pre);
    do {
        size = 0;
        fp = popen(cmd, "r");
        if (fp) {
            size = fread(buffer, 1, sizeof(buffer), fp);
            pclose(fp);
            if (size > 0)
                break;
        }
        sleep(1);
        printf("%s %s\n", __func__, pre);
    } while (1);
}

static void *init_thread(void *arg)
{
    char cmd[256];

    check_pre_path(PRE_PATH);
    int ret = rockface_control_init();

    if (ret)
        play_wav_signal(AUTHORIZE_FAIL_WAV);

    check_pre_path(BAK_PATH);
    snprintf(cmd, sizeof(cmd), "cp %s %s", LICENCE_PATH, BAK_LICENCE_PATH);
    system(cmd);

    database_bak();

    pthread_detach(pthread_self());
    pthread_exit(NULL);
}

void rockface_control_init_thread(void)
{
    pthread_t tid;
    if (pthread_create(&tid, NULL, init_thread, NULL))
        printf("%s fail!\n", __func__);
}

static rockface_det_t *get_max_face(rockface_det_array_t *face_array)
{
    rockface_det_t *max_face = NULL;
    if (face_array->count == 0)
        return NULL;

    for (int i = 0; i < face_array->count; i++) {
        rockface_det_t *cur_face = &(face_array->face[i]);
        if (max_face == NULL) {
            max_face = cur_face;
            continue;
        }
        int cur_face_box_area = (cur_face->box.right - cur_face->box.left) *
                                (cur_face->box.bottom - cur_face->box.top);
        int max_face_box_area = (max_face->box.right - max_face->box.left) *
                                (max_face->box.bottom - max_face->box.top);
        if (cur_face_box_area > max_face_box_area)
            max_face = cur_face;
    }

    return max_face;
}

static int _rockface_control_detect(rockface_image_t *image, rockface_det_t *out_face, int *track)
{
    int r = 0;
    rockface_ret_t ret;
    rockface_det_array_t face_array0;
    rockface_det_array_t face_array;

    memset(&face_array0, 0, sizeof(rockface_det_array_t));
    memset(&face_array, 0, sizeof(rockface_det_array_t));
    memset(out_face, 0, sizeof(rockface_det_t));

    ret = rockface_detect(face_handle, image, &face_array0);
    if (ret != ROCKFACE_RET_SUCCESS)
        return -1;

    if (track) {
        ret = rockface_track(face_handle, image, FACE_TRACK_FRAME, &face_array0, &face_array);
        if (ret != ROCKFACE_RET_SUCCESS)
            return -1;
    } else {
        memcpy(&face_array, &face_array0, sizeof(rockface_det_array_t));
    }

    rockface_det_t* face = get_max_face(&face_array);
    if (face == NULL || face->score < FACE_SCORE_RGB ||
        face->box.right - face->box.left < MIN_FACE_WIDTH(image->width) ||
        face->box.left < 0 || face->box.top < 0 ||
        face->box.right > image->width || face->box.bottom > image->height)
        return -1;

    memcpy(out_face, face, sizeof(rockface_det_t));

    if (track) {
        pthread_mutex_lock(&g_rgb_track_mutex);
        if (g_delete || g_register || !strlen(last_name))
            *track = -1;
        else if (*track == face->id)
            r = -2;
        else
            *track = face->id;
        pthread_mutex_unlock(&g_rgb_track_mutex);
    }

    return r;
}

static int rockface_control_detect(rockface_image_t *image, rockface_det_t *face)
{
    int ret;
    static struct timeval t0;
    struct timeval t1;

    memset(face, 0, sizeof(rockface_det_t));

    if (!t0.tv_sec && !t0.tv_usec)
        gettimeofday(&t0, NULL);
    gettimeofday(&t1, NULL);
    pthread_mutex_lock(&g_rgb_track_mutex);
    if (g_rgb_track >= 0 && t1.tv_sec - t0.tv_sec > FACE_RETRACK_TIME) {
        g_rgb_track = -1;
        gettimeofday(&t0, NULL);
    }
    pthread_mutex_unlock(&g_rgb_track_mutex);
    ret = _rockface_control_detect(image, face, &g_rgb_track);
    if (face->score > FACE_SCORE_RGB) {
        int left, top, right, bottom;
        left = face->box.left * g_ratio;
        top = face->box.top * g_ratio;
        right = face->box.right * g_ratio;
        bottom = face->box.bottom * g_ratio;
        if (rkfacial_paint_box_cb)
            rkfacial_paint_box_cb(left, top, right, bottom);
        rkisp_control_expo_weights(left, top, right, bottom);
    } else {
        if (rkfacial_paint_box_cb)
            rkfacial_paint_box_cb(0, 0, 0, 0);
        rkisp_control_expo_weights_default();
    }

    return ret;
}

static int rockface_control_init_library(void *data, int num, size_t size, size_t off)
{
    rockface_ret_t ret;

    ret = rockface_face_library_init(face_handle, data, num, size, off);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: int library error %d!\n", __func__, ret);
        return -1;
    }

    return 0;
}

static void rockface_control_release_library(void)
{
    rockface_face_library_release(face_handle);
}

static int rockface_control_get_feature(rockface_image_t *in_image,
                                        rockface_feature_t *out_feature,
                                        rockface_det_t *in_face,
                                        float score)
{
    rockface_ret_t ret;

    rockface_landmark_t landmark;
    ret = rockface_landmark5(face_handle, in_image, &(in_face->box), &landmark);
    if (ret != ROCKFACE_RET_SUCCESS || landmark.score < score)
        return -1;

    rockface_image_t out_img;
    memset(&out_img, 0, sizeof(rockface_image_t));
    ret = rockface_align(face_handle, in_image, &(in_face->box), &landmark, &out_img);
    if (ret != ROCKFACE_RET_SUCCESS)
        return -1;

    ret = rockface_feature_extract(face_handle, &out_img, out_feature);
    rockface_image_release(&out_img);
    if (ret != ROCKFACE_RET_SUCCESS)
        return -1;

    return 0;
}

int rockface_control_get_path_feature(const char *path, void *feature)
{
    int ret = -1;
    rockface_feature_t *out_feature = (rockface_feature_t*)feature;
    rockface_image_t in_img;
    rockface_det_t face;
    int cnt = 10;

    while (access(path, F_OK) && --cnt)
        usleep(100000);
    if (rockface_image_read(path, &in_img, 1))
        return -1;
    if (!_rockface_control_detect(&in_img, &face, NULL))
        ret = rockface_control_get_feature(&in_img, out_feature, &face, FACE_SCORE_LANDMARK_IMAGE);
    rockface_image_release(&in_img);
    return ret;
}

static bool rockface_control_search(rockface_image_t *image, void *data, int *index, int cnt,
                              size_t size, size_t offset, rockface_det_t *face, int reg,
                              struct face_data* face_data)
{
    rockface_ret_t ret;
    rockface_search_result_t result;
    rockface_feature_t feature;

    if (rockface_control_get_feature(image, &feature, face, FACE_SCORE_LANDMARK_RUNNING) == 0) {
        //printf("g_total_cnt = %d\n", ++g_total_cnt);
        pthread_mutex_lock(&g_lib_lock);
        ret = rockface_feature_search(face_handle, &feature, FACE_SIMILARITY_SCORE, &result);
        if (ret == ROCKFACE_RET_SUCCESS) {
            memcpy(face_data, result.feature, sizeof(struct face_data));
            pthread_mutex_unlock(&g_lib_lock);
            if (g_register && ++g_register_cnt > FACE_REGISTER_CNT) {
                g_register = false;
                g_register_cnt = 0;
                play_wav_signal(REGISTER_ALREADY_WAV);
            }
            return true;
        }
        pthread_mutex_unlock(&g_lib_lock);
        if (g_register && *index < cnt && face->score > FACE_SCORE_REGISTER && reg && strlen(g_white_list)) {
            char name[NAME_LEN];
            int id = database_get_user_name_id();
            if (id < 0) {
                printf("%s: get id fail!\n", __func__);
                return false;
            }
            snprintf(name, sizeof(name), "%s/%s_%d.jpg", g_white_list, USER_NAME, id);
#ifdef USE_WEB_SERVER
            strncpy(g_snap.name, name, sizeof(g_snap.name));
            if (!snapshot_run(&g_snap, image, NULL, RK_FORMAT_RGB_888, 0, 0))
                printf("save %s success\n", name);
#endif

            rockface_control_add(id, name, &feature);

            g_register = false;
            g_register_cnt = 0;
            play_wav_signal(REGISTER_SUCCESS_WAV);
            return false;
        }
#ifdef USE_WEB_SERVER
        memset(g_snap.name, 0, sizeof(g_snap.name));
        if (!snapshot_run(&g_snap, image, face, RK_FORMAT_RGB_888, SNAP_TIME, 'S'))
            db_monitor_snapshot_record_set(g_snap.name);
#endif
    }

    return false;
}

void rockface_control_set_delete(void)
{
    g_register = false;
    g_register_cnt = 0;
    g_delete = true;
}

void rockface_control_set_register(void)
{
    g_delete = false;
    if (g_register_cnt == 0)
        g_register = true;
}

static void rockface_control_detect_wait(void)
{
    pthread_mutex_lock(&g_detect_mutex);
    if (g_detect_flag)
        pthread_cond_wait(&g_detect_cond, &g_detect_mutex);
    g_detect_flag = true;
    pthread_mutex_unlock(&g_detect_mutex);
}

static void rockface_control_detect_signal(void)
{
    pthread_mutex_lock(&g_detect_mutex);
    g_detect_flag = false;
    pthread_cond_signal(&g_detect_cond);
    pthread_mutex_unlock(&g_detect_mutex);
}

static int rockface_control_wait(void)
{
    int ret;
    pthread_mutex_lock(&g_mutex);
    if (g_feature_flag) {
#define TIMEOUT_US 100000
#define ONE_MIN_US 1000000
        struct timeval now;
        struct timespec out;
        gettimeofday(&now, NULL);
        if (now.tv_usec + TIMEOUT_US >= ONE_MIN_US) {
            out.tv_sec = now.tv_sec + 1;
            out.tv_nsec = (now.tv_usec + TIMEOUT_US - ONE_MIN_US) * 1000;
        } else {
            out.tv_sec = now.tv_sec;
            out.tv_nsec = (now.tv_usec + TIMEOUT_US) * 1000;
        }
        ret = pthread_cond_timedwait(&g_cond, &g_mutex, &out);
    }
    pthread_mutex_unlock(&g_mutex);
    return ret;
}

static void rockface_control_signal(void)
{
    pthread_mutex_lock(&g_mutex);
    g_feature_flag = false;
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
}

int rockface_control_convert_detect(void *ptr, int width, int height, RgaSURF_FORMAT fmt, int rotation, int id)
{
    rga_info_t src, dst;
    struct face_buf *buf;

    if (!g_run)
        return -1;

    pthread_mutex_lock(&g_det_lock);
    if (g_det_free.empty()) {
        pthread_mutex_unlock(&g_det_lock);
        return -1;
    } else {
        buf = g_det_free.front();
        g_det_free.pop_front();
        pthread_mutex_unlock(&g_det_lock);
    }

    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.virAddr = ptr;
    src.mmuFlag = 1;
    src.rotation = rotation;
    rga_set_rect(&src.rect, 0, 0, width, height, width, height, fmt);
    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.virAddr = buf->bo.ptr;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, DET_WIDTH, DET_HEIGHT,
                 DET_WIDTH, DET_HEIGHT, RK_FORMAT_RGB_888);
    if (c_RkRgaBlit(&src, &dst, NULL)) {
        printf("%s: rga fail\n", __func__);
        goto exit;
    }
    memset(&buf->img, 0, sizeof(rockface_image_t));
    buf->img.width = DET_WIDTH;
    buf->img.height = DET_HEIGHT;
    buf->img.pixel_format = ROCKFACE_PIXEL_FORMAT_RGB888;
    buf->img.data = (uint8_t *)buf->bo.ptr;
    buf->id = id;

    pthread_mutex_lock(&g_det_lock);
    g_det_ready.push_back(buf);
    pthread_mutex_unlock(&g_det_lock);
    rockface_control_detect_signal();

    return 0;

exit:
    pthread_mutex_lock(&g_det_lock);
    g_det_free.push_back(buf);
    pthread_mutex_unlock(&g_det_lock);
    return -1;
}

int rockface_control_convert_feature(void *ptr, int width, int height, RgaSURF_FORMAT fmt, int rotation, int id)
{
    rga_info_t src, dst;
    if (!g_feature_flag || g_feature.id)
        return -1;
    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.virAddr = ptr;
    src.mmuFlag = 1;
    src.rotation = rotation;
    rga_set_rect(&src.rect, 0, 0, width, height, width, height, fmt);
    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.virAddr = g_feature.bo.ptr;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, height, width, height, width, RK_FORMAT_RGB_888);
    if (c_RkRgaBlit(&src, &dst, NULL)) {
        printf("%s: rga fail\n", __func__);
        return -1;
    }
    memset(&g_feature.img, 0, sizeof(g_feature.img));
    g_feature.img.width = height;
    g_feature.img.height = width;
    g_feature.img.pixel_format = ROCKFACE_PIXEL_FORMAT_RGB888;
    g_feature.img.data = (uint8_t *)g_feature.bo.ptr;
    g_feature.id = id;

    return 0;
}

static bool rockface_control_liveness_ir(void)
{
    rockface_ret_t ret;
    rockface_image_t image;
    rockface_det_array_t face_array;
    rockface_liveness_t result;

    int src_w = g_ir_img.width, src_h = g_ir_img.height;
    int dst_w = DET_WIDTH, dst_h = DET_HEIGHT;
    rga_info_t src, dst;
    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.virAddr = g_ir_bo.ptr;
    src.mmuFlag = 1;
    rga_set_rect(&src.rect, 0, 0, src_w, src_h, src_w, src_h, RK_FORMAT_YCbCr_420_SP);
    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.virAddr = g_ir_det_bo.ptr;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, dst_w, dst_h, dst_w, dst_h, RK_FORMAT_RGB_888);
    if (c_RkRgaBlit(&src, &dst, NULL)) {
        printf("%s: rga fail\n", __func__);
        return false;
    }

    rockface_image_t ir_det_img;
    memset(&ir_det_img, 0, sizeof(rockface_image_t));
    ir_det_img.width = DET_WIDTH;
    ir_det_img.height = DET_HEIGHT;
    ir_det_img.pixel_format = ROCKFACE_PIXEL_FORMAT_RGB888;
    ir_det_img.data = (uint8_t *)g_ir_det_bo.ptr;
    ret = rockface_detect(face_handle, &ir_det_img, &face_array);
    if (ret != ROCKFACE_RET_SUCCESS)
        return false;

    rockface_det_t* face = get_max_face(&face_array);
    if (face == NULL || face->score < FACE_SCORE_IR ||
        face->box.right - face->box.left < MIN_FACE_WIDTH(ir_det_img.width) ||
        face->box.left < 0 || face->box.top < 0 ||
        face->box.right > ir_det_img.width || face->box.bottom > ir_det_img.height)
        return false;

    face->box.left *= g_ratio;
    face->box.top *= g_ratio;
    face->box.right *= g_ratio;
    face->box.bottom *= g_ratio;
    ret = rockface_liveness_detect(face_handle, &g_ir_img, &face->box, &result);
    if (ret != ROCKFACE_RET_SUCCESS)
        return false;

    if (result.real_score < FACE_REAL_SCORE)
        return false;

    return true;
}

void rockface_control_set_ir_prepared(void)
{
    if (g_ir_state == IR_STATE_CANCELED)
        g_ir_state = IR_STATE_PREPARED;
}

int rockface_control_convert_ir(void *ptr, int width, int height, RgaSURF_FORMAT fmt, int rotation)
{
    int ret = -1;
    rga_info_t src, dst;

    if (!g_run)
        return ret;

    if (g_ir_state != IR_STATE_PREPARED)
        return ret;

    memset(&g_ir_img, 0, sizeof(rockface_image_t));
    g_ir_img.pixel_format = ROCKFACE_PIXEL_FORMAT_GRAY8;
    g_ir_img.data = (uint8_t *)g_ir_bo.ptr;
    switch (rotation) {
    case 0:
        g_ir_img.width = width;
        g_ir_img.height = height;
        break;
    case HAL_TRANSFORM_ROT_90:
    case HAL_TRANSFORM_ROT_270:
        g_ir_img.width = height;
        g_ir_img.height = width;
        break;
    default:
        printf("%s: unsupport rotation!\n", __func__);
        goto exit;
    }

    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.virAddr = ptr;
    src.mmuFlag = 1;
    src.rotation = rotation;
    rga_set_rect(&src.rect, 0, 0, width, height, width, height, fmt);
    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.virAddr = g_ir_bo.ptr;
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, g_ir_img.width, g_ir_img.height,
                 g_ir_img.width, g_ir_img.height, fmt);
    if (c_RkRgaBlit(&src, &dst, NULL)) {
        printf("%s: rga fail\n", __func__);
        goto exit;
    }

    g_ir_state = IR_STATE_COMPLETE;
    ret = 0;

exit:
    return ret;
}

static void *rockface_control_detect_thread(void *arg)
{
    rockface_ret_t ret;
    rga_info_t src, dst;
    int det;
    struct face_buf *buf = NULL;

    while (g_run) {
        if (buf) {
            pthread_mutex_lock(&g_det_lock);
            g_det_free.push_back(buf);
            pthread_mutex_unlock(&g_det_lock);
        }
        buf = NULL;
        pthread_mutex_lock(&g_det_lock);
        if (g_det_ready.empty()) {
            pthread_mutex_unlock(&g_det_lock);
            rockface_control_detect_wait();
        } else {
            buf = g_det_ready.front();
            g_det_ready.pop_front();
            pthread_mutex_unlock(&g_det_lock);
        }
        if (!buf)
            continue;

        if (!g_run)
            break;

        det = rockface_control_detect(&buf->img, &buf->face);
        if (det) {
            if (det == -1)
                memset(last_name, 0, sizeof(last_name));
            continue;
        }

        if (!g_feature_flag)
            continue;

        if (g_feature.id == buf->id) {
            memcpy(&g_feature.face, &buf->face, sizeof(rockface_det_t));
            g_feature.face.box.left = buf->face.box.left * g_ratio;
            g_feature.face.box.top = buf->face.box.top * g_ratio;
            g_feature.face.box.right = buf->face.box.right * g_ratio;
            g_feature.face.box.bottom = buf->face.box.bottom * g_ratio;
            rockface_control_signal();
            g_feature.id = 0;
        } else if (g_feature.id < buf->id) {
            g_feature.id = 0;
            g_ir_state = IR_STATE_CANCELED;
        }
    }

    pthread_exit(NULL);
}

static void *rockface_control_feature_thread(void *arg)
{
    int index;
    struct face_data result_data;
    struct face_data *result;
    rockface_det_t face;
    struct timeval t0, t1;
    int del_timeout = 0;
    int reg_timeout = 0;
    bool real = false;
    bool ret;
    char result_name[NAME_LEN];
    int timeout;

    while (g_run) {
        pthread_mutex_lock(&g_mutex);
        g_feature_flag = true;
        pthread_mutex_unlock(&g_mutex);
        real = false;
        g_ir_state = IR_STATE_CANCELED;
        timeout = rockface_control_wait();
        if (!g_run)
            break;
        if (g_delete) {
            if (!del_timeout) {
                play_wav_signal(DELETE_START_WAV);
            }
            del_timeout++;
            if (del_timeout > 100) {
                del_timeout = 0;
                g_delete = false;
                play_wav_signal(DELETE_TIMEOUT_WAV);
            }
        } else {
            del_timeout = 0;
        }
        if (g_register && g_face_index < g_face_cnt) {
            if (!reg_timeout) {
                play_wav_signal(REGISTER_START_WAV);
            }
            reg_timeout++;
            if (reg_timeout > 100) {
                reg_timeout = 0;
                g_register = false;
                play_wav_signal(REGISTER_TIMEOUT_WAV);
            }
        } else if (g_register && g_face_index >= g_face_cnt) {
            g_register = false;
            g_register_cnt = 0;
            play_wav_signal(REGISTER_LIMIT_WAV);
        } else {
            reg_timeout = 0;
        }
        if (timeout == ETIMEDOUT)
            continue;
        memcpy(&face, &g_feature.face, sizeof(face));
        gettimeofday(&t0, NULL);
        ret = (struct face_data*)rockface_control_search(&g_feature.img, g_face_data, &g_face_index,
                        g_face_cnt, sizeof(struct face_data), 0, &face, reg_timeout, &result_data);
        if (ret) {
            result = &result_data;
        } else {
            result = NULL;
        }
        gettimeofday(&t1, NULL);
        if (g_delete && del_timeout && result) {
            rockface_control_delete(result->id, NULL, true);
            del_timeout = 0;
            g_delete = false;
            play_wav_signal(DELETE_SUCCESS_WAV);
            if (rkfacial_paint_info_cb)
                rkfacial_paint_info_cb(NULL, false);
        } else if (result && face.score > FACE_SCORE_RGB) {
            if (database_is_id_exist(result->id, result_name, NAME_LEN)) {
                if (rkcif_control_run()) {
                    if (g_ir_state == IR_STATE_COMPLETE) {
                        g_ir_state = IR_STATE_USING;
                        if (rockface_control_liveness_ir())
                            real = true;
                    }
                }
                if (rkfacial_paint_info_cb) {
                    struct user_info info;
                    memset(&info, 0, sizeof(info));
                    strncpy(info.sPicturePath, result_name, sizeof(info.sPicturePath) - 1);
                    db_monitor_get_user_info(&info, result->id);
                    rkfacial_paint_info_cb(&info, real);
                }
                if (!g_register && real && memcmp(last_name, result_name, sizeof(last_name))) {
                    char status[64];
                    char similarity[64];
                    char mark;
                    printf("name: %s\n", result_name);
                    memset(last_name, 0, sizeof(last_name));
                    strncpy(last_name, result_name, sizeof(last_name) - 1);
                    if (strstr(result_name, "black_list")) {
                        printf("%s in black_list\n", result_name);
                        snprintf(status, sizeof(status), "close");
                        mark = 'B';
                    } else {
                        snprintf(status, sizeof(status), "open");
                        mark = 'W';
                        play_wav_signal(PLEASE_GO_THROUGH_WAV);
                    }
                    snprintf(similarity, sizeof(similarity), "%f", face.score);
#ifdef USE_WEB_SERVER
                    memset(g_snap.name, 0, sizeof(g_snap.name));
                    if (!snapshot_run(&g_snap, &g_feature.img, &face, RK_FORMAT_RGB_888, 0, mark))
                        db_monitor_control_record_set(result->id, g_snap.name,
                                status, similarity);
#endif
                }
            }
        } else {
            if (rkfacial_paint_info_cb)
                rkfacial_paint_info_cb(NULL, false);
        }
        if (!real) {
            memset(last_name, 0, sizeof(last_name));
            pthread_mutex_lock(&g_rgb_track_mutex);
            g_rgb_track = -1;
            pthread_mutex_unlock(&g_rgb_track_mutex);
        }
#if 0
        if (face.score > FACE_SCORE_RGB)
            printf("box = (%d %d %d %d) score = %f\n", face.box.left, face.box.top,
                    face.box.right, face.box.bottom, face.score);
#endif
    }

    pthread_exit(NULL);
}

int rockface_control_init(void)
{
    int width = g_face_width;
    int height = g_face_height;
    rockface_ret_t ret;
    char cmd[256];

    if (!g_face_en)
        return 0;

    face_handle = rockface_create_handle();

    if (access(LICENCE_PATH, F_OK)) {
        check_pre_path(BAK_PATH);
        if (access(BAK_LICENCE_PATH, F_OK) == 0) {
            snprintf(cmd, sizeof(cmd), "cp %s %s", BAK_LICENCE_PATH, LICENCE_PATH);
            system(cmd);
        }
    }

    ret = rockface_set_licence(face_handle, LICENCE_PATH);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: authorization error %d!\n", __func__, ret);
        return -1;
    }
    ret = rockface_set_data_path(face_handle, FACE_DATA_PATH);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: set data path error %d!\n", __func__, ret);
        return -1;
    }

    ret = rockface_init_recognizer(face_handle);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: init recognizer error %d!\n", __func__, ret);
        return -1;
    }

    ret = rockface_init_detector2(face_handle, 1);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: init detector error %d!\n", __func__, ret);
        return -1;
    }

    ret = rockface_init_landmark(face_handle, 5);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: init landmark error %d!\n", __func__, ret);
        return -1;
    }

    ret = rockface_init_liveness_detector(face_handle);
    if (ret != ROCKFACE_RET_SUCCESS) {
        printf("%s: init liveness detector error %d!\n", __func__, ret);
        return -1;
    }

    if (g_face_cnt <= 0)
        g_face_cnt = DEFAULT_FACE_NUMBER;
    g_face_data = calloc(g_face_cnt, sizeof(struct face_data));
    if (!g_face_data) {
        printf("face data alloc failed!\n");
        return -1;
    }

    if (access(DATABASE_PATH, F_OK)) {
        check_pre_path(BAK_PATH);
        if (access(BAK_DATABASE_PATH, F_OK) == 0) {
            snprintf(cmd, sizeof(cmd), "cp %s %s", BAK_DATABASE_PATH, DATABASE_PATH);
            system(cmd);
        }
    }
    if (access(DATABASE_PATH, F_OK) == 0) {
        printf("load face feature from %s\n", DATABASE_PATH);
        if (database_init())
            return -1;
        g_face_index += database_get_data(g_face_data, g_face_cnt, sizeof(rockface_feature_t), 0,
                                          sizeof(int), sizeof(rockface_feature_t));
        database_exit();
    }

    if (database_init())
        return -1;
#ifndef USE_WEB_SERVER
    printf("load face feature from %s\n", DEFAULT_FACE_PATH);
    g_face_index += load_feature(DEFAULT_FACE_PATH, ".jpg",
                        (struct face_data*)g_face_data + g_face_index, g_face_cnt - g_face_index);
#endif
    printf("face number is %d\n", g_face_index);
    sync();
    if (rockface_control_init_library(g_face_data, g_face_index, sizeof(struct face_data), 0))
        return -1;

    for (int i = 0; i < DET_BUFFER_NUM; i++) {
        if (rga_control_buffer_init(&g_detect[i].bo, &g_detect[i].fd, DET_WIDTH, DET_HEIGHT, 24))
            return -1;
        pthread_mutex_lock(&g_det_lock);
        g_det_free.push_back(&g_detect[i]);
        pthread_mutex_unlock(&g_det_lock);
    }

    if (rga_control_buffer_init(&g_feature.bo, &g_feature.fd, width, height, 24))
        return -1;

    if (rga_control_buffer_init(&g_ir_bo, &g_ir_fd, width, height, 12))
        return -1;
    if (rga_control_buffer_init(&g_ir_det_bo, &g_ir_det_fd, DET_WIDTH, DET_HEIGHT, 24))
        return -1;

    g_run = true;
    if (pthread_create(&g_detect_tid, NULL, rockface_control_detect_thread, NULL)) {
        printf("%s: pthread_create error!\n", __func__);
        g_run = false;
        return -1;
    }
    if (pthread_create(&g_tid, NULL, rockface_control_feature_thread, NULL)) {
        printf("%s: pthread_create error!\n", __func__);
        g_run = false;
        return -1;
    }

    return 0;
}

void rockface_control_exit(void)
{
    if (!g_face_en)
        return;

    g_run = false;
    rockface_control_detect_signal();
    if (g_detect_tid) {
        pthread_join(g_detect_tid, NULL);
        g_detect_tid = 0;
    }
    rockface_control_signal();
    if (g_tid) {
        pthread_join(g_tid, NULL);
        g_tid = 0;
    }

    rockface_control_release_library();
    rockface_release_handle(face_handle);

    database_exit();

    if (g_face_data) {
        free(g_face_data);
        g_face_data = NULL;
    }

    for (int i = 0; i < DET_BUFFER_NUM; i++) {
        rga_control_buffer_deinit(&g_detect[i].bo, g_detect[i].fd);
    }
    rga_control_buffer_deinit(&g_feature.bo, g_feature.fd);
    rga_control_buffer_deinit(&g_ir_bo, g_ir_fd);
    rga_control_buffer_deinit(&g_ir_det_bo, g_ir_det_fd);
    snapshot_exit(&g_snap);
}

void rockface_control_database(void)
{
    pthread_mutex_lock(&g_lib_lock);
    memset(g_face_data, 0, g_face_cnt * sizeof(struct face_data));
    g_face_index = database_get_data(g_face_data, g_face_cnt,
            sizeof(rockface_feature_t), 0, sizeof(int), sizeof(rockface_feature_t));
    rockface_control_release_library();
    rockface_control_init_library(g_face_data, g_face_index,
            sizeof(struct face_data), 0);
    pthread_mutex_unlock(&g_lib_lock);
}

int rockface_control_delete(int id, const char *pname, bool notify)
{
    char name[NAME_LEN];

    if (!database_is_id_exist(id, name, NAME_LEN)) {
        if (pname && strlen(pname))
            unlink(pname);
        return -1;
    }

    printf("delete %d from %s\n", id, DATABASE_PATH);
    database_delete(id, true);
    if (strlen(name))
        unlink(name);
    if (notify)
        db_monitor_face_list_delete(id);

    rockface_control_database();

    return 0;
}

int rockface_control_add(int id, const char *name, void *feature)
{
    printf("add %s, %d to %s\n", name, id, DATABASE_PATH);
    if (feature) {
        char user[] = USER_NAME;
        char type[] = "whiteList";
        database_insert(feature, sizeof(rockface_feature_t), name, NAME_LEN, id, true);
        db_monitor_face_list_add(id, (char*)name, user, type);
    } else {
        rockface_feature_t f;
        if (!rockface_control_get_path_feature(name, &f)) {
            database_insert(&f, sizeof(rockface_feature_t), name, NAME_LEN, id, true);
        } else {
            printf("%s %s fail!\n", __func__, name);
            return -1;
        }
    }

    rockface_control_database();

    return 0;
}
