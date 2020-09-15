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
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>

#include <list>

#include "aiq_control.h"
#include <mediactl/mediactl.h>
#include <mediactl/v4l2subdev.h>

#define IQFILES_PATH "/etc/iqfiles"
#define AIQ_WIDTH 2688
#define AIQ_HEIGHT 1520
#define IR_SENSOR_MEDIA_MODEL   "rkcif_mipi_lvds"
#define RGB_SENSOR_MEDIA_MODEL  "rkisp0"

struct aiq_control {
    rk_aiq_sys_ctx_t *ctx;
    aiq_control_type type;
};

static std::list<struct aiq_control *> g_aiq;

rk_aiq_sys_ctx_t *aiq_control_init(int width, int height, rk_aiq_working_mode_t mode, const char *sensor_entity_name)
{
    rk_aiq_sys_ctx_t *ctx;

    ctx = rk_aiq_uapi_sysctl_init(sensor_entity_name, IQFILES_PATH, NULL, NULL);
    if (!ctx) {
        printf("%s: rk_aiq_uapi_sysctl_init fail!\n", __func__);
        return NULL;
    }
    rk_aiq_uapi_sysctl_setMulCamConc(ctx, true);
    if (rk_aiq_uapi_sysctl_prepare(ctx, width, height, mode)) {
        printf("%s: rk_aiq_uapi_sysctl_prepare fail!\n", __func__);
        rk_aiq_uapi_sysctl_deinit(ctx);
        return NULL;
    }
    if (rk_aiq_uapi_sysctl_start(ctx) < 0) {
        printf("%s: rk_aiq_uapi_sysctl_start fail!\n", __func__);
        rk_aiq_uapi_sysctl_deinit(ctx);
        return NULL;
    }
    return ctx;
}

void aiq_control_deinit(rk_aiq_sys_ctx_t *ctx)
{
    if (ctx) {
        rk_aiq_uapi_sysctl_stop(ctx);
        rk_aiq_uapi_sysctl_deinit(ctx);
    }
}

static int enumrate_modules(struct media_device *device, char *name, size_t size)
{
    uint32_t nents, i;
    const char *dev_name = NULL;
    int active_sensor = -1;

    nents = media_get_entities_count(device);
    for (i = 0; i < nents; ++i) {
        struct media_entity *e;
        const struct media_entity_desc *ef;
        const struct media_link *link;

        e = media_get_entity(device, i);
        ef = media_entity_get_info(e);
        if (ef->type != MEDIA_ENT_T_V4L2_SUBDEV_SENSOR)
            continue;

        if (ef->name[0] != 'm' && ef->name[3] != '_') {
            fprintf(stderr, "sensor entity name format is incorrect\n");
            return -1;
        }

        dev_name = media_entity_get_devname(e);

        switch (ef->type) {
        case MEDIA_ENT_T_V4L2_SUBDEV_SENSOR:
            link = media_entity_get_link(e, 0);
            if (link && (link->flags & MEDIA_LNK_FL_ENABLED)) {
                active_sensor = 1;
                strncpy(name, ef->name, size);
            }
            break;
        default:
            break;
        }
    }

    if (active_sensor < 0) {
        fprintf(stderr, "Not sensor link is enabled, does sensor probe correctly?\n");
        return -1;
    }

    return 0;
}

static int aiq_control_get_sensor_entity_name(aiq_control_type type, char *name, size_t size)
{
    char mdev_path[64];
    int ret = -1;
    struct media_device *device = NULL;

    for (int i = 0;; i++) {
        sprintf(mdev_path, "/dev/media%d", i);
        if (access(mdev_path, F_OK))
            break;

        device = media_device_new(mdev_path);
        if (device == NULL) {
            printf("Failed to create media %s\n", mdev_path);
            continue;
        }

        if (media_device_enumerate(device) < 0) {
            printf("Failed to enumerate %s (%d)\n", mdev_path, ret);
            media_device_unref(device);
            continue;
        }

        const struct media_device_info *info = media_get_info(device);
        if (AIQ_CONTROL_IR == type && !strcmp(info->model, IR_SENSOR_MEDIA_MODEL) ||
                AIQ_CONTROL_RGB == type && !strcmp(info->model, RGB_SENSOR_MEDIA_MODEL)) {
            ret = enumrate_modules(device, name, size);
            media_device_unref(device);
            break;
        }

        media_device_unref(device);
    }

    return ret;
}

static void aiq_alloc(aiq_control_type type)
{
    char name[64];
    if (!aiq_control_get_sensor_entity_name(type, name, sizeof(name))) {
        rk_aiq_working_mode_t mode;
        if (type == AIQ_CONTROL_RGB)
            mode = RK_AIQ_WORKING_MODE_ISP_HDR2;
        else
            mode = RK_AIQ_WORKING_MODE_NORMAL;
        rk_aiq_sys_ctx_t *ctx = aiq_control_init(AIQ_WIDTH, AIQ_HEIGHT, mode, name);
        if (ctx) {
            struct aiq_control *aiq = (struct aiq_control *)calloc(1, sizeof(struct aiq_control));
            if (!aiq) {
                printf("%s alloc fail!\n", __func__);
                return;
            }
            aiq->type = type;
            aiq->ctx = ctx;
            g_aiq.push_back(aiq);
        }
    }
}

int aiq_control_alloc(void)
{
    aiq_alloc(AIQ_CONTROL_RGB);
    aiq_alloc(AIQ_CONTROL_IR);
    return 0;
}

void aiq_control_free(void)
{
    while (!g_aiq.empty()) {
        struct aiq_control *aiq = g_aiq.front();
        g_aiq.pop_front();
        aiq_control_deinit(aiq->ctx);
        free(aiq);
    }
}

void aiq_control_setExpGainRange(enum aiq_control_type type, paRange_t range)
{
    for (auto &aiq : g_aiq) {
        if (aiq->type == type) {
            rk_aiq_uapi_getExpGainRange(aiq->ctx, &range);
            rk_aiq_uapi_setExpGainRange(aiq->ctx, &range);
            break;
        }
    }
}
