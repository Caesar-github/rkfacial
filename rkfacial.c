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
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <getopt.h>

#include "face_common.h"
#include "rockface_control.h"
#include "load_feature.h"
#include "play_wav.h"
#include "rkisp_control.h"
#include "rkcif_control.h"
#include "video_common.h"
#include "usb_camera.h"
#include "db_monitor.h"
#include "rkfacial.h"

int rkfacial_init(void)
{
    register_get_path_feature(rockface_control_get_path_feature);

    rkisp_control_init();

    rkcif_control_init();

    usb_camera_init();

    play_wav_thread_init();
    play_wav_signal(WELCOME_WAV);

    rockface_control_init_thread();

    db_monitor_init();

    return 0;
}

void rkfacial_exit(void)
{
    rkisp_control_exit();

    rkcif_control_exit();

    usb_camera_exit();

    rockface_control_exit();

    play_wav_thread_exit();
}

void rkfacial_delete(void)
{
    rockface_control_set_delete();
}

void rkfacial_register(void)
{
    rockface_control_set_register();
}

rkfacial_paint_box_callback rkfacial_paint_box_cb = NULL;
void register_rkfacial_paint_box(rkfacial_paint_box_callback cb)
{
    rkfacial_paint_box_cb = cb;
}

rkfacial_paint_info_callback rkfacial_paint_info_cb = NULL;
void register_rkfacial_paint_info(rkfacial_paint_info_callback cb)
{
    rkfacial_paint_info_cb = cb;
}
