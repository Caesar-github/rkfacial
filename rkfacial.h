#ifndef __RKFACIAL_H__
#define __RKFACIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*display_callback)(void *ptr, int fd, int fmt, int w, int h, int rotation);

void set_isp_param(int width, int height, display_callback cb, bool expo);
void set_cif_param(int width, int height, display_callback cb);
void set_usb_param(int width, int height, display_callback cb);
void set_face_param(int width, int height, int cnt);

int rkfacial_init(void);
void rkfacial_exit(void);
void rkfacial_register(void);
void rkfacial_delete(void);

typedef void (*rkfacial_paint_box_callback)(int left, int top, int right, int bottom);
void register_rkfacial_paint_box(rkfacial_paint_box_callback cb);
extern rkfacial_paint_box_callback rkfacial_paint_box_cb;

typedef void (*rkfacial_paint_name_callback)(char *name, bool real);
void register_rkfacial_paint_name(rkfacial_paint_name_callback cb);
extern rkfacial_paint_name_callback rkfacial_paint_name_cb;

#ifdef __cplusplus
}
#endif

#endif
