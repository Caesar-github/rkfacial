#ifndef __RKFACIAL_H__
#define __RKFACIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <rockface/rockface.h>

struct user_info {
    unsigned int id;
    char sPicturePath[256];
    char sRegistrationTime[32];
    unsigned int iAge;
    char sListType[32];
    char sType[32];
    char sName[256];
    char sGender[32];
    char sNation[32];
    char sCertificateType[32];
    char sCertificateNumber[32];
    char sBirthday[32];
    char sTelephoneNumber[32];
    char sHometown[32];
    char sAddress[32];
    unsigned int iAccessCardNumber;
};

typedef void (*display_callback)(void *ptr, int fd, int fmt, int w, int h, int rotation);

void set_isp_param(int width, int height, display_callback cb, bool expo);
void set_cif_param(int width, int height, display_callback cb);
void set_usb_param(int width, int height, display_callback cb);
void set_face_param(int width, int height, int cnt);

void set_isp_rotation(int angle);
void set_cif_rotation(int angle);
void set_usb_rotation(int angle);

int rkfacial_init(void);
void rkfacial_exit(void);
void rkfacial_register(void);
void rkfacial_delete(void);

typedef void (*rkfacial_paint_box_callback)(int left, int top, int right, int bottom);
void register_rkfacial_paint_box(rkfacial_paint_box_callback cb);
extern rkfacial_paint_box_callback rkfacial_paint_box_cb;

typedef void (*rkfacial_paint_info_callback)(struct user_info *info, bool real);
void register_rkfacial_paint_info(rkfacial_paint_info_callback cb);
extern rkfacial_paint_info_callback rkfacial_paint_info_cb;

void rkfacial_get_ir_face(rockface_det_t *face);

#ifdef __cplusplus
}
#endif

#endif
