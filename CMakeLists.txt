cmake_minimum_required(VERSION 3.8.2)

project(rkfacial)

set(SRC
    database.c
    rockface_control.cpp
    load_feature.c
    play_wav.c
    rkisp_control.c
    rkcif_control.c
    rga_control.c
    video_common.c
    usb_camera.c
    vpu_decode.c
    db_monitor.cpp
    vpu_encode.c
    snapshot.c
    rkfacial.c
    turbojpeg_decode.c
    rkdrm_display.c
    display.c
)

include_directories(${DRM_HEADER_DIR})

set(LIB rkisp rkisp_api rockface rknn_api drm rga pthread sqlite3 asound rockchip_mpp turbojpeg m)

if(DEFINED USE_WEB_SERVER)
include(FindPkgConfig)
pkg_check_modules (GLIB REQUIRED IMPORTED_TARGET glib-2.0)
pkg_check_modules (DBUS REQUIRED IMPORTED_TARGET dbus-1)
pkg_check_modules (SQLITE3 REQUIRED IMPORTED_TARGET sqlite3)
pkg_check_modules (JSON-C REQUIRED IMPORTED_TARGET json-c)
set(LIB ${LIB}
    IPCProtocol gdbus
    PkgConfig::GLIB
    PkgConfig::DBUS
    PkgConfig::SQLITE3
    PkgConfig::JSON-C
)
add_definitions(-DUSE_WEB_SERVER)
endif()

add_library(rkfacial SHARED ${SRC})
target_link_libraries(rkfacial ${LIB})

install(TARGETS rkfacial DESTINATION lib)
install(FILES rkfacial.h rga_control.h turbojpeg_decode.h display.h DESTINATION include/rkfacial)

install(DIRECTORY wav/cn/ DESTINATION ../etc)
