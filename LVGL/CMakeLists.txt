# The LVGL source tree to build against, selected in Kconfig. Only the upstream
# sources differ per version; the porting layer below is shared by both.
if(CONFIG_LVGL_9_3)
    set(c_LVGL_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/lvgl-9.3/lvgl)
elseif(CONFIG_LVGL_9_6)
    set(c_LVGL_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/lvgl-9.6/lvgl)
endif()

# Root of the shared porting layer: apps / config / hal / libs / platform.
set(c_LVGL_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

ameba_add_subdirectories_if(CONFIG_LVGL_9_3 lvgl-9.3)
ameba_add_subdirectories_if(CONFIG_LVGL_9_6 lvgl-9.6)

if(CONFIG_LVGL_9_3 OR CONFIG_LVGL_9_6)
    ameba_add_subdirectory(hal)
    ameba_add_subdirectory(libs)
    ameba_add_subdirectory(platform)
    ameba_add_subdirectories(apps)
endif()
