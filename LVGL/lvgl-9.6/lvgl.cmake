##########################################################################################
## * This part defines public part of the component

set(public_includes)
set(public_definitions)
set(public_libraries)

#----------------------------------------#
# Component public part, user config begin
ameba_list_append(public_includes
    lvgl/include
    lvgl/include/lvgl
    lvgl/src
    lvgl/src/core
    lvgl/src/display
    lvgl/src/draw
    lvgl/src/drivers
    lvgl/src/font
    lvgl/src/indev
    lvgl/src/layouts
    lvgl/src/misc
    lvgl/src/osal
    lvgl/src/stdlib
    lvgl/src/themes
    lvgl/src/tick
    lvgl/src/widgets
    lvgl/demos/widgets
    lvgl/demos/stress
    lvgl/demos/benchmark
    lvgl/demos/music
    ${c_LVGL_ROOT_DIR}/config/${c_SOC_TYPE}
)

# Component public part, user config end
#----------------------------------------#

ameba_global_include(${public_includes})
ameba_global_define(${public_definitions})
ameba_global_library(${public_libraries})

##########################################################################################
## * This part defines private part of the component

set(private_sources)
set(private_includes)
set(private_definitions)
set(private_compile_options)

#------------------------------#
# Component private part, user config begin

file(GLOB_RECURSE LVGL_SRCS
    "lvgl/src/*.c"
    "lvgl/src/libs/libjpeg_turbo/lv_libjpeg_turbo.c"
)

list(FILTER LVGL_SRCS EXCLUDE REGEX "lvgl/src/draw/sw/blend/(neon|helium|arm2d|riscv_v|sve2)/.*\\.c$")

if(CONFIG_AMEBASMART AND CONFIG_NEON)
    ameba_list_append(LVGL_SRCS
        "lvgl/src/draw/sw/blend/neon/lv_draw_sw_blend_neon_to_rgb565.c"
        "lvgl/src/draw/sw/blend/neon/lv_draw_sw_blend_neon_to_rgb888.c"
    )
endif()

ameba_list_append(private_sources
    ${LVGL_SRCS}
)

if(CONFIG_LV_DEMO_ENABLE)
    file(GLOB LVGL_DEMOS_COMMON "lvgl/demos/*.c")
    ameba_list_append(private_sources ${LVGL_DEMOS_COMMON})
endif()

if(CONFIG_LV_DEMO_WIDGETS)
    file(GLOB_RECURSE LVGL_DEMO_WIDGETS "lvgl/demos/widgets/*.c")
    ameba_list_append(private_sources ${LVGL_DEMO_WIDGETS})
endif()

if(CONFIG_LV_DEMO_STRESS)
    file(GLOB_RECURSE LVGL_DEMO_STRESS "lvgl/demos/stress/*.c")
    ameba_list_append(private_sources ${LVGL_DEMO_STRESS})
endif()

if(CONFIG_LV_DEMO_MUSIC)
    file(GLOB_RECURSE LVGL_DEMO_MUSIC "lvgl/demos/music/*.c")
    ameba_list_append(private_sources ${LVGL_DEMO_MUSIC})
endif()

if(CONFIG_LV_DEMO_BENCHMARK)
    file(GLOB_RECURSE LVGL_DEMO_BENCHMARK "lvgl/demos/benchmark/*.c")
    file(GLOB_RECURSE LVGL_DEMO_WIDGETS "lvgl/demos/widgets/*.c")
    ameba_list_append(private_sources ${LVGL_DEMO_BENCHMARK} ${LVGL_DEMO_WIDGETS})
endif()

ameba_list_append(private_includes
    ${c_LVGL_ROOT_DIR}/config/${c_SOC_TYPE}
    ${c_CMPT_UI_DIR}/third_party/libjpeg-turbo
    ${c_CMPT_UI_DIR}/third_party/libpng
)

ameba_list_append(private_definitions
    LV_CONF_INCLUDE_SIMPLE
)

ameba_list_append(private_compile_options
    -Wno-error
)

# Component private part, user config end
#------------------------------#

ameba_add_internal_library(lvgl
    p_SOURCES
        ${private_sources}
    p_INCLUDES
        ${private_includes}
    p_DEFINITIONS
        ${private_definitions}
    p_COMPILE_OPTIONS
        ${private_compile_options}
)

##########################################################################################
