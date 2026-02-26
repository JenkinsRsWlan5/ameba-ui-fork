/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * All rights reserved.
 *
 * Licensed under the Realtek License, Version 1.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License from Realtek
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ameba_soc.h"
#include "os_wrapper.h"
#include "ameba_ppe.h"

#include "lvgl.h"
#include "lv_draw_ppe.h"

#include "src/misc/lv_types.h"
#include "src/draw/lv_draw.h"
#include "src/draw/lv_draw_private.h"
#include "src/misc/lv_area_private.h"
#include "src/draw/lv_draw_image_private.h"
#include "src/draw/lv_image_decoder_private.h"
#include "src/draw/lv_draw_mask_private.h"
#include "src/draw/sw/blend/lv_draw_sw_blend_private.h"
#include "src/draw/sw/lv_draw_sw.h"
#include "src/draw/lv_draw_image.h"

#if LV_USE_DRAW_PPE
#if LV_USE_PARALLEL_DRAW_DEBUG
    #include "lv_global.h"
#endif

#define LV_USE_PPE_THREAD           1

#define TIME_DEBUG                  0
#define PPE_DEBUG                   0
#define LOG_TAG                     "LV-PPE"

#define MIN_SIZE                    50
#define DRAW_UNIT_ID_PPE            4
#define PPE_BLOCK_ALIGN             16  // PP works best with 16x16 blocks

typedef struct {
    lv_draw_unit_t base_unit;
    lv_draw_task_t *task_act;
#if LV_USE_PPE_THREAD
    lv_thread_t thread;
    lv_thread_sync_t sync;
    bool exit_status;
    bool inited;
#endif
    rtos_sema_t ppe_sema;
    rtos_sema_t trans_sema;
} lv_draw_ppe_unit_t;

static lv_draw_ppe_unit_t *g_ppe_ctx = NULL;
static int32_t _ppe_evaluate(lv_draw_unit_t *draw_unit, lv_draw_task_t *task);
static int32_t _ppe_dispatch(lv_draw_unit_t *draw_unit, lv_layer_t *layer);
static int32_t _ppe_delete(lv_draw_unit_t *draw_unit);
static void _ppe_execute_drawing(lv_draw_ppe_unit_t *draw_ppe_unit);

#if LV_USE_PPE_THREAD
static void _ppe_render_thread_cb(void *param);
#endif

static void PPE_INTHandler_display(void)
{
    uint32_t irq_status = PPE_GetAllIntStatus();

    if (irq_status & PPE_BIT_INTR_ST_ALL_OVER) {
        PPE_ClearINTPendingBit(PPE_BIT_INTR_ST_ALL_OVER);
        rtos_sema_give(g_ppe_ctx->ppe_sema);
    }
}

void lv_draw_ppe_init(void)
{
    lv_draw_ppe_unit_t *draw_ppe_unit = lv_draw_create_unit(sizeof(lv_draw_ppe_unit_t));
    draw_ppe_unit->base_unit.evaluate_cb = _ppe_evaluate;
    draw_ppe_unit->base_unit.dispatch_cb = _ppe_dispatch;
    draw_ppe_unit->base_unit.delete_cb = _ppe_delete;
    draw_ppe_unit->base_unit.name = "PPE";
    draw_ppe_unit->task_act = NULL;
    g_ppe_ctx = draw_ppe_unit;

    RCC_PeriphClockCmd(APBPeriph_PPE, APBPeriph_PPE_CLOCK, ENABLE);
    rtos_sema_create(&g_ppe_ctx->ppe_sema, 0, RTOS_SEMA_MAX_COUNT);
    rtos_sema_create(&g_ppe_ctx->trans_sema, 0, RTOS_SEMA_MAX_COUNT);
    rtos_sema_give(g_ppe_ctx->trans_sema);
#if LV_USE_PPE_THREAD
    lv_thread_init(&draw_ppe_unit->thread, "ppdraw", LV_THREAD_PRIO_HIGH,
                _ppe_render_thread_cb, 8 * 1024, draw_ppe_unit);
#endif
}

void lv_draw_ppe_deinit(void)
{
    rtos_sema_delete(g_ppe_ctx->ppe_sema);
}

static inline bool _ppe_src_cf_supported(lv_color_format_t cf)
{
    switch(cf) {
        case LV_COLOR_FORMAT_RGB565:
        case LV_COLOR_FORMAT_RGB888:
            return true;
        case LV_COLOR_FORMAT_XRGB8888:
        case LV_COLOR_FORMAT_ARGB8888:
        default:
            return false;
    }
}

static bool _ppe_image_transform_supported(const lv_draw_image_dsc_t *draw_dsc)
{
    const lv_image_dsc_t *img_dsc = draw_dsc->src;

    bool has_recolor = draw_dsc->recolor_opa > LV_OPA_MIN;
    bool has_transform = draw_dsc->rotation != 0 ||
                        draw_dsc->scale_x != LV_SCALE_NONE ||
                        draw_dsc->scale_y != LV_SCALE_NONE;
    if (has_recolor && has_transform) return false;  // Can't do both

    if (img_dsc->header.w < PPE_BLOCK_ALIGN || img_dsc->header.h < PPE_BLOCK_ALIGN) return false;

    if (draw_dsc->rotation % 900 != 0) return false;  // Only 90° multiples

    if (draw_dsc->blend_mode != LV_BLEND_MODE_NORMAL) return false; //Unspupport

    // PP block alignment
    if (has_transform && (img_dsc->header.w % PPE_BLOCK_ALIGN || img_dsc->header.h % PPE_BLOCK_ALIGN)) {
        return false;
    }

    if (!_ppe_src_cf_supported(img_dsc->header.cf)) return false;

    return true;
}

static int32_t _ppe_evaluate(lv_draw_unit_t *u, lv_draw_task_t *t)
{
    LV_UNUSED(u);

#if PPE_DEBUG
    RTK_LOGI(LOG_TAG, "%s, type:%d.\n", __func__, t->type);
#endif
    switch(t->type) {
        case LV_DRAW_TASK_TYPE_FILL: {
            const lv_draw_fill_dsc_t *fill_dsc = (lv_draw_fill_dsc_t *)t->draw_dsc;
            if (fill_dsc->radius != 0 || fill_dsc->grad.dir != LV_GRAD_DIR_NONE) {
                return 0;  // No radius or gradient
            }

            if (t->preference_score > 70) {
                t->preference_score = 70;
                t->preferred_draw_unit_id = DRAW_UNIT_ID_PPE;
            }
            return 1;
        }

        case LV_DRAW_TASK_TYPE_IMAGE: {
            lv_draw_image_dsc_t *dsc = (lv_draw_image_dsc_t *)t->draw_dsc;
            if (!_ppe_image_transform_supported(dsc)) {
                //printf("pp image transform not supported.\n");
                return 0;
            }
            if (t->preference_score > 70) {
                t->preference_score = 70;
                t->preferred_draw_unit_id = DRAW_UNIT_ID_PPE;
            }

            return 1;
        }

        case LV_DRAW_TASK_TYPE_LAYER: {
            const lv_draw_image_dsc_t *img_dsc = (lv_draw_image_dsc_t *)t->draw_dsc;
            if (!_ppe_image_transform_supported(img_dsc)) {
                //printf("pp image transform not supported.\n");
                return 0;
            }
            if (t->preference_score > 70) {
                t->preference_score = 70;
                t->preferred_draw_unit_id = DRAW_UNIT_ID_PPE;
            }
            return 1;
        }
        case LV_DRAW_TASK_TYPE_LINE:
        {
            lv_draw_line_dsc_t *dsc = (lv_draw_line_dsc_t *)t->draw_dsc;
            int h_len = dsc->p2.x - dsc->p1.x;
            int v_len = dsc->p2.y - dsc->p1.y;
            if (dsc->round_end || dsc->round_start || (dsc->p1.x != dsc->p2.x && dsc->p1.y != dsc->p2.y)
                || (h_len < MIN_SIZE && h_len > 0) || (v_len < MIN_SIZE && v_len > 0) || dsc->dash_gap > 0) {
#if PPE_DEBUG
                RTK_LOGI(LOG_TAG, "SW (%d,%d) - (%d-%d)\n", (int)dsc->p1.x, (int)dsc->p1.y, (int)dsc->p2.x, (int)dsc->p2.y);
#endif
            } else if (t->preference_score > 70) {
                t->preference_score = 70;
                t->preferred_draw_unit_id = DRAW_UNIT_ID_PPE;
                return 1;
            }

            return 0;
        }
        case LV_DRAW_TASK_TYPE_MASK_RECTANGLE: {
            lv_draw_mask_rect_dsc_t *mask_rect_dsc = (lv_draw_mask_rect_dsc_t *)t->draw_dsc;
            if (mask_rect_dsc->radius != 0) {
                return 0;  // No radius
            }

            if (t->preference_score > 70) {
                t->preference_score = 70;
                t->preferred_draw_unit_id = DRAW_UNIT_ID_PPE;
            }
            return 1;
        }

        default:
            return 0;
    }
}

static int32_t _ppe_dispatch(lv_draw_unit_t *draw_unit, lv_layer_t *layer)
{
    lv_draw_ppe_unit_t *u = (lv_draw_ppe_unit_t *)draw_unit;

    if (u->task_act) {
        return 0;
    }

    lv_draw_task_t *t = lv_draw_get_available_task(layer, NULL, DRAW_UNIT_ID_PPE);
    if (t == NULL || t->preferred_draw_unit_id != DRAW_UNIT_ID_PPE) {
#if PPE_DEBUG
        //RTK_LOGI(LOG_TAG, "Not DRAW_UNIT_ID_PPE.\n");
        if (t) {
            RTK_LOGI(LOG_TAG, "t->preferred_draw_unit_id = %d.\n", t->preferred_draw_unit_id);
        }
#endif
        return LV_DRAW_UNIT_IDLE;
    }

    if (lv_draw_layer_alloc_buf(layer) == NULL) {
        RTK_LOGW(LOG_TAG, "draw malloc buffer failed.\n");
        return LV_DRAW_UNIT_IDLE;
    }

    t->state = LV_DRAW_TASK_STATE_IN_PROGRESS;
    u->task_act = t;

#if LV_USE_PPE_THREAD
    if (u->inited) {
        lv_thread_sync_signal(&u->sync);
    }
#else
    _ppe_execute_drawing(u);
    u->task_act->state = LV_DRAW_TASK_STATE_READY;
    u->task_act = NULL;
    lv_draw_dispatch_request();
#endif

    return 1;
}

static int32_t _ppe_delete(lv_draw_unit_t *draw_unit)
{
#if LV_USE_PPE_THREAD
    lv_draw_ppe_unit_t *u = (lv_draw_ppe_unit_t *)draw_unit;
    u->exit_status = true;
    if (u->inited)
        lv_thread_sync_signal(&u->sync);
    return lv_thread_delete(&u->thread);
#else
    LV_UNUSED(draw_unit);
    return 0;
#endif
}

static int _ppe_get_px_format(lv_color_format_t cf)
{
    switch(cf) {
        case LV_COLOR_FORMAT_RGB565:   return PPE_RGB565;
        case LV_COLOR_FORMAT_RGB888:   return PPE_RGB888;
        case LV_COLOR_FORMAT_XRGB8888:
        case LV_COLOR_FORMAT_ARGB8888: return PPE_ARGB8888;
        default: return PPE_ARGB8888;
    }
}

static int _ppe_get_px_bytes(lv_color_format_t cf)
{
    return lv_color_format_get_bpp(cf) / 8;
}

static void _ppe_draw_fill(lv_draw_task_t *t)
{
    lv_draw_fill_dsc_t *dsc = (lv_draw_fill_dsc_t *)t->draw_dsc;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    lv_area_t *area = &t->area;
    lv_area_t *clip = &t->clip_area;
    lv_area_t draw_area;
#if TIME_DEBUG
    uint64_t start, end;
    uint64_t time_used;
    start = rtos_time_get_current_system_time_ns();
#endif

    if (!lv_area_intersect(&draw_area, area, clip)) return;

    lv_draw_ppe_header_t src_header = {0};
    lv_draw_ppe_header_t dest_header = {0};
    lv_draw_ppe_configuration_t ppe_draw_conf = {0};

    uint32_t fill_width = lv_area_get_width(&draw_area);
    uint32_t fill_height = lv_area_get_height(&draw_area);
    lv_color32_t col32 = lv_color_to_32(dsc->color, dsc->opa);
    uint32_t color_abgr = (col32.alpha << 24) | (col32.blue << 16) | (col32.green << 8) | col32.red;
    void *dest_buf = lv_draw_layer_go_to_xy(layer, draw_area.x1 - layer->buf_area.x1,
                                                draw_area.y1 - layer->buf_area.y1);
    if (((draw_area.x2 - draw_area.x1 + 1) * (draw_area.y2 - draw_area.y1 + 1)) < (MIN_SIZE * MIN_SIZE))
    {
#if PPE_DEBUG
        RTK_LOGI(LOG_TAG, "Area too small, use sw fill.\n");
#endif
        lv_draw_sw_fill(t, t->draw_dsc, &t->area);
#if TIME_DEBUG
        end = rtos_time_get_current_system_time_ns();
        time_used = end - start;
        RTK_LOGI(LOG_TAG, "SW Fill (at:%ld-%ld, w:%lu, h:%lu), opa=%d, Time used: %lld ns\n",
            draw_area.x1, draw_area.y1, fill_width, fill_height, dsc->opa, time_used);
#endif
        return;
    }

    src_header.cf = LV_COLOR_FORMAT_ARGB8888;
    src_header.w = fill_width;
    src_header.h = fill_height;
    src_header.stride = draw_buf->header.w * _ppe_get_px_bytes(layer->color_format);
    src_header.color = color_abgr;
    dest_header.cf = layer->color_format;
    dest_header.w = fill_width;
    dest_header.h = fill_height;
    dest_header.stride = draw_buf->header.w * _ppe_get_px_bytes(layer->color_format);
    dest_header.color = 0xFFFFFFFF;
    ppe_draw_conf.src_buf = NULL;
    ppe_draw_conf.dest_buf = dest_buf;
    ppe_draw_conf.src_header = &src_header;
    ppe_draw_conf.dest_header = &dest_header;
    ppe_draw_conf.scale_x = 1.0f;
    ppe_draw_conf.scale_y = 1.0f;
    ppe_draw_conf.angle = 0;
    ppe_draw_conf.opa = dsc->opa;
    lv_draw_ppe_configure_and_start_transfer(&ppe_draw_conf);
#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    RTK_LOGI(LOG_TAG, "PPE Fill (%-3ld %-3ld %-3lu %-3lu) Time:%8lld, opa=%d\n",
        draw_area.x1, draw_area.y1, fill_width, fill_height, time_used, dsc->opa);
#endif
}

static void _ppe_img_draw_core(lv_draw_task_t *t,
    const lv_draw_image_dsc_t *draw_dsc,
    const lv_image_decoder_dsc_t *decoder_dsc,
    lv_draw_image_sup_t *sup,
    const lv_area_t *img_coords,
    const lv_area_t *clipped_img_area) {
    UNUSED(sup);
    UNUSED(clipped_img_area);

    const lv_draw_buf_t *decoded = decoder_dsc->decoded;
    const uint8_t *src_buf = decoded->data;
    const lv_image_header_t *header = &decoded->header;
    uint32_t img_stride = decoded->header.stride;

    if (!src_buf) {
        RTK_LOGE(LOG_TAG, "Image data is NULL\n");
        return;
    }

    lv_layer_t *layer = t->target_layer;
    uint32_t img_cf = header->cf;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    uint32_t bytes_per_pixel = _ppe_get_px_bytes(draw_buf->header.cf);
    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, img_coords, &t->clip_area)) return;
    if (!lv_area_intersect(&blend_area, &blend_area, img_coords)) return;

    lv_draw_ppe_header_t src_header = {0};
    lv_draw_ppe_header_t dest_header = {0};
    lv_draw_ppe_configuration_t ppe_draw_conf = {0};
    uint32_t layer_stride_byte = layer->draw_buf->header.stride;
    uint32_t img_width = lv_area_get_width(&blend_area);
    uint32_t img_height = lv_area_get_height(&blend_area);
    uint32_t src_px_size = lv_color_format_get_bpp(img_cf);

    uint32_t scale_width = img_width;
    uint32_t scale_height = img_height;
    uint32_t target_width = img_width;
    uint32_t target_height = img_height;
    float scale_x = 1.0f;
    float scale_y = 1.0f;

    if (draw_dsc->scale_x != LV_SCALE_NONE || draw_dsc->scale_y != LV_SCALE_NONE) {
        scale_x = 1.0f * 65536 / (uint32_t)(65536 / draw_dsc->scale_x) / LV_SCALE_NONE;
        scale_y = 1.0f * 65536 / (uint32_t)(65536 / draw_dsc->scale_y) / LV_SCALE_NONE;
        scale_width = img_width * scale_x;
        scale_height = img_height * scale_y;
    }

    src_buf += img_stride * (blend_area.y1 - img_coords->y1);
    src_buf += ((blend_area.x1 - img_coords->x1) * src_px_size) >> 3;
    lv_area_move(&blend_area, -layer->buf_area.x1, -layer->buf_area.y1);

    int32_t dest_offset = LV_MAX((blend_area.y1 * draw_buf->header.w + blend_area.x1) * bytes_per_pixel, 0);

#if TIME_DEBUG
    uint64_t start, end, time_used;
    start = rtos_time_get_current_system_time_ns();
#endif
    target_width = LV_MAX(img_width, scale_width);
    target_height = LV_MAX(img_height, scale_height);
    src_header.cf = img_cf;
    src_header.w = target_width;
    src_header.h = target_height;
    src_header.stride = img_stride;
    src_header.color = 0xFFFFFFFF;
    src_header.min_x = scale_x < 1.0f ? (img_width - scale_width) / 2 : 0;
    src_header.min_y = scale_y < 1.0f ? (img_height - scale_height) / 2 : 0;

    dest_header.cf = layer->color_format;
    dest_header.w = target_width;
    dest_header.h = target_height;
    dest_header.stride = layer_stride_byte;
    dest_header.color = 0xFFFFFFFF;
    ppe_draw_conf.src_buf = (void*)src_buf;
    ppe_draw_conf.dest_buf = draw_buf->data + dest_offset;
    ppe_draw_conf.src_header = &src_header;
    ppe_draw_conf.dest_header = &dest_header;
    ppe_draw_conf.scale_x = scale_x;
    ppe_draw_conf.scale_y = scale_y;
    ppe_draw_conf.angle = draw_dsc->rotation / 10;
    ppe_draw_conf.opa = (lv_color_format_has_alpha(img_cf) && !layer->all_tasks_added) ? LV_OPA_TRANSP : LV_OPA_COVER;
    lv_draw_ppe_configure_and_start_transfer(&ppe_draw_conf);

#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    RTK_LOGI(LOG_TAG, "PPE Imag (%-3ld %-3ld %-3lu %-3lu) Time:%8lld, cf:%lu-%d offset:%lu, layer:%d\n",
        layer->buf_area.x1, layer->buf_area.y1, target_width, target_height,
        time_used, img_cf, layer->all_tasks_added, dest_offset, (int)draw_dsc->base.user_data);
#endif
}

static void lv_draw_ppe_image(lv_draw_task_t *t, const lv_draw_image_dsc_t *draw_dsc,
    const lv_area_t *coords)
{
    if(!draw_dsc->tile) {
        lv_draw_image_normal_helper(t, draw_dsc, coords, _ppe_img_draw_core);
    } else {
        lv_draw_image_tiled_helper(t, draw_dsc, coords, _ppe_img_draw_core);
    }
}

static void _ppe_draw_line(lv_draw_task_t *t)
{
#if TIME_DEBUG
    uint64_t start, end, time_used;
    start = rtos_time_get_current_system_time_ns();
#endif
    lv_draw_line_dsc_t *dsc = (lv_draw_line_dsc_t *)t->draw_dsc;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    lv_area_t *clip = &t->clip_area;
    lv_area_t draw_area;

    if (dsc->width == 0) return;
    if (dsc->opa <= LV_OPA_MIN) return;

    if (dsc->p1.x == dsc->p2.x && dsc->p1.y == dsc->p2.y) return;

    lv_area_t clip_line;
    clip_line.x1 = (int32_t)LV_MIN(dsc->p1.x, dsc->p2.x) - dsc->width / 2;
    clip_line.x2 = (int32_t)LV_MAX(dsc->p1.x, dsc->p2.x) + dsc->width / 2;
    clip_line.y1 = (int32_t)LV_MIN(dsc->p1.y, dsc->p2.y) - dsc->width / 2;
    clip_line.y2 = (int32_t)LV_MAX(dsc->p1.y, dsc->p2.y) + dsc->width / 2;

    if (!lv_area_intersect(&draw_area, &clip_line, clip)) return;

    lv_draw_ppe_header_t src_header = {0};
    lv_draw_ppe_header_t dest_header = {0};
    lv_draw_ppe_configuration_t ppe_draw_conf = {0};
    lv_area_t buf_area = layer->buf_area;
    lv_area_move(&draw_area, -buf_area.x1, -buf_area.y1);
    uint32_t line_width = lv_area_get_width(&draw_area);
    uint32_t line_height = lv_area_get_height(&draw_area);
    lv_color32_t col32 = lv_color_to_32(dsc->color, dsc->opa);
    uint32_t color_abgr = (col32.alpha << 24) | (col32.blue << 16) | (col32.green << 8) | col32.red;
    int32_t offset = (draw_area.y1 * draw_buf->header.w + draw_area.x1) * _ppe_get_px_bytes(layer->color_format);

    src_header.cf = LV_COLOR_FORMAT_ARGB8888;
    src_header.w = line_width;
    src_header.h = line_height;
    src_header.stride = line_width * _ppe_get_px_bytes(layer->color_format);
    src_header.color = color_abgr;
    dest_header.cf = layer->color_format;
    dest_header.w = line_width;
    dest_header.h = line_height;
    dest_header.stride = draw_buf->header.w * _ppe_get_px_bytes(layer->color_format);
    ppe_draw_conf.src_buf = NULL;
    ppe_draw_conf.dest_buf = draw_buf->data + offset;
    ppe_draw_conf.src_header = &src_header;
    ppe_draw_conf.dest_header = &dest_header;
    ppe_draw_conf.scale_x = 1.0f;
    ppe_draw_conf.scale_y = 1.0f;
    ppe_draw_conf.angle = 0;
    ppe_draw_conf.opa = dsc->opa;
    lv_draw_ppe_configure_and_start_transfer(&ppe_draw_conf);

#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    RTK_LOGI(LOG_TAG, "PPE Line (%-3ld %-3ld %-3lu %-3lu) Time:%8lld\n",
        draw_area.x1, draw_area.y1, line_width, line_height, time_used);
#endif
}

static void _ppe_draw_mask_rect(lv_draw_task_t *t)
{
#if TIME_DEBUG
    uint64_t start, end, time_used;
    start = rtos_time_get_current_system_time_ns();
#endif

    lv_draw_mask_rect_dsc_t *dsc = (lv_draw_mask_rect_dsc_t *)t->draw_dsc;
    lv_area_t draw_area;
    if (!lv_area_intersect(&draw_area, &dsc->area, &t->clip_area)) return;

    lv_layer_t *layer = t->target_layer;
    lv_area_t *buf_area = &layer->buf_area;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    lv_draw_ppe_header_t src_header = {0};
    lv_draw_ppe_header_t dest_header = {0};
    lv_draw_ppe_configuration_t ppe_draw_conf = {0};
    /*Clear the top part*/
    lv_area_set(&draw_area, t->clip_area.x1, t->clip_area.y1, t->clip_area.x2,
                dsc->area.y1 - 1);
    lv_area_move(&draw_area, -buf_area->x1, -buf_area->y1);

    /*Clear the bottom part*/
    lv_area_set(&draw_area, t->clip_area.x1, dsc->area.y2 + 1, t->clip_area.x2,
                t->clip_area.y2);
    lv_area_move(&draw_area, -buf_area->x1, -buf_area->y1);

    /*Clear the left part*/
    lv_area_set(&draw_area, t->clip_area.x1, dsc->area.y1, dsc->area.x1 - 1, dsc->area.y2);
    lv_area_move(&draw_area, -buf_area->x1, -buf_area->y1);

    /*Clear the right part*/
    lv_area_set(&draw_area, dsc->area.x2 + 1, dsc->area.y1, t->clip_area.x2, dsc->area.y2);
    lv_area_move(&draw_area, -buf_area->x1, -buf_area->y1);

    src_header.cf = LV_COLOR_FORMAT_ARGB8888;
    src_header.w = draw_buf->header.w;
    src_header.h = draw_buf->header.h;
    dest_header.cf = layer->color_format;
    dest_header.w = draw_buf->header.w - (draw_area.x2 - draw_area.x1);
    dest_header.h = draw_buf->header.h - (draw_area.y2 - draw_area.y1);
    dest_header.stride = draw_buf->header.w * _ppe_get_px_bytes(layer->color_format);
    ppe_draw_conf.src_buf = NULL;
    ppe_draw_conf.dest_buf = draw_buf->data;
    ppe_draw_conf.src_header = &src_header;
    ppe_draw_conf.dest_header = &dest_header;
    ppe_draw_conf.scale_x = 1.0f;
    ppe_draw_conf.scale_y = 1.0f;
    ppe_draw_conf.angle = 0;
    ppe_draw_conf.opa = LV_OPA_COVER;
    lv_draw_ppe_configure_and_start_transfer(&ppe_draw_conf);
#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    RTK_LOGI(LOG_TAG, "PPE Mask (%-3ld %-3ld %-3lu %-3lu) Time:%8lld\n",
        draw_area.x1, draw_area.y1, draw_buf->header.w, draw_buf->header.h, time_used);
#endif
}

void lv_draw_ppe_configure_and_start_transfer(lv_draw_ppe_configuration_t *ppe_draw_conf) {
    rtos_sema_take(g_ppe_ctx->trans_sema, RTOS_MAX_TIMEOUT);
    uint8_t input_layer_id = PPE_INPUT_LAYER1_INDEX;
    PPE_InputLayer_InitTypeDef Input_Layer;
    PPE_InputLayer_StructInit(&Input_Layer);
    Input_Layer.src_addr       = (uint32_t)ppe_draw_conf->src_buf;
    Input_Layer.pic_width      = ppe_draw_conf->src_header->w;
    Input_Layer.pic_height     = ppe_draw_conf->src_header->h;
    Input_Layer.format         = _ppe_get_px_format(ppe_draw_conf->src_header->cf);
    Input_Layer.pic_src        = ppe_draw_conf->src_buf ? PPE_LAYER_SRC_FROM_DMA : PPE_LAYER_SRC_CONST;
    Input_Layer.interp         = PPE_INTERP_TYPE_Nearest_Neighbor;
    Input_Layer.key_mode       = PPE_KEY_MODE_DISABLE;
    Input_Layer.line_len       = ppe_draw_conf->src_header->stride;
    Input_Layer.const_ABGR8888_value = ppe_draw_conf->src_header->color;
    Input_Layer.win_min_x      = ppe_draw_conf->src_header->min_x;
    Input_Layer.win_min_y      = ppe_draw_conf->src_header->min_y;
    Input_Layer.win_max_x      = Input_Layer.pic_width;
    Input_Layer.win_max_y      = Input_Layer.pic_height;
    Input_Layer.key_min_bgr    = 0;
    Input_Layer.key_max_bgr    = 0;
    Input_Layer.scale_x        = ppe_draw_conf->scale_x;
    Input_Layer.scale_y        = ppe_draw_conf->scale_y;
    // Layer2 and layer3 can't support rotation
    if (ppe_draw_conf->angle && !lv_color_format_has_alpha(ppe_draw_conf->src_header->cf)) {
        Input_Layer.angle = ppe_draw_conf->angle;
        if (ppe_draw_conf->angle == 90 || ppe_draw_conf->angle == 270) {
            Input_Layer.win_max_x = ppe_draw_conf->src_header->h;
            Input_Layer.win_max_y = ppe_draw_conf->src_header->w;
        }
    }

    if (ppe_draw_conf->opa < LV_OPA_MAX) {
        input_layer_id = PPE_INPUT_LAYER2_INDEX;
        PPE_InputLayer_InitTypeDef BG_Layer;
        PPE_InputLayer_StructInit(&BG_Layer);
        BG_Layer.src_addr               = (uint32_t)ppe_draw_conf->dest_buf;
        BG_Layer.pic_width              = ppe_draw_conf->dest_header->w;
        BG_Layer.pic_height             = ppe_draw_conf->dest_header->h;
        BG_Layer.format                 = _ppe_get_px_format(ppe_draw_conf->dest_header->cf);
        BG_Layer.pic_src                = PPE_LAYER_SRC_FROM_DMA;
        BG_Layer.interp                 = PPE_INTERP_TYPE_Nearest_Neighbor;
        BG_Layer.key_mode               = PPE_KEY_MODE_DISABLE;
        BG_Layer.line_len               = ppe_draw_conf->dest_header->stride;
        BG_Layer.const_ABGR8888_value   = 0xFFFFFFFF;
        BG_Layer.win_min_x              = 0;
        BG_Layer.win_min_y              = 0;
        BG_Layer.win_max_x              = BG_Layer.pic_width;
        BG_Layer.win_max_y              = BG_Layer.pic_height;
        BG_Layer.key_min_bgr            = 0;
        BG_Layer.key_max_bgr            = 0;
        BG_Layer.scale_x                = 1.0f;
        BG_Layer.scale_y                = 1.0f;
        BG_Layer.angle                  = 0;
        PPE_InitInputLayer(PPE_INPUT_LAYER1_INDEX, &BG_Layer);
    }

    PPE_InitInputLayer(input_layer_id, &Input_Layer);
    PPE_ResultLayer_InitTypeDef Result_Layer;
    PPE_ResultLayer_StructInit(&Result_Layer);
    Result_Layer.src_addr       = (uint32_t)ppe_draw_conf->dest_buf;
    Result_Layer.pic_width      = ppe_draw_conf->dest_header->w;
    Result_Layer.pic_height     = ppe_draw_conf->dest_header->h;
    Result_Layer.format         = _ppe_get_px_format(ppe_draw_conf->dest_header->cf);
    Result_Layer.bg_src         = PPE_BACKGROUND_SOURCE_LAYER1;
    Result_Layer.line_len       = ppe_draw_conf->dest_header->stride;
    Result_Layer.const_bg       = 0xFFFFFFFF;

    if (Input_Layer.angle == 90 || Input_Layer.angle == 270) {
        Result_Layer.blk_width      = 16;
        Result_Layer.blk_height     = 16;
    } else {
        Result_Layer.blk_width      = Result_Layer.pic_width;
        Result_Layer.blk_height     = Result_Layer.pic_height;
    }

    PPE_InitResultLayer(&Result_Layer);
    DCache_CleanInvalidate(0xFFFFFFFF, 0xFFFFFFFF);

    if (input_layer_id == PPE_INPUT_LAYER2_INDEX) {
        PPE_LayerEn(PPE_INPUT_LAYER1_BIT | PPE_INPUT_LAYER2_BIT);
    } else {
        PPE_LayerEn(PPE_INPUT_LAYER1_BIT);
    }

    InterruptRegister((IRQ_FUN)PPE_INTHandler_display, PPE_IRQ, (uint32_t)NULL, INT_PRI_MIDDLE);
    InterruptEn(PPE_IRQ, INT_PRI_MIDDLE);
    PPE_MaskINTConfig(PPE_BIT_INTR_ST_ALL_OVER, ENABLE);
    PPE_Cmd(ENABLE);
    rtos_sema_take(g_ppe_ctx->ppe_sema, RTOS_MAX_TIMEOUT);
    rtos_sema_give(g_ppe_ctx->trans_sema);
}

static void _ppe_execute_drawing(lv_draw_ppe_unit_t *u)
{
    lv_draw_task_t *t = u->task_act;
    lv_layer_t *layer = t->target_layer;

#if LV_USE_PARALLEL_DRAW_DEBUG
    t->draw_unit = &u->base_unit;
#endif

    lv_draw_buf_invalidate_cache(layer->draw_buf, &t->area);

    switch(t->type) {
        case LV_DRAW_TASK_TYPE_FILL:
            _ppe_draw_fill(t);
            break;
        case LV_DRAW_TASK_TYPE_IMAGE:
            //lv_draw_sw_image(t, t->draw_dsc, &t->area);
            lv_draw_ppe_image(t, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_LAYER: {
            // Same as image draw, but src is layer
            lv_draw_image_dsc_t *draw_dsc = (lv_draw_image_dsc_t *)t->draw_dsc;
            lv_layer_t *layer_to_draw = (lv_layer_t *)draw_dsc->src;

            /*It can happen that nothing was draw on a layer and therefore its buffer is not allocated.
            *In this case just return. */

            if (layer_to_draw->draw_buf == NULL) return;

            lv_draw_image_dsc_t new_draw_dsc = *draw_dsc;
            new_draw_dsc.src = layer_to_draw->draw_buf;
            if (draw_dsc->bitmap_mask_src) {
                lv_draw_sw_image(t, &new_draw_dsc, &t->area);
            } else {
                /*The source should be a draw_buf, not a layer*/
                new_draw_dsc.base.user_data = (void*)0x1;
                lv_draw_ppe_image(t, &new_draw_dsc, &t->area);
                //lv_draw_sw_image(t, &new_draw_dsc, &t->area);
            }
            break;
        }
        case LV_DRAW_TASK_TYPE_LINE:
            _ppe_draw_line(t);
            //lv_draw_sw_line(t, t->draw_dsc);
            break;
        case LV_DRAW_TASK_TYPE_MASK_RECTANGLE: {
            _ppe_draw_mask_rect(t);
            break;
        }
        default:
            RTK_LOGW(LOG_TAG, "%s: %d type else.\n", __func__, __LINE__);
            break;
    }
}

#if LV_USE_PPE_THREAD
static void _ppe_render_thread_cb(void *ptr)
{
    lv_draw_ppe_unit_t *u = (lv_draw_ppe_unit_t *)ptr;
    lv_thread_sync_init(&u->sync);
    u->inited = true;

    while(1) {
        while(u->task_act == NULL) {
            if (u->exit_status) break;
            lv_thread_sync_wait(&u->sync);
        }
        if (u->exit_status) break;

        _ppe_execute_drawing(u);

        u->task_act->state = LV_DRAW_TASK_STATE_READY;
        u->task_act = NULL;
        lv_draw_dispatch_request();
    }

    u->inited = false;
    lv_thread_sync_delete(&u->sync);
}
#endif

#endif /* LV_USE_DRAW_PPE */