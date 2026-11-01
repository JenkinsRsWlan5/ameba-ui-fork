/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
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

#include "lv_ameba_jpeg.h"

#include "ameba_soc.h"
#include "os_wrapper.h"
#include "jpegdecapi.h"
#include "ppapi.h"

#include "lvgl.h"

#if LVGL_VERSION_MAJOR > 9 || (LVGL_VERSION_MAJOR == 9 && LVGL_VERSION_MINOR >= 6)
    #include "src/image/lv_image_decoder_private.h"
#else
    #include "src/draw/lv_image_decoder_private.h"
#endif

#define DECODER_NAME "JPEG_RTK"

#define TIME_DEBUG 0
#define FILE_TIME_DEBUG 0

static uint8_t *read_file(const char *filename, uint32_t *size)
{
#if FILE_TIME_DEBUG
    uint64_t start, end;
    uint64_t time_used;
    start = rtos_time_get_current_system_time_ns();
#endif

    uint8_t *data = NULL;
    lv_fs_file_t f;
    uint32_t data_size;
    uint32_t rn;
    lv_fs_res_t res;

    *size = 0;

    res = lv_fs_open(&f, filename, LV_FS_MODE_RD);
    if (res != LV_FS_RES_OK) {
        LV_LOG_WARN("can't open %s", filename);
        return NULL;
    }

    res = lv_fs_seek(&f, 0, LV_FS_SEEK_END);
    if (res != LV_FS_RES_OK) {
        goto failed;
    }

    res = lv_fs_tell(&f, &data_size);
    if (res != LV_FS_RES_OK) {
        goto failed;
    }

    res = lv_fs_seek(&f, 0, LV_FS_SEEK_SET);
    if (res != LV_FS_RES_OK) {
        goto failed;
    }

    /*Read file to buffer*/
    data = lv_malloc(data_size);
    if (data == NULL) {
        LV_LOG_WARN("malloc failed for data");
        goto failed;
    }

    res = lv_fs_read(&f, data, data_size, &rn);

    if (res == LV_FS_RES_OK && rn == data_size) {
        *size = rn;
    } else {
        LV_LOG_WARN("read file failed");
        lv_free(data);
        data = NULL;
    }

failed:
    lv_fs_close(&f);

#if FILE_TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    printf("Decode info Time used: %lld ns\n", time_used);
#endif

    return data;
}

static lv_color_format_t trans_format_hw2sw(int format)
{
    /* Refer to: jpeg_decoder/inc/jpegdecapi.h */

    switch (format) {
    case JPEGDEC_YCbCr400:
        return LV_COLOR_FORMAT_I400;

    case JPEGDEC_YCbCr420_SEMIPLANAR:
        return LV_COLOR_FORMAT_I420;

    case JPEGDEC_YCbCr422_SEMIPLANAR:
        return LV_COLOR_FORMAT_I422;

    case JPEGDEC_YCbCr440:
        return LV_COLOR_FORMAT_NV12;  // For YCbCr440

    case JPEGDEC_YCbCr411_SEMIPLANAR:
        return LV_COLOR_FORMAT_NV21;  // For YCbCr411

    case JPEGDEC_YCbCr444_SEMIPLANAR:
        return LV_COLOR_FORMAT_I444;
    default:
    }

    printf("Image decode-type invalid.\n");
    return LV_COLOR_FORMAT_UNKNOWN;
}

static int trans_format_sw2hw(lv_color_format_t format)
{
    /* Refer to: jpeg_decoder/inc/ppapi.h */
    switch (format) {
    case LV_COLOR_FORMAT_I420:
        return PP_PIX_FMT_YCBCR_4_2_0_SEMIPLANAR;

    case LV_COLOR_FORMAT_I422:
        return PP_PIX_FMT_YCBCR_4_2_2_SEMIPLANAR;

    case LV_COLOR_FORMAT_I400:
        return PP_PIX_FMT_YCBCR_4_0_0;

    case LV_COLOR_FORMAT_I444:
        return PP_PIX_FMT_YCBCR_4_4_4_SEMIPLANAR;

    case LV_COLOR_FORMAT_NV21:
        return PP_PIX_FMT_YCBCR_4_1_1_SEMIPLANAR;

    case LV_COLOR_FORMAT_NV12:
        return PP_PIX_FMT_YCBCR_4_4_0;

    default:
    }

    printf("Image pp-type invalid.\n");
    return LV_RESULT_INVALID;
}

static int purpose_pp_format(void)
{
#if LV_COLOR_DEPTH == 16
    return PP_PIX_FMT_RGB16_5_6_5;
#elif LV_COLOR_DEPTH == 24
    printf("PP does not support RGB888, PP does not support 24-bits mode.\n");
    return PP_PIX_FMT_RGB32;
#elif LV_COLOR_DEPTH == 32
    return PP_PIX_FMT_RGB32;
#endif
    printf("Purpose pp-type invalid.\n");
    return LV_RESULT_INVALID;
}

static int purpose_lv_format(void)
{
#if LV_COLOR_DEPTH == 16
    return LV_COLOR_FORMAT_RGB565;
#elif LV_COLOR_DEPTH == 24
    printf("PP does not support RGB888, PP does not support 24-bits mode.\n");
    return LV_COLOR_FORMAT_XRGB8888;
#elif LV_COLOR_DEPTH == 32
    return LV_COLOR_FORMAT_XRGB8888;
#endif
    printf("Purpose lv-type invalid.\n");
    return LV_RESULT_INVALID;
}

static lv_result_t decoder_info_cb(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc, lv_image_header_t *header)
{
    LV_UNUSED(decoder);
    const void *src = dsc->src;
    lv_image_src_t src_type = dsc->src_type;

    JpegDecImageInfo image_info;
    JpegDecInput jpeg_in;
    uint8_t *data = NULL;
    lv_result_t res = LV_RESULT_INVALID;

    if (src_type == LV_IMAGE_SRC_FILE) {
        uint32_t data_size;
        data = read_file((const char *)dsc->src, &data_size);
        if (data == NULL) {
            LV_LOG_WARN("can't load file %s", dsc->src);
            return LV_RESULT_INVALID;
        }

        memset(&jpeg_in, 0, sizeof(jpeg_in));
        jpeg_in.streamBuffer.pVirtualAddress = (u32 *)data;
        jpeg_in.streamBuffer.busAddress = (u32)data;
        jpeg_in.streamLength = data_size;
        DCache_Clean((u32)data, data_size);
    } else if (src_type == LV_IMAGE_SRC_VARIABLE) {
        const lv_image_dsc_t *img_dsc = src;
        if (img_dsc->data_size < 4) {
            return LV_RESULT_INVALID;
        }
        if (img_dsc->data[0] != 0xFF || img_dsc->data[1] != 0xD8 || img_dsc->data[2] != 0xFF) {
            return LV_RESULT_INVALID;
        }
        memset(&jpeg_in, 0, sizeof(jpeg_in));
        jpeg_in.streamBuffer.pVirtualAddress = (u32 *)img_dsc->data;
        jpeg_in.streamBuffer.busAddress = (u32)img_dsc->data;
        jpeg_in.streamLength = img_dsc->data_size;
    } else {
        return LV_RESULT_INVALID;
    }
#if TIME_DEBUG
    uint64_t start, end;
    uint64_t time_used;
    start = rtos_time_get_current_system_time_ns();
#endif

    JpegDecInst jpeg_inst;

    if (JpegDecInit(&jpeg_inst) != JPEGDEC_OK) {
        printf("Error: JpegDecInit Failed.\n");
        res = LV_RESULT_INVALID;
        goto end;
    }

    if (JpegDecGetImageInfo(jpeg_inst, &jpeg_in, &image_info) == JPEGDEC_OK) {
        header->w = image_info.outputWidth;
        header->h = image_info.outputHeight;
        header->cf = trans_format_hw2sw(image_info.outputFormat);
        res = LV_RESULT_OK;
    }

#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    printf("Decode info Time used: %lld ns\n", time_used);
#endif

    if (jpeg_inst) {
        JpegDecRelease(jpeg_inst);
    }
end:
    if (data) {
        lv_free(data);
    }

#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    printf("Get Info used: %lld ns\n", time_used);
#endif
    return res;
}

static lv_result_t decoder_open_cb(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);

    const uint8_t *jpeg_data_ptr = NULL;
    uint8_t *data = NULL;
    uint32_t jpeg_data_len = 0;

    if (dsc->src_type == LV_IMAGE_SRC_FILE) {

        data = read_file(dsc->src, &jpeg_data_len);
        if (data == NULL) {
            LV_LOG_WARN("can't load file %s", dsc->src);
            return LV_RESULT_INVALID;
        }

        jpeg_data_ptr = data;
        //DCache_Clean((uint32_t)jpeg_data_ptr, jpeg_data_len); // Clean before JpegDecDecode

    } else if (dsc->src_type == LV_IMAGE_SRC_VARIABLE) {
        const lv_image_dsc_t *img_dsc = dsc->src;
        jpeg_data_ptr = img_dsc->data;
        jpeg_data_len = img_dsc->data_size;
    } else {
        return LV_RESULT_INVALID;
    }
#if TIME_DEBUG
    uint64_t start, end;
    uint64_t time_used;
    start = rtos_time_get_current_system_time_ns();
#endif

    lv_result_t res = LV_RESULT_INVALID;

    JpegDecInst jpeg_inst;
    PPInst pp_inst;

    JpegDecInput jpeg_in;
    JpegDecOutput jpeg_out;
    PPConfig pp_conf;

    memset(&jpeg_in, 0, sizeof(jpeg_in));
    memset(&jpeg_out, 0, sizeof(jpeg_out));
    memset(&pp_conf, 0, sizeof(pp_conf));

    jpeg_in.streamBuffer.pVirtualAddress = (u32 *)jpeg_data_ptr;
    jpeg_in.streamBuffer.busAddress = (u32)jpeg_data_ptr;
    jpeg_in.streamLength = jpeg_data_len;

    if (JpegDecInit(&jpeg_inst) != JPEGDEC_OK) {
        printf("Error: JpegDecInit Failed.\n");
        res = LV_RESULT_INVALID;
        goto end;
    }

    if (PPInit(&pp_inst) != PP_OK) {
        printf("Error: PPInit Failed.\n");
        res = LV_RESULT_INVALID;
        goto end1;
    }

    if (PPDecCombinedModeEnable(pp_inst, jpeg_inst, PP_PIPELINED_DEC_TYPE_JPEG) != PP_OK) {
        printf("Error: PPDecCombinedModeEnable Failed.\n");
        res = LV_RESULT_INVALID;
        goto end2;
    }

    if (PPGetConfig(pp_inst, &pp_conf) != PP_OK) {
        printf("Error: PPGetConfig Failed.\n");
        goto end3;
    }

    pp_conf.ppInImg.width = dsc->header.w;
    pp_conf.ppInImg.height = dsc->header.h;
    /* Jessica */
    pp_conf.ppInImg.videoRange = 1;
    pp_conf.ppOutRgb.rgbTransform = PP_YCBCR2RGB_TRANSFORM_BT_709;

    pp_conf.ppInImg.pixFormat = trans_format_sw2hw(dsc->header.cf);
    pp_conf.ppOutImg.width = dsc->header.w;
    pp_conf.ppOutImg.height = dsc->header.h;

    pp_conf.ppOutImg.pixFormat = purpose_pp_format();

    lv_draw_buf_t *decoded_buf = NULL;
    uint32_t stride = lv_draw_buf_width_to_stride(dsc->header.w, purpose_lv_format());
    decoded_buf = lv_draw_buf_create(dsc->header.w, dsc->header.h, purpose_lv_format(), stride);

    if (!decoded_buf) {
        printf("decoded_buf create failed.\n");
        goto end3;
    }
    pp_conf.ppOutImg.bufferBusAddr = (u32)decoded_buf->data;

    //DCache_CleanInvalidate(pp_conf.ppOutImg.bufferBusAddr, dsc->header.w * dsc->header.h * LV_COLOR_DEPTH / 8);
    DCache_CleanInvalidate(0xFFFFFFFF, 0xFFFFFFFF); // Clean !!!!
    if (PPSetConfig(pp_inst, &pp_conf) != PP_OK) {
        printf("Error: PPSetConfig Failed.\n");
        goto end3;
    }

    if (JpegDecDecode(jpeg_inst, &jpeg_in, &jpeg_out) == JPEGDEC_FRAME_READY) {
        dsc->header.cf = purpose_lv_format(); // Format changed after PP process
        dsc->decoded = decoded_buf;
        res = LV_RESULT_OK;
    }
end3:
    if (pp_inst) {
        PPDecCombinedModeDisable(pp_inst, jpeg_inst);
    }
end2:
    if (pp_inst) {
        PPRelease(pp_inst);
    }
end1:
    if (jpeg_inst) {
        JpegDecRelease(jpeg_inst);
    }
end:
    if (data) {
        lv_free(data);
    }
    if (decoded_buf && (res != LV_RESULT_OK)) {
        printf("Decode open flow failed and release decoded_buf.\n");
        lv_draw_buf_destroy(decoded_buf);
    }
#if TIME_DEBUG
    end = rtos_time_get_current_system_time_ns();
    time_used = end - start;
    printf("Decode Time used: %lld ns\n", time_used);
#endif

    return res;
}

static void decoder_close_cb(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);

    if (dsc->decoded) {
        lv_draw_buf_destroy((lv_draw_buf_t *)dsc->decoded);
    }
}

void lv_ameba_jpeg_init(void) {
    lv_image_decoder_t *dec = lv_image_decoder_create();
    lv_image_decoder_set_info_cb(dec, decoder_info_cb);
    lv_image_decoder_set_open_cb(dec, decoder_open_cb);
    lv_image_decoder_set_close_cb(dec, decoder_close_cb);
    dec->name = DECODER_NAME;

    RCC_PeriphClockCmd(APBPeriph_MJPEG, APBPeriph_MJPEG_CLOCK, ENABLE);
    hx170dec_init();
}

void lv_ameba_jpeg_deinit(void) {
    // TODO
}
