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

#define LCD_BLEN_SRGB               _PB_3

#include "st7262.h"
#include "os_wrapper.h"
#include "stdlib.h"

static ST7262VBlankCallback *g_callback = NULL;
static void *g_data = NULL;

#define WIDTH                       800
#define HEIGHT                      480
#define MEM_SIZE                    (WIDTH * HEIGHT * 3)
#define LCDC_LINE_NUM_INTR_DEF      (HEIGHT * 5/ 6)

static u8 *g_buffer = NULL;
static int g_image_format = 0;
volatile int refresh = 0;

static struct LCDC_IRQInfoDef {
    u32 IrqNum;
    u32 IrqData;
    u32 IrqPriority;
} gLcdcIrqInfo;

/* config pinmux and control blen pad */
static void lcdc_pinmux_config(void)
{
    RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "%s \r\n", __func__);

    GPIO_InitTypeDef GPIO_InitStruct_Display;

    GPIO_InitStruct_Display.GPIO_Pin = _PA_17;
    GPIO_InitStruct_Display.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(&GPIO_InitStruct_Display);
    GPIO_WriteBit(_PA_17, 1);

    /* LCD BLEN Pin for ST7262.
    high: BL enable; low: BL disable */
    GPIO_InitTypeDef GPIO_InitStruct_BLEN;
    GPIO_InitStruct_BLEN.GPIO_Pin = LCD_BLEN_SRGB;
    GPIO_InitStruct_BLEN.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(&GPIO_InitStruct_BLEN);

    GPIO_WriteBit(LCD_BLEN_SRGB, 1);

    /* LCD Signal for RGB interface in HV mode
    DE signal is required for LCD SYNC-DE mode */
    Pinmux_Config(_PB_15, PINMUX_FUNCTION_LCD_D0);    /*D0 - B0*/
    Pinmux_Config(_PB_17, PINMUX_FUNCTION_LCD_D1);    /*D1*/
    Pinmux_Config(_PB_21, PINMUX_FUNCTION_LCD_D2);    /*D2*/
    Pinmux_Config(_PB_18, PINMUX_FUNCTION_LCD_D3);    /*D3*/
    Pinmux_Config(_PA_6, PINMUX_FUNCTION_LCD_D4);     /*D4*/
    Pinmux_Config(_PA_8, PINMUX_FUNCTION_LCD_D5);     /*D5*/
    Pinmux_Config(_PA_7, PINMUX_FUNCTION_LCD_D6);     /*D6*/
    Pinmux_Config(_PA_10, PINMUX_FUNCTION_LCD_D7);    /*D7 - B7 */

    Pinmux_Config(_PB_9, PINMUX_FUNCTION_LCD_D8);     /*D8 - G0*/
    Pinmux_Config(_PB_11, PINMUX_FUNCTION_LCD_D9);    /*D9*/
    Pinmux_Config(_PB_10, PINMUX_FUNCTION_LCD_D10);   /*D10*/
    Pinmux_Config(_PB_16, PINMUX_FUNCTION_LCD_D11);   /*D11*/
    Pinmux_Config(_PB_22, PINMUX_FUNCTION_LCD_D12);   /*D12*/
    Pinmux_Config(_PB_23, PINMUX_FUNCTION_LCD_D13);   /*D13*/
    Pinmux_Config(_PB_14, PINMUX_FUNCTION_LCD_D14);   /*D14*/
    Pinmux_Config(_PB_12, PINMUX_FUNCTION_LCD_D15);   /*D15 - G7*/

    Pinmux_Config(_PA_22, PINMUX_FUNCTION_LCD_D16);   /*D16 - R0*/
    Pinmux_Config(_PA_25, PINMUX_FUNCTION_LCD_D17);   /*D17 */
    Pinmux_Config(_PA_29, PINMUX_FUNCTION_LCD_D18);   /*D18*/
    Pinmux_Config(_PB_4, PINMUX_FUNCTION_LCD_D19);    /*D19*/
    Pinmux_Config(_PB_5, PINMUX_FUNCTION_LCD_D20);    /*D20*/
    Pinmux_Config(_PB_6, PINMUX_FUNCTION_LCD_D21);    /*D21*/
    Pinmux_Config(_PB_7, PINMUX_FUNCTION_LCD_D22);    /*D22*/
    Pinmux_Config(_PB_8, PINMUX_FUNCTION_LCD_D23);    /*D23 - R7*/

    Pinmux_Config(_PA_16, PINMUX_FUNCTION_LCD_RGB_HSYNC);    /*RD, HSYNC*/
    Pinmux_Config(_PA_13, PINMUX_FUNCTION_LCD_RGB_VSYNC);    /*VSYNC-TE, VSYNC*/
    Pinmux_Config(_PA_9, PINMUX_FUNCTION_LCD_RGB_DCLK);      /*WR, DCLK*/
    Pinmux_Config(_PA_14, PINMUX_FUNCTION_LCD_RGB_DE);       /*SYNC-DE*/
}

static void lcdc_irq_handler(void)
{
    volatile u32 IntId;

    IntId = LCDC_GetINTStatus(LCDC);
    LCDC_ClearINT(LCDC, IntId);

    RTK_LOGS(NOTAG, RTK_LOG_DEBUG, "irq 0x%x \r\n", IntId);

    if (IntId & LCDC_BIT_LCD_FRD_INTS) {
        RTK_LOGS(NOTAG, RTK_LOG_DEBUG, "intr: frame done \r\n");
    }

    if (IntId & LCDC_BIT_LCD_LIN_INTS) {
        RTK_LOGS(NOTAG, RTK_LOG_DEBUG, "intr: line hit \r\n");
        if (refresh) {
            LCDC_DMAImgCfg(LCDC, (u32)g_buffer);
            LCDC_ShadowReloadConfig(LCDC);
        }
    }

    if (IntId & LCDC_BIT_LCD_LIN_INTEN) {
        if (refresh) {
            refresh = 0;
        }
        if (g_callback) {
            g_callback->VBlank(g_data);
        }
    }

    if (IntId & LCDC_BIT_DMA_UN_INTS) {
        RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "intr: dma udf !!! \r\n");
    }
}

static void lcdc_driver_init(void)
{
    LCDC_RGBInitTypeDef LCDC_RGBInitStruct;

    LCDC_Cmd(LCDC, DISABLE);
    LCDC_RGBStructInit(&LCDC_RGBInitStruct);

    /* set HV para according to lcd spec */
    LCDC_RGBInitStruct.Panel_RgbTiming.RgbVsw = 1;
    LCDC_RGBInitStruct.Panel_RgbTiming.RgbVbp = 4;
    LCDC_RGBInitStruct.Panel_RgbTiming.RgbVfp = 6;

    LCDC_RGBInitStruct.Panel_RgbTiming.RgbHsw = 4;
    LCDC_RGBInitStruct.Panel_RgbTiming.RgbHbp = 40;
    LCDC_RGBInitStruct.Panel_RgbTiming.RgbHfp = 40;

    LCDC_RGBInitStruct.Panel_Init.IfWidth = LCDC_RGB_IF_24_BIT;
    LCDC_RGBInitStruct.Panel_Init.ImgWidth = WIDTH;
    LCDC_RGBInitStruct.Panel_Init.ImgHeight = HEIGHT;

    LCDC_RGBInitStruct.Panel_RgbTiming.Flags.RgbEnPolar = LCDC_RGB_EN_PUL_HIGH_LEV_ACTIVE;
    LCDC_RGBInitStruct.Panel_RgbTiming.Flags.RgbDclkActvEdge = LCDC_RGB_DCLK_FALLING_EDGE_FETCH;
    LCDC_RGBInitStruct.Panel_RgbTiming.Flags.RgbHsPolar = LCDC_RGB_HS_PUL_LOW_LEV_SYNC;
    LCDC_RGBInitStruct.Panel_RgbTiming.Flags.RgbVsPolar = LCDC_RGB_VS_PUL_LOW_LEV_SYNC;

    if (g_image_format == RGB565) {
        LCDC_RGBInitStruct.Panel_Init.InputFormat = LCDC_INPUT_FORMAT_RGB565;
    } else if (g_image_format == ARGB8888) {
        LCDC_RGBInitStruct.Panel_Init.InputFormat = LCDC_INPUT_FORMAT_ARGB8888;
    } else {
        LCDC_RGBInitStruct.Panel_Init.InputFormat = LCDC_INPUT_FORMAT_RGB888;
    }
    LCDC_RGBInitStruct.Panel_Init.OutputFormat = LCDC_OUTPUT_FORMAT_RGB888;
    LCDC_RGBInitStruct.Panel_Init.RGBRefreshFreq = 60;

    LCDC_RGBInit(LCDC, &LCDC_RGBInitStruct);

    /* configure DMA burst size */
    LCDC_DMABurstSizeConfig(LCDC, 2);

    InterruptRegister((IRQ_FUN)lcdc_irq_handler, gLcdcIrqInfo.IrqNum, (u32)gLcdcIrqInfo.IrqData, gLcdcIrqInfo.IrqPriority);
    InterruptEn(gLcdcIrqInfo.IrqNum, gLcdcIrqInfo.IrqPriority);

    LCDC_LineINTPosConfig(LCDC, LCDC_LINE_NUM_INTR_DEF);
    LCDC_INTConfig(LCDC, LCDC_BIT_LCD_FRD_INTEN | LCDC_BIT_FRM_START_INTEN | LCDC_BIT_DMA_UN_INTEN | LCDC_BIT_LCD_LIN_INTEN, ENABLE);

    /*enable the LCDC*/
    LCDC_Cmd(LCDC, ENABLE);
}

void st7262_init(int image_format)
{
    g_image_format = image_format;
    if (g_image_format == ARGB8888) {
        g_buffer = (uint8_t *)malloc(MEM_SIZE);
    }
    /* init lcdc irq info */
    gLcdcIrqInfo.IrqNum = LCDC_IRQ;//49
    gLcdcIrqInfo.IrqPriority = INT_PRI_MIDDLE;
    gLcdcIrqInfo.IrqData = (u32)LCDC;

    /* config pin info */
    lcdc_pinmux_config();

    /* enable function and clock */
    LCDC_RccEnable();

    /* register irq handler */
    InterruptRegister((IRQ_FUN)lcdc_irq_handler, gLcdcIrqInfo.IrqNum, NULL, gLcdcIrqInfo.IrqPriority);
    InterruptEn(gLcdcIrqInfo.IrqNum, gLcdcIrqInfo.IrqPriority);

    /* init lcdc driver */
    lcdc_driver_init();

    /* config irq event */
    LCDC_LineINTPosConfig(LCDC, LCDC_LINE_NUM_INTR_DEF);
    LCDC_INTConfig(LCDC, LCDC_BIT_LCD_FRD_INTEN | LCDC_BIT_DMA_UN_INTEN | LCDC_BIT_LCD_LIN_INTEN, ENABLE);

    /* enable lcdc */
    LCDC_Cmd(LCDC, ENABLE);
}

void convert_rgb888_to_bgr888(uint8_t *argb_buffer, uint8_t *rgb_buffer)
{
    for (int i = 0; i < WIDTH * HEIGHT*3; i++) {
        uint8_t r = argb_buffer[i];     // Alpha
        uint8_t g = argb_buffer[i + 1]; // Red
        uint8_t b = argb_buffer[i + 2]; // Green

        rgb_buffer[i] = b;
        rgb_buffer[i + 1] = g;
        rgb_buffer[i + 2] = r;
    }
}

void st7262_clean_invalidate_buffer(u8 *buffer)
{
    if (g_image_format == ARGB8888) {
        g_buffer = buffer;
        DCache_Clean((u32)g_buffer, WIDTH * HEIGHT * 4);
    } else if(g_image_format == RGB888) {
        g_buffer = buffer;
        DCache_Clean((u32)g_buffer, WIDTH * HEIGHT * 3);
    }else {
        g_buffer = buffer;
        DCache_Clean((u32)g_buffer, WIDTH * HEIGHT * 2);
    }
    refresh = 1;
}

void st7262_get_info(int *width, int *height)
{
    *width = WIDTH;
    *height = HEIGHT;
}

void st7262_register_callback(ST7262VBlankCallback *callback, void *data)
{
    g_callback = callback;
    g_data = data;
}
