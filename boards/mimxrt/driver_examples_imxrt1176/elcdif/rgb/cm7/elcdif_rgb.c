/*
 * Copyright  2017-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_elcdif.h"
#include "fsl_debug_console.h"

#include "fsl_soc_src.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "elcdif_support.h"
#include "fsl_dc_fb_dsi_cmd.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef APP_LCDIF_DATA_BUS
#define APP_LCDIF_DATA_BUS kELCDIF_DataBus24Bit
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool s_frameDone = false;

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_frameBuffer[2][APP_IMG_HEIGHT][APP_IMG_WIDTH][APP_BUF_BYTE_PER_PIXEL], FRAME_BUFFER_ALIGN);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void BOARD_ResetDisplayMix(void)
{
    /*
     * Reset the displaymix, otherwise during debugging, the
     * debugger may not reset the display, then the behavior
     * is not right.
     */
    SRC_AssertSliceSoftwareReset(SRC, kSRC_DisplaySlice);
    while (kSRC_SliceResetInProcess == SRC_GetSliceResetState(SRC, kSRC_DisplaySlice))
    {
    }
}

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
void CM7_GPIO2_3_IRQHandler(void)
{
    uint32_t intStatus;

    intStatus = GPIO_PortGetInterruptFlags(BOARD_MIPI_PANEL_TE_GPIO);

    GPIO_PortClearInterruptFlags(BOARD_MIPI_PANEL_TE_GPIO, intStatus);

    if (intStatus & (1U << BOARD_MIPI_PANEL_TE_PIN))
    {
        BOARD_DisplayTEPinHandler();
        s_frameDone = true;
    }
    SDK_ISR_EXIT_BARRIER;
}
#endif
void APP_LCDIF_IRQHandler(void)
{
    uint32_t intStatus;

    intStatus = ELCDIF_GetInterruptStatus(APP_ELCDIF);

    ELCDIF_ClearInterruptStatus(APP_ELCDIF, intStatus);

    if (intStatus & kELCDIF_CurFrameDone)
    {
        s_frameDone = true;
    }
    SDK_ISR_EXIT_BARRIER;
}

void APP_ELCDIF_Init(void)
{
    const elcdif_rgb_mode_config_t config = {
        .panelWidth    = APP_IMG_WIDTH,
        .panelHeight   = APP_IMG_HEIGHT,
        .hsw           = APP_HSW,
        .hfp           = APP_HFP,
        .hbp           = APP_HBP,
        .vsw           = APP_VSW,
        .vfp           = APP_VFP,
        .vbp           = APP_VBP,
        .polarityFlags = APP_POL_FLAGS,
        .bufferAddr    = (uint32_t)s_frameBuffer[0],
        .pixelFormat   = APP_LCDIF_PIXEL_FORMAT,
        .dataBus       = APP_LCDIF_DATA_BUS,
    };

#if (defined(APP_ELCDIF_HAS_DISPLAY_INTERFACE) && APP_ELCDIF_HAS_DISPLAY_INTERFACE)
    BOARD_InitDisplayInterface();
#endif
    ELCDIF_RgbModeInit(APP_ELCDIF, &config);
}

void APP_FillFrameBuffer(uint8_t frameBuffer[APP_IMG_HEIGHT][APP_IMG_WIDTH][APP_BUF_BYTE_PER_PIXEL])
{
    /* Background color. */
    static const uint32_t bgColor = 0U;
    /* Foreground color. */
    static uint8_t fgColorIndex          = 0U;
    static const uint32_t fgColorTable[] = {0x000000FFU, 0x0000FF00U, 0x0000FFFFU, 0x00FF0000U,
                                            0x00FF00FFU, 0x00FFFF00U, 0x00FFFFFFU};
    uint32_t fgColor                     = fgColorTable[fgColorIndex];

    /* Position of the foreground rectangle. */
    static uint16_t upperLeftX  = 0U;
    static uint16_t upperLeftY  = 0U;
    static uint16_t lowerRightX = (APP_IMG_WIDTH - 1U) / 2U;
    static uint16_t lowerRightY = (APP_IMG_HEIGHT - 1U) / 2U;

    static int8_t incX = 1;
    static int8_t incY = 1;

    /* Change color in next forame or not. */
    static bool changeColor = false;

    uint32_t i, j;

    /* Background color. */
    for (i = 0; i < APP_IMG_HEIGHT; i++)
    {
        for (j = 0; j < APP_IMG_WIDTH; j++)
        {
            *(uint32_t *)(frameBuffer[i][j]) = bgColor;
        }
    }

    /* Foreground color. */
    for (i = upperLeftY; i < lowerRightY; i++)
    {
        for (j = upperLeftX; j < lowerRightX; j++)
        {
            *(uint32_t *)(frameBuffer[i][j]) = fgColor;
        }
    }

    /* Update the format: color and rectangle position. */
    upperLeftX += incX;
    upperLeftY += incY;
    lowerRightX += incX;
    lowerRightY += incY;

    changeColor = false;

    if (0U == upperLeftX)
    {
        incX        = 1;
        changeColor = true;
    }
    else if (APP_IMG_WIDTH - 1 == lowerRightX)
    {
        incX        = -1;
        changeColor = true;
    }

    if (0U == upperLeftY)
    {
        incY        = 1;
        changeColor = true;
    }
    else if (APP_IMG_HEIGHT - 1 == lowerRightY)
    {
        incY        = -1;
        changeColor = true;
    }

    if (changeColor)
    {
        fgColorIndex++;

        if (ARRAY_SIZE(fgColorTable) == fgColorIndex)
        {
            fgColorIndex = 0U;
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t frameBufferIndex = 0;

    BOARD_ConfigMPU();
    BOARD_BootClockRUN();
    BOARD_ResetDisplayMix();
    BOARD_InitLpuartPins();
    BOARD_InitMipiPanelPins();
    BOARD_InitDebugConsole();
    BOARD_InitLcdifClock();

    /* Clear the frame buffer. */
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));

    APP_FillFrameBuffer(s_frameBuffer[frameBufferIndex]);

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
    PRINTF("Smart MIPI RGB example start...\r\n");
    status_t status;
    BOARD_PrepareDisplayController();
    /* Initialize the display controller. */
    status = g_dc.ops->init(&g_dc);
    if (kStatus_Success != status)
    {
        return status;
    }

    dc_fb_dsi_cmd_handle_t *dcHandle;
    dc_fb_dsi_cmd_layer_t *layer;
    dc_fb_info_t *fbInfo;
    dcHandle = (dc_fb_dsi_cmd_handle_t *)g_dc.prvData;
    /* Currently only support one layer, so the layer index is always 0. */
    layer = &(dcHandle->layers[0]);
    fbInfo = &(layer->fbInfo);
    fbInfo->pixelFormat = APP_VIDEO_PIXEL_FORMAT;
    fbInfo->startX      = APP_BUF_START_X;
    fbInfo->startY      = APP_BUF_START_Y;
    fbInfo->width       = APP_BUF_WIDTH;
    fbInfo->height      = APP_BUF_HEIGHT;
    fbInfo->strideBytes = APP_BUF_STRIDE_BYTE;

    layer->frameBuffer = (void *)((uint32_t)s_frameBuffer[0]);

    g_dc.ops->setFrameBuffer(&g_dc, 0, layer->frameBuffer);
    g_dc.ops->enableLayer(&g_dc, 0);
#else
    PRINTF("LCDIF RGB example start...\r\n");
    APP_ELCDIF_Init();
    BOARD_EnableLcdInterrupt();
    ELCDIF_EnableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
    ELCDIF_RgbModeStart(APP_ELCDIF);
#endif

    while (1)
    {
        frameBufferIndex ^= 1U;

        APP_FillFrameBuffer(s_frameBuffer[frameBufferIndex]);

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
        layer->fbWaitTE = (void *)((uint32_t)s_frameBuffer[frameBufferIndex]);
        SDK_DelayAtLeastUs(500000, SystemCoreClock);
        /* Wait for frame buffer sent to display controller video memory. */
        //while ((g_dc.ops->getProperty(&g_dc) & (uint32_t)kDC_FB_ReserveFrameBuffer))
        //{}
        s_frameDone = false;
        /* Wait for previous frame complete. */
        while (!s_frameDone)
        {
        }
#else
        ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_frameBuffer[frameBufferIndex]);

        s_frameDone = false;
        /* Wait for previous frame complete. */
        while (!s_frameDone)
        {
        }
#endif
    }
}
