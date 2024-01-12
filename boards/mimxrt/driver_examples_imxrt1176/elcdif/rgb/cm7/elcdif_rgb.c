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

static volatile bool s_newFrameShown = false;
static dc_fb_info_t s_fbInfo;
static volatile uint8_t s_frameBufferIndex = 0;

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

#if APP_BUF_BYTE_PER_PIXEL == 2
#define BG_TYPE uint16_t
#elif APP_BUF_BYTE_PER_PIXEL == 4
#define BG_TYPE uint32_t
#endif
void APP_FillFrameBuffer(uint8_t frameBuffer[APP_IMG_HEIGHT][APP_IMG_WIDTH][APP_BUF_BYTE_PER_PIXEL])
{
    /* Foreground color. */
    static uint8_t fgColorIndex          = 0U;
    /* Background color. */
    static const BG_TYPE bgColor = 0U;
#if APP_BUF_BYTE_PER_PIXEL == 2
    /* Foreground color. */
    static const BG_TYPE fgColorTable[] = {0x001FU, 0x07E0U, 0x07FFU, 0xF800U,
                                            0xF81FU, 0xFFE0U, 0xFFFFU};
#elif APP_BUF_BYTE_PER_PIXEL == 4
    static const BG_TYPE fgColorTable[] = {0x000000FFU, 0x0000FF00U, 0x0000FFFFU, 0x00FF0000U,
                                            0x00FF00FFU, 0x00FFFF00U, 0x00FFFFFFU};
#endif
    BG_TYPE fgColor                     = fgColorTable[fgColorIndex];

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
            *(BG_TYPE *)(frameBuffer[i][j]) = bgColor;
        }
    }

    /* Foreground color. */
    for (i = upperLeftY; i < lowerRightY; i++)
    {
        for (j = upperLeftX; j < lowerRightX; j++)
        {
            *(BG_TYPE *)(frameBuffer[i][j]) = fgColor;
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

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer)
{
    s_newFrameShown = true;
    s_frameBufferIndex ^= 1;
}

/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_BootClockRUN();
    BOARD_ResetDisplayMix();
    BOARD_InitLpuartPins();
    BOARD_InitMipiPanelPins();
    BOARD_InitDebugConsole();
    BOARD_InitLcdifClock();

    /* Clear the frame buffer. */
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
    APP_FillFrameBuffer(s_frameBuffer[s_frameBufferIndex]);

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

    g_dc.ops->getLayerDefaultConfig(&g_dc, 0, &s_fbInfo);
    s_fbInfo.pixelFormat = APP_VIDEO_PIXEL_FORMAT;
    s_fbInfo.width       = APP_BUF_WIDTH;
    s_fbInfo.height      = APP_BUF_HEIGHT;
    s_fbInfo.startX      = APP_BUF_START_X;
    s_fbInfo.startY      = APP_BUF_START_Y;
    s_fbInfo.strideBytes = APP_BUF_STRIDE_BYTE;
    g_dc.ops->setLayerConfig(&g_dc, 0, &s_fbInfo);
    g_dc.ops->setCallback(&g_dc, 0, DEMO_BufferSwitchOffCallback, NULL);
    s_frameBufferIndex = 0;
    s_newFrameShown  = false;
    g_dc.ops->setFrameBuffer(&g_dc, 0, s_frameBuffer[s_frameBufferIndex]);
    /* For the DBI interface display, application must wait for the first
     * frame buffer sent to the panel.
     */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0)
    {
        while (s_newFrameShown == false)
        {
        }
    }
    s_newFrameShown = true;
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
        uint8_t temp = s_frameBufferIndex ^ 1U;

        APP_FillFrameBuffer(s_frameBuffer[temp]);

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
        /* Wait for the previous frame buffer is shown, and there is available
           inactive buffer to fill. */
        while (s_newFrameShown == false)
        {
        }
        /* Show the new frame. */
        s_newFrameShown = false;
        g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)((uint32_t)s_frameBuffer[temp]));
#else
        s_frameBufferIndex = temp;
        ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_frameBuffer[s_frameBufferIndex]);

        s_frameDone = false;
        /* Wait for previous frame complete. */
        while (!s_frameDone)
        {
        }
#endif
    }
}
