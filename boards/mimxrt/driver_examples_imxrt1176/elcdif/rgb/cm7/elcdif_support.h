/*
 * Copyright 2019-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _ELCDIF_SUPPORT_H_
#define _ELCDIF_SUPPORT_H_

#include "fsl_mipi_dsi.h"
#include "fsl_dc_fb.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_ELCDIF            LCDIF
#define APP_ELCDIF_IRQn       eLCDIF_IRQn
#define APP_ELCDIF_IRQHandler eLCDIF_IRQHandler

#define MIPI_PANEL_RK055AHD091   0 /* 720 * 1280 */
#define MIPI_PANEL_RK055IQH091   1 /* 540 * 960  */
#define MIPI_PANEL_RK055MHD091   2 /* 720 * 1280 */
#define MIPI_PANEL_KD050FWFIA019 3 /* 480 * 854 */
#define MIPI_PANEL_G1120B0MIPI   4 /* 400 * 392 */

#ifndef USE_MIPI_PANEL
#define USE_MIPI_PANEL MIPI_PANEL_G1120B0MIPI
#endif

#if (USE_MIPI_PANEL == MIPI_PANEL_RK055AHD091)
#define APP_PANEL_HEIGHT 1280
#define APP_PANEL_WIDTH  720
#define APP_HSW          8
#define APP_HFP          32
#define APP_HBP          32
#define APP_VSW          2
#define APP_VFP          16
#define APP_VBP          14
#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055MHD091)
#define APP_PANEL_HEIGHT 1280
#define APP_PANEL_WIDTH  720
#define APP_HSW          6
#define APP_HFP          12
#define APP_HBP          24
#define APP_VSW          2
#define APP_VFP          16
#define APP_VBP          14
#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055IQH091)
#define APP_PANEL_HEIGHT 960
#define APP_PANEL_WIDTH  540
#define APP_HSW          2
#define APP_HFP          32
#define APP_HBP          30
#define APP_VSW          2
#define APP_VFP          16
#define APP_VBP          14
#elif (USE_MIPI_PANEL == MIPI_PANEL_KD050FWFIA019)
#define APP_PANEL_HEIGHT 854
#define APP_PANEL_WIDTH  480
#define APP_HSW          4
#define APP_HFP          18
#define APP_HBP          30
#define APP_VSW          4
#define APP_VFP          20
#define APP_VBP          30
#elif (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
#define APP_PANEL_HEIGHT 392
#define APP_PANEL_WIDTH  400
#define APP_HSW          0
#define APP_HFP          0
#define APP_HBP          0
#define APP_VSW          0
#define APP_VFP          0
#define APP_VBP          0
#endif

#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnFallingClkEdge)

/* Frame buffer data alignment, for better performance, the LCDIF frame buffer should be 64B align. */
#define FRAME_BUFFER_ALIGN 64

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
#define APP_IMG_HEIGHT     APP_BUF_HEIGHT
#define APP_IMG_WIDTH      APP_BUF_WIDTH

#define APP_BUF_BYTE_PER_PIXEL 4
#define BOARD_MIPI_CLK_HZ      500000000U /*500MHz*/
#define APP_VIDEO_PIXEL_FORMAT kVIDEO_PixelFormatXRGB8888

/* Where the frame buffer is shown in the screen. */
#define APP_BUF_START_X 4U
#define APP_BUF_START_Y 4U
#define APP_BUF_WIDTH   (400U)
#define APP_BUF_HEIGHT  (400U)
#define APP_BUF_STRIDE_BYTE (APP_BUF_WIDTH * APP_BUF_BYTE_PER_PIXEL)

/* Pixel format macro mapping. */
#define DEMO_RM67162_BUFFER_RGB565   0
#define DEMO_RM67162_BUFFER_RGB888   1
#define DEMO_RM67162_BUFFER_XRGB8888 2
#define DEMO_RM67162_BUFFER_FORMAT DEMO_RM67162_BUFFER_XRGB8888
#else
#define APP_IMG_HEIGHT     APP_PANEL_HEIGHT
#define APP_IMG_WIDTH      APP_PANEL_WIDTH

#define APP_BUF_BYTE_PER_PIXEL 4
#define APP_VIDEO_PIXEL_FORMAT kVIDEO_PixelFormatXRGB8888
#endif

extern const MIPI_DSI_Type g_mipiDsi;
#define APP_MIPI_DSI          (&g_mipiDsi)

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
#define APP_MIPI_DSI_LANE_NUM 1
#else
#define APP_MIPI_DSI_LANE_NUM 2
#endif

#define APP_MIPI_DSI_IRQn     MIPI_DSI_IRQn

/* Frame buffer data alignment, for better performance, the LCDIF frame buffer should be 64B align. */
#define FRAME_BUFFER_ALIGN 64

/*
 * The DPHY bit clock must be fast enough to send out the pixels, it should be
 * larger than:
 *
 *         (Pixel clock * bit per output pixel) / number of MIPI data lane
 *
 * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
 * it is fast enough.
 */
#define APP_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)

/* Should call BOARD_InitDisplayInterface to initialize display interface. */
#define APP_ELCDIF_HAS_DISPLAY_INTERFACE 1

/* When working with MIPI DSI, the output pixel is 24-bit pixel */
#define APP_DATA_BUS           24
#define APP_LCDIF_DATA_BUS     kELCDIF_DataBus24Bit
#define APP_LCDIF_PIXEL_FORMAT kELCDIF_PixelFormatXRGB8888

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
// 
status_t BOARD_InitDisplayInterface(void);
void BOARD_InitLcdifClock(void);
void BOARD_EnableLcdInterrupt(void);
// 
status_t BOARD_PrepareDisplayController(void);
void BOARD_DisplayTEPinHandler(void);
extern const dc_fb_t g_dc;

#endif /* _ELCDIF_SUPPORT_H_ */
