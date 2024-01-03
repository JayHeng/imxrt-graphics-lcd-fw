/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_display.h"
#include "fsl_ili9806e.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ILI9806E_DelayMs VIDEO_DelayMs

typedef struct
{
    const uint8_t *cmd;
    uint8_t cmdLen;
} ili9806e_cmd_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
const display_operations_t ili9806e_ops = {
    .init   = ILI9806E_Init,
    .deinit = ILI9806E_Deinit,
    .start  = ILI9806E_Start,
    .stop   = ILI9806E_Stop,
};

#define ILI9806_DATA		BIT(8)

#define ILI9806_MAX_MSG_LEN	6

struct ili9806e_msg {
	unsigned int len;
	u16 msg[ILI9806_MAX_MSG_LEN];
};

#define ILI9806_SET_PAGE(page)	\
	{				\
		.len = 6,		\
		.msg = {		\
			0xFF,			\
			ILI9806_DATA | 0xFF,	\
			ILI9806_DATA | 0x98,	\
			ILI9806_DATA | 0x06,	\
			ILI9806_DATA | 0x04,	\
			ILI9806_DATA | (page)	\
		},				\
	}

#define ILI9806_SET_REG_PARAM(reg, data)	\
	{					\
		.len = 2,			\
		.msg = {			\
			(reg),			\
			ILI9806_DATA | (data),	\
		},				\
	}

#define ILI9806_SET_REG(reg)	\
	{				\
		.len = 1,		\
		.msg = { (reg) },		\
	}

static const struct ili9806e_msg panel_init[] = {
	ILI9806_SET_PAGE(1),

	/* interface mode
	 *   SEPT_SDIO = 0 (spi interface transfer through SDA pin)
	 *   SDO_STATUS = 1 (always output, but without output tri-state)
	 */
	ILI9806_SET_REG_PARAM(0x08, 0x10),
	/* display control
	 * VSPL = 1 (vertical sync polarity)
	 * HSPL = 0 (horizontal sync polarity)
	 * DPL = 0 (PCLK polarity)
	 * EPL = 1 (data enable polarity)
	 */
	ILI9806_SET_REG_PARAM(0x21, 0x0d),
	/* resolution control (0x02 = 480x800) */
	ILI9806_SET_REG_PARAM(0x30, 0x02),
	/* display inversion control (0x00 = column inversion) */
	ILI9806_SET_REG_PARAM(0x31, 0x00),
	/* power control
	 *  EXB1T = 0 (internal charge pump)
	 *  EXT_CPCK_SEL = 1 (pump clock control signal = output 2 x waveform)
	 *  BT = 0 (DDVDH / DDVDL voltage = VCI x 2 / VCI x -2)
	 */
	ILI9806_SET_REG_PARAM(0x40, 0x10),
	/* power control
	 *  DDVDH_CLP = 5.6 (DDVDH clamp leve)
	 *  DDVDL_CLP = -5.6 (DDVDL clamp leve)
	 */
	ILI9806_SET_REG_PARAM(0x41, 0x55),
	/* power control
	 *  VGH_CP = 2DDVDH - DDVDL (step up factor for VGH)
	 *  VGL_CP = DDVDL + VCL - VCIP (step up factor for VGL)
	 */
	ILI9806_SET_REG_PARAM(0x42, 0x02),
	/* power control
	 *  VGH_CLPEN = 0 (disable VGH clamp level)
	 *  VGH_CLP = 9 (15.0 VGH clamp level - but this is disabled so not used?)
	 */
	ILI9806_SET_REG_PARAM(0x43, 0x84),
	/* power control
	 *  VGL_CLPEN = 0 (disable VGL clamp level)
	 *  VGL_CLP = 9 (-11.0 VGL clamp level - but this is disabled so not used?)
	 */
	ILI9806_SET_REG_PARAM(0x44, 0x84),

	/* power control
	 *  VREG1OUT voltage for positive gamma?
	 */
	ILI9806_SET_REG_PARAM(0x50, 0x78),
	/* power control
	 *  VREG2OUT voltage for negative gamma?
	 */
	ILI9806_SET_REG_PARAM(0x51, 0x78),

	ILI9806_SET_REG_PARAM(0x52, 0x00),
	ILI9806_SET_REG_PARAM(0x53, 0x77),
	ILI9806_SET_REG_PARAM(0x57, 0x60),
	ILI9806_SET_REG_PARAM(0x60, 0x07),
	ILI9806_SET_REG_PARAM(0x61, 0x00),
	ILI9806_SET_REG_PARAM(0x62, 0x08),
	ILI9806_SET_REG_PARAM(0x63, 0x00),
	ILI9806_SET_REG_PARAM(0xA0, 0x00),
	ILI9806_SET_REG_PARAM(0xA1, 0x07),
	ILI9806_SET_REG_PARAM(0xA2, 0x0C),
	ILI9806_SET_REG_PARAM(0xA3, 0x0B),
	ILI9806_SET_REG_PARAM(0xA4, 0x03),
	ILI9806_SET_REG_PARAM(0xA5, 0x07),
	ILI9806_SET_REG_PARAM(0xA6, 0x06),
	ILI9806_SET_REG_PARAM(0xA7, 0x04),
	ILI9806_SET_REG_PARAM(0xA8, 0x08),
	ILI9806_SET_REG_PARAM(0xA9, 0x0C),
	ILI9806_SET_REG_PARAM(0xAA, 0x13),
	ILI9806_SET_REG_PARAM(0xAB, 0x06),
	ILI9806_SET_REG_PARAM(0xAC, 0x0D),
	ILI9806_SET_REG_PARAM(0xAD, 0x19),
	ILI9806_SET_REG_PARAM(0xAE, 0x10),
	ILI9806_SET_REG_PARAM(0xAF, 0x00),
	/* negative gamma control
	 * set the gray scale voltage to adjust the gamma characteristics of the panel
	 */
	ILI9806_SET_REG_PARAM(0xC0, 0x00),
	ILI9806_SET_REG_PARAM(0xC1, 0x07),
	ILI9806_SET_REG_PARAM(0xC2, 0x0C),
	ILI9806_SET_REG_PARAM(0xC3, 0x0B),
	ILI9806_SET_REG_PARAM(0xC4, 0x03),
	ILI9806_SET_REG_PARAM(0xC5, 0x07),
	ILI9806_SET_REG_PARAM(0xC6, 0x07),
	ILI9806_SET_REG_PARAM(0xC7, 0x04),
	ILI9806_SET_REG_PARAM(0xC8, 0x08),
	ILI9806_SET_REG_PARAM(0xC9, 0x0C),
	ILI9806_SET_REG_PARAM(0xCA, 0x13),
	ILI9806_SET_REG_PARAM(0xCB, 0x06),
	ILI9806_SET_REG_PARAM(0xCC, 0x0D),
	ILI9806_SET_REG_PARAM(0xCD, 0x18),
	ILI9806_SET_REG_PARAM(0xCE, 0x10),
	ILI9806_SET_REG_PARAM(0xCF, 0x00),

	ILI9806_SET_PAGE(6),

	ILI9806_SET_REG_PARAM(0x00, 0x20),
	ILI9806_SET_REG_PARAM(0x01, 0x0A),
	ILI9806_SET_REG_PARAM(0x02, 0x00),
	ILI9806_SET_REG_PARAM(0x03, 0x00),
	ILI9806_SET_REG_PARAM(0x04, 0x01),
	ILI9806_SET_REG_PARAM(0x05, 0x01),
	ILI9806_SET_REG_PARAM(0x06, 0x98),
	ILI9806_SET_REG_PARAM(0x07, 0x06),
	ILI9806_SET_REG_PARAM(0x08, 0x01),
	ILI9806_SET_REG_PARAM(0x09, 0x80),
	ILI9806_SET_REG_PARAM(0x0A, 0x00),
	ILI9806_SET_REG_PARAM(0x0B, 0x00),
	ILI9806_SET_REG_PARAM(0x0C, 0x01),
	ILI9806_SET_REG_PARAM(0x0D, 0x01),
	ILI9806_SET_REG_PARAM(0x0E, 0x00),
	ILI9806_SET_REG_PARAM(0x0F, 0x00),
	ILI9806_SET_REG_PARAM(0x10, 0xF0),
	ILI9806_SET_REG_PARAM(0x11, 0xF4),
	ILI9806_SET_REG_PARAM(0x12, 0x01),
	ILI9806_SET_REG_PARAM(0x13, 0x00),
	ILI9806_SET_REG_PARAM(0x14, 0x00),
	ILI9806_SET_REG_PARAM(0x15, 0xC0),
	ILI9806_SET_REG_PARAM(0x16, 0x08),
	ILI9806_SET_REG_PARAM(0x17, 0x00),
	ILI9806_SET_REG_PARAM(0x18, 0x00),
	ILI9806_SET_REG_PARAM(0x19, 0x00),
	ILI9806_SET_REG_PARAM(0x1A, 0x00),
	ILI9806_SET_REG_PARAM(0x1B, 0x00),
	ILI9806_SET_REG_PARAM(0x1C, 0x00),
	ILI9806_SET_REG_PARAM(0x1D, 0x00),
	ILI9806_SET_REG_PARAM(0x20, 0x01),
	ILI9806_SET_REG_PARAM(0x21, 0x23),
	ILI9806_SET_REG_PARAM(0x22, 0x45),
	ILI9806_SET_REG_PARAM(0x23, 0x67),
	ILI9806_SET_REG_PARAM(0x24, 0x01),
	ILI9806_SET_REG_PARAM(0x25, 0x23),
	ILI9806_SET_REG_PARAM(0x26, 0x45),
	ILI9806_SET_REG_PARAM(0x27, 0x67),
	ILI9806_SET_REG_PARAM(0x30, 0x11),
	ILI9806_SET_REG_PARAM(0x31, 0x11),
	ILI9806_SET_REG_PARAM(0x32, 0x00),
	ILI9806_SET_REG_PARAM(0x33, 0xEE),
	ILI9806_SET_REG_PARAM(0x34, 0xFF),
	ILI9806_SET_REG_PARAM(0x35, 0xBB),
	ILI9806_SET_REG_PARAM(0x36, 0xAA),
	ILI9806_SET_REG_PARAM(0x37, 0xDD),
	ILI9806_SET_REG_PARAM(0x38, 0xCC),
	ILI9806_SET_REG_PARAM(0x39, 0x66),
	ILI9806_SET_REG_PARAM(0x3A, 0x77),
	ILI9806_SET_REG_PARAM(0x3B, 0x22),
	ILI9806_SET_REG_PARAM(0x3C, 0x22),
	ILI9806_SET_REG_PARAM(0x3D, 0x22),
	ILI9806_SET_REG_PARAM(0x3E, 0x22),
	ILI9806_SET_REG_PARAM(0x3F, 0x22),
	ILI9806_SET_REG_PARAM(0x40, 0x22),
	/* register doesn't exist on page 6? */
	ILI9806_SET_REG_PARAM(0x52, 0x10),
	/* doesn't make sense, not valid according to datasheet */
	ILI9806_SET_REG_PARAM(0x53, 0x10),
	/* doesn't make sense, not valid according to datasheet */
	ILI9806_SET_REG_PARAM(0x54, 0x13),

	ILI9806_SET_PAGE(7),

	/* enable VREG */
	ILI9806_SET_REG_PARAM(0x18, 0x1D),
	/* enable VGL_REG */
	ILI9806_SET_REG_PARAM(0x17, 0x22),
	/* register doesn't exist on page 7? */
	ILI9806_SET_REG_PARAM(0x02, 0x77),
	/* register doesn't exist on page 7? */
	ILI9806_SET_REG_PARAM(0x26, 0xB2),
	/* register doesn't exist on page 7? */
	ILI9806_SET_REG_PARAM(0xE1, 0x79),

	ILI9806_SET_PAGE(0),

	ILI9806_SET_REG_PARAM(MIPI_DCS_SET_PIXEL_FORMAT,
			      MIPI_DCS_PIXEL_FMT_18BIT << 4),
	ILI9806_SET_REG_PARAM(MIPI_DCS_SET_TEAR_ON, 0x00),
	ILI9806_SET_REG(MIPI_DCS_EXIT_SLEEP_MODE),
};

#define NUM_INIT_REGS ARRAY_SIZE(panel_init)



/*******************************************************************************
 * Code
 ******************************************************************************/

status_t ILI9806E_Init(display_handle_t *handle, const display_config_t *config)
{
    uint8_t i;
    status_t status                   = kStatus_Success;
    const ili9806e_resource_t *resource = (const ili9806e_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    /* Only support 480 * 854 */
    if (config->resolution != FSL_VIDEO_RESOLUTION(480, 854))
    {
        return kStatus_InvalidArgument;
    }

    /* Power on. */
    resource->pullPowerPin(true);
    ILI9806E_DelayMs(1);

    /* Perform reset. */
    resource->pullResetPin(false);
    ILI9806E_DelayMs(1);
    resource->pullResetPin(true);
    ILI9806E_DelayMs(50U);
    
    for (i = 0; i < ARRAY_SIZE(panel_init); i++)
    {
        status = MIPI_DSI_GenericWrite(dsiDevice, panel_init[i].msg, panel_init[i].len);

        if (kStatus_Success != status)
        {
            break;
        }
    }
    
    if (kStatus_Success == status)
    {
        status = MIPI_DSI_DCS_EnterSleepMode(dsiDevice, false);
    }

    if (kStatus_Success == status)
    {
        ILI9806E_DelayMs(120U);

        status = MIPI_DSI_DCS_SetDisplayOn(dsiDevice, true);
    }

    return status;
}

status_t ILI9806E_Deinit(display_handle_t *handle)
{
    const ili9806e_resource_t *resource = (const ili9806e_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    (void)MIPI_DSI_DCS_EnterSleepMode(dsiDevice, true);

    resource->pullResetPin(false);
    resource->pullPowerPin(false);

    return kStatus_Success;
}

status_t ILI9806E_Start(display_handle_t *handle)
{
    const ili9806e_resource_t *resource = (const ili9806e_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, true);
}

status_t ILI9806E_Stop(display_handle_t *handle)
{
    const ili9806e_resource_t *resource = (const ili9806e_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, false);
}
