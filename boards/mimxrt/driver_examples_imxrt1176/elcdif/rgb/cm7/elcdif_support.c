/*
 * Copyright 2019-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_gpio.h"
#include "fsl_mipi_dsi.h"
#include "fsl_rm68191.h"
#include "fsl_rm68200.h"
#include "fsl_hx8394.h"
#include "fsl_ili9806e.h"
#include "fsl_dc_fb_dsi_cmd.h"
#include "fsl_rm67162.h"
#include "mipi_dsi_aux.h"
//#include "auo141_display.h"
#include "elcdif_support.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*
 * The max TX array size:
 *
 * 1. One byte in FIFO is reserved for DSC command
 * 2. One pixel should not be split to two transfer.
 */
#define APP_DSI_TX_ARRAY_MAX   (((FSL_DSI_TX_MAX_PAYLOAD_BYTE - 1U) / APP_BUF_BYTE_PER_PIXEL) * APP_BUF_BYTE_PER_PIXEL)

typedef struct _dsi_mem_write_ctx
{
    volatile bool onGoing;
    const uint8_t *txData;
    uint32_t leftByteLen;
    uint8_t dscCmd;
} dsi_mem_write_ctx_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern void APP_LCDIF_IRQHandler(void);
void BOARD_InitMipiPanelTEPin(void);
status_t PANEL_DSI_MemWrite(uint8_t virtualChannel, const uint8_t *data, uint32_t length);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static dsi_mem_write_ctx_t s_dsiMemWriteCtx;
static dsi_transfer_t      s_dsiMemWriteXfer = {0};
static dsi_handle_t        s_dsiDriverHandle;
static uint8_t             s_dsiMemWriteTmpArray[APP_DSI_TX_ARRAY_MAX];

const dc_fb_dsi_cmd_config_t s_panelConfig = {
    .commonConfig =
        {
            .resolution   = FSL_VIDEO_RESOLUTION(APP_PANEL_WIDTH, APP_PANEL_HEIGHT),
            .hsw          = APP_HSW,
            .hfp          = APP_HFP,
            .hbp          = APP_HBP,
            .vsw          = APP_VSW,
            .vfp          = APP_VFP,
            .vbp          = APP_VBP,
            .controlFlags = 0,
            .dsiLanes     = APP_MIPI_DSI_LANE_NUM,
            .pixelFormat  = APP_VIDEO_PIXEL_FORMAT,
        },
#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
    .useTEPin = true,
#else
    .useTEPin = false,
#endif
};

uint32_t mipiDsiTxEscClkFreq_Hz;
uint32_t mipiDsiDphyBitClkFreq_Hz;
uint32_t mipiDsiDphyRefClkFreq_Hz;
uint32_t mipiDsiDpiClkFreq_Hz;

const MIPI_DSI_Type g_mipiDsi = {
    .host = DSI_HOST,
    .apb  = DSI_HOST_APB_PKT_IF,
    .dpi  = DSI_HOST_DPI_INTFC,
    .dphy = DSI_HOST_DPHY_INTFC,
};

static void PANEL_PullResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 0);
    }
}

/* From the schematic, the power pin is pinned to high. */
static void PANEL_PullPowerPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 0);
    }
}

status_t PANEL_DSI_Transfer(dsi_transfer_t *xfer)
{
    return DSI_TransferBlocking(APP_MIPI_DSI, xfer);
}

#if (USE_MIPI_PANEL == MIPI_PANEL_RK055AHD091)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = PANEL_DSI_Transfer,
};

static const rm68200_resource_t rm68200Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = PANEL_PullResetPin,
    .pullPowerPin = PANEL_PullPowerPin,
};

static display_handle_t rm68200Handle = {
    .resource = &rm68200Resource,
    .ops      = &rm68200_ops,
};

#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055MHD091)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = PANEL_DSI_Transfer,
};

static const hx8394_resource_t hx8394Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = PANEL_PullResetPin,
    .pullPowerPin = PANEL_PullPowerPin,
};

static display_handle_t hx8394Handle = {
    .resource = &hx8394Resource,
    .ops      = &hx8394_ops,
};

#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055IQH091)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = PANEL_DSI_Transfer,
};

static const rm68191_resource_t rm68191Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = PANEL_PullResetPin,
    .pullPowerPin = PANEL_PullPowerPin,
};

static display_handle_t rm68191Handle = {
    .resource = &rm68191Resource,
    .ops      = &rm68191_ops,
};

#elif (USE_MIPI_PANEL == MIPI_PANEL_KD050FWFIA019)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = PANEL_DSI_Transfer,
};

static const hx8394_resource_t ili9806eResource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = PANEL_PullResetPin,
    .pullPowerPin = PANEL_PullPowerPin,
};

static display_handle_t ili9806eHandle = {
    .resource = &ili9806eResource,
    .ops      = &ili9806e_ops,
};

#elif (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = PANEL_DSI_Transfer,
    .memWriteFunc   = PANEL_DSI_MemWrite,
};

static const rm67162_resource_t rm67162Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = PANEL_PullResetPin,
    .pullPowerPin = PANEL_PullPowerPin,
};

static display_handle_t rm67162Handle = {
    .resource = &rm67162Resource,
    .ops      = &rm67162_ops,
};

//static const auo141_resource_t auo141Resource = {
//    .dsiDevice    = &dsiDevice,
//    .pullResetPin = PANEL_PullResetPin,
//    .pullPowerPin = PANEL_PullPowerPin,
//};
//
//static display_handle_t auo141Handle = {
//    .resource = &auo141Resource,
//    .ops      = &auo141_ops,
//};

static dc_fb_dsi_cmd_handle_t s_dcFbDsiCmdHandle = {
    .dsiDevice   = &dsiDevice,
    .panelHandle = &rm67162Handle,
//    .panelHandle = &auo141Handle,
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsDsiCmd,
    .prvData = &s_dcFbDsiCmdHandle,
    .config  = &s_panelConfig,
};
#endif

void BOARD_InitLcdifClock(void)
{
    /*
     * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) * frame rate.
     *
     * Use PLL_528 as clock source.
     *
     * For 60Hz frame rate, the KD050FWFIA019 pixel clock should be 29MHz.
     * For 60Hz frame rate, the RK055IQH091 pixel clock should be 36MHz.
     * the RK055AHD091 pixel clock should be 62MHz.
     */
    const clock_root_config_t lcdifClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
#if (USE_MIPI_PANEL == MIPI_PANEL_RK055AHD091) || (USE_MIPI_PANEL == MIPI_PANEL_RK055MHD091)
        .div = 9,
#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055IQH091)
        .div = 15,
#elif (USE_MIPI_PANEL == MIPI_PANEL_KD050FWFIA019)
        .div = 18,
#elif (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
        .div = 56,
#endif
    };

    CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdif);
}

static status_t BOARD_InitLcdPanel(void)
{
    status_t status = kStatus_Success;

    const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

#if (USE_MIPI_PANEL == MIPI_PANEL_RK055AHD091)
    status = RM68200_Init(&rm68200Handle, &s_panelConfig.commonConfig);
#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055MHD091)
    status = HX8394_Init(&hx8394Handle, &s_panelConfig.commonConfig);
#elif (USE_MIPI_PANEL == MIPI_PANEL_RK055IQH091)
    status = RM68191_Init(&rm68191Handle, &s_panelConfig.commonConfig);
#elif (USE_MIPI_PANEL == MIPI_PANEL_KD050FWFIA019)
    status = ILI9806E_Init(&ili9806eHandle, &s_panelConfig.commonConfig);
#elif (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
    BOARD_InitMipiPanelTEPin();
#endif

    if (status == kStatus_Success)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
    }

    return status;
}

#if (USE_MIPI_PANEL == MIPI_PANEL_G1120B0MIPI)
static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = 11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false, .resetDiv = 2, .div0 = 2, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
    mipiDsiDphyBitClkFreq_Hz = BOARD_MIPI_CLK_HZ;

}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = APP_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    /* Init the DSI module. */
    DSI_Init(APP_MIPI_DSI, &dsiConfig);

    /* Init DPHY */
    DSI_InitDphy(APP_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);
}
#else
static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 24MHz / 1 = 24MHz. */
    /* TxClkEsc = 24MHz / 1 / 2 = 12MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false, .resetDiv = 1, .div0 = 1, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 2;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = APP_PANEL_WIDTH,
#if (APP_DATA_BUS == 24)
                                        .dpiColorCoding   = kDSI_Dpi24Bit,
                                        .pixelPacket      = kDSI_PixelPacket24Bit,
#elif (APP_DATA_BUS == 16)
                                        .dpiColorCoding   = kDSI_Dpi16BitConfig1,
                                        .pixelPacket      = kDSI_PixelPacket16Bit,
#endif
                                        .videoMode        = kDSI_DpiBurst,
                                        .bllpMode         = kDSI_DpiBllpLowPower,
                                        .polarityFlags    = kDSI_DpiVsyncActiveLow | kDSI_DpiHsyncActiveLow,
                                        .hfp              = APP_HFP,
                                        .hbp              = APP_HBP,
                                        .hsw              = APP_HSW,
                                        .vfp              = APP_VFP,
                                        .vbp              = APP_VBP,
                                        .panelHeight      = APP_PANEL_HEIGHT,
                                        .virtualChannel   = 0};

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = APP_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    /* Init the DSI module. */
    DSI_Init(APP_MIPI_DSI, &dsiConfig);

    /* Init DPHY.
     *
     * The DPHY bit clock must be fast enough to send out the pixels, it should be
     * larger than:
     *
     *         (Pixel clock * bit per output pixel) / number of MIPI data lane
     *
     * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
     * it is fast enough.
     *
     * Note that the DSI output pixel is 24bit per pixel.
     */
    mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz * (APP_DATA_BUS / APP_MIPI_DSI_LANE_NUM);

    mipiDsiDphyBitClkFreq_Hz = APP_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    mipiDsiDphyBitClkFreq_Hz = DSI_InitDphy(APP_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

    /* Init DPI interface. */
    DSI_SetDpiConfig(APP_MIPI_DSI, &dpiConfig, APP_MIPI_DSI_LANE_NUM, mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}
#endif

status_t BOARD_InitDisplayInterface(void)
{
    /* LCDIF v2 output to MIPI DSI. */
    CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.CLR = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;

    /* 1. Power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /* 2. Assert MIPI reset. */
    IOMUXC_GPR->GPR62 &=
        ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK |
          IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 3. Setup clock. */
    BOARD_InitMipiDsiClock();

    /* 4. Deassert PCLK and ESC reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

    /* 5. Configures peripheral. */
    BOARD_SetMipiDsiConfig();

    /* 6. Deassert BYTE and DBI reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 7. Configure the panel. */
    return BOARD_InitLcdPanel();
}

void BOARD_EnableLcdInterrupt(void)
{
    EnableIRQ(APP_ELCDIF_IRQn);
}

void APP_ELCDIF_IRQHandler(void)
{
    APP_LCDIF_IRQHandler();
    __DSB();
}

static status_t BOARD_DsiMemWriteSendChunck(void)
{
    uint32_t curSendLen;
    uint32_t i;

    curSendLen =
        APP_DSI_TX_ARRAY_MAX > s_dsiMemWriteCtx.leftByteLen ? s_dsiMemWriteCtx.leftByteLen : APP_DSI_TX_ARRAY_MAX;

    s_dsiMemWriteXfer.txDataType = kDSI_TxDataDcsLongWr;
    s_dsiMemWriteXfer.dscCmd     = s_dsiMemWriteCtx.dscCmd;
    s_dsiMemWriteXfer.txData     = s_dsiMemWriteTmpArray;

    /* For each pixel, the MIPI DSI sends out low byte first, but according to
     * the MIPI DSC spec, the high byte should be send first, so swap the pixel byte
     * first.
     */
#if (DEMO_RM67162_BUFFER_FORMAT == DEMO_RM67162_BUFFER_RGB565)
        s_dsiMemWriteXfer.txDataSize = curSendLen;
    for (i = 0; i < curSendLen; i += 2)
    {
        s_dsiMemWriteTmpArray[i]     = *(s_dsiMemWriteCtx.txData + 1);
        s_dsiMemWriteTmpArray[i + 1] = *(s_dsiMemWriteCtx.txData);

        s_dsiMemWriteCtx.txData += 2;
    }
#elif (DEMO_RM67162_BUFFER_FORMAT == DEMO_RM67162_BUFFER_RGB888)
    s_dsiMemWriteXfer.txDataSize = curSendLen;
    for (i = 0; i < curSendLen; i += 3)
    {
        s_dsiMemWriteTmpArray[i]     = *(s_dsiMemWriteCtx.txData + 2);
        s_dsiMemWriteTmpArray[i + 1] = *(s_dsiMemWriteCtx.txData + 1);
        s_dsiMemWriteTmpArray[i + 2] = *(s_dsiMemWriteCtx.txData);

        s_dsiMemWriteCtx.txData += 3;
    }
#else
    s_dsiMemWriteXfer.txDataSize = (curSendLen * 3 / 4);
    for (i = 0; i < (curSendLen * 3 / 4); i += 3)
    {
        s_dsiMemWriteTmpArray[i]     = *(s_dsiMemWriteCtx.txData + 2);
        s_dsiMemWriteTmpArray[i + 1] = *(s_dsiMemWriteCtx.txData + 1);
        s_dsiMemWriteTmpArray[i + 2] = *(s_dsiMemWriteCtx.txData);

        s_dsiMemWriteCtx.txData += 4;
    }
#endif

    s_dsiMemWriteCtx.leftByteLen -= (curSendLen);
    s_dsiMemWriteCtx.dscCmd = kMIPI_DCS_WriteMemoryContinue;

    return DSI_TransferNonBlocking(APP_MIPI_DSI, &s_dsiDriverHandle, &s_dsiMemWriteXfer);
}

static void BOARD_DsiMemWriteCallback(const MIPI_DSI_Type *base, dsi_handle_t *handle, status_t status, void *userData)
{
    if ((kStatus_Success == status) && (s_dsiMemWriteCtx.leftByteLen > 0))
    {
        status = BOARD_DsiMemWriteSendChunck();

        if (kStatus_Success == status)
        {
            return;
        }
    }

    s_dsiMemWriteCtx.onGoing = false;
    MIPI_DSI_MemoryDoneDriverCallback(status, &dsiDevice);
}

status_t PANEL_DSI_MemWrite(uint8_t virtualChannel, const uint8_t *data, uint32_t length)
{
    status_t status;

    if (s_dsiMemWriteCtx.onGoing)
    {
        return kStatus_Fail;
    }

    s_dsiMemWriteXfer.virtualChannel = virtualChannel;
    s_dsiMemWriteXfer.flags          = kDSI_TransferUseHighSpeed;
    s_dsiMemWriteXfer.sendDscCmd     = true;

    s_dsiMemWriteCtx.onGoing     = true;
    s_dsiMemWriteCtx.txData      = data;
    s_dsiMemWriteCtx.leftByteLen = length;
    s_dsiMemWriteCtx.dscCmd      = kMIPI_DCS_WriteMemoryStart;

    status = BOARD_DsiMemWriteSendChunck();

    if (status != kStatus_Success)
    {
        /* Memory write does not start actually. */
        s_dsiMemWriteCtx.onGoing = false;
    }

    return status;
}

void BOARD_InitMipiPanelTEPin(void)
{
    const gpio_pin_config_t tePinConfig = {
        .direction = kGPIO_DigitalInput,
        .outputLogic  = 0,
        .interruptMode = kGPIO_IntRisingEdge,
    };

    /*
     * TE pin configure method:
     *
     * The TE pin interrupt is like this:
     *
     *            VSYNC
     *         +--------+
     *         |        |
     *         |        |
     * --------+        +----------------
     *
     * 1. If one frame send time is shorter than one frame refresh time, then set
     *    TE pin interrupt at the start of VSYNC.
     * 2. If one frame send time is longer than one frame refresh time, and shorter
     *    than two frames refresh time, then set TE pin interrupt at the end of VSYNC.
     * 3. If one frame send time is longer than two frame refresh time, tearing effect
     *    could not be removed.
     *
     * For RM67162 @60Hz frame rate in single core version, frame refresh time is 16.7 ms. After test,
     * one frame send time is shorter than one frame refresh time. So TE interrupt is
     * set to start of VSYNC.
     */

    GPIO_PinInit(BOARD_MIPI_PANEL_TE_GPIO, BOARD_MIPI_PANEL_TE_PIN, &tePinConfig);

    GPIO_PortClearInterruptFlags(BOARD_MIPI_PANEL_TE_GPIO, 1<<BOARD_MIPI_PANEL_TE_PIN);
    GPIO_PortEnableInterrupts(BOARD_MIPI_PANEL_TE_GPIO, 1<<BOARD_MIPI_PANEL_TE_PIN);

    NVIC_SetPriority(BOARD_MIPI_PANEL_TE_IRQ, 3);
    NVIC_EnableIRQ(BOARD_MIPI_PANEL_TE_IRQ);
}

status_t BOARD_PrepareDisplayController(void)
{
    status_t status;

    status = BOARD_InitDisplayInterface();
    if (kStatus_Success == status)
    {
        /*
         * Suggest setting to low priority. Because a new DSI transfer is prepared
         * in the callback BOARD_DsiMemWriteCallback, so the core spends more time
         * in ISR. Setting the low priority, then the important ISR won't be blocked.
         */
        NVIC_SetPriority(APP_MIPI_DSI_IRQn, 6);
    }
    else
    {
        return status;
    }

    memset(&s_dsiMemWriteCtx, 0, sizeof(dsi_mem_write_ctx_t));

    /* Create the MIPI DSI trasnfer handle for non-blocking data trasnfer. */
    return DSI_TransferCreateHandle(APP_MIPI_DSI, &s_dsiDriverHandle, BOARD_DsiMemWriteCallback, NULL);
}

/* Smart panel TE pin IRQ handler. */
void BOARD_DisplayTEPinHandler(void)
{
    DC_FB_DSI_CMD_TE_IRQHandler(&g_dc);
}

