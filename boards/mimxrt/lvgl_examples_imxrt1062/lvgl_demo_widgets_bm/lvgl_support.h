/*
 * Copyright 2021, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LVGL_SUPPORT_H
#define LVGL_SUPPORT_H

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_PANEL_RK043FN02H  0 /* RK043FN02H-CT */
#define DEMO_PANEL_RK043FN66HS 1 /* RK043FN66HS-CTG */
#define DEMO_PANEL_RK050HR18   2 /* RK050HR18-CTG */
#define DEMO_PANEL_RK050HR01   3 /* RK050HR01-CT */

/* @TEST_ANCHOR */

#ifndef DEMO_PANEL
#define DEMO_PANEL DEMO_PANEL_RK050HR01
#endif

#if ((DEMO_PANEL == DEMO_PANEL_RK043FN66HS) || (DEMO_PANEL == DEMO_PANEL_RK043FN02H))
#define LCD_WIDTH  480
#define LCD_HEIGHT 272
#elif ((DEMO_PANEL == DEMO_PANEL_RK050HR18) || (DEMO_PANEL == DEMO_PANEL_RK050HR01))
#define LCD_WIDTH  800
#define LCD_HEIGHT 480
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

void lv_port_pre_init(void);
void lv_port_disp_init(void);
void lv_port_indev_init(void);

#if defined(__cplusplus)
}
#endif

#endif /*LVGL_SUPPORT_H */
