/* Himax Android Driver Sample Code for Himax chipset
 *
 * Copyright (C) 2015 Himax Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef HIMAX_DEBUG_H
#define HIMAX_DEBUG_H

#include "himax_common.h"
#include "himax_platform.h"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)

#define HIMAX_PROC_TOUCH_FOLDER		"android_touch"
#define HIMAX_PROC_DEBUG_LEVEL_FILE	"debug_level"
#define HIMAX_PROC_VENDOR_FILE		"vendor"
#define HIMAX_PROC_ATTN_FILE		"attn"
#define HIMAX_PROC_INT_EN_FILE		"int_en"
#define HIMAX_PROC_LAYOUT_FILE		"layout"

int himax_touch_proc_init(void);
void himax_touch_proc_deinit(void);
bool getFlashDumpGoing(void);

#ifdef HX_TP_PROC_DIAG
#ifdef HX_TP_PROC_2T2R
int16_t *getMutualBuffer_2(void);
uint8_t	getXChannel_2(void);
uint8_t	getYChannel_2(void);

void	setMutualBuffer_2(void);
void	setXChannel_2(uint8_t x);
void	setYChannel_2(uint8_t y);
#endif
extern uint8_t diag_coor[128];// = {0xFF};

void	himax_ts_diag_func(void);
int16_t *getMutualBuffer(void);
int16_t *getMutualNewBuffer(void);
int16_t *getMutualOldBuffer(void);
int16_t *getSelfBuffer(void);
uint8_t	getDiagCommand(void);
uint8_t	getXChannel(void);
uint8_t	getYChannel(void);

void	setMutualBuffer(void);
void	setMutualNewBuffer(void);
void	setMutualOldBuffer(void);
void	setXChannel(uint8_t x);
void	setYChannel(uint8_t y);
#endif

#endif

#endif

