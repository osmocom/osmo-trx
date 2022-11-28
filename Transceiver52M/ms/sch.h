#pragma once
/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015 by Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 * (C) 2016 by Tom Tsou <tom.tsou@ettus.com>
 * (C) 2017 by Harald Welte <laforge@gnumonks.org>
 * (C) 2022 by sysmocom s.f.m.c. GmbH <info@sysmocom.de> / Eric Wild <ewild@sysmocom.de>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <osmocom/core/bits.h>

struct sch_info  {
	int bsic;
	int t1;
	int t2;
	int t3p;
};

#define GSM_SCH_INFO_LEN		25
#define GSM_SCH_UNCODED_LEN		35
#define GSM_SCH_CODED_LEN		78

int gsm_sch_decode(uint8_t *sb_info, sbit_t *burst);
int gsm_sch_parse(const uint8_t *sb_info, struct sch_info *desc);
int gsm_sch_to_fn(struct sch_info *sch);
int gsm_sch_check_fn(int fn);
int gsm_fcch_check_fn(int fn);
int gsm_fcch_check_ts(int ts, int fn);
int gsm_sch_check_ts(int ts, int fn);

double gsm_fcch_offset(float *burst, int len);

int float_to_sbit(const float *in, sbit_t *out, float scale, int len);

