/*
	sim_profiler.h

	Copyright 2015 Michael Hughes <squirmyworms@embarqmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SIM_PROFILER_H
#define __SIM_PROFILER_H

#define CONFIG_PROFILER_ACTIVE 1

static uint8_t __attribute__ ((__unused__)) _sim_profiler_active = CONFIG_PROFILER_ACTIVE;

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_PROFILER_ACTIVE == 1
	#define PROFILE_ISEQ(x, y) \
		sim_profiler_profile_iseq_start(x); \
		y; \
		sim_profiler_profile_iseq_stop(x);
#else
	#define PROFILE_ISEQ(x, y) \
		y;
#endif

extern void sim_profiler_init(const char *op_name_table[256]);
extern void sim_profiler_generate_report(void);

extern void sim_profiler_profile_iseq_start(uint8_t handler);
extern void sim_profiler_profile_iseq_stop(uint8_t handler);

#ifdef __cplusplus
extern "C" {
#endif

#endif /* #ifndef __SIM_PROFILER_H */

