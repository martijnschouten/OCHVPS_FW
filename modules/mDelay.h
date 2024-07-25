/*
 * @project EPFL-HXL_PS_v1.0
 * @file    mDelay.h
 * @brief   Author:             MBE / VPY
 *          Institute:          EPFL
 *          Laboratory:         LMTS
 *          Firmware version:   v1.09
 *          Created on:         12.02.2024
 *          Last modifications: 22.03.2024
 *
 * HXL_PS © 2021-2024 by MBE
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the 
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public 
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this library.  If not, see:
 * <http://www.gnu.org/licenses/>.
 *
 * NO HELP WILL BE GIVEN IF YOU MODIFY THIS CODE !!!
 */

#ifndef MDELAY_H_
#define MDELAY_H_

//Personal Includes
#include "source/globals.h"

static const uint32_t nb_delays = 20;
static const uint32_t delay_pit0_us = 1000;
static const uint32_t delay_pit1_us = 100;

// Delay struct
typedef struct {
	int32_t Counter;
	bool isFree;
	bool DelayDone;
} CounterStruct;

typedef struct {
	CounterStruct CounterTab[nb_delays];
} DlyStruct;

static DlyStruct sDly __attribute__((used));
static uint32_t sTimestamp_ms_1ms __attribute__((used));
static uint32_t sTimestamp_us_100us __attribute__((used));

class mDelay {
public:
	mDelay();
	mDelay(int32_t aDelayInitValue);
	~mDelay();
	bool isDelayDone(void);
	void release(void);
	void restart(int32_t aDelay);
	void waitUntilFinished();

private:
	static uint32_t numberOfInstances;
	uint32_t instance_id;

	uint32_t setup(void);
	int32_t getDelay(int32_t aDelay);
};

class mTimestamp {
public:
	mTimestamp();
	~mTimestamp();
	static uint32_t now_ms_1ms();
	static uint32_t now_us_100us();
};

#endif /* MDELAY_H_ */
