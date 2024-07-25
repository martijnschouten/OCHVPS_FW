/*
 * @project EPFL-HXL_PS_v1.1
 * @file    mDelay.cpp
 * @brief   Author:             MBE / VPY
 *          Institute:          EPFL
 *          Laboratory:         LMTS
 *          Firmware version:   v1.11
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

//Personal Includes
#include "source/globals.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t mDelay::numberOfInstances = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
extern "C" {
void PIT0_IRQHandler(void) {
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

	// increase timestamp counter
	sTimestamp_ms_1ms++;

	// Counter update
	for (uint32_t i = 0; i < nb_delays; i++) {
		//Decrement counter
		if ((false == sDly.CounterTab[i].isFree)
				&& (sDly.CounterTab[i].Counter > 0)) {
			sDly.CounterTab[i].Counter--;
		}

		// Indicate delay done when 0
		if ((false == sDly.CounterTab[i].isFree)
				&& (sDly.CounterTab[i].Counter == 0)) {
			sDly.CounterTab[i].DelayDone = true;
		}
	}
}

void PIT1_IRQHandler(void) {
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_1, kPIT_TimerFlag);

	// increase timestamp counter
	sTimestamp_us_100us += 100;
}
}

mDelay::mDelay() {
	mDelay(-1);
}

mDelay::mDelay(int32_t aDelayInitValue) {
	numberOfInstances++;
	instance_id = -1;

	//If first instance
	if (numberOfInstances <= 1) {
		//dbg("first instance of mDelay created!\n");
		this->setup();

		instance_id = getDelay(aDelayInitValue);
		if (instance_id < 0) {
			//dbg("Unable to get delay!");
			while (true)
				;
		}
	}
	// If instance > 1
	else if (numberOfInstances < nb_delays) {
		//dbg("new instance of mDelay created!\n");
		instance_id = getDelay(aDelayInitValue);
		if (instance_id < 0) {
			//dbg("Unable to get delay!");
			while (true)
				;
		}
	}

	// If too many counters!
	else {
		//dbg("failed to create a new instance of mDelay because already instantiated to many times!\n");
		while (true)
			;
	}
}

mDelay::~mDelay() {
	//dbg("destructing 1 instance of mDelay");
	release();
	numberOfInstances--;
}

//void mDelay_setup(void){
uint32_t mDelay::setup(void) {

	//-------------------------------------------------------------------------
	// PIT 0
	//-------------------------------------------------------------------------
	/* Structure of initialize PIT */
	pit_config_t pitConfig0;

	/*
	 * pitConfig.enableRunInDebug = false;
	 */
	PIT_GetDefaultConfig(&pitConfig0);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig0);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0,
			USEC_TO_COUNT(delay_pit0_us, CLOCK_GetFreq(kCLOCK_BusClk)));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT0_IRQn);

	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);

	// Init delays struct
	for (uint32_t i = 0; i < nb_delays; i++) {
		sDly.CounterTab[i].DelayDone = false;
		sDly.CounterTab[i].Counter = 0;
		sDly.CounterTab[i].isFree = true;
	}

	// init timestamp
	sTimestamp_ms_1ms = 0;

	//-------------------------------------------------------------------------
	// PIT 1
	//-------------------------------------------------------------------------
	/* Structure of initialize PIT */
	pit_config_t pitConfig1;

	/*
	 * pitConfig.enableRunInDebug = false;
	 */
	PIT_GetDefaultConfig(&pitConfig1);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig1);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_1,
			USEC_TO_COUNT(delay_pit1_us, CLOCK_GetFreq(kCLOCK_BusClk)));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_1, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT1_IRQn);

	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_1);

	// init timestamp
	sTimestamp_ms_1ms = 0;
	return 0;
}

//------------------------------------------------------------
// Get and setup a delay
// aDelay	: the delay (ms)
// Retour	: the delay number
//------------------------------------------------------------
int32_t mDelay::getDelay(int32_t aDelay) {
	int32_t aDelayNb = 0;
	uint32_t i = 0;

	// Find and configure a free delay
	for (i = 0; (i < nb_delays) && (false == sDly.CounterTab[i].isFree);
			i++, aDelayNb = i)
		;

	// Check if a delay was found
	if ((i == nb_delays) && (false == sDly.CounterTab[i - 1].isFree)) {
		aDelayNb = -1;
	} else {
		// Delay setup
		sDly.CounterTab[aDelayNb].isFree = false;
		sDly.CounterTab[aDelayNb].Counter = aDelay;
		sDly.CounterTab[aDelayNb].DelayDone = false;
	}

	return aDelayNb;
}

//------------------------------------------------------------
// Check if the delay is done
// aDelayNb	: the delay number
// Retour	: state of the delay
//------------------------------------------------------------
bool mDelay::isDelayDone(void) {
	return sDly.CounterTab[instance_id].DelayDone;
}

//------------------------------------------------------------
// Release a delay
// aDelayNb	: the delay number
//------------------------------------------------------------
void mDelay::release(void) {
	sDly.CounterTab[instance_id].DelayDone = false;
	sDly.CounterTab[instance_id].Counter = 0;
	sDly.CounterTab[instance_id].isFree = true;
}

//------------------------------------------------------------
// Restart a delay
// aPit			: which PIT
// aDelayNb	: the delay number
// aDelay		: the delay time
//------------------------------------------------------------
void mDelay::restart(int32_t aDelay_ms) {
	sDly.CounterTab[instance_id].DelayDone = false;
	sDly.CounterTab[instance_id].Counter = aDelay_ms;
	sDly.CounterTab[instance_id].isFree = false;
}

void mDelay::waitUntilFinished() {
	while (isDelayDone() == false)
		;
	return;
}

mTimestamp::mTimestamp() {}
mTimestamp::~mTimestamp() {}

uint32_t mTimestamp::now_ms_1ms() {
	return sTimestamp_ms_1ms;
}

uint32_t mTimestamp::now_us_100us() {
	return sTimestamp_us_100us;
}
