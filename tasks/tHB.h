/*
 * @project EPFL-HXL_PS_v1.0
 * @file    tHB.h
 * @brief   Author:             MBE
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

#ifndef THB_H_
#define THB_H_

//Personal Includes
#include "source/globals.h"

#define HALF_BRIDGES 8

#define MIN_DUTY_CYCLE 1
#define MAX_DUTY_CYCLE 100

#define MIN_PHASE 0
#define MAX_PHASE 360

class tHB {
public:
	tHB();
	void setup();
	void run();

	enum hbPinStateEnum {
		HB_OFF = 0, HB_ON = 1
	};
	
	// Maximal and minimal frequencies for switching
	float MAX_FREQ;
	float MIN_FREQ;

	float MIN_PULSE;

	void setOperationMode(mode_cmd_t cmd_recv);
	void clearOperationMode(mode_cmd_t cmd_recv);

private:
	static uint32_t numberOfInstances;

	void set_hb_hv(uint8_t hb_channel);
	void set_hb_gnd(uint8_t hb_channel);
};

#endif /* THB_H_ */
