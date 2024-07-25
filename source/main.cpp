/*
 * @project EPFL-HXL_PS_v1.0
 * @file    main.cpp
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
 
//Personal Includes
#include "globals.h"

//----------------------------------------------
// Global objects (don't forget to call setup()!)
//----------------------------------------------
// tasks
tComm gTComm;
tDCDC gTDCDC;
tHB gTHB;
// modules
mEEPROM eeprom;
mSerial serial;
mSerialCommand sCmd;

int main(void) {
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	//-----------------------------------
	// init EEPROM module
	//-----------------------------------
	eeprom.setup();

	//-----------------------------------
	// init all tasks
	//-----------------------------------
	gTComm.setup();
	gTDCDC.setup();
	gTHB.setup();

	while (1) {

		//-----------------------------------
		// run all tasks, speed limited internally
		//-----------------------------------
		gTComm.run();
		gTDCDC.run();
		gTHB.run();
	}
}
