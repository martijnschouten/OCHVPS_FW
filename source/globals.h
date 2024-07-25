/*
 * @project EPFL-HXL_PS_v1.0
 * @file    userdef.h
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

#ifndef GLOBALS_H_
#define GLOBALS_H_

//Standard Includes
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

/* SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "MK60D10.h"
#include "peripherals.h"

#include "fsl_adc16.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "fsl_pit.h"
#include "fsl_uart.h"
#include "pin_mux.h"

//Personal Includes
#include "misc/debug.h"
#include "misc/hPID_v1.h"
#include "misc/hxl_ps_struct.h"

#include "modules/mADC.h"
#include "modules/mBuckBoost.h"
#include "modules/mDelay.h"
#include "modules/mEEPROM.h"
#include "modules/mSerial.h"
#include "modules/mSerialCommand.h"

#include "tasks/tComm.h"
#include "tasks/tDCDC.h"
#include "tasks/tHB.h"

// external declarations - tasks
extern tComm gTComm;
extern tDCDC gTDCDC;
extern tHB gTHB;
// external declarations - modules
extern mEEPROM eeprom;
extern mSerial serial;
extern mSerialCommand sCmd;

#endif /* GLOBALS_H_ */
