/*
 * @project EPFL-HXL_PS_v1.0
 * @file    tComm.h
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

#ifndef _TCOMM_H
#define _TCOMM_H

//Personal Includes
#include "source/globals.h"

class tComm {
public:
	tComm();
	void setup();
	void run();

private:
	static uint32_t numberOfInstances;
	static bool monitoringEnabled;
	static bool debugEnabled;

	static uint8_t sm_error;

	const uint32_t loopDelayVal_ms = 1;
	const uint32_t monitoring_delay_ms = 5;
	//---------------------------------
	// Configuration command
	//---------------------------------
	/* set the board with different settings */
	static void Conf();
	/* perform a test on EEPROM */
	static void MEMt();
	static void MEMr();
	static void MEMw();

	//------------------------------------------------------
	// getters
	//------------------------------------------------------

	// Miscellaneous
	/* get hardware version */
	static void QHW();
	/* get firmware version */
	static void QFW();
	/* get board name */
	static void QName();
	/* get status of debug mode*/
	static void QDebug();
	/* get parameters */
	static void QPRM();

	// Voltage
	/* get the voltage monitors for high voltage and low voltage*/
	static void QMes();
	/* get the voltage set by the user for high voltage and low voltage */
	static void QSet();

	// Monitoring correction
	/* get coefficient correction for monitoring (high voltage) */
	static void QCC();
	/* get coefficient correction for monitoring (high voltage) */
	static void QCH();
	/* get coefficient correction for monitoring (low voltage) */
	static void QCL();

	//PID
	/* get the PID status and coefficients*/
	static void QPID();

	//------------------------------------------------------
	// setters
	//------------------------------------------------------/
	// Miscellaneous
	/* set board name */
	static void SName();
	/* set status for debug mode*/
	static void SDebug();

	// Voltage
	/* set high voltage */
	static void SHV();
	/* set low voltage */
	static void SLV();

	// Monitoring correction
	/* set coefficient correction for monitoring (current) */
	static void SCC();
	/* set coefficient correction for monitoring (high voltage) */
	static void SCH();
	/* set coefficient correction for monitoring (low voltage) */
	static void SCL();

	// PID
	/* set PID coefficients */
	static void SKpid();
	/* set PID status */
	static void SPID();

	// HB Modes
	/* set mode x */
	static void SMx();
	/* clear mode x */
	static void CMx();

	//------------------------------------------------------
	// others
	//------------------------------------------------------
	/* monitoring */
	static void moni();
	static void monitoringPrintColumns();
	static void monitoringPrint();

	/* emergency stop */
	static void EStop();
	/* help message */
	static void help();
	static void admin();
	static void unlock();
	/* Handler for command that isn't matched */
	static void unrecognized(const char *command);

	/* test values functions */
	static bool test_channel(mode_cmd_t cmd_recv);
	static bool test_values(mode_cmd_t cmd_recv);

};

#endif
