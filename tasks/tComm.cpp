/*
 * @project EPFL-HXL_PS_v1.1
 * @file    tComm.cpp
 * @brief   Author:             MBE
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
pcb_t PCB;
mode_cmd_t sm_cmd;
mode_cmd_t cm_cmd;

uint32_t tComm::numberOfInstances = 0;

bool tComm::monitoringEnabled = false;
bool tComm::debugEnabled = true;
char send_buffer[100];

/*******************************************************************************
 * Code
 ******************************************************************************/
tComm::tComm() {
	numberOfInstances++;
	if (numberOfInstances > 1) {
		while (true); //should never happen
	}
}

void tComm::setup() {
	// test
	PCB.version = eeprom.read_float(eeprom.ADDR_HW_VER);
	if (isnan(PCB.version)){
		// PCB not configured
		PCB.conf = false;
		PCB.admin = true;
		PCB.lock = true;
	} else {
		PCB.conf = (bool) (eeprom.read_uint8(eeprom.ADDR_CONF));
		PCB.admin = (bool) (eeprom.read_uint8(eeprom.ADDR_ADMIN));
		PCB.lock = (bool) (eeprom.read_uint8(eeprom.ADDR_LOCK));
	}

	serial.setup();
	serial.writeInit();

	/*********************************************************************
	 // NORMAL MODE
	 *********************************************************************/
	// Getters - Miscellaneous
	//--------------------------------------------------------------------
	/* get the name of the board */
	sCmd.addCommand("QName", this->QName);
	/* get firmware version */
	sCmd.addCommand("QFW", this->QFW);
	/* get hardware version */
	sCmd.addCommand("QHW", this->QHW);
	/* get parameters */
	sCmd.addCommand("QPRM", this->QPRM);
	//--------------------------------------------------------------------
	// Getters - Voltage
	//--------------------------------------------------------------------
	/* get the voltage monitors for high voltage and low voltage*/
	sCmd.addCommand("QMes", this->QMes);
	/* get the voltage set by the user for high voltage and low voltage */
	sCmd.addCommand("QSet", this->QSet);
	//--------------------------------------------------------------------
	// Setters - Voltage
	//--------------------------------------------------------------------
	/* set high voltage */
	sCmd.addCommand("SHV", this->SHV);
	//--------------------------------------------------------------------
	// Setters - HB Modes
	//--------------------------------------------------------------------
	/* set mode (on) for channel X */
	sCmd.addCommand("SMx", this->SMx);
	/* clear mode (off) for channel X */
	sCmd.addCommand("CMx", this->CMx);
	//--------------------------------------------------------------------
	// others
	//--------------------------------------------------------------------
	/* monitoring */
	sCmd.addCommand("Moni", this->moni);
	/* emergency stop */
	sCmd.addCommand("EStop", this->EStop);
	/* help message */
	sCmd.addCommand("help", this->help);

	/*********************************************************************
	 // ADMIN MODE
	 *********************************************************************/
	// Configuration command
	//--------------------------------------------------------------------
	/* set or clear admin mode */
	sCmd.addCommand("ADM", this->admin);
	/* set the board with different settings */
	sCmd.addCommand("Conf", this->Conf);
	/* perform a test on EEPROM */
	sCmd.addCommand("MEMt", this->MEMt);
	sCmd.addCommand("MEMr", this->MEMr);
	sCmd.addCommand("MEMw", this->MEMw);
	//--------------------------------------------------------------------
	// Getters - miscellaneous
	//--------------------------------------------------------------------
	/* get status of debug mode*/
	sCmd.addCommand("QDebug", this->QDebug);
	//--------------------------------------------------------------------
	// Getters - PID
	//--------------------------------------------------------------------
	/* get PID status and coefficient */
	sCmd.addCommand("QPID", this->QPID);
	//--------------------------------------------------------------------
	// Getters - Monitoring correction
	//--------------------------------------------------------------------
	/* get coefficient correction for monitoring (current) */
	sCmd.addCommand("QCC", this->QCC);
	/* get coefficient correction for monitoring (high voltage) */
	sCmd.addCommand("QCH", this->QCH);
	/* get coefficient correction for monitoring (low voltage) */
	sCmd.addCommand("QCL", this->QCL);
	//--------------------------------------------------------------------
	// setters - Miscellaneous
	//--------------------------------------------------------------------
	/* set board name */
	sCmd.addCommand("SName", this->SName);
	/* set status for debug mode*/
	sCmd.addCommand("SDebug", this->SDebug);
	//--------------------------------------------------------------------
	// setters - Voltage
	//--------------------------------------------------------------------
	/* set low voltage */
	sCmd.addCommand("SLV", this->SLV);
	//--------------------------------------------------------------------
	// Setters - Monitoring correction
	//--------------------------------------------------------------------
	// set coefficient correction for monitoring (current)
	sCmd.addCommand("SCC", this->SCC);
	// set coefficient correction for monitoring (high voltage)
	sCmd.addCommand("SCH", this->SCH);
	// set coefficient correction for monitoring (low voltage)
	sCmd.addCommand("SCL", this->SCL);
	//--------------------------------------------------------------------
	// Setters - PID
	//--------------------------------------------------------------------
	/* PID status */
	sCmd.addCommand("SPID", this->SPID);
	// PID coefficient
	sCmd.addCommand("SKpid", this->SKpid);
	//--------------------------------------------------------------------
	// others
	//--------------------------------------------------------------------
	/* unlocking board */
	sCmd.addCommand("LMTS", this->unlock);
	/* Handler for command that isn't matched */
	sCmd.setDefaultHandler(this->unrecognized);
}

void tComm::run() {
	//dbg("new iteration of tComm::run()!");

	static mDelay loopDelay(loopDelayVal_ms);
	static mDelay monitoringDelay(monitoring_delay_ms);
	static uint32_t counter = 0;

	if (loopDelay.isDelayDone()) {
		loopDelay.restart(loopDelayVal_ms);
		counter++;
		sCmd.readSerial();     // We don't do much, just process serial commands
	}

	if (monitoringDelay.isDelayDone()) {
		monitoringDelay.restart(monitoring_delay_ms);
		if (tComm::monitoringEnabled) {
			monitoringPrint();
		}
	}
}

//---------------------------------
// Configuration command
//---------------------------------
/* set the board with different settings */
void tComm::Conf() {
	if ((PCB.lock == true) and (PCB.admin == true)) {
		char *buffptr;
		buffptr = sCmd.next();
		float hv_max = sCmd.parseFloatArg();
		if (hv_max <= 4500){
			eeprom.conf(buffptr, hv_max);
			// PCB
			PCB.conf = true;
			//
			serial.println("!!REBOOT REQUIRED!!");

		} else {
			serial.println("\n[ERR] Need to use a 2kV or 3kV DCDC");
		}
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* perform a test on EEPROM */
void tComm::MEMt() {
	if ((PCB.lock == true) and (PCB.admin == true)) {
		char *buffptr;
		buffptr = sCmd.next();
		const char *u_caractere = "u";
		const char *s_caractere = "s";
		const char *f_caractere = "f";

		if (strcmp(buffptr, u_caractere) == 0) {
			serial.println("\n[MEM] Test uint8");
			eeprom.test_uint8();
		} else if (strcmp(buffptr, f_caractere) == 0) {
			uint8_t addr = (uint8_t) sCmd.parseLongArg();
			float buff_float = sCmd.parseFloatArg();
			serial.println("\n[MEM] Test float");
			eeprom.test_float(addr, buff_float);
		} else if (strcmp(buffptr, s_caractere) == 0) {
			uint8_t addr = (uint8_t) sCmd.parseLongArg();
			char *buffptr_string;
			buffptr_string = sCmd.next();
			serial.println("\n[MEM] Test string");
			eeprom.test_string(addr, buffptr_string);
		} else {
			snprintf(send_buffer, 100, "\n[MEM] Bad command");
		}
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* perform a test on EEPROM */
void tComm::MEMr() {
	if ((PCB.lock == true) and (PCB.admin == true)) {
		char *buffptr;
		buffptr = sCmd.next();
		const char *u_caractere = "u";
		const char *s_caractere = "s";
		const char *f_caractere = "f";

		uint8_t addr = (uint8_t) sCmd.parseLongArg();

		if (strcmp(buffptr, u_caractere) == 0) {
			serial.print("\n[MEM] Read uint8  ");
			uint8_t read_data_uint8 = eeprom.read_uint8(addr);
			snprintf(send_buffer, 100, "[%d]=%d", addr, read_data_uint8);
			serial.println(send_buffer);
		} else if (strcmp(buffptr, f_caractere) == 0) {
			serial.print("\n[MEM] Read float  ");
			float read_data_float = eeprom.read_float(addr);
			snprintf(send_buffer, 100, "[%d]=%f", addr, read_data_float);
			serial.println(send_buffer);
		} else if (strcmp(buffptr, s_caractere) == 0) {
			uint8_t length = (uint8_t) sCmd.parseLongArg();
			serial.print("\n[MEM] Read string ");
			char read_data_string[length];
			eeprom.read_string(addr, length, read_data_string);
			snprintf(send_buffer, 100, "[%d]=", addr);
			serial.print(send_buffer);
			serial.println(read_data_string);
		} else {
			snprintf(send_buffer, 100, "\n[MEM] Bad command");
		}
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* perform a test on EEPROM */
void tComm::MEMw() {
	if ((PCB.lock == true) and (PCB.admin == true)) {
		char *buffptr;
		buffptr = sCmd.next();
		const char *i_caractere = "i";
		const char *u_caractere = "u";
		const char *s_caractere = "s";
		const char *f_caractere = "f";

		uint8_t addr = (uint8_t) sCmd.parseLongArg();

		if (strcmp(buffptr, i_caractere) == 0) {
			eeprom.init();
		} else if (strcmp(buffptr, u_caractere) == 0) {
			uint8_t buff_uint8 = (uint8_t) sCmd.parseLongArg();
			serial.print("\n[MEM] Write uint8  ");
			eeprom.write_uint8(addr, buff_uint8);
			snprintf(send_buffer, 100, "[%d]=%d", addr, buff_uint8);
			serial.println(send_buffer);
		} else if (strcmp(buffptr, f_caractere) == 0) {
			float buff_float = sCmd.parseFloatArg();
			serial.print("\n[MEM] Write float  ");
			eeprom.write_float(addr, buff_float);
			snprintf(send_buffer, 100, "[%d]=%f", addr, buff_float);
			serial.println(send_buffer);
		} else if (strcmp(buffptr, s_caractere) == 0) {
			uint8_t length = (uint8_t) sCmd.parseLongArg();
			char *buffptr_string;
			buffptr_string = sCmd.next();
			serial.print("\n[MEM] Write string ");
			eeprom.write_string(addr, length, buffptr_string);
			snprintf(send_buffer, 100, "[%d]=%s", addr, buffptr_string);
			serial.println(send_buffer);
		} else {
			snprintf(send_buffer, 100, "\n[MEM] Bad command");
		}
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

//---------------------------------
// getters
//---------------------------------

// Miscellaneous
/* get hardware version */
void tComm::QHW() {
	if (PCB.conf == true) {
		snprintf(send_buffer, 100, "\n[QHW] %.1f", PCB.version);
		serial.println(send_buffer);
	} else {
		serial.println("\n[ERR] PCB not configured");
	}
}

/* get firmware version */
void tComm::QFW() {
	if (PCB.conf == true) {
		serial.print("\n[QFW] ");
		serial.println(FW_VERSION);
	} else {
		serial.println("\n[ERR] PCB not configured");
	}
}

/* get board name */
void tComm::QName() {
	if (PCB.conf == true) {
		char buff[21];
		eeprom.read_string(eeprom.ADDR_NAME, 21, buff);
		serial.print("\n[QName] ");
		serial.println(buff);
	} else {
		serial.println("\n[ERR] PCB not configured");
	}
}

/* get status of debug mode*/
void tComm::QDebug() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		serial.print("\n[Debug] ");
		if (tComm::debugEnabled == true) {
			serial.println("ON");
		} else {
			serial.println("OFF");
		}
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* get parameters */
void tComm::QPRM() {
	if (PCB.conf == true) {
		/* get board name */
		char buff[21];
		eeprom.read_string(eeprom.ADDR_NAME, 21, buff);
		serial.print("\n[PRM] PCB \t\t ");
		serial.println(buff);
		/* get hardware version */
		snprintf(send_buffer, 100, "[PRM] HW Version \t v%.1f", PCB.version);
		serial.println(send_buffer);
		/* get firmware version */
		serial.print("[PRM] FW Version \t ");
		serial.println(FW_VERSION);
		/* get DCDC range*/
		snprintf(send_buffer, 100, "[PRM] MAX HV    (V)\t %.1f", gTDCDC.MAX_HV);
		serial.println(send_buffer);
		snprintf(send_buffer, 100, "[PRM] MIN HV    (V)\t %.1f", gTDCDC.MIN_HV);
		serial.println(send_buffer);
		/* get switching specifications */
		snprintf(send_buffer, 100, "[PRM] MAX FREQ  (Hz)\t %.1f", gTHB.MAX_FREQ);
		serial.println(send_buffer);
		snprintf(send_buffer, 100, "[PRM] MIN FREQ  (Hz)\t %.2f", gTHB.MIN_FREQ);
		serial.println(send_buffer);
		snprintf(send_buffer, 100, "[PRM] MIN PULSE (us)\t %.1f", gTHB.MIN_PULSE);
		serial.println(send_buffer);
	} else {
		serial.println("\n[ERR] PCB not configured");
	}
}

// Voltage
/* get the voltage monitors for high voltage and low voltage*/
void tComm::QMes() {
	if ((PCB.conf == true) and (PCB.lock == true)) {
		//float lv_now = gTDCDC.get_ps_lv_vm();
		float lv_now = gTDCDC.get_ps_lv_vm_corrected();
		//float hv_now = gTDCDC.get_ps_hv_vm();
		float hv_now = gTDCDC.get_ps_hv_vm_corrected();

		//snprintf(send_buffer, 100, "\n[QMes] LV = %2.2f V", gTDCDC.get_ps_lv_vm_corrected());
		snprintf(send_buffer, 100, "\n[QMes] HV = %2.2f V ", hv_now);
		serial.print(send_buffer);
		snprintf(send_buffer, 100, "// LV = %2.2f V", lv_now);
		serial.println(send_buffer);
	} else {
		serial.println("");
	}
}

/* get the voltage set by the user for high voltage and low voltage */
void tComm::QSet() {
	if ((PCB.conf == true) and (PCB.lock == true)) {
		snprintf(send_buffer, 100, "\n[QSet] HV_set = %2.1f V ", gTDCDC.hv_set);
		serial.print(send_buffer);
		snprintf(send_buffer, 100, "// LV_set = %2.2f V", gTDCDC.lv_set);
		serial.println(send_buffer);
	} else {
		serial.println("");
	}
}

// Monitoring correction
/* get coefficient correction for monitoring (current) */
void tComm::QCC() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		serial.print("\n[CC] ");
		serial.print(gTDCDC.C0C);
		serial.print(", ");
		serial.print(gTDCDC.C1C);
		serial.print(", ");
		serial.println(gTDCDC.C2C);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}
/* get coefficient correction for monitoring (high voltage) */
void tComm::QCH() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		serial.print("\n[CH] ");
		serial.print(gTDCDC.C0H);
		serial.print(", ");
		serial.print(gTDCDC.C1H);
		serial.print(", ");
		serial.println(gTDCDC.C2H);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* get coefficient correction for monitoring (low voltage) */
void tComm::QCL() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		serial.print("\n[CL] ");
		serial.print(gTDCDC.C0L);
		serial.print(", ");
		serial.print(gTDCDC.C1L);
		serial.print(", ");
		serial.println(gTDCDC.C2L);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

//PID
/* get the PID status and coefficients*/
void tComm::QPID() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		serial.print("\n[PID] ");
		if (gTDCDC.HVPS_PID_PRM.state == true) {
			serial.print("ON (");
		} else {
			serial.print("OFF (");
		}
		serial.print(gTDCDC.HVPS_PID_PRM.Kp);
		serial.print(", ");
		serial.print(gTDCDC.HVPS_PID_PRM.Ki);
		serial.print(", ");
		serial.print(gTDCDC.HVPS_PID_PRM.Kd);
		serial.println(")");
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

//---------------------------------
// setters
//---------------------------------
/* set name */
void tComm::SName() {
	char *buffptr;
	buffptr = sCmd.next();

	if ((PCB.conf == true) and (PCB.admin == true)) {
		eeprom.write_string(eeprom.ADDR_NAME, 21, buffptr);
		serial.print("\n[SName] ");
		serial.println(buffptr);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* set status for debug mode*/
void tComm::SDebug() {
	bool new_debug_val = (unsigned int) sCmd.parseLongArg();

	if ((PCB.conf == true) and (PCB.admin == true)) {
		if (new_debug_val == true) {
			tComm::debugEnabled = true;
			serial.println("\n[Debug] ON");
		} else {
			tComm::debugEnabled = false;
			serial.println("\n[Debug] OFF");
		}
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

// Voltage
/* set high voltage */
void tComm::SHV() {
	float new_hv_val = (float) sCmd.parseDoubleArg();

	if ((PCB.conf == true) and (PCB.lock == true)) {
		if (new_hv_val > gTDCDC.MAX_HV) {
			// limit to max voltage
			new_hv_val = gTDCDC.MAX_HV;
		} else if (new_hv_val < gTDCDC.MIN_HV){
			// limit to max voltage
			new_hv_val = 0;
		}
		gTDCDC.set_ps_hv(new_hv_val);
		snprintf(send_buffer, 100, "\n[HV] %.1f", gTDCDC.hv_set);
		serial.println(send_buffer);
	} else if ((PCB.conf == false) and (PCB.lock == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("");
	}
}

/* set low voltage */
void tComm::SLV() {
	float new_lv_val = (float) sCmd.parseDoubleArg();

	if ((PCB.conf == true) and (PCB.admin == true)) {
		if (new_lv_val > MAX_LV) { // limit to max voltage
			new_lv_val = MAX_LV;
		}
		gTDCDC.set_ps_lv(new_lv_val);
		snprintf(send_buffer, 100, "\n[LV] %.2f", gTDCDC.lv_set);
		serial.println(send_buffer);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

// Monitoring correction
/* set coefficient correction for monitoring (high voltage) */
void tComm::SCC() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		gTDCDC.C0C = sCmd.parseFloatArg();
		gTDCDC.C1C = sCmd.parseFloatArg();
		gTDCDC.C2C = sCmd.parseFloatArg();
		// save new values to EEPROM
		eeprom.write_float(eeprom.ADDR_C0C, gTDCDC.C0C);
		eeprom.write_float(eeprom.ADDR_C1C, gTDCDC.C1C);
		eeprom.write_float(eeprom.ADDR_C2C, gTDCDC.C2C);
		// print confirmation message
		serial.print("[CC] ");
		serial.print(gTDCDC.C0C);
		serial.print(", ");
		serial.print(gTDCDC.C1C);
		serial.print(", ");
		serial.println(gTDCDC.C2C);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* set coefficient correction for monitoring (high voltage) */
void tComm::SCH() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		gTDCDC.C0H = sCmd.parseFloatArg();
		gTDCDC.C1H = sCmd.parseFloatArg();
		gTDCDC.C2H = sCmd.parseFloatArg();
		// save new values to EEPROM
		eeprom.write_float(eeprom.ADDR_C0H, gTDCDC.C0H);
		eeprom.write_float(eeprom.ADDR_C1H, gTDCDC.C1H);
		eeprom.write_float(eeprom.ADDR_C2H, gTDCDC.C2H);
		// print confirmation message
		serial.print("[CH] ");
		serial.print(gTDCDC.C0H);
		serial.print(", ");
		serial.print(gTDCDC.C1H);
		serial.print(", ");
		serial.println(gTDCDC.C2H);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* set coefficient correction for monitoring (low voltage) */
void tComm::SCL() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		gTDCDC.C0L = sCmd.parseFloatArg();
		gTDCDC.C1L = sCmd.parseFloatArg();
		gTDCDC.C2L = sCmd.parseFloatArg();
		// save new values to EEPROM
		eeprom.write_float(eeprom.ADDR_C0L, gTDCDC.C0L);
		eeprom.write_float(eeprom.ADDR_C1L, gTDCDC.C1L);
		eeprom.write_float(eeprom.ADDR_C2L, gTDCDC.C2L);
		// print confirmation message
		serial.print("[CL] ");
		serial.print(gTDCDC.C0L);
		serial.print(", ");
		serial.print(gTDCDC.C1L);
		serial.print(", ");
		serial.println(gTDCDC.C2L);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

// PID
/* set the coefficients for PID */
void tComm::SKpid() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		// get new values
		gTDCDC.HVPS_PID_PRM.Kp = sCmd.parseFloatArg();
		gTDCDC.HVPS_PID_PRM.Ki = sCmd.parseFloatArg();
		gTDCDC.HVPS_PID_PRM.Kd = sCmd.parseFloatArg();
		// set PIUD with new values
		gTDCDC.set_PID();
		// save new values to EEPROM
		eeprom.write_float(eeprom.ADDR_PID_KP, gTDCDC.HVPS_PID_PRM.Kp);
		eeprom.write_float(eeprom.ADDR_PID_KI, gTDCDC.HVPS_PID_PRM.Ki);
		eeprom.write_float(eeprom.ADDR_PID_KD, gTDCDC.HVPS_PID_PRM.Kd);
		// print confirmation message
		serial.print("\n[PID] ");
		serial.print(gTDCDC.HVPS_PID_PRM.Kp);
		serial.print(", ");
		serial.print(gTDCDC.HVPS_PID_PRM.Ki);
		serial.print(", ");
		serial.println(gTDCDC.HVPS_PID_PRM.Kd);
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

/* set or clear the PID */
void tComm::SPID() {
	if ((PCB.conf == true) and (PCB.admin == true)) {
		// get new state
		gTDCDC.HVPS_PID_PRM.state = (bool) sCmd.parseDoubleArg();
		// save new state to EEPROM
		eeprom.write_uint8(eeprom.ADDR_PID_STATE, (uint8_t) gTDCDC.HVPS_PID_PRM.state);
		// print confirmation message
		if (gTDCDC.HVPS_PID_PRM.state == true) {
			serial.println("\n[PID] ON");
		} else {
			serial.println("\n[PID] OFF");
		}
	} else if ((PCB.conf == false) and (PCB.admin == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("\n[ERR] Command not allowed in normal mode");
	}
}

// HB Modes
/* set mode 1 (on) for hb(x) */
void tComm::SMx() {
	if ((PCB.conf == true) and (PCB.lock == true)) {
		sm_cmd.mode = (operationModeEnum) sCmd.parseLongArg();
		sm_cmd.channel = (uint8_t) sCmd.parseLongArg();
		sm_cmd.frequency = sCmd.parseFloatArg();
		sm_cmd.pos_duty = sCmd.parseFloatArg();
		sm_cmd.neg_duty = sCmd.parseFloatArg();
		sm_cmd.pulse_phase = sCmd.parseFloatArg();
		sm_cmd.phase_shift = sCmd.parseFloatArg();

		// check selection
		if (test_values(sm_cmd) == false) {
			return;
		}

		// set mode 1 for the selected channel(s)
		gTHB.setOperationMode(sm_cmd);

		// display the selected parameters for mode x
		snprintf(send_buffer, 100,
				"\n[SM%d] ON (%3.1f Hz | %3.1f | %3.1f | %3.1f | %3.1f)",
				(uint8_t) sm_cmd.mode, sm_cmd.frequency, sm_cmd.pos_duty,
				sm_cmd.neg_duty, sm_cmd.pulse_phase, sm_cmd.phase_shift);
		serial.println(send_buffer);
	} else if ((PCB.conf == false) and (PCB.lock == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("");
	}
}

/* clear mode 1 (off) for hb(x) */
void tComm::CMx() {
	if ((PCB.conf == true) and (PCB.lock == true)) {
		cm_cmd.mode = (operationModeEnum) sCmd.parseLongArg();
		cm_cmd.channel = (uint8_t) sCmd.parseLongArg();

		// check selection
		if (test_channel(cm_cmd) == false) {
			return;
		}
		// clear mode 1 for the selected channel(s)
		gTHB.clearOperationMode(cm_cmd);

		// display the selected parameters for mode x
		snprintf(send_buffer, 100, "\n[CM%d] OFF (%d)", (uint8_t) cm_cmd.mode,
				cm_cmd.channel);
		serial.println(send_buffer);
	} else if ((PCB.conf == false) and (PCB.lock == true)) {
		serial.println("\n[ERR] PCB not configured");
	} else {
		serial.println("");
	}
}

//---------------------------------
// others
//---------------------------------
/* set or clear the PID */
void tComm::admin() {
	if (PCB.lock == true) {
		PCB.admin = (bool) sCmd.parseDoubleArg();
		// save new values to EEPROM
		eeprom.write_uint8(eeprom.ADDR_ADMIN, (uint8_t) PCB.admin);
		// print confirmation message
		serial.print("\n[ADM] ");
		if (PCB.admin == 1) {
			serial.println("ON");
		} else {
			serial.println("OFF");
		}
	} else {
		serial.println("");
	}
}

// unlock
void tComm::unlock() {
	if (PCB.lock == false){
		PCB.lock = true;
	} else {
		PCB.lock = false;
	}
	eeprom.write_uint8(eeprom.ADDR_LOCK, (uint8_t) PCB.lock);
	serial.println("");
}

/* monitoring */
void tComm::moni() {
	uint32_t moni_val = (uint8_t) sCmd.parseLongArg();

	if (moni_val == 1) {
		//tComm::monitoringPrintColumns();
		monitoringPrint();
		tComm::monitoringEnabled = false;
		serial.print("\n");
	} else if (moni_val == 2) {
		//tComm::monitoringPrintColumns();
		monitoringPrint();
		tComm::monitoringEnabled = true;
	} else {
		tComm::monitoringEnabled = false;
	}
}

void tComm::monitoringPrintColumns() {
	serial.println("\n[columns], timestamp, Vset, LV, HV ...");
}

void tComm::monitoringPrint() {
	//TODO: this is a debug method, it might be changed at any moment, probably breaking GUI
	serial.print("\n[moni], ");
	serial.print_time(mTimestamp::now_ms_1ms());
	serial.print(", ");
	serial.print_hv_vm(gTDCDC.hv_set);
	serial.print(", ");
	serial.print_lv_vm(gTDCDC.lv_set);
	serial.print(", ");
	serial.print_lv_vm(gTDCDC.get_ps_lv_vm_corrected());
	serial.print(", ");
	serial.print_hv_vm(gTDCDC.get_ps_hv_vm_corrected());
	serial.print(", ");
	serial.print_current(gTDCDC.get_ps_hv_cm_corrected());
	serial.print(", ");

	for (uint8_t i = 1; i < 9; i++) {
		serial.print_current(gTDCDC.get_hb_cm_chx(i));
		serial.print(", ");
	}
}

/* ermergency stop */
void tComm::EStop() {
	gTDCDC.set_ps_hv(0);
	gTHB.setup();

	if (tComm::debugEnabled == true) {
		serial.println("\n[EStop] DONE");
	}
}

/* help message */
void tComm::help() {
	serial.println(
			"\n[HLP] This is the help message - ask MBE for implementation");
	if (PCB.admin == false) {
		sCmd.printCommandListUser();

	} else {
		sCmd.printCommandListAdmin();
	}

}

// this gets set as the default handler, and gets called when no other command matches.
void tComm::unrecognized(const char *command) {
	if (tComm::debugEnabled == true) {
		serial.print("\n[ERR] ");
		serial.println(command);
	}
}

bool tComm::test_channel(mode_cmd_t cmd_recv) {
	// channel selection
	if (((cmd_recv.mode == OP_M1) or (cmd_recv.mode == OP_M3))
			and (cmd_recv.channel > (1 << (HALF_BRIDGES)) - 1)) {
		// display a message error
		serial.println("\n[ERR] Half-Brdige selection");
		return false;
	} else if (((cmd_recv.mode == OP_M2) or (cmd_recv.mode == OP_M4))
			and (cmd_recv.channel > (1 << (HALF_BRIDGES / 2)) - 1)) {
		// display a message error
		serial.println("\n[ERR] Full-Brdige selection");
		return false;
	} else {
		return true;
	}
}

bool tComm::test_values(mode_cmd_t cmd_recv) {
	// channel selection
	if (test_channel(cmd_recv) == false) {
		return false;
	}
	// frequency
	if (((cmd_recv.frequency < gTHB.MIN_FREQ) and (cmd_recv.frequency > 0))
			or (cmd_recv.frequency > gTHB.MAX_FREQ)) {
		// display a message error
		serial.println("\n[ERR] Frequency selection");
		return false;
	}
	// positive duty
	if (((cmd_recv.pos_duty > 0)
			and (cmd_recv.pos_duty < (float) MIN_DUTY_CYCLE))
			or (cmd_recv.pos_duty > MAX_DUTY_CYCLE)) {
		// display a message error
		serial.println("\n[ERR] PosDuty value");
		return false;
	}
	// positive pulse width
	float pos_pulse_width = ((cmd_recv.pos_duty / cmd_recv.frequency) * 10)
			* 1000;
	if (pos_pulse_width < gTHB.MIN_PULSE) {
		// display a message error
		snprintf(send_buffer, 100, "\n[ERR] Pulse width (%.0f us < %f.0 us)",
				pos_pulse_width, gTHB.MIN_PULSE);
		serial.println(send_buffer);
		return false;
	}
	if ((cmd_recv.mode == OP_M2) or (cmd_recv.mode == OP_M4)) {
		// negative duty cycle
		if ((cmd_recv.neg_duty < (float) MIN_DUTY_CYCLE)
				or (cmd_recv.neg_duty > MAX_DUTY_CYCLE)) {
			// display a message error
			serial.println("\n[ERR] NegDuty");
			return false;
		}
		// negative pulse width
		float neg_pulse_width = ((cmd_recv.neg_duty / cmd_recv.frequency) * 10)
				* 1000;
		if (neg_pulse_width < gTHB.MIN_PULSE) {
			// display a message error
			snprintf(send_buffer, 100,
					"\n[ERR] Pulse width (%.0f us < %f.0 us)", neg_pulse_width,
					gTHB.MIN_PULSE);
			serial.println(send_buffer);
			return false;
		}
		// test everything is correct
		if (cmd_recv.pos_duty + cmd_recv.neg_duty > 100) {
			// display a message error
			serial.println("\n[ERR] duty sum");
			return false;
		}
		// pulse phase ( phase between 2 hb signals)
		if ((cmd_recv.pulse_phase < (float) MIN_PHASE)
				or (cmd_recv.pulse_phase > MAX_PHASE)) {
			// display a message error
			serial.println("\n[ERR] pulse phase value");
			return false;
		}
		// test everything is correct
		if (((100 / cmd_recv.pos_duty) < (360 / cmd_recv.pulse_phase))
				or ((cmd_recv.pulse_phase / 360) + (cmd_recv.neg_duty / 100) > 1)) {
			// display a message error
			serial.println("\n[ERR] pulse phase value 2");
			return false;
		}
	}
	if ((cmd_recv.mode == OP_M3) or (cmd_recv.mode == OP_M4)) {
		if ((cmd_recv.phase_shift < (float) MIN_PHASE)
				or (cmd_recv.phase_shift > MAX_PHASE)) {
			// display a message error
			serial.println("\n[ERR] FB Phase value");
			return false;
		}
	}
	return true;
}
