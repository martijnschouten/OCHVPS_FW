/*
 * @project EPFL-HXL_PS_v1.1
 * @file    mEEPROM.h
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
 
#ifndef MEEPROM_H_
#define MEEPROM_H_

//Personal Includes
#include "source/globals.h"

class mEEPROM {
public:
	mEEPROM();
	static void setup();
	// EEPROM config
	static void conf(const char *name, float hv_max);
	// EEPROM initialize
	static void init();
	// EEPROM read/write function
	static float read_float(uint8_t addr);
	static bool write_float(uint8_t addr, float data);

	static void read_string(uint8_t addr, uint8_t lgth, char *data);
	static void write_string(uint8_t addr, uint8_t lgth, const char *data);

	static uint8_t read_uint8(uint8_t addr);
	static bool write_uint8(uint8_t addr, uint8_t data);

	static uint32_t read_uint32(uint8_t addr);
	static bool write_uint32(uint8_t addr, uint32_t data);

	// EEPROM tests
	static void test_float(uint8_t addr, float data_float);
	static void test_string(uint8_t addr, const char *data_string);
	static void test_uint8();
	static void test_uint32();

	static void I2C_Init(void);
	static void I2C_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
			status_t status, void *userData);
	static bool I2C_Read_Data_uint8_t(I2C_Type *base, uint8_t device_addr,
			uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
	static bool I2C_Write_Data_uint8_t(I2C_Type *base, uint8_t device_addr,
			uint8_t reg_addr, uint8_t *txBuff, uint32_t txSize);

	//------------------------------------------------------
	//EEPROM map
	//------------------------------------------------------
	// Voltage
	static const uint8_t ADDR_MAX_HV = 0;     // (04B) Voltage rating of the board
	static const uint8_t ADDR_MIN_HV = 4;     // (04B) Voltage rating of the board
	// Half-Bridges Frequency
	static const uint8_t ADDR_MAX_FREQ = 8;   // (04B) Maximum switching frequency
	static const uint8_t ADDR_MIN_FREQ = 12;  // (04B) Minimum switching frequency
	// Half-Bridges Pulse
	static const uint8_t ADDR_MIN_PULSE = 16; // (04B) Minimum pulse duration
	// Low Voltage Monitoring correction
	/*calibration factor for the voltage reading to account for resistive bridge imprecision. VoutCorr=C2*1E-6*V^2+C1*V+C0 (double values).*/
	static const uint8_t ADDR_C0L = 20;       // (04B)
	static const uint8_t ADDR_C1L = 24;       // (04B)
	static const uint8_t ADDR_C2L = 28;       // (04B)
	// High Voltage Monitoring correction
	/*calibration factor for the voltage reading to account for resistive bridge imprecision. VoutCorr=C2*1E-6*V^2+C1*V+C0 (double values).*/
	static const uint8_t ADDR_C0H = 32;       // (04B)
	static const uint8_t ADDR_C1H = 36;       // (04B)
	static const uint8_t ADDR_C2H = 40;       // (04B)
	// Current Monitoring correction
	/*calibration factor for the voltage reading to account for resistive bridge imprecision. VoutCorr=C2*1E-6*V^2+C1*V+C0 (double values).*/
	static const uint8_t ADDR_C0C = 44;       // (04B)
	static const uint8_t ADDR_C1C = 48;       // (04B)
	static const uint8_t ADDR_C2C = 52;       // (04B)
	// PID
	static const uint8_t ADDR_PID_STATE = 56; // (01B) PID state
	static const uint8_t ADDR_PID_KP = 60;    // (04B) PID gain term double
	static const uint8_t ADDR_PID_KI = 64;    // (04B) PID integral term double
	static const uint8_t ADDR_PID_KD = 68;    // (04B) PID derivative term double
	// Debug
	static const uint8_t ADDR_DEBUG = 72;     // (01B) Debug status
	// PCB
	static const uint8_t ADDR_CONF = 91;      // (01B) Configuration status
	static const uint8_t ADDR_ADMIN = 92;     // (01B)
	static const uint8_t ADDR_LOCK = 93;      // (01B)
	static const uint8_t ADDR_HW_VER = 96;    // (04B)
	// Board name
	/* store the board name (21 bytes) located at 100 to leave some space to add new stored variables without having to resave the name */
	static const uint8_t ADDR_NAME = 100;     // (21B)

private:
	static uint32_t numberOfInstances;
};

#endif /* MEEPROM_H_ */
