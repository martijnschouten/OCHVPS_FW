/*
 * @project EPFL-HXL_PS_v1.0
 * @file    mBuckBoost.h
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

#ifndef MBUCKBOOST_H_
#define MBUCKBOOST_H_

//Personal Includes
#include "source/globals.h"

#define MAX_LV 12
#define MIN_LV 0.7

enum stateMachineEnum {
	off, start, on, stop
};

class mBuckBoost {
public:
	static void setup();
	static void run();
	static void set_output(float voltage);

private:
	static void I2C_Init(void);
	static void I2C_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
			status_t status, void *userData);
	static bool I2C_Read_8Addr_8Data(I2C_Type *base, uint8_t device_addr,
			uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
	static bool I2C_Write_8Addr_8Data(I2C_Type *base, uint8_t device_addr,
			uint8_t reg_addr, uint8_t *txBuff, uint32_t txSize);
	static void loadSettings();
};

#endif /* MBUCKBOOST_H_ */
