/*
 * @project EPFL-HXL_PS_v1.1
 * @file    mEEPROM.cpp
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
 * Definitions
 ******************************************************************************/
/* I2C Master parameters (uC)*/
#define I2C_MASTER_BASEADDR I2C0
#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
/* I2C Slave parameters (TPS55288) */
#define I2C_SLAVE_ADDR_7BIT 0X50U
/*I2C communication parameters */
#define I2C_BAUDRATE 100000U

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t mEEPROM::numberOfInstances = 0;

i2c_master_handle_t g_mEEPROM_handle;

volatile bool mEEPROM_completionFlag = false;
volatile bool mEEPROM_nakFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
mEEPROM::mEEPROM() {
	numberOfInstances++;
	if (numberOfInstances > 1) {
		while (true); //should never happen
	}
}

void mEEPROM::setup() {
	// initialize the buckboost converter
	I2C_MasterTransferCreateHandle(I2C_MASTER_BASEADDR, &g_mEEPROM_handle,
			eeprom.I2C_master_callback, NULL);
	I2C_Init();
}

void mEEPROM::conf(const char *name, float hv_max) {
	// initialize EEPROM to remove everything
	init();
	// Voltage
	write_float(eeprom.ADDR_MAX_HV, (float) hv_max);    // V
	float hv_min = 0;
	if (hv_max == 2000){
		hv_min = 350;
	} else if (hv_max == 3000){
		hv_min = 550;
	} else if (hv_max == 4500){
		hv_min = 950;
	}
	write_float(eeprom.ADDR_MIN_HV, hv_min);     // V
	// Half-Bridges Frequency
	write_float(eeprom.ADDR_MAX_FREQ, (float) 1000.0);  // Hz
	write_float(eeprom.ADDR_MIN_FREQ, (float) 0.01);    // Hz
	// Half-Bridges Pulse
	write_float(eeprom.ADDR_MIN_PULSE, (float) 500);    // us
	// Low Voltage Monitoring correction
	write_float(eeprom.ADDR_C0L, (float) 0.0);
	write_float(eeprom.ADDR_C1L, (float) 1.0);
	write_float(eeprom.ADDR_C2L, (float) 0.0);
	// High Voltage Monitoring correction
	write_float(eeprom.ADDR_C0H, (float) 0.0);
	write_float(eeprom.ADDR_C1H, (float) 1.0);
	write_float(eeprom.ADDR_C2H, (float) 0.0);
	// Current Monitoring correction
	write_float(eeprom.ADDR_C0C, (float) 0.0);
	write_float(eeprom.ADDR_C1C, (float) 1.0);
	write_float(eeprom.ADDR_C2C, (float) 0.0);
	// PID
	write_uint8(eeprom.ADDR_PID_STATE, (bool) true);
	write_float(eeprom.ADDR_PID_KP, (float) 0.3);
	write_float(eeprom.ADDR_PID_KI, (float) 10.0);
	write_float(eeprom.ADDR_PID_KD, (float) 0.0);
	// Debug
	write_uint8(eeprom.ADDR_DEBUG, 1);
	// Name
	write_string(eeprom.ADDR_NAME, 21, name);

	// PCB
	write_uint8(eeprom.ADDR_CONF, (bool) true);
	write_uint8(eeprom.ADDR_ADMIN, (bool) false);
	write_uint8(eeprom.ADDR_LOCK, (bool) true);
	write_float(eeprom.ADDR_HW_VER, 1.1);
}

void mEEPROM::init() {
	// can be used to initialize EEPROM
	serial.print("\n[MEM] Initialize... ");
	uint8_t writeBuff = 0;
	uint8_t errors = 0;
	for (uint16_t i = 0; i <= 255; i++) {
		bool test = eeprom.write_uint8(i, writeBuff);
		// if we can write to this address, errors is incremented
		if (test != true) {
			errors += 1;
		}
		// EEPROM module needs some delay to correctly write
		mDelay dly(5);
		dly.waitUntilFinished();
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
}

float mEEPROM::read_float(uint8_t addr) {
	union fourbyte readBuff;

	eeprom.I2C_Read_Data_uint8_t(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT, addr,
			readBuff.byte, 4);

	return readBuff.f;
}

bool mEEPROM::write_float(uint8_t addr, float data) {
	union fourbyte writeBuff;

	writeBuff.f = data;

	bool test = eeprom.I2C_Write_Data_uint8_t(I2C_MASTER_BASEADDR,
			I2C_SLAVE_ADDR_7BIT, addr, writeBuff.byte, 4);

	// EEPROM module needs some delay to correctly write
	mDelay dly(5);
	dly.waitUntilFinished();

	return test;
}

void mEEPROM::read_string(uint8_t addr, uint8_t lgth, char *data) {
	if (eeprom.read_uint8(addr) == addr) { //If string has not yet been defined in EEPROM
		data[0] = '\0';
		return;
	} else {
		for (uint8_t i = 0; i < lgth; i++) {
			data[i] = eeprom.read_uint8(addr + i);
			if (data[i] == '\0') {
				break;
				return;
			}
		}
		return;
	}
}

void mEEPROM::write_string(uint8_t addr, uint8_t lgth, const char *data) {
	for (uint8_t i = 0; i < lgth; i++) {
		eeprom.write_uint8(addr + i, data[i]);
		if (data[i] == '\0') {
			break;
		}
	}
	// EEPROM module needs some delay to correctly write
	mDelay dly(5);
	dly.waitUntilFinished();
}

uint8_t mEEPROM::read_uint8(uint8_t addr) {
	uint8_t readAddr = addr;
	uint8_t readBuff = 0;
	eeprom.I2C_Read_Data_uint8_t(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
			readAddr, &readBuff, 1);
	return readBuff;
}

bool mEEPROM::write_uint8(uint8_t addr, uint8_t data) {
	uint8_t writeAddr = addr;
	uint8_t writeBuff = data;
	bool test = eeprom.I2C_Write_Data_uint8_t(I2C_MASTER_BASEADDR,
			I2C_SLAVE_ADDR_7BIT, writeAddr, &writeBuff, 1);

	// EEPROM module needs some delay to correctly write
	mDelay dly(5);
	dly.waitUntilFinished();

	return test;
}

uint32_t mEEPROM::read_uint32(uint8_t addr) {
	uint8_t readAddr = 0;
	union fourbyte readBuff;

	eeprom.I2C_Read_Data_uint8_t(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
			readAddr, readBuff.byte, 4);

	return readBuff.dword;
}

bool mEEPROM::write_uint32(uint8_t addr, uint32_t data) {
	uint8_t writeAddr = addr;
	union fourbyte writeBuff;

	writeBuff.dword = data;

	bool test = eeprom.I2C_Write_Data_uint8_t(I2C_MASTER_BASEADDR,
			I2C_SLAVE_ADDR_7BIT, writeAddr, writeBuff.byte, 4);

	// EEPROM module needs some delay to correctly write
	mDelay dly(5);
	dly.waitUntilFinished();

	return test;
}

void mEEPROM::test_float(uint8_t addr, float data_float) {
	serial.print("[MEM] Write [");
	serial.print(addr);
	serial.print("]=");
	serial.print(data_float);
	serial.print("... ");

	bool write_test = eeprom.write_float(addr, data_float);
	if (write_test) {
		serial.println("OK");
	} else {
		serial.println("fail");
	}

	float read_data_float = eeprom.read_float(addr);
	serial.print("[MEM] Read  [");
	serial.print(addr);
	serial.print("]=");
	serial.println(read_data_float);
}

void mEEPROM::test_string(uint8_t addr, const char *data_string) {
	serial.print("[MEM] Write [");
	serial.print(addr);
	serial.print("]=");
	serial.println(data_string);

	uint8_t length = 21;
	eeprom.write_string(addr, length, data_string);

	char read_data_string[length];
	eeprom.read_string(addr, length, read_data_string);
	serial.print("[MEM] Read  [");
	serial.print(addr);
	serial.print("]=");
	serial.println(read_data_string);
}

void mEEPROM::test_uint8() {
	uint8_t writeBuff = 0;
	uint8_t readBuff = 0;
	uint8_t errors = 0;

	//------------------------------------------------------------------------------------------------------------
	serial.print("[MEM] Write [i]=  0... ");

	for (uint16_t i = 0; i <= 255; i++) {
		bool test = eeprom.I2C_Write_Data_uint8_t(I2C_MASTER_BASEADDR,
				I2C_SLAVE_ADDR_7BIT, i, &writeBuff, 1);	// if we can write to this address, errors is incremented
		if (test != true) {
			errors += 1;
		}

		// EEPROM module needs some delay to correctly write
		mDelay dly(5);
		dly.waitUntilFinished();
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
	//------------------------------------------------------------------------------------------------------------
	serial.print("[MEM] Read  [i]=  0... ");
	errors = 0;

	for (uint16_t i = 0; i <= 255; i++) {
		eeprom.I2C_Read_Data_uint8_t(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
				i, &readBuff, 1);// if we can write to this address, errors is incremented
		if (readBuff != 0) {
			errors += 1;
		}
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
	//------------------------------------------------------------------------------------------------------------
	serial.print("[MEM] Write [i]=255... ");
	writeBuff = 255;
	errors = 0;

	for (uint16_t i = 0; i <= 255; i++) {
		bool test = eeprom.I2C_Write_Data_uint8_t(I2C_MASTER_BASEADDR,
				I2C_SLAVE_ADDR_7BIT, i, &writeBuff, 1);	// if we can write to this address, errors is incremented
		if (test != true) {
			errors += 1;
		}

		// EEPROM module needs some delay to correctly write
		mDelay dly(5);
		dly.waitUntilFinished();
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
	//------------------------------------------------------------------------------------------------------------
	serial.print("[MEM] Read  [i]=255... ");
	errors = 0;

	for (uint16_t i = 0; i <= 255; i++) {
		eeprom.I2C_Read_Data_uint8_t(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
				i, &readBuff, 1);// if we can write to this address, errors is incremented
		if (readBuff != 255) {
			errors += 1;
		}
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
	//------------------------------------------------------------------------------------------------------------
	serial.print("[MEM] Write [i]=  i... ");
	errors = 0;

	for (uint16_t i = 0; i <= 255; i++) {
		writeBuff = i;
		bool test = eeprom.I2C_Write_Data_uint8_t(I2C_MASTER_BASEADDR,
				I2C_SLAVE_ADDR_7BIT, i, &writeBuff, 1);	// if we can write to this address, errors is incremented
		if (test != true) {
			errors += 1;
		}

		// EEPROM module needs some delay to correctly write
		mDelay dly(5);
		dly.waitUntilFinished();
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
	//------------------------------------------------------------------------------------------------------------
	serial.print("[MEM] Read  [i]=  i... ");
	errors = 0;

	for (uint16_t i = 0; i <= 255; i++) {
		eeprom.I2C_Read_Data_uint8_t(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
				i, &readBuff, 1);// if we can write to this address, errors is incremented
		if (readBuff != i) {
			errors += 1;
		}
	}
	// check if any error happened
	if (errors != 0) {
		serial.print(" fail (");
		serial.print(errors);
		serial.println(" errors)");
	} else {
		serial.println("OK");
	}
}

void mEEPROM::test_uint32() {
	uint8_t addr = 0;
	uint32_t write_data_uint32 = 4027518960;
	bool write_test = eeprom.write_uint32(addr, write_data_uint32);
	if (write_test) {
		serial.print("\nWrite uint32: OK");
	} else {
		serial.print("\nWrite uint32: fail");
	}

	uint32_t read_data_uint32 = eeprom.read_uint32(addr);
	if (read_data_uint32 == write_data_uint32) {
		serial.print("\nRead uint32: OK");
	} else {
		serial.print("\nRead uint32: fail");
	}
}

/*******************************************************************************
 ******************************************************************************/
void mEEPROM::I2C_Init(void) {
	/*
	 How to read the device who_am_I value ?
	 Start + Device_address_Write , who_am_I_register;
	 Repeart_Start + Device_address_Read , who_am_I_value.
	 */

	uint32_t sourceClock = 0;

	i2c_master_config_t masterConfig;

	/*
	 * masterConfig.baudRate_Bps = 100000U;
	 * masterConfig.enableStopHold = false;
	 * masterConfig.glitchFilterWidth = 0U;
	 * masterConfig.enableMaster = true;
	 */
	I2C_MasterGetDefaultConfig(&masterConfig);

	masterConfig.baudRate_Bps = I2C_BAUDRATE;

	sourceClock = I2C_MASTER_CLK_SRC;

	I2C_MasterInit(I2C_MASTER_BASEADDR, &masterConfig, sourceClock);
}

void mEEPROM::I2C_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData) {
	/* Signal transfer success when received success status. */
	if (status == kStatus_Success) {
		mEEPROM_completionFlag = true;
	}
	/* Signal transfer success when received success status. */
	if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak)) {
		mEEPROM_nakFlag = true;
	}
}

bool mEEPROM::I2C_Read_Data_uint8_t(I2C_Type *base, uint8_t device_addr,
		uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize) {
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = rxBuff;
	masterXfer.dataSize = rxSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*  direction=write : start+device_write;cmdbuff;xBuff; */
	/*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

	I2C_MasterTransferNonBlocking(I2C_MASTER_BASEADDR, &g_mEEPROM_handle,
			&masterXfer);

	/*  wait for transfer completed. */
	while ((!mEEPROM_nakFlag) && (!mEEPROM_completionFlag)) {
	}

	mEEPROM_nakFlag = false;

	if (mEEPROM_completionFlag == true) {
		mEEPROM_completionFlag = false;
		return true;
	} else {
		return false;
	}
}

bool mEEPROM::I2C_Write_Data_uint8_t(I2C_Type *base, uint8_t device_addr,
		uint8_t reg_addr, uint8_t *txBuff, uint32_t txSize) {
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = txBuff;
	masterXfer.dataSize = txSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/* direction = write:   start+device_write; cmdbuff; xBuff; */
	/* direction = receive: start+device_write; cmdbuff; repeatStart+device_read;xBuff; */

	I2C_MasterTransferNonBlocking(I2C_MASTER_BASEADDR, &g_mEEPROM_handle,
			&masterXfer);

	/*  wait for transfer completed. */
	while ((!mEEPROM_nakFlag) && (!mEEPROM_completionFlag)) {
	}

	mEEPROM_nakFlag = false;

	if (mEEPROM_completionFlag == true) {
		mEEPROM_completionFlag = false;
		return true;
	} else {
		return false;
	}

	return true;
}
