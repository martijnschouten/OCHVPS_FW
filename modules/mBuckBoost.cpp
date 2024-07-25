/*
 * @project EPFL-HXL_PS_v1.0
 * @file    mBuckBoost.cpp
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
#include "source/globals.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C Master parameters (uC)*/
#define I2C_MASTER_BASEADDR I2C1
#define I2C_MASTER_CLK_SRC I2C1_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)
/* I2C Slave parameters (TPS55288) */
#define I2C_SLAVE_ADDR_7BIT 0x74U
/*I2C communication parameters */
#define I2C_BAUDRATE 100000U

/*******************************************************************************
 * Variables
 ******************************************************************************/
i2c_master_handle_t g_mBuckBoost_handle;

volatile bool mBuckBoost_completionFlag = false;
volatile bool mBuckBoost_nakFlag = false;

static uint16_t Vref = 0;
static float last_lv_set = 0;
static bool PS_SW_STATE = false;

// state machine enum + var
stateMachineEnum stateMachine_mBuckBoost;

/*******************************************************************************
 * Code
 ******************************************************************************/
void mBuckBoost::I2C_Init(void) {
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

void mBuckBoost::I2C_master_callback(I2C_Type *base,
		i2c_master_handle_t *handle, status_t status, void *userData) {
	/* Signal transfer success when received success status. */
	if (status == kStatus_Success) {
		mBuckBoost_completionFlag = true;
	}
	/* Signal transfer success when received success status. */
	if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak)) {
		mBuckBoost_nakFlag = true;
	}
}

bool mBuckBoost::I2C_Read_8Addr_8Data(I2C_Type *base, uint8_t device_addr,
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

	I2C_MasterTransferNonBlocking(I2C_MASTER_BASEADDR, &g_mBuckBoost_handle,
			&masterXfer);

	/*  wait for transfer completed. */
	while ((!mBuckBoost_nakFlag) && (!mBuckBoost_completionFlag)) {
	}

	mBuckBoost_nakFlag = false;

	if (mBuckBoost_completionFlag == true) {
		mBuckBoost_completionFlag = false;
		return true;
	} else {
		return false;
	}
}

bool mBuckBoost::I2C_Write_8Addr_8Data(I2C_Type *base, uint8_t device_addr,
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

	I2C_MasterTransferNonBlocking(I2C_MASTER_BASEADDR, &g_mBuckBoost_handle,
			&masterXfer);

	/*  wait for transfer completed. */
	while ((!mBuckBoost_nakFlag) && (!mBuckBoost_completionFlag)) {
	}

	mBuckBoost_nakFlag = false;

	if (mBuckBoost_completionFlag == true) {
		mBuckBoost_completionFlag = false;
		return true;
	} else {
		return false;
	}
}

void mBuckBoost::loadSettings() {
	uint8_t slave_reg_data[8];

	// set REF_LSB Register    (Address = 0h)    [reset = 11010010h]    (page 27)
	//0b [1   1   0   1   0   0   1   0]    VREF                    1001 1010b = 282mV-mV reference voltage
	//0b  1   1   0   1   0   0   1   0     AKA 0xD2
	slave_reg_data[0] = (Vref & 0xFF);

	// set REF_MSB Register    (Address = 1h)    [reset = 00000000h]    (page 27)
	//0b [0   0   0   0   0   0] [X] [X]    RESERVED                 00 0000b
	//0b  0   0   0   0   0   0  [0   0]    VREF                     00b = 282m-V reference voltage
	//0b  0   0   0   0   0   0   0   0     AKA 0x00
	slave_reg_data[1] = (Vref & 0x0F00) >> 8;

	// set IOUT_LIMIT Register (Address = 2h)    [reset = 11100100h]    (page 28)
	//0b [1] [X] [X] [X] [X] [X] [X] [X]    Current_Limit_EN         1b = Current limit enabled (Default)
	//0b  1  [0   0   1   1   0   0   1]    Current_Limit_Setting    001 1001b = VISP-VISN = 12.5 (mV)
	//0b  1   0   0   1   1   0   0   1     AKA 0x99
	slave_reg_data[2] = 0x99;

	// set VOUT_SR Register    (Address = 3h)    [reset = 00000001h]    (page 29)
	//0b [0]  0] [X] [X] [X] [X] [X] [X]    RESERVED                 00b
	//0b  0   0  [0]  0] [X] [X] [X] [X]    OCP_DELAY                00b = 128 μs (Default)
	//0b  0   0   0   0  [0]  0] [X] [X]    RESERVED                 00b
	//0b  0   0   0   0   0   0  [0   1]    SR                       01b = 2.5 mV/μs output change slew rate (Default)
	//0b  0   0   0   0   0   0   0   1     AKA 0x01
	slave_reg_data[3] = 0x01;

	// set VOUT_FS Register    (Address = 4h)    [reset = 00000011h]    (page 30)
	//0b [0] [X] [X] [X] [X] [X] [X] [X]    FB                       0b = Use internal output voltage feedback.
	//0b  0  [0   0   0   0   0] [X] [X]    RESERVED                 0 0000b
	//0b  0   0   0   0   0   0  [1   1]    INTFB                    11b = Set internal feedback ratio to 0.0564(Default)
	//0b  0   0   0   0   0   0   1   1     AKA 0x02
	slave_reg_data[4] = 0x02;

	// set CDC Register        (Address = 5h)    [reset = 11100000h]    (page 31)
	//0b [1] [X] [X] [X] [X] [X] [X] [X]    SC_MASK                  1b = Enable SC indication (Default)
	//0b  1  [1] [X] [X] [X] [X] [X] [X]    OCP_MASK                 1b = Enable OCP indication (Default)
	//0b  1   1  [1] [X] [X] [X] [X] [X]    OVP_MASK                 1b = Enable OVP indication (Default)
	//0b  1   1   1  [0] [X] [X] [X] [X]    RESERVED                 0b
	//0b  1   1   1   0  [0] [X] [X] [X]    CDC_OPTION               0b = Internal CDC compensation by the register 05H (Default)
	//0b  1   1   1   0   0  [0   0   0]    CDC                      000b = 0-V output voltage rise with 50 mV at VISP - VISN (Default)
	//0b  1   1   1   0   0   0   0   0     AKA 0xE0
	slave_reg_data[5] = 0xE0;

	// set MODE Register       (Address = 6h)    [reset = 00100000h]    (page 32)
	//0b [1] [X] [X] [X] [X] [X] [X] [X]    OE                       1b = Output enable
	//0b  1  [0] [X] [X] [X] [X] [X] [X]    FSWDBL                   0b = Keep the switching frequency unchanged during buck-boost mode (Default)
	//0b  1   0  [1] [X] [X] [X] [X] [X]    HICCUP                   1b = Enable the hiccup during output short circuit protection (Default)
	//0b  1   0   1  [0] [X] [X] [X] [X]    DISCHG                   0b = Disabled VOUT discharge when the device is in shutdown mode (Default)
	//0b  1   0   1   0  [0] [X] [X] [X]    VCC                      0b = Select internal LDO for VCC (Default)
	//0b  1   0   1   0   0  [0] [X] [X]    I2CADD                   0b = Set I2C slave address to 74h (Default)
	//0b  1   0   1   0   0   0  [0] [X]    PFM                      0b = PFM operating mode at light load condition (Default)
	//0b  1   0   1   0   0   0   0  [0]    MODE                     0b = Set VCC, I2CADD, and PFM controlled by external resistor (Default)
	//0b  1   0   1   0   0   0   0   0     AKA 0xA0
	slave_reg_data[6] = 0xA0;

	// set STATUS Register     (Address = 7h)    [reset = 00000011h]    (page 33)
	//0b [0] [X] [X] [X] [X] [X] [X] [X]    SCP                      0b = No short circuit
	//0b  0  [0] [X] [X] [X] [X] [X] [X]    OCP                      0b = No output overcurrent
	//0b  0   0  [0] [X] [X] [X] [X] [X]    OVP                      0b = No OVP
	//0b  0   0   0  [0] [X] [X] [X] [X]    RESERVED                 0b
	//0b  0   0   0   0  [0] [X] [X] [X]    RESERVED                 0b
	//0b  0   0   0   0   0  [0] [X] [X]    RESERVED                 0b
	//0b  0   0   0   0   0   0  [1   1]    STATUS                   11b = Reserved
	//0b  0   0   0   0   0   0   0   0     AKA 0x03
	slave_reg_data[7] = 0x03;

	uint8_t write_reg = 0;
	uint8_t writeBuff[8];

	for (int i = 0; i < 8; i++) {
		writeBuff[i] = slave_reg_data[i];
	}

	mBuckBoost::I2C_Write_8Addr_8Data(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
			write_reg, writeBuff, 8);
}

void mBuckBoost::setup() {
	PS_SW_STATE = false;
	// Buck-boost converter = OFF state
	GPIO_WritePinOutput(PS_12V_EN_GPIO, PS_12V_EN_PIN, 0);
	// HV DCDC = OFF state
	GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 1);
	last_lv_set = 0;
	Vref = 0;
	stateMachine_mBuckBoost = off;
	return;
}

void mBuckBoost::run() {
	// Read the HV switch state
	PS_SW_STATE = GPIO_ReadPinInput(PS_SW_STATE_GPIO, PS_SW_STATE_PIN);

	// State machine
	switch (stateMachine_mBuckBoost) {
	case off: {
		// Buck-boost converter = OFF state
		GPIO_WritePinOutput(PS_12V_EN_GPIO, PS_12V_EN_PIN, 0);
		// HV DCDC = OFF state
		GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 1);
		if (PS_SW_STATE == true) {
			// Buck-boost converter = ON state
			GPIO_WritePinOutput(PS_12V_EN_GPIO, PS_12V_EN_PIN, 1);
			// initialize the buckboost converter
			I2C_MasterTransferCreateHandle(I2C_MASTER_BASEADDR, &g_mBuckBoost_handle,
					mBuckBoost::I2C_master_callback, NULL);
			mBuckBoost::I2C_Init();
			stateMachine_mBuckBoost = start;
		}
		break;
	}
	case start: {
		// Buck-boost converter = ON state
		GPIO_WritePinOutput(PS_12V_EN_GPIO, PS_12V_EN_PIN, 1);
		// HV DCDC = OFF state
		GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 1);

		mBuckBoost::loadSettings();
		stateMachine_mBuckBoost = on;

		break;
	}
	case on: {
		// Buck-boost converter = ON state
		GPIO_WritePinOutput(PS_12V_EN_GPIO, PS_12V_EN_PIN, 1);
		if (PS_SW_STATE == true) {
			if (last_lv_set > 0.7) {
				// HV DCDC = ON state
				GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 0);
			} else {
				// HV DCDC = OFF state
				GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 1);
			}
			mBuckBoost::loadSettings();
		} else {
			// HV DCDC = OFF state
			GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 1);
			stateMachine_mBuckBoost = stop;
		}
		break;
	}
	case stop: {
		uint8_t write_reg = 0X06;
		uint8_t writeBuff = 0X20;
		I2C_Write_8Addr_8Data(I2C_MASTER_BASEADDR, I2C_SLAVE_ADDR_7BIT,
				write_reg, &writeBuff, 1);

		// Buck-boost converter = OFF state
		GPIO_WritePinOutput(PS_12V_EN_GPIO, PS_12V_EN_PIN, 0);

		// HV DCDC = OFF state
		GPIO_WritePinOutput(PS_HV_EN_GPIO, PS_HV_EN_PIN, 1);
		stateMachine_mBuckBoost = off;
		break;
	}

	default: {
		// Should never happen, reset stateMachine machine
		stateMachine_mBuckBoost = off;
		break;
	}
	}
	return;
}

void mBuckBoost::set_output(float voltage) {
	// save last voltage set by the user
	if (voltage <= 0.7) {
		last_lv_set = 0;
		Vref = 0;
	} else if ((voltage > 0.7) && (voltage < 12)) {
		last_lv_set = voltage;
		Vref = (((last_lv_set - 0.6) * 760) / 11.4);
	} else if (voltage > 12) {
		last_lv_set = 12;
		Vref = (((last_lv_set - 0.6) * 760) / 11.4);
	}
}
