/*
 * @project EPFL-HXL_PS_v1.0
 * @file    tDCDC.cpp
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
 * Variables
 ******************************************************************************/
uint32_t tDCDC::numberOfInstances = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
tDCDC::tDCDC() :
		HVPS_PID(&input, &output, &Vset, 0.02, 0.0, 0.0, DIRECT_PID) {
	numberOfInstances++;
	if (numberOfInstances > 1) {
		while (true)
			; //should never happen
	}
}

void tDCDC::setup() {
	// initialize task by checking values from EEPROM
	// voltage
	MAX_HV = eeprom.read_float(eeprom.ADDR_MAX_HV);
	MIN_HV = eeprom.read_float(eeprom.ADDR_MIN_HV);
	// current correction coefficients
	C0C = eeprom.read_float(eeprom.ADDR_C0C);
	C1C = eeprom.read_float(eeprom.ADDR_C1C);
	C2C = eeprom.read_float(eeprom.ADDR_C2C);
	// high voltage correction coefficients
	C0H = eeprom.read_float(eeprom.ADDR_C0H);
	C1H = eeprom.read_float(eeprom.ADDR_C1H);
	C2H = eeprom.read_float(eeprom.ADDR_C2H);
	// low voltage correction coefficients
	C0L = eeprom.read_float(eeprom.ADDR_C0L);
	C1L = eeprom.read_float(eeprom.ADDR_C1L);
	C2L = eeprom.read_float(eeprom.ADDR_C2L);
	// PID (state + coefficients)
	HVPS_PID_PRM.state = eeprom.read_uint8(eeprom.ADDR_PID_STATE);
	HVPS_PID_PRM.Kp = eeprom.read_float(eeprom.ADDR_PID_KP);
	HVPS_PID_PRM.Ki = eeprom.read_float(eeprom.ADDR_PID_KI);
	HVPS_PID_PRM.Kd = eeprom.read_float(eeprom.ADDR_PID_KD);
	// initialize Buck-Boost Converter module
	mBuckBoost::setup();
	// PID settings
	HVPS_PID.SetOutputLimits(0, 4095); // set the PID output to a 10bit value
	HVPS_PID.SetSampleTime(5);         // PID output is updated every 5ms
	HVPS_PID.SetTunings(HVPS_PID_PRM.Kp, HVPS_PID_PRM.Ki, HVPS_PID_PRM.Kd); // set the regulators parameters with data read from EEPROM
	HVPS_PID.SetMode(AUTOMATIC);
	HVPS_PID.SetControllerDirection(DIRECT_PID);
}

void tDCDC::run() {
	//dbg("new iteration of tDCDC::run()!");

	static mDelay loopDelay(loopDelayVal_ms);

	if (loopDelay.isDelayDone()) {
		loopDelay.restart(loopDelayVal_ms);
		// Read the HV switch state
		PS_SW_STATE = GPIO_ReadPinInput(PS_SW_STATE_GPIO, PS_SW_STATE_PIN);

		// HV switch = ON and PID = OFF
		if ((PS_SW_STATE == true) and (HVPS_PID_PRM.state == false)) {
			mBuckBoost::set_output(lv_set);
			// HV switch = ON and PID = ON
		} else if ((PS_SW_STATE == true) and (HVPS_PID_PRM.state == true)) {
			if (hv_set > 0) {
				Vset = hv_set;
				input = get_ps_hv_vm(); // Tell PID what was the measured voltage
				if (HVPS_PID.Compute()) {
					lv_val_to_set = 12 * output / 4095;
				}
			} else {
				Vset = 0;
				input = get_ps_hv_vm(); // Tell PID what was the measured voltage
				if (HVPS_PID.Compute()) {
					lv_val_to_set = 12 * output / 4095;
				}
				lv_val_to_set = 0;
			}
			mBuckBoost::set_output(lv_val_to_set);
			// HV switch = OFF and PID = OFF
		} else if ((PS_SW_STATE == false) and (HVPS_PID_PRM.state == true)) {
			Vset = 0;
			input = get_ps_hv_vm(); // Tell PID what was the measured voltage
			if (HVPS_PID.Compute()) {
				lv_val_to_set = 12 * output / 4095;
			}
			lv_val_to_set = 0;
		}

		mBuckBoost::run();
	}
}

//------------------------------------------------------
// getters
//------------------------------------------------------
/* get current monitor (HB) (not corrected)*/
float tDCDC::get_hb_cm_chx(uint32_t channel) {
	i_mes[channel] = hb_cm_conv * (float) (mADC::adc1_se_run(channel + 7));
	if (i_mes[channel] < 3) {
		i_mes[channel] = 0;
	}
	return i_mes[channel];
}

/* get current monitor (DCDC) (not corrected)*/
float tDCDC::get_ps_hv_cm() {
	uint16_t adc_result = mADC::adc0_df_run(0) - 90;
	if (adc_result > 36000) {
		i_mes[0] = hv_cm_conv * (65535 - adc_result);
	} else {
		i_mes[0] = hv_cm_conv * adc_result;
	}
	return i_mes[0];
}

/* get high voltage monitor (DCDC) (corrected)*/
double tDCDC::get_ps_hv_cm_corrected() {
	return second_order_correction(tDCDC::get_ps_hv_cm(), C0C, C1C, C2C);
}

/* get high voltage monitor (DCDC) (not corrected)*/
float tDCDC::get_ps_hv_vm() {
	if ((MAX_HV == 2000) or (MAX_HV == 3000)) {
		v_mes[1] = hv_2kV_conv * ((float) mADC::adc0_se_run(18));
	} else if (MAX_HV == 4500) {
		v_mes[1] = hv_6kV_conv * ((float) mADC::adc0_se_run(18));
	}
	return v_mes[1];
}

/* get high voltage monitor (DCDC) (corrected)*/
double tDCDC::get_ps_hv_vm_corrected() {
	return second_order_correction(tDCDC::get_ps_hv_vm(), C0H, C1H, C2H);
}

/* get low voltage monitor (Buck-Boost Converter) (not corrected)*/
float tDCDC::get_ps_lv_vm() {
	v_mes[0] = lv_12v_conv * (float) (mADC::adc0_se_run(17));
	if (v_mes[0] < 0.7) {
		v_mes[0] = 0;
	}
	return v_mes[0];
}

/* get low voltage monitor (Buck-Boost Converter) (corrected)*/
double tDCDC::get_ps_lv_vm_corrected() {
	return second_order_correction(tDCDC::get_ps_lv_vm(), C0L, C1L, C2L);
}

/* */
float tDCDC::get_PidLastOutput() {
	return (float) output;
}

/* */
float tDCDC::get_PidLastError() {
	return (float) hv_set - get_ps_hv_vm_corrected();
}

//---------------------------------
// setters
//---------------------------------
/* */
void tDCDC::set_PID() {
	HVPS_PID.SetTunings(HVPS_PID_PRM.Kp, HVPS_PID_PRM.Ki, HVPS_PID_PRM.Kd);
}

/* set high voltage */
void tDCDC::set_ps_hv(float hv_value) {
	float lv_value = 0;
	if (MAX_HV == 2000) {
		lv_value = (hv_value - 148.84) / 239.92;	// 2kV
	} else if (MAX_HV == 3000) {
		lv_value = (hv_value - 221.87) / 325.41;	// 3kV
	} else if (MAX_HV == 4500) {
		lv_value = (hv_value - 446.23) / 700.12;	// 6kV
	}

	if (lv_value < MIN_LV) {
		// clear
		lv_set = 0;
		hv_set = 0;
	} else {
		hv_set = (float) hv_value;
		lv_set = (float) lv_value;
	}
	Vset = hv_set;
}

/* set low voltage */
void tDCDC::set_ps_lv(float lv_value) {
	if (lv_value < MIN_LV) {
		// low voltage
		lv_set = 0;
		// high voltage
		hv_set = 0;
	} else {
		// low voltage
		lv_set = lv_value;
		// high voltage
		if (MAX_HV == 2000) {
			hv_set = (float) (239.92 * lv_value) + 148.84;	// 2kV
		} else if (MAX_HV == 3000) {
			hv_set = (float) (325.41 * lv_value) + 221.87;  // 3kV
		} else if (MAX_HV == 4500) {
			hv_set = (float) (700.12 * lv_value) + 446.23;  // 6kV
		}
	}
	Vset = hv_set;
}

//------------------------------------------------------
// others
//------------------------------------------------------
double tDCDC::second_order_correction(double in, double C0, double C1,
		double C2) {
	return (C0 + (C1 * in) + (C2 * in * in));
}
