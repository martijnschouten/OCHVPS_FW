/*
 * @project EPFL-HXL_PS_v1.1
 * @file    tDCDC.h
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

#ifndef TDCDC_H_
#define TDCDC_H_

//Personal Includes
#include "source/globals.h"

class tDCDC {
public:
	tDCDC();
	void setup();
	void run();

	float MAX_HV;
	float MIN_HV;

	float hv_set;
	float lv_set;
	
	//------------------------------------------------------
	// monitoring
	//------------------------------------------------------
	// current correction coefficients
	float C0C = 0;
	float C1C = 1;
	float C2C = 0;
	// high voltage correction coefficients
	float C0H = 0;
	float C1H = 1;
	float C2H = 0;
	// low voltage correction coefficients
	float C0L = 0;
	float C1L = 1;
	float C2L = 0;

	//------------------------------------------------------
	// PID
	//------------------------------------------------------
	pid_prm_t HVPS_PID_PRM;

	//------------------------------------------------------
	// getters
	//------------------------------------------------------
	/* get current monitor (HB) (not corrected)*/
	float get_hb_cm_chx(uint32_t channel);

	/* get current monitor (DCDC) (not corrected)*/
	float get_ps_hv_cm();
	/* get current monitor (DCDC) (corrected)*/
	double get_ps_hv_cm_corrected();

	/* get high voltage monitor (DCDC) */
	float get_ps_hv_vm();
	/* get high voltage monitor (DCDCC) (corrected)*/
	double get_ps_hv_vm_corrected();

	/* get low voltage monitor (Buck-Boost Converter) */
	float get_ps_lv_vm();
	/* get low voltage monitor (Buck-Boost Converter) (corrected)*/
	double get_ps_lv_vm_corrected();

	/* */
	float get_PidLastOutput();
	/* */
	float get_PidLastError();

	//------------------------------------------------------
	// setters
	//------------------------------------------------------
	/* set PID */
	void set_PID();
	/* set high voltage */
	void set_ps_hv(float voltage);
	/* set low voltage */
	void set_ps_lv(float voltage);

private:
	static uint32_t numberOfInstances;
	const uint32_t loopDelayVal_ms = 1;

	bool PS_SW_STATE; // high voltage switch state
	float v_mes[2];   // voltage measurements:
	//v_mes[0] = low voltage measurement (buck-boost converter)
	//v_mes[1] = high voltage measurement (DCDC)
	float i_mes[8];   // current measurements:
	//i_mes[0] = current measurement at the output of DCDC
	//i_mes[1-8] = current measurement at the low side of half-bridges

	//------------------------------------------------------
	// monitoring
	//------------------------------------------------------
	// high voltage voltage divider
	static constexpr float hv_2kV_conv = (2501 * 3.3 / 4095.0);
																		 
	// low voltage voltage divider
	static constexpr float lv_12v_conv = (4.01 * 3.3 / 4095.0);

	static constexpr float hv_cm_conv = 2 * (1e6 / (41 * 10)) * (3.3 / 65535);
	static constexpr float hb_cm_conv = (1e6 / (200 * 3.1)) * (3.3 / 4095);

	double second_order_correction(double in, double C0, double C1, double C2);

	//------------------------------------------------------
	// PID
	//------------------------------------------------------
	double input, output, Vset; //3 parameters for PID regulator
	PID HVPS_PID;
	float lv_val_to_set;

};

#endif /* TDCDC_H_ */
