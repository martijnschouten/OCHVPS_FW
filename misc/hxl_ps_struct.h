/*
 * @project EPFL-HXL_PS_v1.1
 * @file    hxl_ps_struct.h
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

#ifndef HXL_PS_STRUCT_H_
#define HXL_PS_STRUCT_H_

union fourbyte {
	uint32_t dword;
	uint16_t word[2];
	uint8_t byte[4];
	float f;
};

enum operationModeEnum {
	OFF = 0, OP_M1 = 1, OP_M2 = 2, OP_M3 = 3, OP_M4 = 4
};

typedef struct pcb {
	bool conf;
	bool admin;
	bool lock;
	float version;
} pcb_t;

typedef struct pid_param {
	bool state;
	float Kp;
	float Ki;
	float Kd;
} pid_prm_t;

typedef struct mode_cmd {
	operationModeEnum mode;
	uint8_t channel;
	float frequency;
	float pos_duty;
	float neg_duty;
	float pulse_phase;
	float phase_shift;
} mode_cmd_t;

typedef struct hb_param {
	uint8_t mode;
	uint32_t on;
	uint32_t off;
	uint32_t cnt;
	float shift;
} hb_param_t;

#endif /* HXL_PS_STRUCT_H_ */
