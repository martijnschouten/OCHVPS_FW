/*
 * @project EPFL-HXL_PS_v1.0
 * @file    tHB.cpp
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
/* The Flextimer instance/channel used for board */
#define BOARD_FTM_BASEADDR FTM0
/* Interrupt number and interrupt handler for the FTM instance used */
#define BOARD_FTM_IRQ_NUM FTM0_IRQn
/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk)/4)

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t tHB::numberOfInstances = 0;

ftm_config_t ftmInfo;
hb_param_t ch[8];

/*******************************************************************************
 * Code
 ******************************************************************************/
tHB::tHB() {
	numberOfInstances++;
	if (numberOfInstances > 1) {
		while (true)
			; //should never happen
	}
}

extern "C" {
/* enable high side of halb-bridge (= HV) */
void set_hb_hv(uint8_t hb_channel) {
	// enable high side (HB_HGi pin = 1U/3U/5U/7U/9U/11U/13U/15U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel) + 1, tHB::HB_ON);
	// disable low side (HB_LGi pin = 0U/2U/4U/6U/8U/10U/12U/14U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel), tHB::HB_OFF);
}

/* enable low side of halb-bridge (= GND) */
void set_hb_gnd(uint8_t hb_channel) {
	// disable high side (HB_HGi pin = 1U/3U/5U/7U/9U/11U/13U/15U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel) + 1, tHB::HB_OFF);
	// enable low side (HB_LGi pin = 0U/2U/4U/6U/8U/10U/12U/14U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel), tHB::HB_ON);
}

void FTM0_IRQHandler(void) {
	/* Clear interrupt flag.*/
	FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, kFTM_TimeOverflowFlag);

	for (uint8_t index = 0; index < 8; index++) {
		if (ch[index].mode != 0) {
			if (ch[index].on == ch[index].off) {
				set_hb_hv(index);
			} else {
				if (ch[index].cnt == ch[index].off - 1) {
					set_hb_gnd(index);
					ch[index].cnt++;
				} else if (ch[index].cnt == ch[index].on - 1) {
					set_hb_hv(index);
					ch[index].cnt = 0;
				} else {
					ch[index].cnt++;
				}
			}
		}
	}
}
}

void tHB::setup() {
	// initialize task by checking values from EEPROM
	MAX_FREQ = mEEPROM::read_float(mEEPROM::ADDR_MAX_FREQ);
	MIN_FREQ = mEEPROM::read_float(mEEPROM::ADDR_MIN_FREQ);
	MIN_PULSE = mEEPROM::read_float(mEEPROM::ADDR_MIN_PULSE);

	// initialize FTM module
	FTM_GetDefaultConfig(&ftmInfo);

	/* divide FTM clock by 4 */
	ftmInfo.prescale = kFTM_Prescale_Divide_4;

	/* initialize FTM module */
	FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

	/* set timer period */
	// FTM_SetTimerPeriod(BOARD_FTM_BASEADDR, USEC_TO_COUNT(500U, FTM_SOURCE_CLOCK));
	FTM_SetTimerPeriod(BOARD_FTM_BASEADDR,
			USEC_TO_COUNT(100U, FTM_SOURCE_CLOCK));

	FTM_EnableInterrupts(BOARD_FTM_BASEADDR, kFTM_TimeOverflowInterruptEnable);

	EnableIRQ(BOARD_FTM_IRQ_NUM);

	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);

	for (uint8_t channel = 0; channel < 8; channel++) {
		// reset all counters for half-bridges/full-bridges
		ch[channel].mode = OFF;
		ch[channel].on = 0;
		ch[channel].off = 0;
		ch[channel].cnt = 0;
		// disable all the high sides (HB_HGi pin = 1U/3U/5U/7U/9U/11U/13U/15U)
		GPIO_WritePinOutput(GPIOD, ((2 * channel) + 1), HB_OFF);
		// enable all the low sides (HB_LGi pin = 0U/2U/4U/6U/8U/10U/12U/14U)
		GPIO_WritePinOutput(GPIOD, ((2 * channel)), HB_ON);
	}
}

void tHB::run() {
}

void tHB::setOperationMode(mode_cmd_t cmd_recv) {
	switch (cmd_recv.mode) {
	case OP_M1:

		for (uint8_t index = 0; index < 8; index++) {
			if ((cmd_recv.channel >> index) & 0x1) {
				// reset in case previous mode was mode 3 or mode 4
				if ((ch[index].mode == 3) or (ch[index].mode == 4)) {
					ch[index].mode = 0;
					ch[index].on = 0;
					ch[index].off = 0;
					ch[index].cnt = 0;
				}

				ch[index].mode = OP_M1;

				if ((cmd_recv.frequency > 0) and (cmd_recv.pos_duty < 100)) {
					ch[index].on = (uint32_t) (10000 / cmd_recv.frequency);
					ch[index].off = (uint32_t) (ch[index].on
							* (cmd_recv.pos_duty / 100));
				} else {
					ch[index].on = 0;
					ch[index].off = 0;
				}
				ch[index].cnt = 0;
				set_hb_hv(index);
			}
		}

		break;
	case OP_M2:

		for (uint8_t index = 0; index < 4; index++) {
			if ((cmd_recv.channel >> index) & 0x1) {
				// reset in case previous mode was mode 3 or mode 4
				if ((ch[index].mode == 3) or (ch[index].mode == 4)) {
					ch[index].mode = 0;
					ch[index].on = 0;
					ch[index].off = 0;
					ch[index].cnt = 0;
				}

				// hb(x) = positive pulse
				uint8_t hb_1 = 2 * index;
				ch[hb_1].mode = OP_M2;
				ch[hb_1].on = (uint32_t) (10000 / cmd_recv.frequency);
				ch[hb_1].off = (uint32_t) (ch[hb_1].on
						* (cmd_recv.pos_duty / 100));
				ch[hb_1].cnt = 0;
				set_hb_hv(hb_1);

				// hb(x+1) = negative pulse
				uint8_t hb_2 = (2 * index) + 1;
				ch[hb_2].mode = OP_M2;
				ch[hb_2].on = (uint32_t) (10000 / cmd_recv.frequency);
				ch[hb_2].off = (uint32_t) (ch[hb_2].on
						* (cmd_recv.neg_duty / 100));
				ch[hb_2].cnt = (uint32_t) (ch[hb_2].on
						* (1 - (cmd_recv.pulse_phase / 360)));
				set_hb_gnd(hb_2);
			}
		}

		break;
	case OP_M3:

		// reset everything before to start
		for (uint8_t index = 0; index < 8; index++) {
			ch[index].mode = 0;
			ch[index].on = 0;
			ch[index].off = 0;
			ch[index].cnt = 0;
		}

		// set on, off and cnt values for hb(x)
		for (uint8_t index2 = 0; index2 < 8; index2++) {
			if ((cmd_recv.channel >> index2) & 0x1) {
				ch[index2].mode = OP_M3;
				ch[index2].on = (uint32_t) (10000 / cmd_recv.frequency);
				ch[index2].off = (uint32_t) (ch[index2].on
						* (cmd_recv.pos_duty / 100));

				// calculation to determine where cnt will start
				if (index2 == 0) {
					ch[0].shift = 0;
				} else if (index2 == 1) {
					ch[1].shift = (1 - (cmd_recv.phase_shift / 360));
				} else {
					ch[index2].shift = ch[index2 - 1].shift + ch[1].shift;
				}

				if (ch[index2].shift >= 1) {
					ch[index2].shift = ch[index2].shift - 1;
				}

				ch[index2].cnt = (uint32_t) (ch[index2].on * ch[index2].shift);

				// HV or GND
				if ((ch[index2].cnt >= 0)
						and (ch[index2].cnt < ch[index2].off)) {
					set_hb_hv(index2);
				} else {
					set_hb_gnd(index2);
				}
			}
		}

		break;
	case OP_M4:

		// reset everything before to start
		for (uint8_t channel = 0; channel < 8; channel++) {
			ch[channel].mode = 0;
			ch[channel].on = 0;
			ch[channel].off = 0;
			ch[channel].cnt = 0;
		}

		// set on, off and cnt values for hb(x)
		for (uint8_t index2 = 0; index2 < 4; index2++) {
			if ((cmd_recv.channel >> index2) & 0x1) {
				/* hb(x) = positive pulse */
				uint8_t hb_1 = 2 * index2;
				ch[hb_1].mode = OP_M4;
				ch[hb_1].on = (uint32_t) (10000 / cmd_recv.frequency);
				ch[hb_1].off = (uint32_t) (ch[hb_1].on
						* (cmd_recv.pos_duty / 100));
				// calculation to determine where cnt will start
				if (hb_1 == 0) {
					ch[0].shift = 0;
				} else if (hb_1 == 2) {
					ch[2].shift = (1 - (cmd_recv.phase_shift / 360));
				} else {
					ch[hb_1].shift = ch[hb_1 - 2].shift + ch[2].shift;
				}
				if (ch[hb_1].shift >= 1) {
					ch[hb_1].shift = ch[hb_1].shift - 1;
				}

				ch[hb_1].cnt = (uint32_t) (ch[hb_1].on * ch[hb_1].shift);

				// HV or GND
				if ((ch[hb_1].cnt >= 0) and (ch[hb_1].cnt < ch[hb_1].off)) {
					set_hb_hv(hb_1);
				} else {
					set_hb_gnd(hb_1);
				}

				/* hb(x+1) = negative pulse */
				uint8_t hb_2 = (2 * index2) + 1;
				ch[hb_2].mode = OP_M4;
				ch[hb_2].on = (uint32_t) (10000 / cmd_recv.frequency);
				ch[hb_2].off = (uint32_t) (ch[hb_2].on
						* (cmd_recv.neg_duty / 100));
				// calculation to determine where cnt will start
				ch[hb_2].shift = ch[hb_1].shift
						+ (1 - (cmd_recv.pulse_phase / 360));
				if (ch[hb_2].shift >= 1) {
					ch[hb_2].shift = ch[hb_2].shift - 1;
				}
				ch[hb_2].cnt = (uint32_t) (ch[hb_1].on * ch[hb_2].shift);

				// HV or GND
				if ((ch[hb_2].cnt >= 0) and (ch[hb_2].cnt < ch[hb_2].off)) {
					set_hb_hv(hb_2);
				} else {
					set_hb_gnd(hb_2);
				}
			}
		}

		break;
	default:
		break;
	}
}

void tHB::clearOperationMode(mode_cmd_t cmd_recvl) {
	if (cmd_recvl.channel == 0) {
		// All the channels will be disabled
		for (uint8_t index = 0; index < 8; index++) {
			ch[index].mode = OFF;
			ch[index].on = 0;
			ch[index].off = 0;
			ch[index].cnt = 0;
			set_hb_gnd(index);
		}
	} else {
		if ((cmd_recvl.mode == 1) or (cmd_recvl.mode == 3)) {
			for (uint8_t index = 0; index < 8; index++) {
				if (((cmd_recvl.channel >> index) & 0x1)
						and (ch[index].mode == cmd_recvl.mode)) {
					//HB(x)
					ch[index].mode = OFF;
					ch[index].on = 0;
					ch[index].off = 0;
					ch[index].cnt = 0;
					set_hb_gnd(index);
				}
			}
		} else if ((cmd_recvl.mode == 2) or (cmd_recvl.mode == 4)) {
			for (uint8_t index = 0; index < 4; index++) {
				if (((cmd_recvl.channel >> index) & 0x1)
						and (ch[index].mode == cmd_recvl.mode)) {
					// hb(x)
					uint8_t hb_1 = 2 * index;
					ch[hb_1].mode = OFF;
					ch[hb_1].on = 0;
					ch[hb_1].off = 0;
					ch[hb_1].cnt = 0;
					set_hb_gnd(hb_1);
					// hb(x+1)
					uint8_t hb_2 = (2 * index) + 1;
					ch[hb_2].mode = OFF;
					ch[hb_2].on = 0;
					ch[hb_2].off = 0;
					ch[hb_2].cnt = 0;
					set_hb_gnd(hb_2);
				}
			}
		}
	}
}

//---------------------------------
// setters
//---------------------------------
/* enable high side of halb-bridge (= HV) */
void tHB::set_hb_hv(uint8_t hb_channel) {
	// enable high side (HB_HGi pin = 1U/3U/5U/7U/9U/11U/13U/15U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel) + 1, HB_ON);
	// disable low side (HB_LGi pin = 0U/2U/4U/6U/8U/10U/12U/14U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel), HB_OFF);
}

/* enable low side of halb-bridge (= GND) */
void tHB::set_hb_gnd(uint8_t hb_channel) {
	// disable high side (HB_HGi pin = 1U/3U/5U/7U/9U/11U/13U/15U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel) + 1, HB_OFF);
	// enable low side (HB_LGi pin = 0U/2U/4U/6U/8U/10U/12U/14U)
	GPIO_WritePinOutput(GPIOD, (2 * hb_channel), HB_ON);
}
