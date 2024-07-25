/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define SOPT5_UART0RXSRC_UART_RX 0x00u /*!<@brief UART 0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*! @name VDD5 (number 5), VDD_005
  @{ */
/* @} */

/*! @name VSS6 (number 6), VSS_006
  @{ */
/* @} */

/*! @name VDD16 (number 16), VDD_016
  @{ */
/* @} */

/*! @name VSS17 (number 17), VSS_017
  @{ */
/* @} */

/*! @name VSS22 (number 18), VSS_018
  @{ */
/* @} */

/*! @name VDDA (number 31), VDDA_031
  @{ */
/* @} */

/*! @name VSSA (number 34), VSSA_034
  @{ */
/* @} */

/*! @name VDD54 (number 43), VDD_043
  @{ */
/* @} */

/*! @name VSS55 (number 44), VSS_044
  @{ */
/* @} */

/*! @name VDD67 (number 56), VDD_056
  @{ */
/* @} */

/*! @name VSS68 (number 57), VSS_057
  @{ */
/* @} */

/*! @name VDD81 (number 70), VDD_070
  @{ */
/* @} */

/*! @name VSS82 (number 71), VSS_071
  @{ */
/* @} */

/*! @name RESET_b (number 74), RESET_b
  @{ */
/* @} */

/*! @name VSS104 (number 93), VSS_093
  @{ */
/* @} */

/*! @name VDD105 (number 94), VDD_094
  @{ */
/* @} */

/*! @name VSS118 (number 107), VSS_107
  @{ */
/* @} */

/*! @name VDD119 (number 108), VDD_108
  @{ */
/* @} */

/*! @name VSS137 (number 121), VSS_121
  @{ */
/* @} */

/*! @name VDD138 (number 122), VDD_122
  @{ */
/* @} */

/*! @name VSS150 (number 134), VSS_134
  @{ */
/* @} */

/*! @name VDD151 (number 135), VDD_135
  @{ */
/* @} */

/*! @name PORTA18 (number 72), EXTAL0
  @{ */

/* Symbols to be used with PORT driver */
#define EXTAL0_PORT PORTA                /*!<@brief PORT peripheral base pointer */
#define EXTAL0_PIN 18U                   /*!<@brief PORT pin number */
#define EXTAL0_PIN_MASK (1U << 18U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTA19 (number 73), XTAL0
  @{ */

/* Symbols to be used with PORT driver */
#define XTAL0_PORT PORTA                /*!<@brief PORT peripheral base pointer */
#define XTAL0_PIN 19U                   /*!<@brief PORT pin number */
#define XTAL0_PIN_MASK (1U << 19U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name VREFH (number 32), VREFH_032
  @{ */
/* @} */

/*! @name VREFL (number 33), VREFL_033
  @{ */
/* @} */

/*! @name PORTA0 (number 50), JTAG_CLK
  @{ */

/* Symbols to be used with PORT driver */
#define JTAG_CLK_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define JTAG_CLK_PIN 0U                   /*!<@brief PORT pin number */
#define JTAG_CLK_PIN_MASK (1U << 0U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name PORTA1 (number 51), JTAG_TDI
  @{ */

/* Symbols to be used with PORT driver */
#define JTAG_TDI_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define JTAG_TDI_PIN 1U                   /*!<@brief PORT pin number */
#define JTAG_TDI_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name PORTA2 (number 52), JTAG_TDO
  @{ */

/* Symbols to be used with PORT driver */
#define JTAG_TDO_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define JTAG_TDO_PIN 2U                   /*!<@brief PORT pin number */
#define JTAG_TDO_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name PORTA3 (number 53), JTAG_TMS
  @{ */

/* Symbols to be used with PORT driver */
#define JTAG_TMS_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define JTAG_TMS_PIN 3U                   /*!<@brief PORT pin number */
#define JTAG_TMS_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name PORTA14 (number 66), USB_UART_TX
  @{ */

/* Symbols to be used with PORT driver */
#define USB_UART_TX_PORT PORTA                /*!<@brief PORT peripheral base pointer */
#define USB_UART_TX_PIN 14U                   /*!<@brief PORT pin number */
#define USB_UART_TX_PIN_MASK (1U << 14U)      /*!<@brief PORT pin mask */
                                              /* @} */

/*! @name PORTA15 (number 67), USB_UART_RX
  @{ */

/* Symbols to be used with PORT driver */
#define USB_UART_RX_PORT PORTA                /*!<@brief PORT peripheral base pointer */
#define USB_UART_RX_PIN 15U                   /*!<@brief PORT pin number */
#define USB_UART_RX_PIN_MASK (1U << 15U)      /*!<@brief PORT pin mask */
                                              /* @} */

/*! @name PORTB2 (number 83), EEPROM_SCL
  @{ */

/* Symbols to be used with PORT driver */
#define EEPROM_SCL_PORT PORTB               /*!<@brief PORT peripheral base pointer */
#define EEPROM_SCL_PIN 2U                   /*!<@brief PORT pin number */
#define EEPROM_SCL_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                            /* @} */

/*! @name PORTB3 (number 84), EEPROM_SDA
  @{ */

/* Symbols to be used with PORT driver */
#define EEPROM_SDA_PORT PORTB               /*!<@brief PORT peripheral base pointer */
#define EEPROM_SDA_PIN 3U                   /*!<@brief PORT pin number */
#define EEPROM_SDA_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                            /* @} */

/*! @name PORTE9 (number 12), PS_SW_STATE
  @{ */

/* Symbols to be used with GPIO driver */
#define PS_SW_STATE_GPIO GPIOE               /*!<@brief GPIO peripheral base pointer */
#define PS_SW_STATE_GPIO_PIN_MASK (1U << 9U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define PS_SW_STATE_PORT PORTE               /*!<@brief PORT peripheral base pointer */
#define PS_SW_STATE_PIN 9U                   /*!<@brief PORT pin number */
#define PS_SW_STATE_PIN_MASK (1U << 9U)      /*!<@brief PORT pin mask */
                                             /* @} */

/*! @name PORTE24 (number 45), PS_12V_VM
  @{ */

/* Symbols to be used with PORT driver */
#define PS_12V_VM_PORT PORTE                /*!<@brief PORT peripheral base pointer */
#define PS_12V_VM_PIN 24U                   /*!<@brief PORT pin number */
#define PS_12V_VM_PIN_MASK (1U << 24U)      /*!<@brief PORT pin mask */
                                            /* @} */

/*! @name PORTC9 (number 114), PS_12V_EN
  @{ */

/* Symbols to be used with GPIO driver */
#define PS_12V_EN_GPIO GPIOC               /*!<@brief GPIO peripheral base pointer */
#define PS_12V_EN_GPIO_PIN_MASK (1U << 9U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define PS_12V_EN_PORT PORTC               /*!<@brief PORT peripheral base pointer */
#define PS_12V_EN_PIN 9U                   /*!<@brief PORT pin number */
#define PS_12V_EN_PIN_MASK (1U << 9U)      /*!<@brief PORT pin mask */
                                           /* @} */

/*! @name PORTC10 (number 115), PS_12V_SCL
  @{ */

/* Symbols to be used with PORT driver */
#define PS_12V_SCL_PORT PORTC                /*!<@brief PORT peripheral base pointer */
#define PS_12V_SCL_PIN 10U                   /*!<@brief PORT pin number */
#define PS_12V_SCL_PIN_MASK (1U << 10U)      /*!<@brief PORT pin mask */
                                             /* @} */

/*! @name PORTC11 (number 116), PS_12V_SDA
  @{ */

/* Symbols to be used with PORT driver */
#define PS_12V_SDA_PORT PORTC                /*!<@brief PORT peripheral base pointer */
#define PS_12V_SDA_PIN 11U                   /*!<@brief PORT pin number */
#define PS_12V_SDA_PIN_MASK (1U << 11U)      /*!<@brief PORT pin mask */
                                             /* @} */

/*! @name PORTC8 (number 113), PS_HV_EN
  @{ */

/* Symbols to be used with GPIO driver */
#define PS_HV_EN_GPIO GPIOC               /*!<@brief GPIO peripheral base pointer */
#define PS_HV_EN_GPIO_PIN_MASK (1U << 8U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define PS_HV_EN_PORT PORTC               /*!<@brief PORT peripheral base pointer */
#define PS_HV_EN_PIN 8U                   /*!<@brief PORT pin number */
#define PS_HV_EN_PIN_MASK (1U << 8U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name VBAT (number 42), VBAT_042
  @{ */
/* @} */

/*! @name PORTD0 (number 127), HB_LG1
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG1_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_LG1_GPIO_PIN_MASK (1U << 0U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG1_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_LG1_PIN 0U                   /*!<@brief PORT pin number */
#define HB_LG1_PIN_MASK (1U << 0U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD1 (number 128), HB_HG1
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG1_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_HG1_GPIO_PIN_MASK (1U << 1U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG1_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_HG1_PIN 1U                   /*!<@brief PORT pin number */
#define HB_HG1_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD2 (number 129), HB_LG2
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG2_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_LG2_GPIO_PIN_MASK (1U << 2U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG2_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_LG2_PIN 2U                   /*!<@brief PORT pin number */
#define HB_LG2_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD4 (number 131), HB_LG3
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG3_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_LG3_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG3_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_LG3_PIN 4U                   /*!<@brief PORT pin number */
#define HB_LG3_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD3 (number 130), HB_HG2
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG2_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_HG2_GPIO_PIN_MASK (1U << 3U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG2_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_HG2_PIN 3U                   /*!<@brief PORT pin number */
#define HB_HG2_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD5 (number 132), HB_HG3
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG3_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_HG3_GPIO_PIN_MASK (1U << 5U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG3_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_HG3_PIN 5U                   /*!<@brief PORT pin number */
#define HB_HG3_PIN_MASK (1U << 5U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD6 (number 133), HB_LG4
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG4_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_LG4_GPIO_PIN_MASK (1U << 6U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG4_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_LG4_PIN 6U                   /*!<@brief PORT pin number */
#define HB_LG4_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD7 (number 136), HB_HG4
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG4_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_HG4_GPIO_PIN_MASK (1U << 7U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG4_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_HG4_PIN 7U                   /*!<@brief PORT pin number */
#define HB_HG4_PIN_MASK (1U << 7U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD8 (number 137), HB_LG5
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG5_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_LG5_GPIO_PIN_MASK (1U << 8U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG5_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_LG5_PIN 8U                   /*!<@brief PORT pin number */
#define HB_LG5_PIN_MASK (1U << 8U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD9 (number 138), HB_HG5
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG5_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define HB_HG5_GPIO_PIN_MASK (1U << 9U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG5_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define HB_HG5_PIN 9U                   /*!<@brief PORT pin number */
#define HB_HG5_PIN_MASK (1U << 9U)      /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name PORTD10 (number 139), HB_LG6
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG6_GPIO GPIOD                /*!<@brief GPIO peripheral base pointer */
#define HB_LG6_GPIO_PIN_MASK (1U << 10U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG6_PORT PORTD                /*!<@brief PORT peripheral base pointer */
#define HB_LG6_PIN 10U                   /*!<@brief PORT pin number */
#define HB_LG6_PIN_MASK (1U << 10U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTD11 (number 140), HB_HG6
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG6_GPIO GPIOD                /*!<@brief GPIO peripheral base pointer */
#define HB_HG6_GPIO_PIN_MASK (1U << 11U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG6_PORT PORTD                /*!<@brief PORT peripheral base pointer */
#define HB_HG6_PIN 11U                   /*!<@brief PORT pin number */
#define HB_HG6_PIN_MASK (1U << 11U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTD12 (number 141), HB_LG7
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG7_GPIO GPIOD                /*!<@brief GPIO peripheral base pointer */
#define HB_LG7_GPIO_PIN_MASK (1U << 12U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG7_PORT PORTD                /*!<@brief PORT peripheral base pointer */
#define HB_LG7_PIN 12U                   /*!<@brief PORT pin number */
#define HB_LG7_PIN_MASK (1U << 12U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTD13 (number 142), HB_HG7
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG7_GPIO GPIOD                /*!<@brief GPIO peripheral base pointer */
#define HB_HG7_GPIO_PIN_MASK (1U << 13U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG7_PORT PORTD                /*!<@brief PORT peripheral base pointer */
#define HB_HG7_PIN 13U                   /*!<@brief PORT pin number */
#define HB_HG7_PIN_MASK (1U << 13U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTD14 (number 143), HB_LG8
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_LG8_GPIO GPIOD                /*!<@brief GPIO peripheral base pointer */
#define HB_LG8_GPIO_PIN_MASK (1U << 14U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_LG8_PORT PORTD                /*!<@brief PORT peripheral base pointer */
#define HB_LG8_PIN 14U                   /*!<@brief PORT pin number */
#define HB_LG8_PIN_MASK (1U << 14U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTD15 (number 144), HB_HG8
  @{ */

/* Symbols to be used with GPIO driver */
#define HB_HG8_GPIO GPIOD                /*!<@brief GPIO peripheral base pointer */
#define HB_HG8_GPIO_PIN_MASK (1U << 15U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define HB_HG8_PORT PORTD                /*!<@brief PORT peripheral base pointer */
#define HB_HG8_PIN 15U                   /*!<@brief PORT pin number */
#define HB_HG8_PIN_MASK (1U << 15U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PORTE25 (number 46), PS_HV_VM
  @{ */

/* Symbols to be used with PORT driver */
#define PS_HV_VM_PORT PORTE                /*!<@brief PORT peripheral base pointer */
#define PS_HV_VM_PIN 25U                   /*!<@brief PORT pin number */
#define PS_HV_VM_PIN_MASK (1U << 25U)      /*!<@brief PORT pin mask */
                                           /* @} */

/*!
 * @brief 
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
