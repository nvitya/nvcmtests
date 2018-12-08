/* -----------------------------------------------------------------------------
 * This file is a part of the NVCM Tests project: https://github.com/nvitya/nvcmtests
 * Copyright (c) 2018 Viktor Nagy, nvitya
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 * --------------------------------------------------------------------------- */
/*
 *  file:     main.cpp (SDRAM)
 *  brief:    Multi-board sdram example for NVCM
 *  version:  1.00
 *  date:     2018-11-24
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "hwsdram.h"
#include "sdram_test.h"

#include "traces.h"

THwUart   conuart;  // console uart

#if defined(BOARD_DISCOVERY_F746)

TGpioPin  led1pin(PORTNUM_I, 1, false);

#define LED_COUNT 1

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 9,  PINCFG_OUTPUT | PINCFG_AF_7);
	hwpinctrl.PinSetup(PORTNUM_B, 7,  PINCFG_INPUT  | PINCFG_AF_7);
	conuart.Init(1); // USART1

	// SDRAM pins

	unsigned pin_flags = PINCFG_AF_12 | PINCFG_SPEED_MEDIUM; // it does not work with FAST !!!

	hwpinctrl.PinSetup(PORTNUM_C,  3, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_D,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  9, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 10, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 15, pin_flags);


	hwpinctrl.PinSetup(PORTNUM_E,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  7, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  9, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 10, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 11, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 12, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 13, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_F,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  2, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  3, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  4, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 11, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 12, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 13, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_G,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  4, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_H,  3, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_H,  5, pin_flags);

	// config the SDRAM device: 8 MByte

	hwsdram.row_bits = 12;
	hwsdram.column_bits = 8;
	hwsdram.bank_count = 4;
	hwsdram.cas_latency = 2;

	hwsdram.row_precharge_delay = 1;
	hwsdram.row_to_column_delay = 1;
	hwsdram.recovery_delay = 1;
	hwsdram.row_cycle_delay = 5;
	hwsdram.exit_self_refresh_delay = 5;
	hwsdram.active_to_precharge_delay = 3; // TRAS

	hwsdram.burst_length = 1;  // it does not like when it bigger than 1

	hwsdram.Init();
}

#endif

#if defined(BOARD_DISCOVERY_F429)

TGpioPin  led1pin(PORTNUM_G, 13, false);
TGpioPin  led2pin(PORTNUM_G, 14, false);

#define LED_COUNT 2

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 9,  PINCFG_OUTPUT | PINCFG_AF_7);
	hwpinctrl.PinSetup(PORTNUM_B, 7,  PINCFG_INPUT  | PINCFG_AF_7);
	conuart.Init(1); // USART1

	// it does not run with PINCFG_SPEED_FAST !
	unsigned pin_flags = PINCFG_AF_12 | PINCFG_SPEED_MED2;

	hwpinctrl.PinSetup(PORTNUM_B,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_B,  6, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_C,  0, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_D,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  9, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 10, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_E,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  7, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  9, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 10, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 11, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 12, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 13, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_F,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  2, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  3, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  4, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 11, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 12, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 13, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_G,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  4, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G, 15, pin_flags);

	// config the SDRAM device: 8 MByte

	hwsdram.bank = 2; // it is connected to bank 2!

	hwsdram.hclk_div = 2;

	hwsdram.row_bits = 12;
	hwsdram.column_bits = 8;
	hwsdram.bank_count = 4;
	hwsdram.cas_latency = 3;

	hwsdram.row_precharge_delay = 2;
	hwsdram.row_to_column_delay = 2;
	hwsdram.recovery_delay = 2;
	hwsdram.row_cycle_delay = 7;
	hwsdram.exit_self_refresh_delay = 7;
	hwsdram.active_to_precharge_delay = 4; // TRAS

	hwsdram.burst_length = 1;

	hwsdram.Init();
}

#endif

#if defined(BOARD_EVK_IMXRT1020)

TGpioPin  led1pin(1, 5, false); // GPIO_AD_B0_05 = GPIO_1_5

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PadSetup(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0);
	hwpinctrl.PadSetup(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0);
	conuart.Init(1); // UART1

	// SDRAM

  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_00_SEMC_DATA00,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_01_SEMC_DATA01,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_02_SEMC_DATA02,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_03_SEMC_DATA03,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_04_SEMC_DATA04,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_05_SEMC_DATA05,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_06_SEMC_DATA06,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_07_SEMC_DATA07,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_08_SEMC_DM00,    0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_09_SEMC_WE,      0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_10_SEMC_CAS,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_11_SEMC_RAS,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_12_SEMC_CS0,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_13_SEMC_BA0,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_14_SEMC_BA1,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_15_SEMC_ADDR10,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_16_SEMC_ADDR00,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_17_SEMC_ADDR01,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_18_SEMC_ADDR02,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_19_SEMC_ADDR03,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_20_SEMC_ADDR04,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_21_SEMC_ADDR05,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_22_SEMC_ADDR06,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_23_SEMC_ADDR07,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_24_SEMC_ADDR08,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_25_SEMC_ADDR09,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_26_SEMC_ADDR11,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_27_SEMC_ADDR12,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_28_SEMC_DQS,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_29_SEMC_CKE,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_30_SEMC_CLK,     0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_31_SEMC_DM01,    0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_32_SEMC_DATA08,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_33_SEMC_DATA09,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_34_SEMC_DATA10,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_35_SEMC_DATA11,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_36_SEMC_DATA12,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_37_SEMC_DATA13,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_38_SEMC_DATA14,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_39_SEMC_DATA15,  0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_40_SEMC_CSX00,   0);
  hwpinctrl.PadSetup(IOMUXC_GPIO_EMC_41_SEMC_READY,   0);

	// config for MT48LC16M16A2-6A: 32 MByte

	hwsdram.row_bits = 13;
	hwsdram.column_bits = 9;
	hwsdram.bank_count = 4;
	hwsdram.cas_latency = 3;

	hwsdram.row_precharge_delay = 3;
	hwsdram.row_to_column_delay = 3;
	hwsdram.recovery_delay = 2;
	hwsdram.row_cycle_delay = 9;

	hwsdram.burst_length = 8;

	hwsdram.Init();
}

#endif

#if defined(BOARD_XPLAINED_SAME70)

TGpioPin  led1pin(2, 8, false);  // C8

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1 - EDBG
	hwpinctrl.PinSetup(0, 21, PINCFG_INPUT | PINCFG_AF_0);  // USART1_RXD
	MATRIX->CCFG_SYSIO |= (1 << 4); // select PB4 instead of TDI !!!!!!!!!
	hwpinctrl.PinSetup(1,  4, PINCFG_OUTPUT | PINCFG_AF_3); // USART1_TXD
	conuart.Init(0x101); // USART1

	// UART3 - Arduino shield
	//hwpinctrl.PinSetup(3, 28, PINCFG_INPUT | PINCFG_AF_0);  // UART3_RXD
	//hwpinctrl.PinSetup(3, 30, PINCFG_OUTPUT | PINCFG_AF_0); // UART3_TXD
	//uartx2.Init(3); // UART3
}

#endif

#if defined(BOARD_VERTIBO_A)

TGpioPin  led1pin(PORTNUM_A, 29, false);

TGpioPin  pin_fpga_cfg(PORTNUM_C, 9, false);
TGpioPin  pin_fpga_irq(PORTNUM_A, 22, false);

void setup_board()
{
	// leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_fpga_cfg.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	pin_fpga_irq.Setup(PINCFG_INPUT);

	// setupt the clock output pin
#if 1
	hwpinctrl.PinSetup(PORTNUM_A,  6,  PINCFG_OUTPUT | PINCFG_AF_1);  // PCK0 = FPGA.CLK_IN

	PMC->PMC_SCER = (1 << 8); // enable PCK0

	PMC->PMC_PCK[0] = 0
		| (1 << 0)  // CSS(3): 1 = MAIN CLK (12 MHz)
		| (0 << 4)  // PRES(8): divisor - 1
	;
#endif

	// Console uart
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.baudrate = 115200;
	conuart.Init(0);

	// The Ethernet pins must be configured every time otherwise the Ethernet PHY will be damaged !!!

	/* Ethernet pins configuration ************************************************

	        RMII_REF_CLK ----------------------> PD0
	        RMII_MDIO -------------------------> PD9
	        RMII_MDC --------------------------> PD8
	        RMII_MII_CRS_DV -------------------> PD4
	        RMII_MII_RXD0 ---------------------> PD5
	        RMII_MII_RXD1 ---------------------> PD6
	        RMII_MII_RXER ---------------------> PD7
	        RMII_MII_TX_EN --------------------> PD1
	        RMII_MII_TXD0 ---------------------> PD2
	        RMII_MII_TXD1 ---------------------> PD3
	*/

	hwpinctrl.PinSetup(PORTNUM_D, 0, PINCFG_INPUT); // REF CLK
	hwpinctrl.PinSetup(PORTNUM_D, 9, PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // MDIO
	hwpinctrl.PinSetup(PORTNUM_D, 8, PINCFG_OUTPUT | PINCFG_GPIO_INIT_0); // MDC
	hwpinctrl.PinSetup(PORTNUM_D, 4, PINCFG_INPUT); // CRS_DV
	hwpinctrl.PinSetup(PORTNUM_D, 5, PINCFG_INPUT); // RXD0
	hwpinctrl.PinSetup(PORTNUM_D, 6, PINCFG_INPUT); // RXD1
	hwpinctrl.PinSetup(PORTNUM_D, 7, PINCFG_INPUT); // RXER       // Tie to the GND on early hw !!!
	hwpinctrl.PinSetup(PORTNUM_D, 1, PINCFG_OUTPUT | PINCFG_GPIO_INIT_0); // TX_EN <- this is the most important !!!
	hwpinctrl.PinSetup(PORTNUM_D, 2, PINCFG_OUTPUT | PINCFG_GPIO_INIT_0); // TXD0
	hwpinctrl.PinSetup(PORTNUM_D, 3, PINCFG_OUTPUT | PINCFG_GPIO_INIT_0); // TXD1

	// SDRAM

	uint32_t pincfgbase = 0; //PINCFG_DRIVE_STRONG;

	hwpinctrl.PinSetup(PORTNUM_A, 20, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // A16/BA0
	hwpinctrl.PinSetup(PORTNUM_A,  0, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // A17/BA1

	hwpinctrl.PinSetup(PORTNUM_C, 0, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D0
	hwpinctrl.PinSetup(PORTNUM_C, 1, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D1
	hwpinctrl.PinSetup(PORTNUM_C, 2, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D2
	hwpinctrl.PinSetup(PORTNUM_C, 3, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D3
	hwpinctrl.PinSetup(PORTNUM_C, 4, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D4
	hwpinctrl.PinSetup(PORTNUM_C, 5, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D5
	hwpinctrl.PinSetup(PORTNUM_C, 6, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D6
	hwpinctrl.PinSetup(PORTNUM_C, 7, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D7

	hwpinctrl.PinSetup(PORTNUM_E, 0, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D8
	hwpinctrl.PinSetup(PORTNUM_E, 1, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D9
	hwpinctrl.PinSetup(PORTNUM_E, 2, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D10
	hwpinctrl.PinSetup(PORTNUM_E, 3, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D11
	hwpinctrl.PinSetup(PORTNUM_E, 4, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D12
	hwpinctrl.PinSetup(PORTNUM_E, 5, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D13

	hwpinctrl.PinSetup(PORTNUM_A, 15, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D14
	hwpinctrl.PinSetup(PORTNUM_A, 16, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // D15

	hwpinctrl.PinSetup(PORTNUM_C, 15, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // NCS1/SDCS

	hwpinctrl.PinSetup(PORTNUM_C, 18, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A0/NBS0
	//hwpinctrl.PinSetup(PORTNUM_C, 19, PINCFG_OUTPUT | PINCFG_AF_0);  // A1

	hwpinctrl.PinSetup(PORTNUM_C, 20, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A2
	hwpinctrl.PinSetup(PORTNUM_C, 21, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A3
	hwpinctrl.PinSetup(PORTNUM_C, 22, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A4
	hwpinctrl.PinSetup(PORTNUM_C, 23, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A5
	hwpinctrl.PinSetup(PORTNUM_C, 24, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A6
	hwpinctrl.PinSetup(PORTNUM_C, 25, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A7
	hwpinctrl.PinSetup(PORTNUM_C, 26, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A8
	hwpinctrl.PinSetup(PORTNUM_C, 27, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A9
	hwpinctrl.PinSetup(PORTNUM_C, 28, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A10
	hwpinctrl.PinSetup(PORTNUM_C, 29, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A11
	hwpinctrl.PinSetup(PORTNUM_C, 30, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A12
	hwpinctrl.PinSetup(PORTNUM_C, 31, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_0);  // A13
	hwpinctrl.PinSetup(PORTNUM_A, 18, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // A14

	hwpinctrl.PinSetup(PORTNUM_D, 13, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // SDA10
	hwpinctrl.PinSetup(PORTNUM_D, 14, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // SDCKE
	hwpinctrl.PinSetup(PORTNUM_D, 15, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // NWR1/NBS1
	hwpinctrl.PinSetup(PORTNUM_D, 16, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // RAS
	hwpinctrl.PinSetup(PORTNUM_D, 17, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // CAS
	hwpinctrl.PinSetup(PORTNUM_D, 23, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // SDCK
	hwpinctrl.PinSetup(PORTNUM_D, 29, pincfgbase | PINCFG_OUTPUT | PINCFG_AF_2);  // SDWE


	// config for MT48LC16M16A2-6A: 32 MByte

	hwsdram.row_bits = 13;
	hwsdram.column_bits = 9;
	hwsdram.bank_count = 4;
	hwsdram.cas_latency = 3;

	hwsdram.row_precharge_delay = 3;
	hwsdram.row_to_column_delay = 3;
	hwsdram.recovery_delay = 2;
	hwsdram.row_cycle_delay = 9;

	hwsdram.burst_length = 1;  // SDRAM does not work properly when larger than 1, but no speed degradation noticed

	hwsdram.Init();
}

#endif

#if defined(BOARD_XPRESSO_LPC54608)

TGpioPin  led1pin(2, 2, true);
TGpioPin  led2pin(3, 3, true);
TGpioPin  led3pin(3, 14, true);

#define LED_COUNT 3

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(0, 30, PINCFG_OUTPUT | PINCFG_AF_1); // UART_TX:
	hwpinctrl.PinSetup(0, 29, PINCFG_INPUT  | PINCFG_AF_1); // UART_RX:
	conuart.Init(0);
}

#endif

#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

volatile unsigned systick = 0;

extern "C" void SysTick_Handler(void)
{
	++systick;
}

void idle_task()
{
}

unsigned hbcounter = 0;

void heartbeat_task() // invoked every 0.5 s
{
	++hbcounter;

	led1pin.SetTo(hbcounter >> 0);
#if LED_COUNT > 1
	led2pin.SetTo(hbcounter >> 1);
#endif
#if LED_COUNT > 2
	led3pin.SetTo(hbcounter >> 2);
#endif
#if LED_COUNT > 3
	led4pin.SetTo(hbcounter >> 3);
#endif
#if LED_COUNT > 4
	led5pin.SetTo(hbcounter >> 4);
#endif

	//TRACE("hbcounter = %u, systick = %u\r\n", hbcounter, systick);

	//conuart.TrySendChar(0x55);
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_disable_interrupts();

  mcu_preinit_code(); // inline code for preparing the MCU, RAM regions. Without this even the stack does not work on some MCUs.

	// Set the interrupt vector table offset, so that the interrupts and exceptions work
	mcu_init_vector_table();

  unsigned clockspeed = MCU_CLOCK_SPEED;

#ifdef MCU_INPUT_FREQ
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, clockspeed))
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, clockspeed))
#endif
	{
		while (1)
		{
			// the external oscillator did not start.
		}
	}

	// now the MCU runs faster, start the memory, and C/C++ initialization:
	cppinit();
	// ...from now on all the variables work, static classes are initialized.

	// provide info to the system about the clock speed:
	hwclkctrl.SetClockInfo(clockspeed);

	mcu_enable_fpu();    // enable coprocessor if present

	mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM SDRAM TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

#if 0
	// The D-Cache must be enabled for effective SDRAM sequential (burst) transfers
	// the SDRAM sequential read performance is 4x (!) times better this way

	mcu_enable_dcache();
	TRACE("D-CACHE Enabled. Keep in mind for DMA transfers.\r\n");  // you have been warned. :)

	// the NVCM libraries does not contain cache invalidation for DMA transfers so this could be
	// a problem sometimes. This test does not use DMA transfers.
#endif


#if 1
	if (hwsdram.initialized)
	{
		TRACE("SDRAM initialized, size = %u kByte\r\n", hwsdram.byte_size >> 10);
		sdram_tests();
	}
	else
	{
		TRACE("SDRAM Init failed !\r\n");
	}
#endif

	TRACE("\r\nStarting main cycle...\r\n");

	//SysTick_Config(SystemCoreClock / 1000);

	//mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock / 1;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		idle_task();

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			//sdram_tests();
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
