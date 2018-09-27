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
 *  file:     board_setup.h (periphtest)
 *  brief:    This is included as code into main.cpp, contains board specific initializations and definitions
 *  version:  1.00
 *  date:     2018-09-27
 *  authors:  nvitya
*/

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(2, 13, false); // PC13

void board_setup()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_0);  // USART1_RX
	conuart.Init(1);

	// USART2
	//hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART2_TX
	//hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_0 | PINCFG_PULLUP);  // USART2_RX
	//conuart.Init(2);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_B, 5, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_B, 6, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_B, 7, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}
#endif

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

#define LED_COUNT 3

void board_setup()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

  // USART3: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_D, 8,  PINCFG_OUTPUT | PINCFG_AF_7); // USART3_TX
	hwpinctrl.PinSetup(PORTNUM_D, 9,  PINCFG_INPUT  | PINCFG_AF_7); // USART3_RX
	conuart.Init(3);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_C, 10, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_C, 11, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_C, 12, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}

#endif

#if defined(BOARD_DEV_STM32F407VG)

TGpioPin  led1pin(PORTNUM_E, 0, true);  // PE0

#define LED_COUNT 1

void board_setup()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_B, 4, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_B, 5, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_B, 6, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}

#endif


#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);

void board_setup()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	conuart.Init(0);
}

#endif

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void board_setup()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	conuart.Init(0);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_A, 28, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_A, 29, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 30, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}

#endif

#if defined(BOARD_DISCOVERY_F072)

TGpioPin  led1pin(PORTNUM_C, 6, false);
TGpioPin  led2pin(PORTNUM_C, 8, false);
TGpioPin  led3pin(PORTNUM_C, 9, false);
TGpioPin  led4pin(PORTNUM_C, 7, false);

#define LED_COUNT 4

void board_setup()
{
	// direction leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_1);  // USART1_RX
	conuart.Init(1);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_B, 5, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_B, 6, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_B, 7, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}

#endif

#if defined(BOARD_DEV_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

void board_setup()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
}

#endif

#if defined(BOARD_ARDUINO_DUE)

TGpioPin  led1pin(PORTNUM_B, 27, false); // D13

void board_setup()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART - On the Arduino programmer interface
	hwpinctrl.PinSetup(0, 8, PINCFG_INPUT | PINCFG_AF_0);  // UART_RXD
	hwpinctrl.PinSetup(0, 9, PINCFG_OUTPUT | PINCFG_AF_0); // UART_TXD
	conuart.Init(0);  // UART

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_C, 25, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_C, 24, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_C, 23, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;

}

#endif

#if defined(BOARD_XPLAINED_SAME70)

TGpioPin  led1pin(2, 8, false);  // C8

void board_setup()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1 - EDBG
	hwpinctrl.PinSetup(0, 21, PINCFG_INPUT | PINCFG_AF_0);  // USART1_RXD
	MATRIX->CCFG_SYSIO |= (1 << 4); // select PB4 instead of TDI !!!!!!!!!
	hwpinctrl.PinSetup(1,  4, PINCFG_OUTPUT | PINCFG_AF_3); // USART1_TXD
	conuart.Init(0x101); // USART1

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_D, 26, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_C, 31, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 19, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}

#endif

#if defined(BOARD_VERTIBO_A)

TGpioPin  led1pin(PORTNUM_A, 29, false);

TGpioPin  pin_fpga_cfg(PORTNUM_C, 9, false);
TGpioPin  pin_fpga_irq(PORTNUM_A, 22, false);

void board_setup()
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


	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_D, 18, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_D, 19, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_D, 20, false);
	ledandkey.Init();
	ledandkey.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
	ledandkey.leds = 0x00;
}

#endif


#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

