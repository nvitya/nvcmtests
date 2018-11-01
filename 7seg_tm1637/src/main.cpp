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
 *  file:     main.cpp (blinkled)
 *  brief:    Multi-board blinking led example for NVCM
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include <stdio.h>
#include <string.h>
//#include <stdlib.h>

#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "clockcnt.h"
#include "hwuart.h"
#include "timedisp_tm1637.h"

#include "cppinit.h"

#include "traces.h"

THwUart   conuart;

TTimeDisp_tm1637   disp;


#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(PORTNUM_C, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	//hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_OUTPUT | PINCFG_AF_1);  // USART1_TX
	//hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_INPUT  | PINCFG_AF_1);  // USART1_RX
	//conuart.Init(1);

	hwpinctrl.PinSetup(PORTNUM_A,  2, PINCFG_OUTPUT | PINCFG_AF_1);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_INPUT  | PINCFG_AF_1);  // USART2_RX
	conuart.Init(2);

	// init disp
	disp.controller.stb_pin.Assign(PORTNUM_B, 5, false);
	disp.controller.clk_pin.Assign(PORTNUM_B, 6, false);
	disp.controller.dio_pin.Assign(PORTNUM_B, 7, false);
	disp.Init();
}
#endif

#if defined(BOARD_XPLORER_LPC4330)

TGpioPin  led1pin(1, 12, true); // D2 (GPIO numbering)
TGpioPin  led2pin(1, 11, true); // D3

#define LED_COUNT 2

void setup_board()
{
	// By the classic LPC (V1) MCUs the pin numbering and the GPIO numbering differs!

	hwpinctrl.PinSetup(2, 12, PINCFG_OUTPUT | PINCFG_AF_0);  // D2: GPIO_1_12, pad B9
	hwpinctrl.PinSetup(2, 11, PINCFG_OUTPUT | PINCFG_AF_0);  // D3: GPIO_1_11, pad A9
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// uart console

	hwpinctrl.PinSetup(6,  4, PINCFG_OUTPUT | PINCFG_AF_2);  // U0_TXD, J8/9
	hwpinctrl.PinSetup(6,  5, PINCFG_INPUT  | PINCFG_AF_2);  // U0_RXD, J8/10
	conuart.Init(0);

	// disp

	hwpinctrl.PinSetup(2,  2, PINCFG_OUTPUT | PINCFG_AF_4);  // GPIO_5_2, J8/11
	hwpinctrl.PinSetup(2,  5, PINCFG_OUTPUT | PINCFG_AF_4);  // GPIO_5_5, J8/12
	hwpinctrl.PinSetup(2,  8, PINCFG_OUTPUT | PINCFG_AF_4);  // GPIO_5_7, J8/15

	disp.controller.clk_pin.Assign(5, 5, false);
	disp.controller.dio_pin.Assign(5, 7, false);
	disp.Init();
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


	// init disp
	disp.controller.clk_pin.Assign(PORTNUM_D, 19, false);
	disp.controller.dio_pin.Assign(PORTNUM_D, 20, false);
	disp.Init();
	//disp.DisplayDirect(0x00000080, 0x00000000); // turn on only the lowest dot
}

#endif

#if defined(BOARD_DISCOVERY_F072)

TGpioPin  led1pin(PORTNUM_C, 6, false);
TGpioPin  led2pin(PORTNUM_C, 8, false);
TGpioPin  led3pin(PORTNUM_C, 9, false);
TGpioPin  led4pin(PORTNUM_C, 7, false);

#define LED_COUNT 4

void setup_board()
{
	// direction leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);

	// USART1_TX: PA9
	hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_OUTPUT | PINCFG_AF_1);
	// USART1_RX: PA10
	hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_INPUT  | PINCFG_AF_1);

	conuart.Init(1);

	// init disp
	disp.controller.dio_pin.Assign(PORTNUM_B, 5, false);
	disp.controller.clk_pin.Assign(PORTNUM_B, 6, false);
	disp.Init();
}

#endif

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	inputpin.Setup(PINCFG_INPUT);
	//inputpin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	conuart.Init(0);

	// init disp
	disp.controller.stb_pin.Assign(PORTNUM_A, 24, false);
	disp.controller.clk_pin.Assign(PORTNUM_A, 25, false);
	disp.controller.dio_pin.Assign(PORTNUM_A, 26, false);
	disp.Init();
}
#endif

#if defined(BOARD_ARDUINO_DUE)

TGpioPin  led1pin(1, 27, false); // D13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART - On the Arduino programmer interface
	hwpinctrl.PinSetup(0, 8, PINCFG_INPUT | PINCFG_AF_0);  // UART_RXD
	hwpinctrl.PinSetup(0, 9, PINCFG_OUTPUT | PINCFG_AF_0); // UART_TXD
	conuart.Init(0);  // UART

	// init disp
	disp.controller.stb_pin.Assign(PORTNUM_D, 0, false);
	disp.controller.clk_pin.Assign(PORTNUM_D, 1, false);
	disp.controller.dio_pin.Assign(PORTNUM_D, 2, false);
	disp.Init();
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

unsigned hbcounter = 0;

void idle_task()
{
	disp.Run();
}

unsigned piodata = 0;

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

	//TRACE("hbcounter = %i\r\n", hbcounter);
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

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
	//if (false)
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, clockspeed))
#endif
	{
		while (1)
		{
			// the external oscillator did not start.
		}
	}

	// now we run faster, start the memory, and C/C++ initialization:
	cppinit();
	// ...from now on all the variables work, static classes are initialized.

	// provide info to the system about the clock speed:
	hwclkctrl.SetClockInfo(clockspeed);

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	TRACE("\r\n-------------------------\r\n");
	TRACE("tm1637 time display test, board: %s\r\n", BOARD_NAME);
	TRACE("MCU Clock Speed: %u Hz\r\n", SystemCoreClock);

	SysTick_Config(clockspeed / 1000);

	mcu_enable_interrupts();

	// timed loop without using interrupts

	TRACE("Starting main cycle...\r\n");

	unsigned t0, t1;

	t0 = CLOCKCNT;

	unsigned cnt = 0;
	unsigned prevscan = disp.controller.scancounter;

	// Infinite loop
	while (1)
	{
		idle_task();

		t1 = CLOCKCNT;

		if (t1 - t0 > SystemCoreClock / 2)
		{
			//TRACE("TIM14 = %u\r\n", TIM14->CNT);
			disp.colon_on = !disp.colon_on;
			heartbeat_task();
			t0 = t1;
		}

		if (disp.controller.scancounter != prevscan)
		{
			++cnt;

			disp.DisplayDecNum(cnt);
			prevscan = disp.controller.scancounter;
		}
	}

}

// ----------------------------------------------------------------------------
