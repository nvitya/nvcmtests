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
#include "d7s_595.h"

#include "cppinit.h"

#include "traces.h"

THwUart   conuart;

Td7s_595  disp;

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

	// init ledandkey
	disp.pin_din.Assign(PORTNUM_B, 4, false);
	disp.pin_rclk.Assign(PORTNUM_B, 5, false);
	disp.pin_sclk.Assign(PORTNUM_B, 6, false);
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

	//disp.Diledandkey.leds = hbcounter;

	//TRACE("ledandkey.keys = %02X\r\n", ledandkey.keys);

	//TRACE("hbcounter = %i\r\n", hbcounter);
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_preinit_code();

	unsigned clockspeed = MAX_CLOCK_SPEED;

#ifdef MCU_INPUT_FREQ
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, MAX_CLOCK_SPEED))
#else
	//if (false)
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, MAX_CLOCK_SPEED))
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
	TRACE("Max7219 LED tube test, board: %s\r\n", BOARD_NAME);

	SysTick_Config(clockspeed / 1000);

	// timed loop without using interrupts

	TRACE("Starting main cycle...\r\n");

	unsigned t0, t1;

	t0 = CLOCKCNT;

	unsigned cnt = 0;
	unsigned prevscan = disp.scancount;

	disp.DisplayHexNum(0x12345678);

	// Infinite loop
	while (1)
	{
		idle_task();

		t1 = CLOCKCNT;

		if (t1 - t0 > SystemCoreClock / 2)
		{
			//TRACE("TIM14 = %u\r\n", TIM14->CNT);
			heartbeat_task();
			t0 = t1;
		}

		if (disp.scancount != prevscan)
		{
			++cnt;
			disp.DisplayDecNum(cnt, false);
			//disp.DisplayHexNum(cnt);
			prevscan = disp.scancount;
		}
	}

}

// ----------------------------------------------------------------------------
