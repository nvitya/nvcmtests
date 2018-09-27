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
 *  file:     main.cpp (uart)
 *  brief:    Multi-board uart example (with MCU speed test) for NVCM
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include "main.h"

#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"

#include "test_spi.h"
extern void test_adc();

#include "traces.h"

#include "ledandkey.h"

THwUart     conuart;  // console uart
TLedAndKey  ledandkey; // some display

#include "board_setup.h" // contains code too

volatile unsigned systick = 0;

extern "C" void SysTick_Handler(void)
{
	++systick;
}

void idle_task()
{
	ledandkey.Run();
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

// STM32F746
  //clockspeed = 144000000; // maximal speed for the ADC
  //clockspeed = 144000000; // maximal speed for the ADC

// STM32F103
  //clockspeed = 64000000; // maximal speed without external oscillator
  //clockspeed = 56000000; // maximal speed for the ADC speed 1 M / s

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
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the boards specific initializations
	board_setup();  // from board_setup.h

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM Peripheral Tests\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);


	// ------------------------------------------------------------------------------------
	// TEST SELECTION

	//test_spi();
	test_adc();

	//test_division();

	// ------------------------------------------------------------------------------------

	TRACE("\r\nStarting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	unsigned prevscannum = ledandkey.controller.scancounter;
	unsigned dcnt = 0;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		idle_task();

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;
		}

#if 1	 // to test the display
		if (prevscannum != ledandkey.controller.scancounter)
		{
			++dcnt;
			ledandkey.DisplayDecNum(dcnt);
			prevscannum = ledandkey.controller.scancounter;
		}
#endif
	}
}

// ----------------------------------------------------------------------------
