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

#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"

#include "hwsdram.h"
#include "lcd_test.h"

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

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);
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


#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

volatile unsigned systick = 0;

extern "C" void SysTick_Handler(void)
{
	++systick;
}

uint32_t disp_counter = 0;

#include "font_FreeMonoBold9pt7b.h"

TGfxFont font_mono_bold(&FreeMonoBold9pt7b);

void idle_task()
{
	if (disp.initialized)
	{
		disp.SetFont(&font_mono_bold);
		disp.SetCursor(10, 200);
		disp.printf("%8u", disp_counter);
		++disp_counter;
	}
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
	TRACE("F746 Discovery Tests\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	lcd_test();

	TRACE("\r\nStarting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	//unsigned * ptr = (unsigned * )0x3000000;
	//*ptr = 1;

	unsigned hbclocks = SystemCoreClock;

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
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
