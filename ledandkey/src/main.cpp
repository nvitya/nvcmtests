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
#include "ledandkey.h"

#include "cppinit.h"

#include "traces.h"

THwUart   conuart;

TLedAndKey  ledandkey;


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

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_B, 5, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_B, 6, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_B, 7, false);
	ledandkey.Init();
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

	// ledandkey

	hwpinctrl.PinSetup(2,  2, PINCFG_OUTPUT | PINCFG_AF_4);  // GPIO_5_2, J8/11
	hwpinctrl.PinSetup(2,  5, PINCFG_OUTPUT | PINCFG_AF_4);  // GPIO_5_5, J8/12
	hwpinctrl.PinSetup(2,  8, PINCFG_OUTPUT | PINCFG_AF_4);  // GPIO_5_7, J8/15

	ledandkey.controller.stb_pin.Assign(5, 2, false);
	ledandkey.controller.clk_pin.Assign(5, 5, false);
	ledandkey.controller.dio_pin.Assign(5, 7, false);
	ledandkey.Init();
}

#endif

#if defined(MCU_STM32L011)

TGpioPin  led1pin(PORTNUM_A, 4, false);


void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// LPUART1_RX: PA0
	hwpinctrl.PinSetup(0, 0, PINCFG_INPUT | PINCFG_AF_6);
	// LPUART1_TX: PA1
	hwpinctrl.PinSetup(0, 1, PINCFG_OUTPUT | PINCFG_AF_6);

	// USART2_TX: PA2
	hwpinctrl.PinSetup(0, 2, PINCFG_OUTPUT | PINCFG_AF_4);
	// USART2_RX: PA3
	hwpinctrl.PinSetup(0, 3, PINCFG_INPUT  | PINCFG_AF_4);

	uartx.baudrate = 115200;
	uartx.Init(2);
}
#endif

#if defined(MCU_STM32F030F4)

// A TSSOP-20 processor soldered into an adapter and used in a breadboard

TGpioPin  led1pin(PORTNUM_A, 3, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1_TX: PA9
	hwpinctrl.PinSetup(0,  9, PINCFG_OUTPUT | PINCFG_AF_1);
	conuart.Init(1);

	ledandkey.controller.stb_pin.Assign(PORTNUM_A, 0, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_A, 1, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 2, false);
	ledandkey.Init();
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
	hwpinctrl.PinSetup(0,  9, PINCFG_OUTPUT | PINCFG_AF_1);
	// USART1_RX: PA10
	hwpinctrl.PinSetup(0, 10, PINCFG_INPUT  | PINCFG_AF_1);

	conuart.Init(1);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_B, 4, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_B, 5, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_B, 6, false);
	ledandkey.Init();
}

#endif

#if defined(BOARD_BOOT_XMC1200)

TGpioPin  led1pin(0, 0, true);
TGpioPin  led2pin(0, 2, true);
TGpioPin  led3pin(0, 5, true);
TGpioPin  led4pin(0, 6, true);
TGpioPin  led5pin(0, 7, true);

#define LED_COUNT 5

void setup_board()
{
	// direction leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led5pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(1,  2, PINCFG_OUTPUT | PINCFG_AF_7);  // UART_TX
	hwpinctrl.PinSetup(1,  3, PINCFG_INPUT  | PINCFG_AF_1);  // UART_RX
	conuart.Init(0x001);  // usic_0_ch_1

	// init ledandkey
	// beware some ports does not have output function, like 2.2-2.9
	ledandkey.controller.stb_pin.Assign(0, 10, false);
	ledandkey.controller.clk_pin.Assign(0, 11, false);
	ledandkey.controller.dio_pin.Assign(0, 12, false);
	ledandkey.Init();
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

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_A, 24, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_A, 25, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 26, false);
	ledandkey.Init();
}
#endif

#if defined(BOARD_MIBO64_ATSAME5X)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SERCOM0
	hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_OUTPUT | PINCFG_AF_3);  // PAD[0] = TX
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_INPUT  | PINCFG_AF_3);  // PAD[1] = RX
	conuart.Init(0);

	// SERCOM2
	//hwpinctrl.PinSetup(PORTNUM_A, 12, PINCFG_AF_2);  // PAD[0] = TX
	//hwpinctrl.PinSetup(PORTNUM_A, 13, PINCFG_AF_2);  // PAD[1] = RX
	//conuart.Init(2);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_A,  9, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_A, 10, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 11, false);
	ledandkey.Init();
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

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_D, 0, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_D, 1, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_D, 2, false);
	ledandkey.Init();
}

#endif

#if defined(MCU_ATSAMD10)

TGpioPin  led1pin(0, 15, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SERCOM0
	hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_OUTPUT | PINCFG_AF_3);  // PAD[0] = TX
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_INPUT  | PINCFG_AF_3);  // PAD[1] = RX
	conuart.Init(0);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_A,  8, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_A,  9, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 14, false);
	ledandkey.Init();
}
#endif

#if defined(MCU_LPC822)

TGpioPin  led1pin(0, 15, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinFuncConnect(SWM_FUNC_U0_RXD, 0, 0);  // pin 19 / TSOP-20
	hwpinctrl.PinFuncConnect(SWM_FUNC_U0_TXD, 0, 4);  // pin  6 / TSOP-20
	conuart.Init(0);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(0, 8, false);
	ledandkey.controller.clk_pin.Assign(0, 9, false);
	ledandkey.controller.dio_pin.Assign(0, 1, false);
	ledandkey.Init();
}
#endif

#if defined(MCU_MKL03)

TGpioPin  led1pin(PORTNUM_B, 0, false);

THwUart   uartx;

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(0, 3, PINCFG_OUTPUT | PINCFG_AF_4); // QFN16/PIN3: LPUART_TX
	hwpinctrl.PinSetup(0, 4, PINCFG_INPUT | PINCFG_AF_4);  // QFN16/PIN4: LPUART_RX
	conuart.Init(0);

	// init ledandkey
	ledandkey.controller.stb_pin.Assign(PORTNUM_A, 5, false);
	ledandkey.controller.clk_pin.Assign(PORTNUM_A, 6, false);
	ledandkey.controller.dio_pin.Assign(PORTNUM_A, 7, false);
	ledandkey.Init();
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
	ledandkey.Run();
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

	ledandkey.leds = hbcounter;

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
	TRACE("LED & KEY test, board: %s\r\n", BOARD_NAME);

	SysTick_Config(clockspeed / 1000);

	TRACE("clockcnt_t bits = %u\r\n", sizeof(clockcnt_t)*8);

#if CLOCKCNT_BITS == 16
	TRACE("16 bit clock timer!\r\n Max. time gap = %u/10 us\r\n", 65535 * 10000 / SystemCoreClock);
#endif

	// timed loop without using interrupts

	TRACE("Starting main cycle...\r\n");

	unsigned t0, t1;

	t0 = CLOCKCNT;

	unsigned mscnt = 0;
	unsigned cnt = 0;
	unsigned msclocks = SystemCoreClock / 1000;

	unsigned ms0 = mscnt;

	unsigned char prevkeys = 0;
	unsigned prevscannum = 0;

	// Infinite loop
	while (1)
	{
		idle_task();

#if 1
		t1 = CLOCKCNT;
		if (ELAPSEDCLOCKS(t1, t0) > msclocks)
		{
			// count milliseconds
			++mscnt;
			t0 += msclocks;
		}
#else
		mscnt = systick;
#endif

		if (mscnt - ms0 > 1000)
		{
			//TRACE("TIM14 = %u\r\n", TIM14->CNT);
			heartbeat_task();
			ms0 = mscnt;
		}

		++cnt;

		if (prevkeys != ledandkey.keys)
		{
			TRACE("Keys changed to %02X\r\n", ledandkey.keys);
			prevkeys = ledandkey.keys;
		}

		if (prevscannum != ledandkey.controller.scancounter)
		{
			ledandkey.DisplayDecNum(cnt);
			prevscannum = ledandkey.controller.scancounter;
		}
	}

}

// ----------------------------------------------------------------------------
