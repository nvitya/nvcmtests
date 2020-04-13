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
 *  file:     main.cpp (USB special test)
 *  brief:    Special USB Device (multi-board)
 *  version:  1.00
 *  date:     2020-03-11
 *  authors:  nvitya
 *  notes:
 *   The "definition" of this special USB device is originated from Niklas Gürtler:
 *     https://github.com/Erlkoenig90/f1usb
 *     https://www.mikrocontroller.net/articles/USB-Tutorial_mit_STM32
 *   This implementation is compatible with the f1usb-master from Niklas Gürtler.
 *
 *   The device does not provide a standard interface, it can be controller with libusb.
 *   The project for testing it: https://github.com/nvitya/nvcmtests/usbclient_spec
 *   Or using the one from Niklas Gürtler: https://github.com/Erlkoenig90/usbclient
*/

#include <usb_spec_test.h>
#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "hwusbctrl.h"
#include "traces.h"

THwUart   conuart;  // console uart

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.Init(0);
}

#endif

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

#define LED_COUNT 3

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

  // USART3: Stlink USB / Serial converter
	hwpinctrl.PinSetup(3, 8,  PINCFG_OUTPUT | PINCFG_AF_7); // USART3_TX: PD.8
	hwpinctrl.PinSetup(3, 9,  PINCFG_INPUT  | PINCFG_AF_7); // USART3_RX: PD.9
	conuart.Init(3); // USART3

	// USB_OTG_FS PINS
	hwpinctrl.PinSetup(PORTNUM_A, 11, PINCFG_INPUT | PINCFG_AF_10 | PINCFG_SPEED_FAST);  // USB_OTG_FS DM
	hwpinctrl.PinSetup(PORTNUM_A, 12, PINCFG_INPUT | PINCFG_AF_10 | PINCFG_SPEED_FAST);  // USB_OTG_FS DP
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
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(2, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_0);  // USART1_RX
	conuart.Init(1);
}
#endif

#if defined(BOARD_MIBO48_STM32F303)

TGpioPin  led1pin(PORTNUM_C, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USB PINS
	hwpinctrl.PinSetup(PORTNUM_A, 11, PINCFG_INPUT | PINCFG_AF_14 | PINCFG_SPEED_FAST);  // USB DM
	hwpinctrl.PinSetup(PORTNUM_A, 12, PINCFG_INPUT | PINCFG_AF_14 | PINCFG_SPEED_FAST);  // USB DP

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
}
#endif

#if defined(BOARD_MIBO48_STM32G473)

TGpioPin  led1pin(PORTNUM_C, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USB PINS
	hwpinctrl.PinSetup(PORTNUM_A, 11, PINCFG_INPUT | PINCFG_AF_14 | PINCFG_SPEED_FAST);  // USB DM
	hwpinctrl.PinSetup(PORTNUM_A, 12, PINCFG_INPUT | PINCFG_AF_14 | PINCFG_SPEED_FAST);  // USB DP

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
}
#endif

#if defined(BOARD_MIBO64_STM32F070)

TGpioPin  led1pin(PORTNUM_C, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USB PINS
	//hwpinctrl.PinSetup(PORTNUM_A, 11, PINCFG_INPUT | PINCFG_AF_0 | PINCFG_SPEED_FAST);  // USB DM
	//hwpinctrl.PinSetup(PORTNUM_A, 12, PINCFG_INPUT | PINCFG_AF_0 | PINCFG_SPEED_FAST);  // USB DP

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_1);  // USART1_RX
	conuart.Init(1);
}
#endif

// ---------------------------------------------------------------------------------------

#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

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
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();
	traces_init();
	tracebuf.waitsend = true;  // start with blocking mode

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM USB SPEC TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	TRACE_FLUSH();

	usb_spec_test_init();

	TRACE("\r\nStarting main cycle...\r\n");

	tracebuf.waitsend = false;  // go to buffered mode
	//tracebuf.waitsend = true;  // better for basic debugging

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		tracebuf.Run(); // send out buffered characters

		usb_spec_test_run();

		if (t1-t0 > hbclocks)
		{
			// heartbeat task
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
