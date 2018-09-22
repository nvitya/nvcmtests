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
 *  file:     main.cpp (serialflash)
 *  brief:    Multi-board serial SPI and QSPI Flash memory test for NVCM
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include "stdio.h"
#include "string.h"
#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "cppinit.h"
#include "clockcnt.h"

#include "traces.h"

#include "spiflashtest.h"
#include "qspiflashtest.h"

THwUart   conuart;  // console uart

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
	// USART3_TX: PD.8
	hwpinctrl.PinSetup(3, 8,  PINCFG_OUTPUT | PINCFG_AF_7);
	// USART3_RX: Pd.9
	hwpinctrl.PinSetup(3, 9,  PINCFG_INPUT  | PINCFG_AF_7);

	conuart.Init(3); // USART3
}

#endif

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
}

#endif


#if defined(BOARD_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
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

	//// USART2
	//hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART2_TX
	//hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_0 | PINCFG_PULLUP);  // USART2_RX
	//conuart.Init(2);
}
#endif

#if defined(BOARD_XPLORER_LPC4330)

TGpioPin  led1pin(1, 12, true); // D2
TGpioPin  led2pin(1, 11, true); // D3

#define LED_COUNT 2

void setup_board()
{
	hwpinctrl.PinSetup(2, 12, PINCFG_OUTPUT | PINCFG_AF_0);  // D2: GPIO_1_12, pad B9
	hwpinctrl.PinSetup(2, 11, PINCFG_OUTPUT | PINCFG_AF_0);  // D3: GPIO_1_11, pad A9

	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_VERTIBO_A)

TGpioPin  led1pin(PORTNUM_A, 29, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.baudrate = 115200;
	conuart.Init(0);
}

#endif

#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART0
	hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_INPUT  | PINCFG_AF_0); // UART0_RXD
	hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_OUTPUT | PINCFG_AF_0); // UART0_TXD (Marked as B10 between B1 and D31 !!!)
	conuart.Init(0x000); // UART0
}

#endif

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART0
	hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_INPUT  | PINCFG_AF_0); // UART0_RXD
	hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_OUTPUT | PINCFG_AF_0); // UART0_TXD (Marked as B10 between B1 and D31 !!!)
	conuart.Init(0x000); // UART0
}

#endif

#if defined(BOARD_DEV_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

#define LED_COUNT 2

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
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

#if defined(BOARD_MIBO100_LPC540)

TGpioPin  led1pin(1, 3, true);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

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

	//swo_printf("hbcounter = %i\r\n", hbcounter);
}

extern unsigned int __stack;  // defined in the linker script

extern "C" void _start(void)
{
	asm("ldr r0, =__stack");
	asm("mov sp, r0");
	asm("ldr r0, =main");
	asm("bx r0");
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void main(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_disable_interrupts();

	// Set the interrupt vector table offset, so that the interrupts and exceptions work
	mcu_init_vector_table();

  mcu_preinit_code(); // inline code for preparing the MCU, RAM regions. Without this even the stack does not work on some MCUs.

  unsigned clockspeed = MAX_CLOCK_SPEED;
  //unsigned clockspeed = 4000000;

#ifdef MCU_INPUT_FREQ
  //if (false)
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, MAX_CLOCK_SPEED))  // activate the external crystal oscillator with multiplication x2
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, MAX_CLOCK_SPEED))  // activate the external crystal oscillator with multiplication x2
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

	//mcu_enable_fpu();    // enable coprocessor if present
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	//mcu_enable_interrupts();

	TRACE("\r\n------------------------------------------\r\n");
	TRACE("Serial Flash Test\r\n");

	//uart_dma_test();

	//spi_flash_test();
	qspi_flash_test();

	TRACE("Starting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	unsigned hbclocks = SystemCoreClock / 20;  // start blinking fast

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

			if (hbcounter > 20)  hbclocks = SystemCoreClock / 2;  // slow down to 0.5 s
		}
	}
}

// ----------------------------------------------------------------------------
