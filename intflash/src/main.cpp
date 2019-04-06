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
 *  file:     main.cpp (intflash)
 *  brief:    Multi-board Internal Flash write example for NVCM
 *  version:  1.00
 *  date:     2019-04-06
 *  authors:  nvitya
*/

#include "stdlib.h"
#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"

#include "traces.h"

#include "hwintflash.h"

THwUart            conuart;  // console uart

volatile unsigned  systick = 0;
unsigned           hbcounter = 0;

extern "C" void SysTick_Handler(void)  // IRQ
{
	++systick;
}

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

#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.Init(0);
}

#endif

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

#if defined(BOARD_DISCOVERY_F072)

TGpioPin  led1pin(PORTNUM_C, 6, false);
TGpioPin  led2pin(PORTNUM_C, 8, false);
TGpioPin  led3pin(PORTNUM_C, 9, false);
TGpioPin  led4pin(PORTNUM_C, 7, false);

#define LED_COUNT 4
#undef USE_DWT_CYCCNT

void setup_board()
{
	// direction leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
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
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led5pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(1,  2, PINCFG_OUTPUT | PINCFG_AF_7);  // UART_TX
	hwpinctrl.PinSetup(1,  3, PINCFG_INPUT  | PINCFG_AF_1);  // UART_RX
	conuart.Init(0x001);  // usic_0_ch_1
}

#endif

#if defined(BOARD_DEV_STM32F407VG)

TGpioPin  led1pin(4, 0, true);  // PE0

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
}

#endif

#if defined(BOARD_DEV_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

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

#if defined(BOARD_ARDUINO_DUE)

TGpioPin  led1pin(1, 27, false); // D13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART - On the Arduino programmer interface
	hwpinctrl.PinSetup(0, 8, PINCFG_INPUT | PINCFG_AF_0);  // UART_RXD
	hwpinctrl.PinSetup(0, 9, PINCFG_OUTPUT | PINCFG_AF_0); // UART_TXD
	conuart.Init(0);  // UART
}

#endif

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(2, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

#if 1
	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_0);  // USART1_RX
	conuart.Init(1);
#else
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_0 | PINCFG_PULLUP);  // USART2_RX
	conuart.Init(2);
#endif

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
}
#endif

#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

unsigned clocks_to_us(unsigned aclocks)
{
	unsigned clocks_per_us = SystemCoreClock / 1000000;
	return (aclocks + (clocks_per_us >> 1)) / clocks_per_us;
}

void test_intflash()
{
	unsigned    t0, t1;
	uint32_t *  p1;
	uint32_t *  p2;
	int i;
	bool bok = true;

	uint32_t testaddr = hwintflash.start_address + (hwintflash.bytesize >> 1);
	TRACE("Test parameters:\r\n", testaddr);
	TRACE("  Address: 0x%08X\r\n", testaddr);

	// the test length depends on how much RAM we can allocate

	uint32_t testlen  = (hwintflash.bytesize >> 1);
	uint8_t * pbuf = nullptr;
	while (1)
	{
		pbuf = (uint8_t *)malloc(testlen);
		if (pbuf)
		{
			break;
		}
		else
		{
			testlen = (testlen >> 1); // try the half
		}
	}

	TRACE("  length : %u k\r\n", testlen >> 10);

	uint32_t testdwcnt = (testlen >> 2);

	// first erase the memory
	TRACE("Erasing test area...\r\n");
	t0 = CLOCKCNT;
	hwintflash.StartEraseMem(testaddr, testlen);
	hwintflash.WaitForComplete();
	t1 = CLOCKCNT;
	TRACE("  Erase time = %u us\r\n", clocks_to_us(t1 - t0));

	TRACE("Verifying...\r\n");

	p1 = (uint32_t *)(testaddr);
	for (i = 0; i < testdwcnt; ++i)
	{
		if (*p1 != 0xFFFFFFFF)
		{
			TRACE("  Not erased at 0x%08X !\r\n", p1);
			bok = false;
		}
		++p1;
	}

	if (bok)
	{
		TRACE("  OK.\r\n");

		TRACE("Filling test area...\r\n");

		p1 = (uint32_t *)(pbuf);
		for (i = 0; i < testdwcnt; ++i)
		{
			*p1++ = 0x04030201 * i;
		}

		t0 = CLOCKCNT;
		hwintflash.StartWriteMem(testaddr, pbuf, testlen);
		hwintflash.WaitForComplete();
		t1 = CLOCKCNT;
		TRACE("  Write time = %u us\r\n", clocks_to_us(t1 - t0));

		TRACE("Verifying...\r\n");

		p1 = (uint32_t *)(pbuf);
		p2 = (uint32_t *)(testaddr);
		for (i = 0; i < testdwcnt; ++i)
		{
			if (*p1++ != *p2++)
			{
				bok = false;
				TRACE("  Mismatch at 0x%08X !\r\n", p2);
				break;
			}
		}
	}

	if (bok)
	{
		TRACE("  OK.\r\n");

		TRACE("Testing Copy...\r\n");

		// prepare another content
		p1 = (uint32_t *)(pbuf);
		for (i = 0; i < testdwcnt; ++i)
		{
			*p1++ = 0x01010101 * i;
		}

		t0 = CLOCKCNT;
		hwintflash.StartCopyMem(testaddr, pbuf, testlen);
		hwintflash.WaitForComplete();
		t1 = CLOCKCNT;
		TRACE("  Copy time = %u us\r\n", clocks_to_us(t1 - t0));

		TRACE("Verifying...\r\n");

		p1 = (uint32_t *)(pbuf);
		p2 = (uint32_t *)(testaddr);
		for (i = 0; i < testdwcnt; ++i)
		{
			if (*p1++ != *p2++)
			{
				bok = false;
				TRACE("  Mismatch at 0x%08X !\r\n", p2);
				break;
			}
		}
	}

	if (bok)
	{
		TRACE("  OK.\r\n");
	}

	free(pbuf);
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
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM Internal Flash Write Test\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	if (!hwintflash.Init())
	{
		TRACE("Error initializing internal flash !\r\n");
		led1pin.Set1();
		while (1) { };
	}

	hwintflash.TraceFlashInfo();

	test_intflash();
	TRACE("Reapeating Write Test\r\n");
	test_intflash();

	TRACE("\r\nStarting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;


		if (t1-t0 > hbclocks)
		{
			// heartbeat

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

			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
