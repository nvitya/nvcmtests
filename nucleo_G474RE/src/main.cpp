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

#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "traces.h"
#include "hwdma.h"
#include "string.h"

#include "cpu_tests_asm.h"

extern void qspi_test();

THwUart   conuart;  // console uart

#if defined(BOARD_NUCLEO_G474RE)

TGpioPin  led1pin(PORTNUM_A, 5, false);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

#if 0
  // LPUART1: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_A, 2,  PINCFG_OUTPUT | PINCFG_AF_12);  // LPUART1.TX
	hwpinctrl.PinSetup(PORTNUM_A, 3,  PINCFG_INPUT  | PINCFG_AF_12);  // LPUART1.RX
	conuart.Init(0x101); // 0x101 = LPUART1, 0x001 = USART1
#else
  // USART2: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_A, 2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART2.TX
	hwpinctrl.PinSetup(PORTNUM_A, 3,  PINCFG_INPUT  | PINCFG_AF_7);  // USART2.RX
	conuart.Init(2);
#endif
}

#endif

#if defined(BOARD_NUCLEO_G431KB)

TGpioPin  led1pin(PORTNUM_B, 8, false);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

#if 0
  // LPUART1: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_A, 2,  PINCFG_OUTPUT | PINCFG_AF_12);  // LPUART1.TX
	hwpinctrl.PinSetup(PORTNUM_A, 3,  PINCFG_INPUT  | PINCFG_AF_12);  // LPUART1.RX
	conuart.Init(0x101); // 0x101 = LPUART1, 0x001 = USART1
#else
  // USART2: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_A, 2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART2.TX
	hwpinctrl.PinSetup(PORTNUM_A, 3,  PINCFG_INPUT  | PINCFG_AF_7);  // USART2.RX
	conuart.Init(2);
#endif
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

	//swo_printf("hbcounter = %i\r\n", hbcounter);
}

void test_code_speed()
{
	unsigned r1, r2, r3;

#if __CORTEX_M >= 3
	TRACE("Testing code execution speed:\r\nInstruction clocks for 64 x single cycle 32bit:\r\n");

	r1 = linear_run_asm();
	r2 = linear_run_asm();
	r3 = linear_run_asm();
	TRACE("Normal: %3u, %3u, %3u\r\n", r1, r2, r3);

	r1 = linear_run_asm_itcram();
	r2 = linear_run_asm_itcram();
	r3 = linear_run_asm_itcram();
	TRACE("ITCRAM: %3u, %3u, %3u\r\n", r1, r2, r3);

#ifndef SKIP_DTCRAM_EXEC_TEST
	r1 = linear_run_asm_dtcram();
	r2 = linear_run_asm_dtcram();
	r3 = linear_run_asm_dtcram();
	TRACE("DTCRAM: %3u, %3u, %3u\r\n", r1, r2, r3);
#endif

	r1 = linear_run_asm_ram2();
	r2 = linear_run_asm_ram2();
	r3 = linear_run_asm_ram2();
	TRACE("RAM2  : %3u, %3u, %3u\r\n", r1, r2, r3);
#endif

#if __CORTEX_M >= 3

	TRACE("Instruction clocks for 64 x single cycle 16bit:\r\n");

	r1 = linear_run_asm_m0((uint32_t *)&CLOCKCNT);
	r2 = linear_run_asm_m0((uint32_t *)&CLOCKCNT);
	r3 = linear_run_asm_m0((uint32_t *)&CLOCKCNT);
	TRACE("Flash:  %3u, %3u, %3u\r\n", r1, r2, r3);

#if 1
	r1 = linear_run_asm_ram_m0((uint32_t *)&CLOCKCNT);
	r2 = linear_run_asm_ram_m0((uint32_t *)&CLOCKCNT);
	r3 = linear_run_asm_ram_m0((uint32_t *)&CLOCKCNT);
	TRACE("RAM:    %3u, %3u, %3u\r\n", r1, r2, r3);
#endif

#endif
}

const char dma_test_string[] = "This text was sent using DMA!\r\n";

void test_uart_dma_tx()
{
	THwDmaChannel dmach;
	THwDmaTransfer xfer;

	dmach.Init(2, 1, 27);  // chnum=1..8, 27: USART2_TX

	conuart.DmaAssign(true, &dmach);

	xfer.srcaddr = (void *)&dma_test_string[0];
	xfer.bytewidth = 1;
	xfer.count = strlen(&dma_test_string[0]);
	xfer.flags = 0;

	TRACE("Testing DMA...\r\n");
	conuart.DmaStartSend(&xfer);
	while (dmach.Active())
	{
		// wait until finishes
	}
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_disable_interrupts();

	SYSCFG->MEMRMP = 3; // remap SRAM1 to 0x00000000

  mcu_preinit_code(); // inline code for preparing the MCU, RAM regions. Without this even the stack does not work on some MCUs.

	// Set the interrupt vector table offset, so that the interrupts and exceptions work
	mcu_init_vector_table();

  unsigned clockspeed = MCU_CLOCK_SPEED;

#ifdef MCU_INPUT_FREQ
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, clockspeed))  // activate the external crystal oscillator with multiplication x2
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, clockspeed))  // activate the external crystal oscillator with multiplication x2
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
	TRACE("NVCM UART TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	TRACE("Flash DBANK=%i\r\n", (FLASH->OPTR >> 22) & 1);
	//|= (FLASH_OPTR_DBANK);  // disable dual bank mode

	test_code_speed();

	test_uart_dma_tx();

	qspi_test();

	mcu_enable_interrupts();

	SysTick_Config(SystemCoreClock / 1000);

	TRACE("\r\nStarting main cycle...\r\n");

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
