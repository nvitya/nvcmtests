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

#include "cpu_tests_asm.h"

#include "traces.h"

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
#undef USE_DWT_CYCCNT

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

#define SKIP_DTCRAM_EXEC_TEST

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

#define SKIP_DTCRAM_EXEC_TEST

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

	// USART1
	//hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART1_TX
	//hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_0);  // USART1_RX
	//conuart.Init(1);

	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_0 | PINCFG_PULLUP);  // USART2_RX
	conuart.Init(2);
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


#if defined(BOARD_XPRESSO_LPC4337)

TGpioPin  led1pin(3, 5, true);
TGpioPin  led2pin(0, 7, true);
TGpioPin  led3pin(3, 7, true);

#define LED_COUNT 3

void setup_board()
{
	// RGB LED
	hwpinctrl.PinSetup(6,  9, PINCFG_OUTPUT | PINCFG_DRIVE_WEAK | PINCFG_AF_0);  // GPIO_3_5
	hwpinctrl.PinSetup(2,  7, PINCFG_OUTPUT | PINCFG_DRIVE_WEAK | PINCFG_AF_0);  // GPIO_0_7
	hwpinctrl.PinSetup(6, 11, PINCFG_OUTPUT | PINCFG_DRIVE_WEAK | PINCFG_AF_0);  // GPIO_3_7

	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART
	hwpinctrl.PinSetup(6, 4, PINCFG_OUTPUT | PINCFG_AF_2);  // UART0_TXD
	hwpinctrl.PinSetup(2, 1, PINCFG_INPUT  | PINCFG_AF_1);  // UART0_RXD
	conuart.Init(0);
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

#if defined(BOARD_MIBO100_LPC546XX)

TGpioPin  led1pin(1, 3, false);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(0, 30, PINCFG_OUTPUT | PINCFG_AF_1); // UART_TX:
	hwpinctrl.PinSetup(0, 29, PINCFG_INPUT  | PINCFG_AF_1); // UART_RX:
	conuart.Init(0);

}

#endif

// ---------------------------------------------------------------------------------------

#if defined(BOARD_NONE_STM32F301)

TGpioPin  led1pin(PORTNUM_A, 0, true);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX, pin 19
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX, pin 20
	conuart.Init(1);

	//// USART2
	//hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX, pin 9
	//hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX, pin 10
	//conuart.Init(2);
}

#endif

#if defined(BOARD_NONE_MKV30F)

// QFN-32 in an adaptor

TGpioPin  led1pin(PORTNUM_B, 0, true);  // pin 20

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART0
	hwpinctrl.PinSetup(PORTNUM_D, 7,  PINCFG_OUTPUT | PINCFG_AF_3);  // USART0_TX, pin 32
	hwpinctrl.PinSetup(PORTNUM_D, 6,  PINCFG_INPUT  | PINCFG_AF_3);  // USART0_RX, pin 31
	conuart.Init(0);
}

#endif

#if defined(BOARD_NONE_LPC822)

TGpioPin  led1pin(0, 15, false);  // pin 11 / TSOP-20

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinFuncConnect(SWM_FUNC_U0_RXD, 0, 0);  // pin 19 / TSOP-20
	hwpinctrl.PinFuncConnect(SWM_FUNC_U0_TXD, 0, 4);  // pin  6 / TSOP-20
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
#if LED_COUNT > 3
	led4pin.SetTo(hbcounter >> 3);
#endif
#if LED_COUNT > 4
	led5pin.SetTo(hbcounter >> 4);
#endif

	TRACE("hbcounter = %u, systick = %u\r\n", hbcounter, systick);

	//conuart.TrySendChar(0x55);
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
	TRACE("NVCM UART TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

#if defined(CMCC)
	// enable cache
	CMCC->CTRL.bit.CEN = 0;
#endif

	TRACE("\r\nINSTRUCTION CACHE DISABLED.\r\n");
#if __ICACHE_PRESENT
	SCB_DisableICache();
#endif
	test_code_speed();

#if __ICACHE_PRESENT
	TRACE("\r\nINSTRUCTION CACHE ENABLED.\r\n");
	SCB_EnableICache();
	test_code_speed();
#endif

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
