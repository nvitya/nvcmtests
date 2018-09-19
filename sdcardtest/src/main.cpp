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
 *  file:     main.cpp (ethernet)
 *  brief:    Multi-board ethernet example for NVCM
 *  version:  1.00
 *  date:     2018-05-30
 *  authors:  nvitya
*/

#include "string.h"
#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "hwsdcard.h"

#include "traces.h"

THwSdcard sdcard;

THwUart   conuart;  // console uart

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

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_A, 28, PINCFG_AF_2); // MCCDA
	hwpinctrl.PinSetup(PORTNUM_A, 25, PINCFG_AF_3); // MCCK
	hwpinctrl.PinSetup(PORTNUM_A, 30, PINCFG_AF_2); // MCDA0
	hwpinctrl.PinSetup(PORTNUM_A, 31, PINCFG_AF_2); // MCDA1
	hwpinctrl.PinSetup(PORTNUM_A, 26, PINCFG_AF_2); // MCDA2
	hwpinctrl.PinSetup(PORTNUM_A, 27, PINCFG_AF_2); // MCDA3

	hwpinctrl.PinSetup(PORTNUM_C, 16, PINCFG_INPUT | PINCFG_PULLUP); // Card detect input

	sdcard.dma.Init(9, 0); // 0 = HSMCI DMA Peripheral Id (Transmit and Receive)
	sdcard.Init();
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

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_A, 28, PINCFG_AF_2); // MCCDA
	hwpinctrl.PinSetup(PORTNUM_A, 25, PINCFG_AF_3); // MCCK
	hwpinctrl.PinSetup(PORTNUM_A, 30, PINCFG_AF_2); // MCDA0
	hwpinctrl.PinSetup(PORTNUM_A, 31, PINCFG_AF_2); // MCDA1
	hwpinctrl.PinSetup(PORTNUM_A, 26, PINCFG_AF_2); // MCDA2
	hwpinctrl.PinSetup(PORTNUM_A, 27, PINCFG_AF_2); // MCDA3

	sdcard.dma.Init(9, 0); // 0 = HSMCI DMA Peripheral Id (Transmit and Receive)
	sdcard.Init();
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

#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

volatile unsigned systick = 0;

extern "C" void SysTick_Handler(void)
{
	++systick;
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

#if 0

uint8_t sdcard_cid[16];

void sdcard_test()
{
	uint32_t n;
	uint32_t d;

	TRACE("SDCARD test\r\n");

	sdcard.SetSpeed(400000); // initial speed = 400 kHz
	sdcard.SetBusWidth(1);

	sdcard.SendSpecialCmd(SD_SPECIAL_CMD_INIT); // never fails
	while (!sdcard.CmdFinished()) { }

	sdcard.SendCmd(0, 0, SDCMD_RES_NO); // never fails
	while (!sdcard.CmdFinished()) { }

	while (1)
	{
		sdcard.SendCmd(55, 0, SDCMD_RES_48BIT | SDCMD_OPENDRAIN);
		while (!sdcard.CmdFinished()) { }
		if (sdcard.cmderror)
		{
			TRACE("Cmd_55 error!\r\n");
			return;
		}

		sdcard.SendCmd(41, 0x001f8000 | 0x40000000, SDCMD_RES_48BIT | SDCMD_OPENDRAIN);
		while (!sdcard.CmdFinished()) { }
		if (sdcard.cmderror)
		{
			TRACE("Cmd_41 error!\r\n");
			return;
		}

		d = sdcard.GetCmdResult32();
		//TRACE("OCR reg = %08X\r\n", d);

		if (d & 0x80000000)
		{
			// card is ready.
			break;
		}
	}

	sdcard.SendCmd(2, 0, SDCMD_RES_136BIT | SDCMD_OPENDRAIN);
	while (!sdcard.CmdFinished()) { }
	if (sdcard.cmderror)
	{
		TRACE("CID command error!\r\n");
		return;
	}
	else
	{
		TRACE("CID command ok.\r\n");
	}

	sdcard.GetCmdResult128(&sdcard_cid[0]);

	for (n = 0; n < 16; ++n)
	{
		TRACE(" %02X", sdcard_cid[n]);
	}
	TRACE("\r\n");
}

void idle_task()
{
}

#else

void idle_task()
{
	sdcard.Run();
}

void sdcard_test()
{

}

#endif

int teststate = 0;

uint8_t testbuf[16384] __attribute__((aligned(4)));
uint32_t testlen = 2048;
uint32_t testcnt = 0;

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
	TRACE("NVCM SDCARD TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	sdcard_test();

	TRACE("\r\nStarting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	unsigned rstart, rend;

	int i;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		idle_task();

		if (0 == teststate)
		{
			if (sdcard.card_initialized)
			{
				TRACE("APP: SD Card initialized.\r\n");
				teststate = 1;
			}
		}
		else if (1 == teststate)
		{
			// start block read
			//sdcard.StartReadBlocks(2561, &testbuf[0], testlen / 512);
			sdcard.StartReadBlocks(0, &testbuf[0], testlen / 512);
			rstart = CLOCKCNT;
			teststate = 2;
		}
		else if (2 == teststate)
		{
			// wait until read completed
			if (sdcard.completed)
			{
				if (sdcard.errorcode)
				{
					TRACE("Read error!\r\n");
					teststate = 10;
				}
				else
				{
					rend = CLOCKCNT;
					TRACE("Read ok, clocks = %u\r\n", rend - rstart);
#if 1
					for (i = 0; i < testlen; ++i)
					{
						if (i != 0)
						{
							if ((i % 16) == 0)  TRACE("\r\n");
							if ((i % 512) == 0) TRACE("\r\n");
						}

						TRACE(" %02X", testbuf[i]);
					}
					TRACE("\r\n");
#endif

					teststate = 10;
				}
			}
		}
		else if (10 == teststate)
		{
			TRACE("Test %i finished.\r\n", testcnt);
			// the end.
			++testcnt;
			if (testcnt < 2)
			{
				teststate = 1; // repeat
			}
			else
			{
				teststate = 11;
			}
		}
		else if (11 == teststate)
		{
			// stay here.
		}

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
