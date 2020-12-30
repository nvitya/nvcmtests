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
#include "storman_sdcard.h"
#include "filesys_fat.h"

#include "traces.h"

THwUart         conuart;  // console uart

THwSdcard       sdcard;

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

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746) || defined(BOARD_NUCLEO_H743)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

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

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_C,  8, PINCFG_AF_12); // SDMMC_D0
	hwpinctrl.PinSetup(PORTNUM_C,  9, PINCFG_AF_12); // SDMMC_D1
	hwpinctrl.PinSetup(PORTNUM_C, 10, PINCFG_AF_12); // SDMMC_D2
	hwpinctrl.PinSetup(PORTNUM_C, 11, PINCFG_AF_12); // SDMMC_D3
	hwpinctrl.PinSetup(PORTNUM_C, 12, PINCFG_AF_12); // SDMMC_CK
	hwpinctrl.PinSetup(PORTNUM_D,  2, PINCFG_AF_12); // SDMMC_CMD

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

#if defined(BOARD_ENEBO_A)

TGpioPin  led1pin(PORTNUM_D, 13, true);
TGpioPin  led2pin(PORTNUM_D, 14, true);
TGpioPin  led3pin(PORTNUM_A, 20, true);

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

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_C,  8, PINCFG_AF_12); // SDMMC_D0
	hwpinctrl.PinSetup(PORTNUM_C,  9, PINCFG_AF_12); // SDMMC_D1
	hwpinctrl.PinSetup(PORTNUM_C, 10, PINCFG_AF_12); // SDMMC_D2
	hwpinctrl.PinSetup(PORTNUM_C, 11, PINCFG_AF_12); // SDMMC_D3
	hwpinctrl.PinSetup(PORTNUM_C, 12, PINCFG_AF_12); // SDMMC_CK
	hwpinctrl.PinSetup(PORTNUM_D,  2, PINCFG_AF_12); // SDMMC_CMD

	sdcard.Init();
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

int teststate = 0;

uint8_t  testbuf[4096] __attribute__((aligned(16)));

TMbrPtEntry     ptable[4];

uint32_t        fs_firstsector;
uint32_t        fs_maxsectors;

TStorManSdcard  storman;
TStorTrans      stra;
TFileSysFat     fatfs;

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

	TRACE("\r\n---------------------------------\r\n");
	TRACE("NVCM SDCARD + FAT Filesystem Test\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	int i;
	unsigned n;

#if 1 // not necessary, but the trace output is nicer and the debugging is easier
	TRACE("Waiting for SDCARD initialization...\r\n");

	while (!sdcard.card_initialized)
	{
		sdcard.Run();
	}
#endif

#if 0

	sdcard.StartReadBlocks(0x800, &testbuf[0], 2);
	while (!sdcard.completed)
	{
		sdcard.Run();
	}

	if (sdcard.errorcode)
	{
		TRACE("Read error!\r\n");
		teststate = 10;
	}
	else
	{
		TRACE("Read ok.\r\n");
		for (i = 0; i < 512*2; ++i)
		{
			if (i != 0)
			{
				if ((i % 16) == 0)  TRACE("\r\n");
				if ((i % 512) == 0) TRACE("\r\n");
			}

			TRACE(" %02X", testbuf[i]);
		}
		TRACE("\r\n");
	}

	__BKPT();
#endif

	storman.Init(&sdcard);

	fs_firstsector = 0;
	fs_maxsectors = 0;

	TRACE("Reading SDCARD partition table...\r\n");

	storman.AddTransaction(&stra, STRA_READ, 446, &ptable[0], 64);
	storman.WaitTransaction(&stra);
	if (stra.errorcode)
	{
		TRACE("Error reading partition table!\r\n");
	}
	else
	{
		TRACE("SDCARD Partition table:\r\n");
		for (n = 0; n < 4; ++n)
		{
			if ((fs_firstsector == 0) && (ptable[n].ptype != 0) && ptable[n].first_lba)
			{
				fs_firstsector = ptable[n].first_lba;
				fs_maxsectors = ptable[n].sector_count;
			}

			TRACE(" %u.: status=%02X, type=%02X, start=%08X, blocks=%i\r\n",
					n, ptable[n].status, ptable[n].ptype, ptable[n].first_lba, ptable[n].sector_count
			);
		}
	}

	if (fs_maxsectors)
	{
		TRACE("Initializing FAT FS at sector %i...\r\n", fs_firstsector);

		fatfs.Init(&storman, (fs_firstsector << 9), (fs_maxsectors << 9));
		while (!fatfs.initialized)
		{
			fatfs.Run();
		}

		if (fatfs.fsok)
		{
			TRACE("FAT file system initialized:\r\n");
			if (fatfs.fat32)  TRACE(" FAT32\r\n");
			TRACE(" cluster size: %u\r\n", fatfs.clusterbytes);
			TRACE(" total size: %u MByte\r\n", fatfs.databytes >> 20);

			TRACE("Reading the root directory...\r\n");

			storman.AddTransaction(&stra, STRA_READ, fatfs.firstaddr + fatfs.sysbytes, &testbuf[0], 4096);
			storman.WaitTransaction(&stra);
			if (stra.errorcode)
			{
				TRACE("Error reading the root directory!\r\n");
			}
			else
			{
				for (i = 0; i < 512; ++i)
				{
					if (i != 0)
					{
						if ((i % 16) == 0)  TRACE("\r\n");
						if ((i % 512) == 0) TRACE("\r\n");
					}

					TRACE(" %02X", testbuf[i]);
				}
				TRACE("\r\n");
			}
		}
	}

	TRACE("\r\nStarting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	unsigned rstart, rend;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		storman.Run();

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
