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
 *  file:     spiflashtest.cpp
 *  brief:    Testing routines for an SPI Flash memory
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwpins.h"
#include "hwspi.h"
#include "traces.h"
#include "spiflash.h"

TSpiFlash  spiflash;

unsigned readlen = 256;

unsigned char databuf[8192];

void show_mem(void * addr, unsigned len)
{
	unsigned char * cp = (unsigned char *)addr;
	TRACE("Dumping memory at %08X, len = %u\r\n", addr, len);
	for (unsigned n = 0; n < len; ++n)
	{
		TRACE(" %02X", *cp++);
		if (n % 16 == 15) TRACE("\r\n");
	}
	TRACE("\r\n");
}

void spi_flash_test()
{
	int i;
	unsigned addr;

	TRACE("SPI Flash Test\r\n");

#ifdef BOARD_XPLORER_LPC4330
	// SSP1 setup
	hwpinctrl.PinSetup(1,  4, PINCFG_OUTPUT | PINCFG_AF_5);  // MOSI
	hwpinctrl.PinSetup(1,  3, PINCFG_INPUT  | PINCFG_AF_5);  // MISO
	//hwpinctrl.PinSetup(1,  5, PINCFG_OUTPUT | PINCFG_AF_5);  // SSEL
	hwpinctrl.PinSetup(1,  5, PINCFG_OUTPUT | PINCFG_AF_0);  // SSEL, as GPIO_1_8
	hwpinctrl.PinSetup(256,  0, PINCFG_OUTPUT | PINCFG_AF_6);  // CLK0!, SCK

	spiflash.pin_cs.Assign(1, 8, false);
	spiflash.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	//spiflash.spi.datasample_late = false;
	spiflash.spi.speed = 16000000;
	spiflash.spi.Init(1);

	spiflash.txdma.Init(0x030301);  // ch1, dmamux 3/3
	spiflash.rxdma.Init(0x030402);  // ch2, dmamux 4/3

#elif defined(BOARD_MIBO100_ATSAME70)

	// SPI0 setup

	unsigned pinflags = PINCFG_OUTPUT | PINCFG_AF_B | PINCFG_PULLUP;

	hwpinctrl.PinSetup(PORTNUM_D, 21, pinflags);  // MOSI
	hwpinctrl.PinSetup(PORTNUM_D, 20, pinflags);  // MISO
	hwpinctrl.PinSetup(PORTNUM_D, 22, pinflags);  // SCK

	spiflash.pin_cs.Assign(PORTNUM_D, 25, false);
	spiflash.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	//spiflash.spi.datasample_late = false;
	spiflash.spi.speed = 8000000;
	spiflash.spi.Init(0);

	spiflash.txdma.Init(1, 1);  // perid = 1: SPI0_TX
	spiflash.rxdma.Init(2, 2);  // perid = 2: SPI0_RX

#elif defined(BOARD_DEV_STM32F407ZE)

	// SPI Flash on the F407ZE board

	spiflash.pin_cs.Assign(PORTNUM_B, 14, false);
	spiflash.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_B, 3, PINCFG_AF_5);  // SPI1_SCK    = SWO !!!!!!!!!!!!!!!!!!!!!
	hwpinctrl.PinSetup(PORTNUM_B, 4, PINCFG_AF_5);  // SPI1_MISO
	hwpinctrl.PinSetup(PORTNUM_B, 5, PINCFG_AF_5);  // SPI1_MOSI

	spiflash.spi.speed = 16000000;
	spiflash.spi.Init(1);

	spiflash.txdma.Init(0x020503);  // dma2/stream5/ch3
	spiflash.rxdma.Init(0x020003);  // dma2/stream0/ch3

#else
  #error "Unknown board!"
#endif

	spiflash.has4kerase = true;
	if (!spiflash.Init())
	{
		TRACE("SPI Flash init failed!\r\n");
		//return;
	}

	TRACE("SPI Flash initialized, ID CODE = %06X, kbyte size = %u\r\n", spiflash.idcode, (spiflash.bytesize >> 10));

/*
	TRACE("Reading memory...\r\n");

	spiflash.StartReadMem(0, &databuf[0], readlen);
	spiflash.WaitForComplete();

	TRACE("Memory read finished\r\n");

	show_mem(&databuf[0], readlen);

	return;
*/

	//TRACE("Issuing reset...\r\n");
	//spiflash.ResetChip();

	return;

	TRACE("Erasing whole chip...\r\n");
	spiflash.StartEraseAll();
	spiflash.WaitForComplete();
	TRACE("Erase complete.\r\n");

	// try to clear the busy flag
/*
	spiflash.txbuf[0] = 0x06; // write enable
	spiflash.ExecCmd(1);

	spiflash.txbuf[0] = 0x01; // write status register
	spiflash.txbuf[1] = 0x00; // clear busy flag
	spiflash.txbuf[2] = 0x00;
	spiflash.ExecCmd(3);
*/

/*
	TRACE("Read status...\r\n");
	spiflash.StartReadStatus();
	while (!spiflash.spi.DmaRecvCompleted())
	{
		// wait
	}
	spiflash.pin_cs.Set();
	TRACE("STATUS REG = %02X\r\n", spiflash.rxbuf[1]);
*/

	TRACE("Reading memory...\r\n");

	spiflash.StartReadMem(0, &databuf[0], readlen);
	spiflash.WaitForComplete();

	TRACE("Memory read finished\r\n");

	TRACE("Writing memory...\r\n");

	for (i = 0; i < sizeof(databuf); ++i)
	{
		databuf[i] = 0xF0 + i;
	}

	spiflash.StartWriteMem(0x20000, &databuf[0], sizeof(databuf));
	spiflash.WaitForComplete();

	TRACE("Write completed.\n\r");

	TRACE("Testing short writes\n\r");

	addr = 0x16;

	while (addr < spiflash.bytesize)
	{
		spiflash.StartWriteMem(addr, &databuf[0], 16);
		spiflash.WaitForComplete();

		addr += 9320;
	}

	TRACE("Writes complete. \n\r");

/*

	return;
*/

	TRACE("Scanning for empty sectors...\r\n");

  addr = 0;
  unsigned usedcnt = 0;
  bool isempty;

  while (addr < spiflash.bytesize)
  {
  	spiflash.StartReadMem(addr, &databuf[0], 0x1000);
  	spiflash.WaitForComplete();

  	// check if sector is empty
  	isempty = true;
  	for (i = 0; i < 0x1000; ++i)
  	{
  		if (databuf[i] != 0xFF)
  		{
  			++usedcnt;
  			isempty = false;
  			break;
  		}
  	}

  	TRACE(isempty ? "1" : "0");

  	addr += 0x1000;
  }

  TRACE("\r\n%i used sectors.\r\n", usedcnt);

/*
	TRACE("DATA:\r\n");

	for (int i = 0; i < readlen; ++i)
	{
		//TRACE(" %02X", rmbuf[i]);
		scr.printf(" %02X", databuf[i]);  // do not call update after every byte!
	}
	TRACE("\r\n\r\n");
*/

		TRACE("\r\nTest finished.\r\n");

}
