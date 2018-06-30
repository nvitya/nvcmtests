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
 *  file:     qspiflashtest.cpp
 *  brief:    Testing routines for a QSPI Flash memory
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwpins.h"
#include "hwspi.h"
#include "traces.h"
#include "qspiflash.h"

TQspiFlash  qspiflash;

//unsigned readlen = 256;

__attribute__((aligned(16)))
unsigned char qdatabuf[8192];

extern void show_mem(void * addr, unsigned len);

void qspi_flash_test()
{
	int i;
	unsigned addr = 0x00000;  // start at 128 k
	unsigned len = 64;

	TRACE("QSPI Flash Test\r\n");

	qspiflash.qspi.speed = 8000000;
	qspiflash.qspi.multi_line_count = 1;
	qspiflash.has4kerase = true;
	if (!qspiflash.Init())
	{
		TRACE("QSPI Flash init failed!\r\n");
		return;
	}

	TRACE("QSPI Flash initialized, ID CODE = %06X, kbyte size = %u\r\n", qspiflash.idcode, (qspiflash.bytesize >> 10));

#if 1
	TRACE("Erasing whole chip...\r\n");
	qspiflash.StartEraseAll();
	qspiflash.WaitForComplete();
	TRACE("Erase complete.\r\n");
#endif

	TRACE("Reading memory...\r\n");

	for (i = 0; i < sizeof(qdatabuf); ++i)	qdatabuf[i] = 0x55 + i;

	qspiflash.StartReadMem(addr, &qdatabuf[0], len);
	qspiflash.WaitForComplete();
	show_mem(&qdatabuf[0], len);

	TRACE("Writing bigger memory...\r\n");

	for (i = 0; i < sizeof(qdatabuf); ++i)
	{
		qdatabuf[i] = 0xF0 + i;
	}

	qspiflash.StartWriteMem(addr, &qdatabuf[0], sizeof(qdatabuf));
	qspiflash.WaitForComplete();
	TRACE("Write completed.\n\r");

	qspiflash.StartReadMem(addr, &qdatabuf[0], len);
	qspiflash.WaitForComplete();
	show_mem(&qdatabuf[0], len);

	TRACE("Testing short writes\n\r");

	addr = 0x00;

	while (addr < qspiflash.bytesize)
	{
		qspiflash.StartWriteMem(addr, &qdatabuf[0], 16);
		qspiflash.WaitForComplete();

		addr += 8192;
	}

	TRACE("Writes complete. \n\r");

	TRACE("Scanning for empty sectors...\r\n");

  addr = 0;
  unsigned usedcnt = 0;
  bool isempty;

  while (addr < qspiflash.bytesize)
  {
  	qspiflash.StartReadMem(addr, &qdatabuf[0], 0x1000);
  	qspiflash.WaitForComplete();

  	// check if sector is empty
  	isempty = true;
  	for (i = 0; i < 0x1000; ++i)
  	{
  		if (qdatabuf[i] != 0xFF)
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


	TRACE("\r\nTest finished.\r\n");
}
