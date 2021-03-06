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
#include "string.h"
#include "hwpins.h"
#include "hwspi.h"
#include "traces.h"
#include "qspiflash.h"

TQspiFlash  qspiflash;

#define QSPI_SPEED       32000000
#define QSPI_LINE_COUNT         2

//unsigned readlen = 256;

__attribute__((aligned(16)))
unsigned char qdatabuf[256 * 1024];

extern void show_mem(void * addr, unsigned len);

void qspi_test_reliability()
{
	TRACE("QSPI reliablility test. speed=%u\r\n", qspiflash.qspi.speed);

	unsigned value;
	const unsigned begin_value = 0xc35a7291 + 1;
	const unsigned value_add = 33;
	uint32_t * dp;
	uint32_t * endp = (uint32_t *)&qdatabuf[sizeof(qdatabuf)];

	const unsigned test_length = 256 * 1024;

#if 1

	TRACE("Erasing the first %u k...\r\n", test_length / 1024);
	qspiflash.StartEraseMem(0, test_length);
	qspiflash.WaitForComplete();
	TRACE("Erase complete.\r\n");


	// fill the buffer:
	dp = (uint32_t *)&qdatabuf[0];
	value = begin_value;
	while (dp < endp)
	{
		*dp++ = value;
		value += value_add;
	}

	TRACE("Writing blocks...\r\n");
	qspiflash.StartWriteMem(0, &qdatabuf[0], sizeof(qdatabuf));
	qspiflash.WaitForComplete();

#endif

	TRACE("Reading back...\r\n");
	memset(&qdatabuf[0], 0, sizeof(qdatabuf));
	qspiflash.StartReadMem(0, &qdatabuf[0], sizeof(qdatabuf));
	qspiflash.WaitForComplete();

	TRACE("Comparing...\r\n");

	unsigned mismatch_cnt = 0;

	value = begin_value;
	dp = (uint32_t *)&qdatabuf[0];
	while (dp < endp)
	{
		if (*dp++ != value)
		{
			++mismatch_cnt;
		}
		value += value_add;
	}

	if (mismatch_cnt)
	{
		TRACE("ERROR! Mismatch count: %u\r\n", mismatch_cnt);
	}
	else
	{
		TRACE("Content check ok.\r\n");
	}

	TRACE("Test Finished.\r\n");
}

void qspi_flash_test()
{
	int i;
	unsigned addr = 0x00000;  // start at 128 k
	unsigned len = 64;

	TRACE("QSPI Flash Test\r\n");

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746) || defined(BOARD_DISCOVERY_F746)

	uint32_t qspipincfg = 0;

	hwpinctrl.PinSetup(PORTNUM_B,  6, qspipincfg | PINCFG_AF_10);  // NCS
	hwpinctrl.PinSetup(PORTNUM_B,  2, qspipincfg | PINCFG_AF_9);   // CLK

	hwpinctrl.PinSetup(PORTNUM_D, 11, qspipincfg | PINCFG_AF_9);   // IO0
	hwpinctrl.PinSetup(PORTNUM_D, 12, qspipincfg | PINCFG_AF_9);   // IO1

	hwpinctrl.PinSetup(PORTNUM_E,  2, qspipincfg | PINCFG_AF_9);   // IO2
	hwpinctrl.PinSetup(PORTNUM_D, 13, qspipincfg | PINCFG_AF_9);   // IO3

#elif	defined(BOARD_NUCLEO_H743)

	uint32_t qspipincfg = 0;

	hwpinctrl.PinSetup(PORTNUM_B,  6, qspipincfg | PINCFG_AF_10);  // NCS     !!!!!!!!!!!
	//hwpinctrl.PinSetup(PORTNUM_G,  6, qspipincfg | PINCFG_AF_10);  // NCS
	hwpinctrl.PinSetup(PORTNUM_B,  2, qspipincfg | PINCFG_AF_9);   // CLK

	hwpinctrl.PinSetup(PORTNUM_D, 11, qspipincfg | PINCFG_AF_9);   // IO0
	hwpinctrl.PinSetup(PORTNUM_D, 12, qspipincfg | PINCFG_AF_9);   // IO1

	hwpinctrl.PinSetup(PORTNUM_E,  2, qspipincfg | PINCFG_AF_9);   // IO2
	hwpinctrl.PinSetup(PORTNUM_D, 13, qspipincfg | PINCFG_AF_9);   // IO3

#endif

	qspiflash.qspi.speed = QSPI_SPEED;
	qspiflash.qspi.multi_line_count = QSPI_LINE_COUNT;
#if 1
	qspiflash.has4kerase = true;
	if (!qspiflash.Init())
	{
		TRACE("QSPI Flash init failed!\r\n");
		return;
	}
#else
	qspiflash.qspi.Init();
#endif

	TRACE("QSPI Flash initialized\r\n  ID CODE = %06X, kbyte size = %u\r\n", qspiflash.idcode, (qspiflash.bytesize >> 10));

	qspi_test_reliability();

	return;


#if 0
	//TRACE("QSPI Erase test...\r\n");
	qdatabuf[0] = 0x55;
	qdatabuf[1] = 0x33;
	qdatabuf[2] = 0xAA;
	qdatabuf[3] = 0xCC;

	qspiflash.qspi.StartWriteData(0x20, 0, &qdatabuf[0], 3);
	qspiflash.qspi.WaitFinish();
	TRACE("QSPI Erase test finished.\r\n");

	return;

#endif

#if 1
	TRACE("repeating ID code query...\r\n");
	if (qspiflash.ReadIdCode())
	{
		TRACE("  repeated ID CODE = %06X, kbyte size = %u\r\n", qspiflash.idcode, (qspiflash.bytesize >> 10));
	}
	else
	{
		TRACE("  error repeating ID code!\r\n");
	}
#endif

#if 1
	TRACE("Reading memory...\r\n");

	for (i = 0; i < sizeof(qdatabuf); ++i)	qdatabuf[i] = 0x55 + i;

	qspiflash.StartReadMem(addr, &qdatabuf[0], len);
	qspiflash.WaitForComplete();
	show_mem(&qdatabuf[0], len);

	//return;
#endif

	unsigned write_test_size = qspiflash.bytesize;
	if (write_test_size > 512 * 1024) // limit the erase time
	{
		write_test_size = 512 * 1024;
	}

#if 1
	TRACE("Erasing the first %u k...\r\n", write_test_size / 1024);
	qspiflash.StartEraseMem(0, write_test_size);
	qspiflash.WaitForComplete();
	TRACE("Erase complete.\r\n");

	#if 1
		TRACE("Erase check read...\r\n");

		qspiflash.StartReadMem(addr, &qdatabuf[0], len);
		qspiflash.WaitForComplete();
		show_mem(&qdatabuf[0], len);

		//return;
	#endif

#endif

	TRACE("Writing bigger memory...\r\n");

	for (i = 0; i < sizeof(qdatabuf); ++i)
	{
		qdatabuf[i] = 0xF0 + i;
	}

	addr = 0x00000;

	qspiflash.StartWriteMem(addr, &qdatabuf[0], sizeof(qdatabuf));
	qspiflash.WaitForComplete();
	TRACE("Write completed.\n\r");

	qspiflash.StartReadMem(addr, &qdatabuf[0], len);
	qspiflash.WaitForComplete();
	show_mem(&qdatabuf[0], len);

	TRACE("Testing short writes\n\r");

	addr = 0x0000;

	while (addr < write_test_size)
	{
		qspiflash.StartWriteMem(addr, &qdatabuf[0], 256);
		qspiflash.WaitForComplete();

		addr += 8192;
	}

	TRACE("Writes complete. \n\r");

	TRACE("Scanning for empty sectors...\r\n");

  addr = 0;
  unsigned usedcnt = 0;
  bool isempty;

  while (addr < write_test_size)
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
