// qspi_test.cpp

#include "platform.h"
#include "hwpins.h"
#include "hwqspi.h"
#include "traces.h"
#include "qspiflash.h"

TQspiFlash  qspiflash;

__attribute__((aligned(16)))
unsigned char qdatabuf[8192];

void qspi_test()
{
	TRACE("QSPI test...\r\n");

	uint32_t qspipincfg = 0;

	// Discovery QSPI pins

	hwpinctrl.PinSetup(PORTNUM_B,  6, qspipincfg | PINCFG_AF_10);  // NCS
	hwpinctrl.PinSetup(PORTNUM_B,  2, qspipincfg | PINCFG_AF_9);   // CLK

	hwpinctrl.PinSetup(PORTNUM_E,  2, qspipincfg | PINCFG_AF_9);   // IO2
	hwpinctrl.PinSetup(PORTNUM_D, 11, qspipincfg | PINCFG_AF_9);   // IO0
	hwpinctrl.PinSetup(PORTNUM_D, 12, qspipincfg | PINCFG_AF_9);   // IO1
	hwpinctrl.PinSetup(PORTNUM_D, 13, qspipincfg | PINCFG_AF_9);   // IO3

	qspiflash.qspi.speed = 8000000;
	qspiflash.qspi.multi_line_count = 1;
	qspiflash.has4kerase = true;
	if (!qspiflash.Init())
	{
		TRACE("QSPI Flash init failed!\r\n");
		return;
	}

	TRACE("QSPI Flash initialized, ID CODE = %06X, kbyte size = %u\r\n", qspiflash.idcode, (qspiflash.bytesize >> 10));

	int i;
	unsigned addr = 0x00000;
	unsigned len = 64;

	TRACE("QSPI test finished.\r\n");
}



