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

/*
	G4 QSPI Pins: AF10
	--------
	A6: IO3
	A7: IO2
	B0: IO1
	B1: IO0
	B10: CLK
	B11: NCS
*/

	// Discovery QSPI pins

	uint32_t qspipincfg = PINCFG_AF_10;

	hwpinctrl.PinSetup(PORTNUM_B, 11, qspipincfg);   // NCS
	hwpinctrl.PinSetup(PORTNUM_B, 10, qspipincfg);   // CLK

	hwpinctrl.PinSetup(PORTNUM_B,  1, qspipincfg);   // IO0
	hwpinctrl.PinSetup(PORTNUM_B,  0, qspipincfg);   // IO1
	hwpinctrl.PinSetup(PORTNUM_A,  7, qspipincfg);   // IO2
	hwpinctrl.PinSetup(PORTNUM_A,  6, qspipincfg);   // IO3

	qspiflash.qspi.speed = 8000000;
	qspiflash.qspi.multi_line_count = 1;
	qspiflash.has4kerase = true;
	if (!qspiflash.Init())
	{
		TRACE("QSPI Flash init failed!\r\n");
		return;
	}

	TRACE("QSPI Flash initialized, ID CODE = %06X, kbyte size = %u\r\n", qspiflash.idcode, (qspiflash.bytesize >> 10));

#if 0
	int i;
	unsigned addr = 0x00000;
	unsigned len = 64;
#endif

	TRACE("QSPI test finished.\r\n");
}



