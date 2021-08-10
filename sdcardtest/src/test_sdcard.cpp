/*
 * test_sdcard.cpp
 *
 *  Created on: 17 Jan 2021
 *      Author: vitya
 */

#include "string.h"
#include "platform.h"
#include "hwpins.h"
#include "clockcnt.h"
#include "hwsdcard.h"
#include "traces.h"

THwSdcard       sdcard;

uint8_t testbuf1[16384] __attribute__((aligned(4)));
uint8_t testbuf2[16384] __attribute__((aligned(4)));

void init_sdcard()
{
	TRACE("Initializing SDCARD...\r\n");

#if defined(BOARD_XPLAINED_SAME70)

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

#elif defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746) || defined(BOARD_NUCLEO_H743)

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_C,  8, PINCFG_AF_12); // SDMMC_D0
	hwpinctrl.PinSetup(PORTNUM_C,  9, PINCFG_AF_12); // SDMMC_D1
	hwpinctrl.PinSetup(PORTNUM_C, 10, PINCFG_AF_12); // SDMMC_D2
	hwpinctrl.PinSetup(PORTNUM_C, 11, PINCFG_AF_12); // SDMMC_D3
	hwpinctrl.PinSetup(PORTNUM_C, 12, PINCFG_AF_12); // SDMMC_CK
	hwpinctrl.PinSetup(PORTNUM_D,  2, PINCFG_AF_12); // SDMMC_CMD

	sdcard.Init();

#elif defined(BOARD_DEV_STM32F407ZE)

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_C,  8, PINCFG_AF_12); // SDMMC_D0
	hwpinctrl.PinSetup(PORTNUM_C,  9, PINCFG_AF_12); // SDMMC_D1
	hwpinctrl.PinSetup(PORTNUM_C, 10, PINCFG_AF_12); // SDMMC_D2
	hwpinctrl.PinSetup(PORTNUM_C, 11, PINCFG_AF_12); // SDMMC_D3
	hwpinctrl.PinSetup(PORTNUM_C, 12, PINCFG_AF_12); // SDMMC_CK
	hwpinctrl.PinSetup(PORTNUM_D,  2, PINCFG_AF_12); // SDMMC_CMD

	sdcard.Init();

#elif defined(BOARD_VERTIBO_A) || defined(BOARD_ENEBO_A)

	// SDCARD Pins
	hwpinctrl.PinSetup(PORTNUM_A, 28, PINCFG_AF_2); // MCCDA
	hwpinctrl.PinSetup(PORTNUM_A, 25, PINCFG_AF_3); // MCCK
	hwpinctrl.PinSetup(PORTNUM_A, 30, PINCFG_AF_2); // MCDA0
	hwpinctrl.PinSetup(PORTNUM_A, 31, PINCFG_AF_2); // MCDA1
	hwpinctrl.PinSetup(PORTNUM_A, 26, PINCFG_AF_2); // MCDA2
	hwpinctrl.PinSetup(PORTNUM_A, 27, PINCFG_AF_2); // MCDA3

	sdcard.dma.Init(9, 0); // 0 = HSMCI DMA Peripheral Id (Transmit and Receive)
	sdcard.Init();

#else
  #error "Unimplemented board!"
#endif

	TRACE("Waiting for SDCARD initialization...\r\n");

	while (!sdcard.card_initialized)
	{
		sdcard.Run();
	}
}

void show_mem(uint8_t * srcptr, unsigned len)
{
	int i;
	for (i = 0; i < len; ++i)
	{
		if (i != 0)
		{
			if ((i % 16) == 0)  TRACE("\r\n");
			if ((i % 512) == 0) TRACE("\r\n");
		}

		TRACE(" %02X", srcptr[i]);
	}
	TRACE("\r\n");
}

void test_read(unsigned astartblock, unsigned abytelen)
{
	int i;
	unsigned tstart, tend;

	TRACE("Testing SDCARD Read at block %u, %u bytes...\r\n", astartblock, abytelen);

	// start block read
	tstart = CLOCKCNT;
	sdcard.StartReadBlocks(astartblock, &testbuf1[0], abytelen / 512);
	while (!sdcard.completed)
	{
		sdcard.Run();
	}
	tend = CLOCKCNT;
	if (sdcard.errorcode)
	{
		TRACE("Read error: %i!\r\n", sdcard.errorcode);
		return;
	}

	TRACE("Read1 ok, clocks = %u\r\n", tend - tstart);
	show_mem((uint8_t *)&testbuf1[0], abytelen);

	TRACE("Repeating read...\r\n");
	// start block read
	tstart = CLOCKCNT;
	sdcard.StartReadBlocks(astartblock, &testbuf2[0], abytelen / 512);
	while (!sdcard.completed)
	{
		sdcard.Run();
	}
	tend = CLOCKCNT;
	if (sdcard.errorcode)
	{
		TRACE("Read error: %i!\r\n", sdcard.errorcode);
		return;
	}
	TRACE("Read2 ok, clocks = %u\r\n", tend - tstart);

	if (0 != memcmp(testbuf1, testbuf2, abytelen))
	{
		TRACE("Read2:\r\n");
		show_mem((uint8_t *)&testbuf2[0], abytelen);
		TRACE("Error: read1 and read2 result is different!\r\n");
	}
	else
	{
		TRACE("read1 and read2 result data are same.\r\n");
	}
}

void test_write(unsigned astartblock, unsigned abytelen, uint8_t astartbyte)
{
	int i;
	unsigned tstart, tend;

	TRACE("Testing SDCARD Write at block %u, %u bytes, startbyte=0x%02X...\r\n", astartblock, abytelen, astartbyte);

	// fill the test buffer
	for (i = 0; i < abytelen; ++i)
	{
		testbuf1[i] = astartbyte + i;
	}

	// start block write
	tstart = CLOCKCNT;
	sdcard.StartWriteBlocks(astartblock, &testbuf1[0], abytelen / 512);
	while (!sdcard.completed)
	{
		sdcard.Run();
	}
	tend = CLOCKCNT;
	if (sdcard.errorcode)
	{
		TRACE("Write error: %i!\r\n", sdcard.errorcode);
		return;
	}

	TRACE("Write1 ok, clocks = %u\r\n", tend - tstart);
	TRACE("Reading back...\r\n");
	sdcard.StartReadBlocks(astartblock, &testbuf2[0], abytelen / 512);
	while (!sdcard.completed)
	{
		sdcard.Run();
	}
	if (sdcard.errorcode)
	{
		TRACE("Read error: %i!\r\n", sdcard.errorcode);
		return;
	}

	TRACE("Read back data:\r\n");
	show_mem((uint8_t *)&testbuf2[0], abytelen);

	if (0 != memcmp(testbuf1, testbuf2, abytelen))
	{
		TRACE("Error: read and write data is different!\r\n");
	}
	else
	{
		TRACE("read and write result data are same.\r\n");
	}

}

void test_sdcard()
{
	TRACE("SDCARD Test Begin\r\n");

	init_sdcard();

	if (!sdcard.initialized)
	{
		return;
	}

#if 0
	test_read(0, 2 * 512);
#endif

#if 1
	test_write(1, 2 * 512, 0xDA);  // sector 1-2 are usually unused
	//TRACE("Repeating the write with different data...\r\n");
	//test_write(1, 2 * 512, 0xA5);  // sector 1-2 are usually unused
#endif

	TRACE("SDCARD Test End\r\n");
}


