/*
 * test_fs.cpp
 *
 *  Created on: 16 Jan 2021
 *      Author: vitya
 */


#include "string.h"
#include "platform.h"
#include "hwpins.h"
#include "clockcnt.h"
#include "hwsdcard.h"
#include "storman_sdcard.h"
#include "filesys_fat.h"
#include "fileman.h"

#include "traces.h"

THwSdcard       sdcard;
TStorManSdcard  storman;
TStorTrans      stra;

TFile *         pfile;

uint8_t  testbuf[4096] __attribute__((aligned(16)));
uint8_t  filebuf[65536];

void init_storage()
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

#if 1 // not necessary, but the trace output is nicer and the debugging is easier
	TRACE("Waiting for SDCARD initialization...\r\n");

	while (!sdcard.card_initialized)
	{
		sdcard.Run();
	}
#endif

	storman.Init(&sdcard);
}

void test_sdcard_read()
{
	int i;

	sdcard.StartReadBlocks(0x800, &testbuf[0], 2);
	while (!sdcard.completed)
	{
		sdcard.Run();
	}

	if (sdcard.errorcode)
	{
		TRACE("Read error!\r\n");
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

	//__BKPT();
}


TFileSysFat     fatfs;

TMbrPtEntry     ptable[4];
uint32_t        fs_firstsector;
uint32_t        fs_maxsectors;

void init_filesystems()
{
	unsigned n;

	TRACE("Initializing File Systems...\r\n");

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

			fileman.AddFileSystem(&fatfs);
		}
	}
}

#if 0

void dump_root_dir()
{
	int i;

	storman.AddTransaction(&stra, STRA_READ, fatfs.rootdirstart, &testbuf[0], 4096);
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

#endif

int      g_dir_maxdepth = 4;
int      g_dir_depth = 0;
char     g_ident[128];

void set_dir_depth(int adepth)
{
	g_dir_depth = adepth;
	int i = 0;
	while (i < g_dir_depth)
	{
		g_ident[i] = ' ';
		++i;
	}
	g_ident[i] = 0;
}

void list_directory(const char * adirpath) //uint64_t adirstart)
{
	int            i;
	TFile *        dfile;  // allocated on the heap !
	TFileDirData   fdata;

	dfile = fileman.NewFileObj("", nullptr, 0);  // get a file object for the first filesystem
	if (!dfile)
	{
		return;
	}

	set_dir_depth(g_dir_depth + 1);

	dfile->Open(adirpath, FOPEN_DIRECTORY);
	if (0 != dfile->WaitComplete())
	{
		TRACE("Error(%i) opening directory at \"%s\".\r\n", dfile->result, adirpath);
	}
	else
	{
		TRACE("Directory opened successfully.\r\n");
		while (1)
		{
			dfile->Read(&fdata, sizeof(fdata));
			if (0 != dfile->WaitComplete())
			{
				break;
			}

			if (fdata.attributes & FSATTR_DIR)
			{
				if ((strcmp(".", fdata.name) != 0) && (strcmp("..", fdata.name) != 0))
				{
					TRACE("%s [%s]\r\n", g_ident, fdata.name);
					if (g_dir_depth < g_dir_maxdepth)
					{
						char newpath[FS_PATH_MAX_LEN];
						strncpy(newpath, adirpath, sizeof(newpath));
						unsigned len = strlen(newpath);
						if (('/' != newpath[len-1]) && ('\\' != newpath[len-1]))
						{
							strcat(newpath, "/");
						}
						strcat(newpath, fdata.name);

						list_directory(newpath);
					}
				}
			}
			else if (0 == (fdata.attributes & FSATTR_NONFILE))
			{
				TRACE("%s %s: %u bytes\r\n", g_ident, fdata.name, uint32_t(fdata.size));
			}
		}
	}

	set_dir_depth(g_dir_depth - 1);
	fileman.ReleaseFileObj(dfile);
}

void test_dir_read(const char * adirstart, unsigned amaxdepth)
{
	TRACE("Testing directory read at \"%s\", depth=%u\r\n", adirstart, amaxdepth);

	g_dir_maxdepth = amaxdepth;
	g_dir_depth = 0;

	list_directory(adirstart);
}

void test_file_read(const char * aname)
{
	TRACE("Testing file open \"%s\"...\r\n", aname);

	pfile->Open(aname, 0);
	pfile->WaitComplete();

	TRACE("File Open Result: %i\r\n", pfile->result);
	if (0 == pfile->result)
	{
		TRACE("  file size: %llu\r\n", pfile->fdata.size);
		TRACE("  location:  %llu\r\n", pfile->fdata.location);

		TRACE("Reading file...\r\n");
		pfile->Read(&filebuf[0], sizeof(filebuf));
		pfile->WaitComplete();

		TRACE("File Read Result: %i\r\n", pfile->result);
		if (0 == pfile->result)
		{
			int i = 0;
			char * cp = (char *)&filebuf[0];
			while (i < 4000)
			{
				if (*cp == 10)  TRACE("\r");
				TRACE("%c", *cp);
				++cp;
				++i;
			}
		}
	}
}

void test_fs()
{
	int i;
	unsigned n;

	TRACE("File System Test Begin\r\n");

	init_storage();

	init_filesystems();

	pfile = fileman.NewFileObj("", nullptr, 0);  // get a file object for the first filesystem
	if (!pfile)
	{
		TRACE("Error allocating a file object, no active filesystems.\r\n");
		return;
	}

	TRACE("File object allocated successfully.\r\n");

	//test_dir_read("/", 2);
	test_file_read("nvcm/core/src/core_cm7.h");

	TRACE("File System Test End\r\n");
}
