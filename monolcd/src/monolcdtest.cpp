// monolcdtest.cpp

#include "monolcd_spi.h"
#include "traces.h"
#include "clockcnt.h"

TMonoLcd_spi  disp;

uint8_t disp_buf[128*64 >> 3];

#include "font_FreeSans9pt7b.h"
TGfxFont font_sans(&FreeSans9pt7b);

#include "font_FreeSansBold9pt7b.h"
TGfxFont font_bold(&FreeSansBold9pt7b);

#include "stdmonofont.h"
TGfxFont font_mono(&stdmonofont);

void monolcd_test()
{
	TRACE("Mono LCD display test\r\n");

#if 0

#elif defined(BOARD_MIN_F103)

	// LCD control
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_OUTPUT | PINCFG_AF_0); // SPI1_SCK
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_OUTPUT | PINCFG_AF_0); // SPI1_MOSI

	disp.pin_reset.Assign(PORTNUM_B, 0, false);
	disp.pin_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // B0: RESET

	disp.pin_cs.Assign(PORTNUM_A, 4, false);
	disp.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	disp.pin_cd.Assign(PORTNUM_B, 1, false);
	disp.pin_cd.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SPI1

	//disp.txdma.Init(1, 3, 1);  // SPI1/TX = DMA channel 3

	disp.spi.idleclk_high = false;
	disp.spi.datasample_late = false;
	disp.spi.speed = 4000000; // 4 MHz
	disp.spi.Init(1);

#else
  #error "unknown board."
#endif


#if 0
	disp.rotation = 0;
	if (!disp.Init(MLCD_CTRL_UC1701, 128, 64, &disp_buf[0]))
	{
		TRACE("Error Initializing LCD display!\r\n");
		return;
	}
#endif

#if 1
	disp.rotation = 0;
	disp.contrast = 40;
	if (!disp.Init(MLCD_CTRL_NOKIA5110, 84, 48, &disp_buf[0]))
	{
		TRACE("Error Initializing LCD display!\r\n");
		return;
	}
#endif


/*
	disp.pin_cd.Set1();
	disp.pin_cs.Set0();
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.SendData(0x55);
	disp.spi.SendData(0xAA);
	disp.spi.WaitSendFinish();
	disp.pin_cs.Set1();

	TRACE("test stopped here.\r\n");

	while (1)
	{
		//
	}
*/


#if 0
	disp.SetFont(&font_sans);
#else
	disp.SetFont(&font_mono);
#endif

	TRACE("LCD display initialized.\r\n");

	uint16_t x = 0, y = 0;
	uint32_t ccnt = 0;
	unsigned t0, t1;

#if 0 // moving test

	TRACE("LCD Update test\r\n");

	TRACE("Starting display test.\r\n");
	t0 = CLOCKCNT;
	while (true)
	{
		t1 = CLOCKCNT;

		if (disp.UpdateFinished())
		{
			disp.FillScreen(0);
			//disp.FillRect(0,0, disp.width, disp.height, 0);
			//disp.DrawPixel(x, y, 1);
			//disp.FillRect(x, y, 20, 20, 1);

			//disp.DrawChar(x, y + disp.GetFontHeight(), 'A');
#if 1
			disp.DrawLine(0, 0, 127, 63);
			disp.DrawRect(10, 10, 100, 30);

			disp.SetCursor(x, 2 + y + disp.font->height);
			disp.DrawString((char *)"AgfMAVLTd.");
			disp.SetCursor(x, 2 + y + 2 * disp.font->height);
			disp.DrawString((char *)"gAMtimilL");

#endif

			//disp.DrawRect(0, 0, disp.width, disp.height);

			disp.SetCursor(1, 40);
			disp.printf("%u", ccnt);

			++x;
			++y;
			if (x >= disp.width)
			{
				x = 0;
				++y;
				if (y >= disp.height)
				{
					y = 0;
				}
			}

			if (y > 16)  y = 0;

			delay_us(200000);

		}

		++ccnt;
	}

#endif

#if 1

	TRACE("Static LCD test\r\n");

	t0 = CLOCKCNT;
	while (true)
	{
		t1 = CLOCKCNT;

		if (disp.UpdateFinished())
		{
			disp.FillScreen(0);

			disp.SetFont(&font_mono);
			disp.SetCursor(2, 2);
			disp.DrawString((char *)"Small Full Font 1234 km");
#if 1
			disp.SetFont(&font_sans);
			disp.SetCursor(2, 16);
			disp.DrawString((char *)"Big 1234 km");
			//disp.DrawString((char *)"kkkkkkkkkkkkkk");
			//disp.DrawString((char *)"iiiiiiiiiiiiiiiiii");

			disp.SetFont(&font_bold);
			disp.SetCursor(2, 40);
			disp.DrawString((char *)"Big 1234 km");
#endif

 			delay_us(200000);
		}

		++ccnt;
	}

#endif

	TRACE("LCD display test finished.\r\n");
}

