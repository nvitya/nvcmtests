// monolcdtest.cpp

#include "monolcd_spi.h"
#include "traces.h"
#include "clockcnt.h"

TMonoLcd_spi  disp;

uint8_t disp_buf[128*64 >> 3];

#include "font_FreeSans9pt7b.h"
TGfxFont font_sans(&FreeSans9pt7b);

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
	disp.spi.idleclk_high = false;
	disp.spi.datasample_late = false;
	disp.spi.speed = 4000000; // 4 MHz
	disp.spi.Init(1);

#else
  #error "unknown board."
#endif

	disp.rotation = 0;
	if (!disp.Init(MLCD_CTRL_UC1701, 128, 64, &disp_buf[0]))
	{
		TRACE("Error Initializing LCD display!\r\n");
		return;
	}

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


#if 1
	disp.SetFont(&font_sans);
#else
	disp.SetFont(&font_mono);
#endif

	TRACE("LCD display initialized.\r\n");

	uint16_t x = 0, y = 0;
	uint32_t ccnt = 0;
	unsigned t0, t1;
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
			disp.SetCursor(x, 44 + y + disp.font->height);
			disp.DrawString((char *)"AgfMAVLTd.");
			disp.SetCursor(x, y + 2 * disp.font->height);
			disp.DrawString((char *)"gAMtimilL");

			disp.DrawLine(0, 0, 127, 63);
			disp.DrawRect(10, 10, 100, 30);
#endif

			disp.SetCursor(1, 40);
			disp.printf("%u", ccnt);

			++x;
			if (x >= disp.width)
			{
				x = 0;
				//++y;
				if (y >= disp.height)
				{
					y = 0;
				}
			}
		}

		delay_us(10000);
		++ccnt;
	}

	TRACE("LCD display test finished.\r\n");
}

