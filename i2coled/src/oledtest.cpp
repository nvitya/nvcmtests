// oledtest.cpp

#include "oleddisp_i2c.h"
#include "traces.h"
#include "clockcnt.h"

TOledDisp_i2c  oled;

uint8_t oled_disp_buf[128*64 >> 3];

#include "FreeSans9pt7b.h"
#include "TomThumb.h"
#include "stdmonofont.h"

void oled_test()
{
	TRACE("OLED display test\r\n");

	oled.i2c.speed = 400000;

#if 0
#elif defined(BOARD_MIBO100_ATSAME70)
	// TWIHS0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SDA/TWD0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SCL/TWCK0

	oled.i2c.Init(0); // TWIHS0
	oled.i2c.txdma.Init(14, 14);  // 14 = TWIHS0.TX (see XDMAC controller peripheral connections)

#elif defined(BOARD_MIBO64_ATSAM4S)
	// TWI0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SDA/TWD0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SCL/TWCK0

	oled.i2c.Init(0); // TWIHS0

	// PdmaInit must be called after i2c.Init !!!
	oled.i2c.PdmaInit(true,  nullptr);   // use internal

#elif defined(BOARD_MIN_F103)
	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_B,  6, PINCFG_AF_0 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  7, PINCFG_AF_0 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SDA

	oled.i2c.Init(1); // I2C1
	oled.i2c.txdma.Init(1, 6, 3);  // DMA1/CH6 = I2C1_TX

#elif defined(BOARD_DISCOVERY_F072)
	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_B,  6, PINCFG_AF_1 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  7, PINCFG_AF_1 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SDA

	oled.i2c.Init(1); // I2C1
	oled.i2c.txdma.Init(1, 2, 0);  // DMA1/CH2 = I2C1_TX

#elif defined(BOARD_NUCLEO_F746)

	// I2C1
	hwpinctrl.PinSetup(PORTNUM_B,  8, PINCFG_AF_4 | PINCFG_OPENDRAIN); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  9, PINCFG_AF_4 | PINCFG_OPENDRAIN); // I2C1_SDA
	//hwpinctrl.PinSetup(PORTNUM_B,  8, PINCFG_AF_4 | PINCFG_PULLUP); // I2C1_SCL
	//hwpinctrl.PinSetup(PORTNUM_B,  9, PINCFG_AF_4 | PINCFG_PULLUP); // I2C1_SDA
	oled.i2c.Init(1); // I2C1
	oled.i2c.txdma.Init(1, 7, 1);  // DMA1/ST7/CH1 = I2C1_TX

#else
  #error "unknown board."
#endif

	oled.rotation = 0;
	if (!oled.Init(OLED_CTRL_SSD1306, 128, 64, &oled_disp_buf[0]))
	{
		TRACE("Error Initializing I2C OLED display at %02X!\r\n", oled.i2caddress);
		return;
	}

#if 1
	oled.SetFont(&FreeSans9pt7b);
#else
	oled.SetFont(&stdmonofont);
#endif

	TRACE("OLED display initialized.\r\n");

	uint8_t fontadvancey, fontascend, fontdescend, fontheight;

	fontadvancey = oled.GetFontMetrics(oled.pfont, &fontascend, &fontdescend);
	fontheight = fontascend + fontdescend;
	TRACE("Current font metrics:\r\n yadvance = %u, ascend = %u, descend = %u\r\n", fontadvancey, fontascend, fontdescend);

	uint16_t x = 0, y = 0;
	uint32_t ccnt = 0;
	unsigned t0, t1;
	TRACE("Starting display test.\r\n");
	t0 = CLOCKCNT;
	while (true)
	{
		t1 = CLOCKCNT;

		if (oled.UpdateFinished())
		{
			oled.FillScreen(0);
			//oled.FillRect(0,0, oled.width, oled.height, 0);
			//oled.DrawPixel(x, y, 1);
			//oled.FillRect(x, y, 20, 20, 1);

			//oled.DrawChar(x, y + oled.GetFontHeight(), 'A');
#if 1
			oled.SetCursor(x, 44 + y + oled.GetFontHeight());
			oled.DrawString((char *)"AgfMAVLTd.");
			oled.SetCursor(x, y + 2 * oled.GetFontHeight());
			oled.DrawString((char *)"gAMtimilL");

			oled.DrawLine(0, 0, 127, 63);
			oled.DrawRect(10, 10, 100, 30);
#endif

			oled.SetCursor(1, 40);
			oled.printf("%u", ccnt);

			++x;
			if (x >= oled.width)
			{
				x = 0;
				//++y;
				if (y >= oled.height)
				{
					y = 0;
				}
			}
		}

		//delay_us(10);  // this can make huge difference between DMA and DMA-less mode
		++ccnt;
	}

	TRACE("OLED display test finished.\r\n");
}

