// chartest.cpp

#include "charlcd_i2c.h"
#include "traces.h"
#include "clockcnt.h"

#define DISPLAY_BITBANG 0

#if DISPLAY_BITBANG
  TCharLcd_bb4  disp;
#else
  TCharLcd_i2c  disp;
#endif

#define CDISP_CHAR_COLS   40
#define CDISP_CHAR_ROWS   20
#define CDISP_CHAR_NUM    (CDISP_CHAR_COLS * CDISP_CHAR_ROWS)

uint8_t cdisp_chars[CDISP_CHAR_NUM];
uint8_t cdisp_changemap[CDISP_CHAR_NUM / 8];

void charlcd_test()
{
	TRACE("Character LCD display test\r\n");

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

#elif defined(BOARD_MIBO48_STM32F303)

	// opendrain allows 5V I2C signal driving, because the pins are 5V tolerant

	// LCD control
	hwpinctrl.PinSetup(PORTNUM_B, 6, PINCFG_AF_4 | PINCFG_OPENDRAIN); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B, 7, PINCFG_AF_4 | PINCFG_OPENDRAIN); // I2C1_SDA

	// I2C1

	disp.i2c.speed = 100000; // 100 kHz
	disp.i2c.Init(1);

	disp.cols = 16;
	disp.rows = 2;


#elif defined(BOARD_MIBO64_ATSAM4S)

	disp.pin_clk.Assign(PORTNUM_B, 0, false);
	disp.pin_clk.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);

	disp.pin_din.Assign(PORTNUM_B, 1, false);
	disp.pin_din.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);

	disp.pin_ce.Assign(PORTNUM_B, 2, false);
	disp.pin_ce.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);

	disp.pin_reset.Assign(PORTNUM_B, 3, false);
	disp.pin_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // B0: RESET

#else
  #error "unknown board."
#endif

	if (!disp.Init(CHLCD_CTRL_HD44780, disp.cols, disp.rows, &cdisp_chars[0], &cdisp_changemap[0]))
	{
		TRACE("Error initializing display!\r\n");
		return;
	}

	TRACE("Display initialized.\r\n");

	disp.printf("Hello World!\n");
	disp.printf("Second line...");

	while (disp.screenchanged)
	{
		disp.Run();
	}

	TRACE("Display test finished.\r\n");
}

