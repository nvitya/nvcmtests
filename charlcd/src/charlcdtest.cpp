// chartest.cpp

#include "traces.h"
#include "clockcnt.h"
#include "hwpwm.h"

#define DISPLAY_BITBANG 1

#if DISPLAY_BITBANG
  #include "charlcd_bb4.h"

  TCharLcd_bb4  disp;
#else
  #include "charlcd_i2c.h"

  TCharLcd_i2c  disp;
#endif

#define CDISP_CHAR_COLS   40
#define CDISP_CHAR_ROWS   20
#define CDISP_CHAR_NUM    (CDISP_CHAR_COLS * CDISP_CHAR_ROWS)

uint8_t cdisp_chars[CDISP_CHAR_NUM];
uint8_t cdisp_changemap[CDISP_CHAR_NUM / 8];

THwPwmChannel  conpwm;

TCharLcdCtrlType  ctrltype = CHLCD_CTRL_HD44780;

void charlcd_test()
{
	TRACE("Character LCD display test\r\n");

#if 0

#elif defined(BOARD_MIBO48_STM32F303)

	#if DISPLAY_BITBANG

		// the 2x16 character display connected over TI 3.3V -> 5V level shifter

		ctrltype = CHLCD_CTRL_ST7066U;

		disp.pin_en.Assign(PORTNUM_B, 0, false);
		disp.pin_rs.Assign(PORTNUM_B, 1, false);
		disp.pin_rw.Assign(PORTNUM_B, 2, false);

		disp.pin_data[0].Assign(PORTNUM_A, 4, false);
		disp.pin_data[1].Assign(PORTNUM_A, 5, false);
		disp.pin_data[2].Assign(PORTNUM_A, 6, false);
		disp.pin_data[3].Assign(PORTNUM_A, 7, false);

		hwpinctrl.PinSetup(PORTNUM_B, 3, PINCFG_AF_1); // select TIM2_CH2

		conpwm.frequency = 10000;
		conpwm.Init(2, 2, 0); // Timer2/Ch2/output0
		conpwm.Enable();
		conpwm.SetOnClocks((conpwm.periodclocks * 30) / 100);

		disp.cols = 16;
		disp.rows = 2;

	#else
		// opendrain allows 5V I2C signal driving, because the pins are 5V tolerant

		// LCD control
		hwpinctrl.PinSetup(PORTNUM_B, 6, PINCFG_AF_4 | PINCFG_OPENDRAIN); // I2C1_SCL
		hwpinctrl.PinSetup(PORTNUM_B, 7, PINCFG_AF_4 | PINCFG_OPENDRAIN); // I2C1_SDA

		// I2C1

		disp.i2c.speed = 100000; // 100 kHz
		disp.i2c.Init(1);

		disp.cols = 16;
		disp.rows = 2;
	#endif

#else
  #error "unknown board."
#endif

	if (!disp.Init(ctrltype, disp.cols, disp.rows, &cdisp_chars[0], &cdisp_changemap[0]))
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

