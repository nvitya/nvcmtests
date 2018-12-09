// lcd_test.cpp

#include "platform.h"
#include "lcd_test.h"

#include "hwpins.h"
#include "traces.h"

#include "hwlcdctrl.h"
#include "hwsdram.h"
#include "framebuffer16.h"

THwLcdCtrl      lcdctrl;

TFrameBuffer16  disp;

void lcd_init()
{
	uint32_t  tmp;
	uint32_t  pinflags = 0;

	// LCD CONTROLLER PINS

	hwpinctrl.PinSetup(PORTNUM_E,  4, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_G, 12, pinflags | PINCFG_AF_12); //

	hwpinctrl.PinSetup(PORTNUM_I,  9, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_I, 10, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_I, 14, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_I, 15, pinflags | PINCFG_AF_14); //

	hwpinctrl.PinSetup(PORTNUM_J,  0, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  1, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  2, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  3, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  4, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  5, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  6, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  7, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  8, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J,  9, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J, 10, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J, 11, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J, 13, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J, 14, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_J, 15, pinflags | PINCFG_AF_14); //

	hwpinctrl.PinSetup(PORTNUM_K,  0, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_K,  1, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_K,  2, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_K,  4, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_K,  5, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_K,  6, pinflags | PINCFG_AF_14); //
	hwpinctrl.PinSetup(PORTNUM_K,  7, pinflags | PINCFG_AF_14); //

	// LCD GPIO PINS

	hwpinctrl.PinSetup(PORTNUM_I, 12, PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // LCD_DISP
	hwpinctrl.PinSetup(PORTNUM_K,  3, PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // LCD_BL_CTRL

	// Configure the LCD clock

	uint32_t lcd_pixel_clock = 8000000;


	//lcdctrl.Init(480, 272, (void *)0x08000000);  // give the rom start as the framebuffer
	lcdctrl.Init(480, 272, (void *)hwsdram.address);

}

void lcd_test()
{
	TRACE("--- LCD TEST ---\r\n");

	lcd_init();

	uint16_t * pp;

	uint16_t color = 0x001F;
	uint32_t cnt = 480 * 10;
	uint32_t n;

	pp = (uint16_t *)hwsdram.address;
	for (n = 0; n < cnt; ++n)
	{
		*(pp++) = color;
	}

	disp.Init(480, 272, (void *)(hwsdram.address));
	disp.FillScreen(0);
	disp.color = RGB16(0, 255, 0);
	disp.FillRect(10, 10, 100, 100, disp.color);

	disp.color = RGB16(255, 0, 0);
	disp.DrawRect(0, 0, disp.width, disp.height);

	disp.color = 0xffff;
	disp.SetCursor(300, 150);
	disp.DrawString("Hello World!");

	TRACE("LCD test finished.\r\n");
}
