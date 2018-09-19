/* -----------------------------------------------------------------------------
 * This file is a part of the NVCM Tests project: https://github.com/nvitya/nvcmtests
 * Copyright (c) 2018 Viktor Nagy, nvitya
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 * --------------------------------------------------------------------------- */
/*
 *  file:     main.cpp (tftgfx)
 *  brief:    Multi-board / connection type TFT LCD screen test
 *  version:  1.00
 *  date:     2018-04-23
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "textscreen.h"
#include "hwdma.h"

#include "sdram.h"

#include "mmtests.h"

#include "traces.h"

THwDmaChannel  dma;
THwDmaTransfer dmaxfer;

uint32_t lcdfill[100*100 / 2];

THwUart   conuart;  // console uart

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

#define LED_COUNT 3

/* 3.5" RPI Display connected to the CN7 ports on the Nucleo-144
 *
 */

#include "tftlcd_spi.h"

TTftLcd_spi  lcd;

void setup_board()
{
  // USART3: Stlink USB / Serial converter
	// USART3_TX: PD.8
	hwpinctrl.PinSetup(3, 8,  PINCFG_OUTPUT | PINCFG_AF_7);
	// USART3_RX: Pd.9
	hwpinctrl.PinSetup(3, 9,  PINCFG_INPUT  | PINCFG_AF_7);
	conuart.Init(3); // USART3

	// LCD control
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_AF_5);  // SPI1_SCK = LCD pin 23
	hwpinctrl.PinSetup(PORTNUM_A, 6, PINCFG_AF_5);  // SPI1_MISO = n.c.
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_AF_5);  // SPI1_MOSI = LCD pin 19

	lcd.pin_reset.Assign(PORTNUM_F, 12, false); // = LCD pin 22
	lcd.pin_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // B0: RESET

	lcd.pin_cs.Assign(PORTNUM_D, 14, false); // LCD pin 24
	lcd.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	lcd.pin_cd.Assign(PORTNUM_D, 15, false); // LCD pin 18
	lcd.pin_cd.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SPI1
	lcd.spi.speed = 48000000;
	lcd.spi.Init(1);

	lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_UNKNOWN, 320, 480);
	lcd.SetRotation(0);
}

#endif

#if defined(BOARD_DEV_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

#define LED_COUNT 2

#include "tftlcd_mm16_f407ze.h"

TTftLcd_mm16_F407ZE  lcd;

TGpioPin pin_reset;

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);

/*
	pin_reset.Assign(PORTNUM_A, 1, false);
	pin_reset.Setup(PINCFG_OUTPUT);
	pin_reset.Set0();
	delay_us(100000);
	pin_reset.Set1();
	delay_us(100000);
*/


	lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_UNKNOWN, 240, 320);
	//lcd.Init(LCD_CTRL_HX8357B, 320, 480);
	//lcd.Init(LCD_CTRL_ILI9486, 320, 480);
	lcd.SetRotation(1);
}

#endif

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(2, 13, false); // PC13

#include "tftlcd_spi.h"

TTftLcd_spi  lcd;
TGpioPin     lcd_reset_pin(PORTNUM_B, 0, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_0);  // USART1_RX
	conuart.Init(1);

	// USART2
	//hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART2_TX
	//hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_0 | PINCFG_PULLUP);  // USART2_RX
	//conuart.Init(2);


	// LCD control
	//hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // SPI1_CS as GPIO
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_OUTPUT | PINCFG_AF_0); // SPI1_SCK
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_OUTPUT | PINCFG_AF_0); // SPI1_MOSI

	lcd.pin_reset.Assign(PORTNUM_B, 0, false);
	lcd.pin_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // B0: RESET

	lcd.pin_cs.Assign(PORTNUM_A, 4, false);
	lcd.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	lcd.pin_cd.Assign(PORTNUM_B, 1, false);
	lcd.pin_cd.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SPI1
	lcd.spi.speed = 48000000;
	lcd.spi.Init(1);

	//lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_ST7735, 128, 160);
	lcd.SetRotation(2);
}

#endif

#if defined(BOARD_ARDUINO_DUE)

/* 16 bit parallel display module designed to arduino mega connected to DUE
 * The pins arent layed out optimal so it is as fast as an SPI display
 *
 * At least optimization -O1 or -Og must be used otherwise it will be too slow
 */

TGpioPin  led1pin(1, 27, false); // D13

#include "tftlcd_gp16_due.h"

TTftLcd_gp16_due  lcd;

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART - On the Arduino programmer interface
	hwpinctrl.PinSetup(0, 8, PINCFG_INPUT | PINCFG_AF_0);  // UART_RXD
	hwpinctrl.PinSetup(0, 9, PINCFG_OUTPUT | PINCFG_AF_0); // UART_TXD
	conuart.Init(0);  // UART

	lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_HX8357B, 320, 480);
	lcd.SetRotation(0);
}
#endif

#if defined(BOARD_VERTIBO_A)

TGpioPin  led1pin(PORTNUM_A, 29, false);

TGpioPin  pin_fpga_cfg(PORTNUM_C, 9, false);

//TGpioPin  pin_fpga_clk(PORTNUM_A, 6, false);
TGpioPin  pin_fpga_clk(PORTNUM_A, 22, false);
//TGpioPin  pin_fpga_irq(PORTNUM_A, 22, false);

#if 1
  #include "tftlcd_mm16_vertibo_a.h"
  TTftLcd_mm16_vertibo_a  lcd;
#else
  #include "tftlcd_gp16_vertibo_a.h"
  TTftLcd_gp16_vertibo_a  lcd;
#endif

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_fpga_cfg.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	//pin_fpga_irq.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	//pin_fpga_clk.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
#if 1
	hwpinctrl.PinSetup(PORTNUM_A,  6,  PINCFG_OUTPUT | PINCFG_AF_1);  // PCK0 = FPGA.CLK_IN

	PMC->PMC_SCER = (1 << 8); // enable PCK0

	PMC->PMC_PCK[0] = 0
		| (1 << 0)  // CSS(3): 1 = MAIN CLK
		| (9 << 4)  // PRES(8): divisor - 1
	;
#endif

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.baudrate = 115200;
	conuart.Init(0);

	lcd.mirrorx = true;
	//lcd.Init(LCD_CTRL_ILI9486, 320, 480);
	lcd.Init(LCD_CTRL_HX8357B, 320, 480);
	//lcd.Init(LCD_CTRL_UNKNOWN, 320, 480);
	lcd.SetRotation(0);
}

#endif


#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

volatile unsigned systick = 0;

extern "C" void SysTick_Handler(void)
{
	++systick;
}

#include "font_FreeSans9pt7b.h"
TGfxFont font_sans(&FreeSans9pt7b);

#include "font_FreeSansBold9pt7b.h"
TGfxFont font_sans_bold(&FreeSansBold9pt7b);

#include "font_FreeSerif9pt7b.h"
TGfxFont font_serif(&FreeSerif9pt7b);

#include "font_FreeSerifBold9pt7b.h"
TGfxFont font_serif_bold(&FreeSerifBold9pt7b);

#include "font_FreeMono9pt7b.h"
TGfxFont font_mono(&FreeMono9pt7b);

#include "font_FreeMonoBold9pt7b.h"
TGfxFont font_mono_bold(&FreeMonoBold9pt7b);

unsigned hbcounter = 0;

const uint16_t testcolors[4] = {0x000F, 0xF000, 0x03C0, 0xFF00};

uint8_t fillcnt = 0;


void heartbeat_task() // invoked every 0.5 s
{
	++hbcounter;

	led1pin.SetTo(hbcounter >> 0);
#if LED_COUNT > 1
	led2pin.SetTo(hbcounter >> 1);
#endif
#if LED_COUNT > 2
	led3pin.SetTo(hbcounter >> 2);
#endif

	//mm_setaddrwindow(10, 10, 100, 100);
	//mm_fillcolor(testcolors[fillcnt & 0x3], 100 * 100); //lcd.width * lcd.height / 2);

#if 0
	lcd.SetAddrWindow(10, 10, 100, 100);
	lcd.FillColor(testcolors[fillcnt & 0x3], 100 * 100); //lcd.width * lcd.height / 2);
#endif

	++fillcnt;

	//lcd.FillScreen(hbcounter);

	//TRACE("hbcounter = %i\r\n", hbcounter);

	lcd.SetFont(&font_mono_bold);
	lcd.SetCursor(10, 200);
	lcd.printf("hbcounter = %i", hbcounter);

}

void idle_task()
{
	//lcd.FillColor(testcolors[hbcounter & 0x3], lcd.width * lcd.height);
	//lcd.FillScreen(testcolors[hbcounter & 0x3]);

	//mm_fillcolor(testcolors[fillcnt & 0x3], lcd.width * lcd.height - 2);

	//++fillcnt;
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_disable_interrupts();

	// Set the interrupt vector table offset, so that the interrupts and exceptions work
	mcu_init_vector_table();

  mcu_preinit_code(); // inline code for preparing the MCU, RAM regions. Without this even the stack does not work on some MCUs.

  unsigned clockspeed = MAX_CLOCK_SPEED;

#ifdef MCU_INPUT_FREQ
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, MAX_CLOCK_SPEED))  // activate the external crystal oscillator with multiplication x2
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, MAX_CLOCK_SPEED))  // activate the external crystal oscillator with multiplication x2
#endif
	{
		while (1)
		{
			// the external oscillator did not start.
		}
	}

	// now the MCU runs faster, start the memory, and C/C++ initialization:
	cppinit();
	// ...from now on all the variables work, static classes are initialized.

	// provide info to the system about the clock speed:
	hwclkctrl.SetClockInfo(clockspeed);

	//mcu_enable_fpu();    // enable coprocessor if present
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	//mcu_enable_interrupts();

	unsigned t0, t1;

	TRACE("\r\n------------------------------------------\r\n");
	TRACE("TFT LCD Test\r\n");

	sdram_init();

	t0 = CLOCKCNT;
	lcd.FillScreen(0x0000);
	t1 = CLOCKCNT;
	TRACE("LCD fill clocks: %u\r\n", t1-t0);

	lcd.color = 0xffff;
	lcd.DrawLine(0, 0, lcd.width-1, lcd.height-1);

#if 1

	const char * testtext = "ThgApIL";

	//lcd.FillRect(10,10, 30, 30, RGB16(0, 255, 0));

	uint16_t x = 1, y = 1;

	lcd.SetFont(&font_sans);

	lcd.bgcolor = RGB16(0, 0, 255);
	lcd.SetCursor(x + 1, y + 1);
	lcd.color = RGB16(0, 50, 0);
	lcd.DrawRect(x, y, lcd.TextWidth(testtext)+2, lcd.font->height+2);
	lcd.color = 0xFFFF;

	lcd.printf(testtext);

	y += lcd.font->height + 5;

	lcd.SetFont(&font_gfx_standard);
	lcd.SetCursor(x, y);
	lcd.printf("ySome text to display with standard font ...");
	y += lcd.font->height + 5;

	lcd.SetFont(&font_serif);
	lcd.SetCursor(x, y);
	lcd.printf("gfFont Serif Text fyAg ");
	y += lcd.font->height + 5;

	lcd.SetFont(&font_serif_bold);
	lcd.SetCursor(x, y);
	lcd.printf("Font Serif Bold Text fyAg ");
	y += lcd.font->height + 5;

	lcd.SetFont(&font_sans);
	lcd.SetCursor(x, y);
	lcd.printf("Font Sans Text fyAg ");
	y += lcd.font->height + 5;

	lcd.SetFont(&font_sans_bold);
	lcd.SetCursor(x, y);
	lcd.printf("Font Sans Bold Text yAg ");
	y += lcd.font->height + 5;

	lcd.SetFont(&font_mono);
	lcd.SetCursor(x, y);
	lcd.printf("Font Mono Text fyAg ");
	y += lcd.font->height + 5;

	lcd.SetFont(&font_mono_bold);
	lcd.SetCursor(x, y);
	lcd.printf("Font Mono Bold Text yAg ");
	y += lcd.font->height + 5;

#endif

	uint32_t wlen = 100;

	lcd.SetAddrWindow(0,0,wlen,wlen);

	for (y = 0; y < wlen; ++y)
	{
		for (x = 0; x < wlen; ++x)
		{

			if (x & 1)
			{
				lcd.WriteData16(0xFFFF);
				//*(uint32_t *)lcd.datareg = 0x0000FFFF;
				//__DSB();
			}
			else
			{
				lcd.WriteData16(0x0000);
				//*lcd.datareg = 0x0000;
			}
		}
	}

#if 1

	lcd.SetAddrWindow(200,200,wlen,wlen + 10);

//#define FILL_COLOR_32  0xFFFFFFFF
#define FILL_COLOR_32  0x0000FFFF
//#define FILL_COLOR_32  0xffffFFFF


	for (x = 0; x < wlen*wlen / 2; ++x)  lcdfill[x] = FILL_COLOR_32;

#if 0
	uint32_t remaining = wlen * wlen / 2;
	while (remaining > 0)
	{

#if 1
		MM_DATAREG32 = 0x0000FFFF;
		__DSB();
#else
		MM_DATAREG = 0x0FF0;
		__DSB();
		MM_DATAREG = 0x0000;
		__DSB();
#endif

		--remaining;
	}
#else
	dma.Init(1, 0);
	dmaxfer.srcaddr = &lcdfill[0];
	dmaxfer.dstaddr = (void *)&MM_DATAREG32;
	dmaxfer.bytewidth = 4;
	dmaxfer.count = wlen * wlen / 2;
	dmaxfer.flags = DMATR_MEM_TO_MEM | DMATR_NO_DST_INC;
	dma.StartTransfer(&dmaxfer);
#endif

#endif

#if 1
	// full screen fill with dma

	int i;

	uint32_t * sdd = (uint32_t *)	0x70000000;

	for (i = 0; i < lcd.width * lcd.height / 2; ++i)
	{
		sdd[i] = FILL_COLOR_32; //;  + (i << 16);
		//sdd[i] = (i << 16);
	}

	lcd.SetAddrWindow(0, 0, lcd.width, lcd.height);

	dmaxfer.srcaddr = (void *)0x70000000;
	dmaxfer.dstaddr = (void *)&MM_DATAREG32;
	dmaxfer.bytewidth = 4;
	dmaxfer.count = lcd.width * lcd.height / 2;
	dmaxfer.flags = DMATR_MEM_TO_MEM | DMATR_NO_DST_INC;

#if 1
	//dmaxfer.srcaddr = (void *)0x20400000;
	//dmaxfer.srcaddr = (void *)0x70000000;
	dmaxfer.srcaddr = &lcdfill[0];
	//dmaxfer.count = 128 * 1024 / 4;
	dmaxfer.flags = DMATR_MEM_TO_MEM | DMATR_NO_ADDR_INC;
#endif

	t0 = CLOCKCNT;
	dma.StartTransfer(&dmaxfer);
	while (dma.Active()) { } // wait until DMA finishes.
	t1 = CLOCKCNT;
	TRACE("LCD DMA fill clocks from SDRAM: %u\r\n", t1-t0);

#endif

	//lcd.FillScreen(0x0004);

	//mm_init();

	//heartbeat_task();

	TRACE("Starting main cycle...\r\n");

	//SysTick_Config(SystemCoreClock / 1000);

	unsigned hbclocks = SystemCoreClock / 2;


	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		idle_task();

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
