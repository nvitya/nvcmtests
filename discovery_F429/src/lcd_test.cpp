// lcd_test.cpp

#include "platform.h"
#include "lcd_test.h"

#include "hwpins.h"
#include "traces.h"

#include "hwlcdctrl.h"
#include "hwsdram.h"
#include "framebuffer16.h"
#include "clockcnt.h"

THwLcdCtrl      lcdctrl;

TFrameBuffer16  disp;

#if defined(BOARD_DISCOVERY_F746)

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

#endif

#if defined(BOARD_DISCOVERY_F429)

#include "hwspi.h"

TGpioPin  pin_disp_cs(PORTNUM_C, 2, false);;
TGpioPin  pin_disp_rs(PORTNUM_D, 13, false);

THwSpi    disp_spi;

/* Level 1 Commands */
#define LCD_SWRESET             0x01   /* Software Reset */
#define LCD_READ_DISPLAY_ID     0x04   /* Read display identification information */
#define LCD_RDDST               0x09   /* Read Display Status */
#define LCD_RDDPM               0x0A   /* Read Display Power Mode */
#define LCD_RDDMADCTL           0x0B   /* Read Display MADCTL */
#define LCD_RDDCOLMOD           0x0C   /* Read Display Pixel Format */
#define LCD_RDDIM               0x0D   /* Read Display Image Format */
#define LCD_RDDSM               0x0E   /* Read Display Signal Mode */
#define LCD_RDDSDR              0x0F   /* Read Display Self-Diagnostic Result */
#define LCD_SPLIN               0x10   /* Enter Sleep Mode */
#define LCD_SLEEP_OUT           0x11   /* Sleep out register */
#define LCD_PTLON               0x12   /* Partial Mode ON */
#define LCD_NORMAL_MODE_ON      0x13   /* Normal Display Mode ON */
#define LCD_DINVOFF             0x20   /* Display Inversion OFF */
#define LCD_DINVON              0x21   /* Display Inversion ON */
#define LCD_GAMMA               0x26   /* Gamma register */
#define LCD_DISPLAY_OFF         0x28   /* Display off register */
#define LCD_DISPLAY_ON          0x29   /* Display on register */
#define LCD_COLUMN_ADDR         0x2A   /* Colomn address register */
#define LCD_PAGE_ADDR           0x2B   /* Page address register */
#define LCD_GRAM                0x2C   /* GRAM register */
#define LCD_RGBSET              0x2D   /* Color SET */
#define LCD_RAMRD               0x2E   /* Memory Read */
#define LCD_PLTAR               0x30   /* Partial Area */
#define LCD_VSCRDEF             0x33   /* Vertical Scrolling Definition */
#define LCD_TEOFF               0x34   /* Tearing Effect Line OFF */
#define LCD_TEON                0x35   /* Tearing Effect Line ON */
#define LCD_MAC                 0x36   /* Memory Access Control register*/
#define LCD_VSCRSADD            0x37   /* Vertical Scrolling Start Address */
#define LCD_IDMOFF              0x38   /* Idle Mode OFF */
#define LCD_IDMON               0x39   /* Idle Mode ON */
#define LCD_PIXEL_FORMAT        0x3A   /* Pixel Format register */
#define LCD_WRITE_MEM_CONTINUE  0x3C   /* Write Memory Continue */
#define LCD_READ_MEM_CONTINUE   0x3E   /* Read Memory Continue */
#define LCD_SET_TEAR_SCANLINE   0x44   /* Set Tear Scanline */
#define LCD_GET_SCANLINE        0x45   /* Get Scanline */
#define LCD_WDB                 0x51   /* Write Brightness Display register */
#define LCD_RDDISBV             0x52   /* Read Display Brightness */
#define LCD_WCD                 0x53   /* Write Control Display register*/
#define LCD_RDCTRLD             0x54   /* Read CTRL Display */
#define LCD_WRCABC              0x55   /* Write Content Adaptive Brightness Control */
#define LCD_RDCABC              0x56   /* Read Content Adaptive Brightness Control */
#define LCD_WRITE_CABC          0x5E   /* Write CABC Minimum Brightness */
#define LCD_READ_CABC           0x5F   /* Read CABC Minimum Brightness */
#define LCD_READ_ID1            0xDA   /* Read ID1 */
#define LCD_READ_ID2            0xDB   /* Read ID2 */
#define LCD_READ_ID3            0xDC   /* Read ID3 */

/* Level 2 Commands */
#define LCD_RGB_INTERFACE       0xB0   /* RGB Interface Signal Control */
#define LCD_FRMCTR1             0xB1   /* Frame Rate Control (In Normal Mode) */
#define LCD_FRMCTR2             0xB2   /* Frame Rate Control (In Idle Mode) */
#define LCD_FRMCTR3             0xB3   /* Frame Rate Control (In Partial Mode) */
#define LCD_INVTR               0xB4   /* Display Inversion Control */
#define LCD_BPC                 0xB5   /* Blanking Porch Control register */
#define LCD_DFC                 0xB6   /* Display Function Control register */
#define LCD_ETMOD               0xB7   /* Entry Mode Set */
#define LCD_BACKLIGHT1          0xB8   /* Backlight Control 1 */
#define LCD_BACKLIGHT2          0xB9   /* Backlight Control 2 */
#define LCD_BACKLIGHT3          0xBA   /* Backlight Control 3 */
#define LCD_BACKLIGHT4          0xBB   /* Backlight Control 4 */
#define LCD_BACKLIGHT5          0xBC   /* Backlight Control 5 */
#define LCD_BACKLIGHT7          0xBE   /* Backlight Control 7 */
#define LCD_BACKLIGHT8          0xBF   /* Backlight Control 8 */
#define LCD_POWER1              0xC0   /* Power Control 1 register */
#define LCD_POWER2              0xC1   /* Power Control 2 register */
#define LCD_VCOM1               0xC5   /* VCOM Control 1 register */
#define LCD_VCOM2               0xC7   /* VCOM Control 2 register */
#define LCD_NVMWR               0xD0   /* NV Memory Write */
#define LCD_NVMPKEY             0xD1   /* NV Memory Protection Key */
#define LCD_RDNVM               0xD2   /* NV Memory Status Read */
#define LCD_READ_ID4            0xD3   /* Read ID4 */
#define LCD_PGAMMA              0xE0   /* Positive Gamma Correction register */
#define LCD_NGAMMA              0xE1   /* Negative Gamma Correction register */
#define LCD_DGAMCTRL1           0xE2   /* Digital Gamma Control 1 */
#define LCD_DGAMCTRL2           0xE3   /* Digital Gamma Control 2 */
#define LCD_INTERFACE           0xF6   /* Interface control register */

/* Extend register commands */
#define LCD_POWERA               0xCB   /* Power control A register */
#define LCD_POWERB               0xCF   /* Power control B register */
#define LCD_DTCA                 0xE8   /* Driver timing control A */
#define LCD_DTCB                 0xEA   /* Driver timing control B */
#define LCD_POWER_SEQ            0xED   /* Power on sequence register */
#define LCD_3GAMMA_EN            0xF2   /* 3 Gamma enable register */
#define LCD_PRC                  0xF7   /* Pump ratio control register */

/* Size of read registers */
#define LCD_READ_ID4_SIZE        3      /* Size of Read ID4 */

void disp_write_data(uint16_t avalue)
{
  pin_disp_rs.Set1(); // Set WRX to send data

  pin_disp_cs.Set0();  // Reset LCD control line(/CS) and Send data
  disp_spi.SendData(avalue);
  disp_spi.WaitSendFinish();
  uint16_t d16;
  while (disp_spi.TryRecvData(&d16))
  {
  	//
  }

  pin_disp_cs.Set1(); // Deselect: Chip Select high
}

void disp_write_reg(uint8_t avalue)
{
  pin_disp_rs.Set0(); // Reset WRX to send command

  pin_disp_cs.Set0();  // Reset LCD control line(/CS) and Send data
  disp_spi.SendData(avalue);
  disp_spi.WaitSendFinish();
  uint16_t d16;
  while (disp_spi.TryRecvData(&d16))
  {
  	//
  }

  pin_disp_cs.Set1(); // Deselect: Chip Select high
}

void init_ili9341()
{
	TRACE("Initializing ILI9341...\r\n");

	pin_disp_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_disp_rs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	//pin_disp_cs.Set0();
	//pin_disp_cs.Set1();

	// init SPI5
	hwpinctrl.PinSetup(PORTNUM_F, 7, PINCFG_AF_5); // SPI5.SCK
	hwpinctrl.PinSetup(PORTNUM_F, 8, PINCFG_AF_5 | PINCFG_PULLDOWN); // SPI5.MISO
	hwpinctrl.PinSetup(PORTNUM_F, 9, PINCFG_AF_5); // SPI5.MOSI
	disp_spi.idleclk_high = false;
	disp_spi.speed = 4000000;
	disp_spi.databits = 8;
	disp_spi.Init(5);

	delay_ms(100);

  /* Configure LCD */
  disp_write_reg(0xCA);
  disp_write_data(0xC3);
  disp_write_data(0x08);
  disp_write_data(0x50);
  disp_write_reg(LCD_POWERB);
  disp_write_data(0x00);
  disp_write_data(0xC1);
  disp_write_data(0x30);
  disp_write_reg(LCD_POWER_SEQ);
  disp_write_data(0x64);
  disp_write_data(0x03);
  disp_write_data(0x12);
  disp_write_data(0x81);
  disp_write_reg(LCD_DTCA);
  disp_write_data(0x85);
  disp_write_data(0x00);
  disp_write_data(0x78);
  disp_write_reg(LCD_POWERA);
  disp_write_data(0x39);
  disp_write_data(0x2C);
  disp_write_data(0x00);
  disp_write_data(0x34);
  disp_write_data(0x02);
  disp_write_reg(LCD_PRC);
  disp_write_data(0x20);
  disp_write_reg(LCD_DTCB);
  disp_write_data(0x00);
  disp_write_data(0x00);
  disp_write_reg(LCD_FRMCTR1);
  disp_write_data(0x00);
  disp_write_data(0x1B);
  disp_write_reg(LCD_DFC);
  disp_write_data(0x0A);
  disp_write_data(0xA2);
  disp_write_reg(LCD_POWER1);
  disp_write_data(0x10);
  disp_write_reg(LCD_POWER2);
  disp_write_data(0x10);
  disp_write_reg(LCD_VCOM1);
  disp_write_data(0x45);
  disp_write_data(0x15);
  disp_write_reg(LCD_VCOM2);
  disp_write_data(0x90);
  disp_write_reg(LCD_MAC);
  disp_write_data(0xC8);
  disp_write_reg(LCD_3GAMMA_EN);
  disp_write_data(0x00);
  disp_write_reg(LCD_RGB_INTERFACE);
  disp_write_data(0xC2);
  disp_write_reg(LCD_DFC);
  disp_write_data(0x0A);
  disp_write_data(0xA7);
  disp_write_data(0x27);
  disp_write_data(0x04);

  /* Colomn address set */
  disp_write_reg(LCD_COLUMN_ADDR);
  disp_write_data(0x00);
  disp_write_data(0x00);
  disp_write_data(0x00);
  disp_write_data(0xEF);
  /* Page address set */
  disp_write_reg(LCD_PAGE_ADDR);
  disp_write_data(0x00);
  disp_write_data(0x00);
  disp_write_data(0x01);
  disp_write_data(0x3F);
  disp_write_reg(LCD_INTERFACE);
  disp_write_data(0x01);
  disp_write_data(0x00);
  disp_write_data(0x06);

  disp_write_reg(LCD_GRAM);
  delay_ms(200);

  disp_write_reg(LCD_GAMMA);
  disp_write_data(0x01);

  disp_write_reg(LCD_PGAMMA);
  disp_write_data(0x0F);
  disp_write_data(0x29);
  disp_write_data(0x24);
  disp_write_data(0x0C);
  disp_write_data(0x0E);
  disp_write_data(0x09);
  disp_write_data(0x4E);
  disp_write_data(0x78);
  disp_write_data(0x3C);
  disp_write_data(0x09);
  disp_write_data(0x13);
  disp_write_data(0x05);
  disp_write_data(0x17);
  disp_write_data(0x11);
  disp_write_data(0x00);
  disp_write_reg(LCD_NGAMMA);
  disp_write_data(0x00);
  disp_write_data(0x16);
  disp_write_data(0x1B);
  disp_write_data(0x04);
  disp_write_data(0x11);
  disp_write_data(0x07);
  disp_write_data(0x31);
  disp_write_data(0x33);
  disp_write_data(0x42);
  disp_write_data(0x05);
  disp_write_data(0x0C);
  disp_write_data(0x0A);
  disp_write_data(0x28);
  disp_write_data(0x2F);
  disp_write_data(0x0F);

  disp_write_reg(LCD_SLEEP_OUT);
  delay_ms(200);
  disp_write_reg(LCD_DISPLAY_ON);
  /* GRAM start writing */
  disp_write_reg(LCD_GRAM);
}

void lcd_init()
{
	uint32_t  tmp;
	uint32_t  pinflags = PINCFG_SPEED_MEDIUM;

/*
 +------------------------+-----------------------+----------------------------+
 +                       LCD pins assignment                                   +
 +------------------------+-----------------------+----------------------------+
 |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06      |
 |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
 |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12      |
 |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
 |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
 |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09      |
 -------------------------------------------------------------------------------
					|  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04 |
					|  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
					 -----------------------------------------------------
*/

	// LCD CONTROLLER PINS

	hwpinctrl.PinSetup(PORTNUM_A,  3, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_A,  4, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_A,  6, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_A, 11, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_A, 12, pinflags | PINCFG_AF_14);

	hwpinctrl.PinSetup(PORTNUM_B,  0, pinflags | PINCFG_AF_9);
	hwpinctrl.PinSetup(PORTNUM_B,  1, pinflags | PINCFG_AF_9);
	hwpinctrl.PinSetup(PORTNUM_B,  8, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_B,  9, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_B, 10, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_B, 11, pinflags | PINCFG_AF_14);

	hwpinctrl.PinSetup(PORTNUM_C,  6, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_C,  7, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_C, 10, pinflags | PINCFG_AF_14);

	hwpinctrl.PinSetup(PORTNUM_D,  3, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_D,  6, pinflags | PINCFG_AF_14);

	hwpinctrl.PinSetup(PORTNUM_F, 10, pinflags | PINCFG_AF_14);

	hwpinctrl.PinSetup(PORTNUM_G,  6, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_G,  7, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_G, 10, pinflags | PINCFG_AF_9);
	hwpinctrl.PinSetup(PORTNUM_G, 11, pinflags | PINCFG_AF_14);
	hwpinctrl.PinSetup(PORTNUM_G, 12, pinflags | PINCFG_AF_9);

	// The ILI9341 must be initialized trough SPI
	init_ili9341();

	//--------------------------------------------------------------
	// Configure the internal LCD controller

	lcdctrl.hsync = 10;
	lcdctrl.hbp = 20;
	lcdctrl.hfp = 10;

	lcdctrl.vsync = 2;
	lcdctrl.vbp = 2;
	lcdctrl.vfp = 5;
	lcdctrl.Init(240, 320, (void *)hwsdram.address);
	//lcdctrl.Init(240, 320, (void *)0x08000000);
}

#endif

void lcd_test()
{
	TRACE("--- LCD TEST ---\r\n");

	lcd_init();

	uint16_t w = lcdctrl.hwwidth;
	uint16_t h = lcdctrl.hwheight;

	uint16_t * pp;

	uint16_t color = 0x001F;
	uint32_t cnt = w * 10;
	uint32_t n;

	pp = (uint16_t *)hwsdram.address;
	for (n = 0; n < cnt; ++n)
	{
		*(pp++) = color;
	}

#if 1
	disp.Init(w, h, (void *)(hwsdram.address));
	disp.FillScreen(0);
	disp.color = RGB16(0, 255, 0);
	disp.FillRect(10, 10, 100, 100, disp.color);

	disp.color = RGB16(255, 0, 0);
	disp.DrawRect(0, 0, disp.width, disp.height);

	disp.color = 0xffff;
	disp.SetCursor(50, 150);
	disp.DrawString("Hello World!");

#endif

	TRACE("LCD test finished.\r\n");
}
