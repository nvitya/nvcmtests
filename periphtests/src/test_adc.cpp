// test_adc.cpp

#include "main.h"
#include "hwadc.h"

THwAdc adc;

void test_adc()
{
	TRACE("*** ADC test ***\r\n");

	uint8_t adc_ch_x = 0;
	uint8_t adc_ch_y = 1;

	uint8_t adc_num = 0;

#if defined(MCUF_STM32)
	hwpinctrl.PinSetup(PORTNUM_A, 0, PINCFG_INPUT | PINCFG_ANALOGUE); // ch0
	hwpinctrl.PinSetup(PORTNUM_A, 1, PINCFG_INPUT | PINCFG_ANALOGUE); // ch1

	adc_num = 1;

#elif defined(BOARD_XPLAINED_SAME70)

	adc_ch_x = 0; // PD30
	adc_ch_y = 6; // PA17

#else
	// The ATSAM4S does not require pin setup, the pins switched automatically to analogue mode on channel enabling

	// CH0, CH1 Pins:
	// Arduino DUE: PA2(AD7), PA3(AD6)
	// ATSAM-E70: PD30, PA21


#endif

	adc.Init(adc_num, (1 << adc_ch_x) | (1 << adc_ch_y));
	//adc.Init(adc_num, (1 << adc_ch_x));

	TRACE("ADC speed: %u conversions / s\r\n", adc.act_conv_rate);

	unsigned t0, t1;

	uint8_t  keys = ledandkey.keys;
	unsigned prevscannum = ledandkey.controller.scancounter;
	unsigned dcnt = 0;

	unsigned sampspeed = 10;

	t0 = CLOCKCNT;
	while (1)
	{
		t1 = CLOCKCNT;
		if (t1 - t0 > SystemCoreClock / sampspeed)
		{
			uint16_t advx = (adc.ChValue(adc_ch_x) >> 2);
			uint16_t advy = (adc.ChValue(adc_ch_y) >> 2);

			//ledandkey.DisplayHexNum(adc.ChValue(0) | (adc.ChValue(1) << 16));
			ledandkey.DisplayDecNum(advx + 10000 * advy);
			//TRACE("ADC0 = %4u, ADC1 = %4u\r\n", adc.ChValue(0), adc.ChValue(1));

			led1pin.Toggle();

			t0 = t1;
		}

		if (keys != ledandkey.keys)
		{
			keys = ledandkey.keys;
			if      (keys & 1)  sampspeed = 1000;
			else if (keys & 2)  sampspeed = 100;
			else if (keys & 4)  sampspeed = 10;
			else if (keys & 8)  sampspeed = 5;
			else if (keys & 16)  sampspeed = 1;
		}

		ledandkey.leds = 0;
		if      (sampspeed == 1000)  ledandkey.leds = 1;
		else if (sampspeed ==  100)  ledandkey.leds = 2;
		else if (sampspeed ==   10)  ledandkey.leds = 4;
		else if (sampspeed ==    5)  ledandkey.leds = 8;
		else if (sampspeed ==    1)  ledandkey.leds = 16;
		else ledandkey.leds = 0;

		idle_task();
	}
}



