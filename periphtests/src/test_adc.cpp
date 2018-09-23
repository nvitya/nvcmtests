// test_adc.cpp

#include "main.h"
#include "hwadc.h"

THwAdc adc;

void test_adc()
{
	TRACE("*** ADC test ***\r\n");

	hwpinctrl.PinSetup(PORTNUM_A, 0, PINCFG_INPUT | PINCFG_ANALOGUE); // ch0
	hwpinctrl.PinSetup(PORTNUM_A, 1, PINCFG_INPUT | PINCFG_ANALOGUE); // ch1

	adc.Init(1, 0x0003);

	TRACE("ADC speed: %u conversions / s\r\n", adc.act_conv_rate);

	unsigned t0, t1;

	uint8_t  keys = ledandkey.keys;
	unsigned prevscannum = ledandkey.controller.scancounter;
	unsigned dcnt = 0;

	unsigned sampspeed = 100;

	t0 = CLOCKCNT;
	while (1)
	{
		t1 = CLOCKCNT;
		if (t1 - t0 > SystemCoreClock / sampspeed)
		{
			uint16_t advx = (adc.ChValue(1) >> 2);
			uint16_t advy = (adc.ChValue(0) >> 2);

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



