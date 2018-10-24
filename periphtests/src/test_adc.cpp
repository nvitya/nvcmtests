// test_adc.cpp

#include "main.h"
#include "hwadc.h"

THwAdc adc;

#define ADC_REC_LEN  1024

uint16_t adc_rec_buffer[ADC_REC_LEN];

#define ADC_ANALYZE_LEN  256

uint16_t adc_value_cnt[ADC_ANALYZE_LEN];

uint8_t adc_ch_x = 0;
uint8_t adc_ch_y = 1;

uint8_t adc_num = 0;

uint8_t adc_shift = 6; // keep only the highest 10 bits

void adc_init()
{
	TRACE("ADC init\r\n");

#if defined(MCUF_STM32)
	adc_num = 1;
	hwpinctrl.PinSetup(PORTNUM_A, 0, PINCFG_INPUT | PINCFG_ANALOGUE); // ch0
	hwpinctrl.PinSetup(PORTNUM_A, 1, PINCFG_INPUT | PINCFG_ANALOGUE); // ch1

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

	hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_INPUT | PINCFG_ANALOGUE); // ch4
	adc_ch_y = 4;
#endif


#elif defined(BOARD_XPLAINED_SAME70)

	adc_ch_x = 0; // PD30
	adc_ch_y = 6; // PA17

#elif defined(BOARD_XPLORER_LPC4330)

	adc_ch_x = 1; // ADC0_1 = J8/19
	adc_ch_y = 2; // ADC0_2 = J8/20

#else
	// The ATSAM4S does not require pin setup, the pins switched automatically to analogue mode on channel enabling

	// CH0, CH1 Pins:
	// Atsam-4S: PA17, PA18
	// Arduino DUE: PA2(AD7), PA3(AD6)
	// ATSAM-E70: PD30, PA21

#endif

	adc.Init(adc_num, (1 << adc_ch_x) | (1 << adc_ch_y));
	//adc.Init(adc_num, (1 << adc_ch_x));

	TRACE("ADC speed: %u conversions / s\r\n", adc.act_conv_rate);
}

void analyze_record()
{
	int i;
	for (i = 0; i < ADC_ANALYZE_LEN; ++i)
	{
		adc_value_cnt[i] = 0;
	}

	// search min-max
	uint16_t minval = 0xFFFF;
	uint16_t maxval = 0;
	for (i = 0; i < ADC_REC_LEN; ++i)
	{
		uint16_t v = (adc_rec_buffer[i] << HWADC_DATA_LSHIFT) >> 4; // 12 bit resolution
		if (v < minval)  minval = v;
		if (v > maxval)  maxval = v;
	}

	if (maxval - minval > ADC_ANALYZE_LEN)
	{
		TRACE("Min-Max range is out of analyze buffer: %i\r\n", maxval-minval);
		return;
	}

	// collect counts

	for (i = 0; i < ADC_REC_LEN; ++i)
	{
		uint16_t v = (adc_rec_buffer[i] << HWADC_DATA_LSHIFT) >> 4; // 12 bit resolution
		int idx = v - minval;
		++adc_value_cnt[idx];
	}

	// displaying data
	uint16_t maxcnt = 0;
	uint16_t maxidx = 0;
	TRACE(" min = %u, max = %u, diff = %u\r\n", minval, maxval, maxval - minval);
	for (i = 0; i < maxval - minval; ++i)
	{
		TRACE("  %4u : %u\r\n", minval + i, adc_value_cnt[i]);
		if (adc_value_cnt[i] > maxcnt)
		{
			maxcnt = adc_value_cnt[i];
			maxidx = i;
		}
	}

	TRACE(" modus = %u\r\n", minval + maxidx);
}

void test_adc_record(uint8_t achnum)
{
	TRACE("ADC record test for channel %i\r\n", achnum);

	unsigned t0, t1;

	int state = 0;

	t0 = CLOCKCNT;
	while (1)
	{
		if (0 == state)
		{
			led1pin.Toggle();

			int i;
			for (i = 0; i < ADC_REC_LEN; ++i)  adc_rec_buffer[i] = 0x1111 + i;

			adc.StartRecord((1 << achnum), ADC_REC_LEN, &adc_rec_buffer[0]);
			t0 = CLOCKCNT;
			state = 1;
		}
		else if (1 == state)
		{
		  if (adc.RecordFinished())
		  {
		  	t1 = CLOCKCNT;
		  	TRACE("Record finished in %u clocks\r\n", t1 - t0);
		  	analyze_record();
		  	t0 = CLOCKCNT;
		  	state = 2; // wait
		  }
		}
		else if (2 == state)
		{
			t1 = CLOCKCNT;
			if (t1 - t0 > SystemCoreClock / 1)
			{
				state = 0; // start again
			}
		}

		idle_task();
	}
}

void adc_test_record()
{
	TRACE("*** ADC Test Record***\r\n");

	adc_init();

	//test_adc_record(adc_ch_x);
	test_adc_record(adc_ch_y);
}

void adc_test_freerun()
{
	TRACE("*** ADC Test Freerun ***\r\n");

	adc_init();

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
			uint16_t advx = (adc.ChValue(adc_ch_x) >> adc_shift);
			uint16_t advy = (adc.ChValue(adc_ch_y) >> adc_shift);

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



