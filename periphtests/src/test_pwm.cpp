// test_pwm.cpp

#include "main.h"
#include "hwadc.h"

extern THwAdc adc;

extern void adc_init();

extern uint8_t adc_ch_x;
extern uint8_t adc_ch_y;

void test_pwm()
{
	adc_init(); // init adc in freerun mode

	TRACE("PWM init\r\n");

	// setup timer 4

	TIM_TypeDef * regs = TIM4;

	// enable TIM4 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	uint32_t baseclock = SystemCoreClock;

	uint32_t pwm_period = 50; // 50 Hz

	uint32_t prescaler = 0;
	int32_t period_counts;
	do
	{
		++prescaler;
		period_counts = baseclock / (prescaler * pwm_period);
	}
	while (period_counts > 65535);

	TRACE("PWM prescaler = %u\r\n", prescaler);
	TRACE("Period counts = %u\r\n", period_counts);

	regs->SMCR = 0;
	regs->EGR = 0;
	regs->PSC = prescaler - 1; // prescaler
	regs->ARR = period_counts;

	// set output compare CH1
	regs->CCR4 = 1700;
	regs->CCMR2 =
	(
		0
		| (0 <<  7)  // OC1CE: Output compare 1 clear enable
		| (6 <<  4)  // OC1M(3): Output compare 1 mode
		| (0 <<  3)  // OC1PE: Output compare 1 preload enable
		| (0 <<  2)  // OC1FE: Output compare 1 fast enable
		| (0 <<  0)  // CC1S(2): Capture/Compare 1 selection, 0 = CH1 is output
	) << 8
	;

	regs->CCER =
	(
		0
		| (0 <<  1)  // CC1P: Capture/Compare 1 output polarity, 0 = active high
		| (1 <<  0)  // CC1E: Capture/Compare 1 output enable
	) << 12
	;

	regs->CR1 = 0
	  | (0 <<  8)  // CKD(2): clock division
	  | (0 <<  7)  // ARPE: Auto-reload preload enable, 1 = buffered ARR
	  | (0 <<  5)  // CMS(2): Center aligned mode, 0 = edge aligned
	  | (0 <<  4)  // DIR: 0 = upcounter, 1 = downcounter
	  | (0 <<  3)  // OPM: one pulse mode
	  | (0 <<  2)  // URS: Update request source
	  | (0 <<  1)  // UDIS: Update disable
	  | (1 <<  0)  // CEN: Counter enable
	;

	regs->CR2 = 0;

	hwpinctrl.PinSetup(PORTNUM_B, 9, PINCFG_AF_0);

	TRACE("*** PWM Test ***\r\n");

	unsigned t0, t1;

	uint16_t adv;

	t0 = CLOCKCNT;
	while (1)
	{
		t1 = CLOCKCNT;
		if (t1 - t0 > SystemCoreClock / 2)
		{
			//TRACE("TIM4 = %u\r\n", regs->CNT);

			led1pin.Toggle();

			t0 = t1;
		}


		int32_t toffset = (3 * period_counts) / (2 * 20); // the base value for position 0


		adv = (adc.ChValue(adc_ch_x) >> 8); // keep the upper 8 bits only

		//adv = 200;

		int32_t xadd = (adv - 128) * period_counts / (20 * 256);

		regs->CCR4 = toffset + xadd;

		idle_task();
	}
}



