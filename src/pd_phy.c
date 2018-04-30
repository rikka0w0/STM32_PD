#include "platform.h"
#include "pd_phy.h"

uint16_t raw_samples_rising[PD_BIT_LEN];
uint16_t raw_samples_falling[PD_BIT_LEN];

static uint64_t rx_edge_ts[PD_RX_TRANSITION_COUNT];
static int rx_edge_ts_idx;

void pd_select_cc(uint8_t cc) {
	GPIO_InitTypeDef GPIO_InitStruct;
	if (cc == PD_CC_1) {	// CC1 is CC, pull-up CC2
		PD_CC_GPIO->ODR |= PD_CC2_PIN;
		GPIO_InitStruct.Pin = PD_CC2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);
		PD_CC_GPIO->ODR |= PD_CC2_PIN;
	} else if (cc == PD_CC_2) {	// CC2 is CC, pull-up CC1
		PD_CC_GPIO->ODR |= PD_CC1_PIN;
		GPIO_InitStruct.Pin = PD_CC1_PIN ;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);
		PD_CC_GPIO->ODR |= PD_CC1_PIN;
	} else {
		PD_CC_GPIO->ODR &= ~(PD_CC1_PIN | PD_CC2_PIN);
		GPIO_InitStruct.Pin = PD_CC1_PIN | PD_CC2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);
	}
}

void pd_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	pd_select_cc(0);
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->ODR |= GPIO_PIN_11;

	// Comparator GPIO
	GPIO_InitStruct.Pin = PD_COMP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);
	pd_rx_disable_monitoring();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	// TIM3

    __HAL_RCC_TIM3_CLK_ENABLE();
	/* --- set counter for RX timing : 12Mhz rate, free-running --- */
	TIM3->CR1 = 0x0000;
	TIM3->CR2 = 0x0000;
	TIM3->DIER = 0x0000;
	TIM3->ARR = 0xFFFF;	// Auto-reload value, 16-bit free running counter
	TIM3->PSC = (48000000 / 12000000) - 1; // Prescaler = fAPB1_Timer / 12MHz - 1;
	TIM3->CCR2 = (12000000 / 1000) * USB_PD_RX_TMOUT_US / 1000;	// Channel 2 - Timeout = 21600 Ticks

	TIM3->DCR = 0x300; // burst len = 4
	// RM0091 page 449, IC3=TI4FP3 input, IC4 = TI4FP4 input
	TIM3->CCMR2 = 0x102;
	// RM0091 page 450, CC3E=CC4E=1, Capture enabled
	// CC3NP=CC4NP=0; CC3P=1, CC4P=0, IC3 falling, IC4 rising
	TIM3->CCER = 0x1300;
	// RM0091 page 441, Enable DMA request for IC3 and IC4
	TIM3->DIER = 0x1800;
	// RM0091 page 444, Enable update, CC3 and CC4 generation
	TIM3->EGR = 0x19;
	// RM0091 page 442, Clear flags
	TIM3->SR = 0;

	// DMA
	__HAL_RCC_DMA1_CLK_ENABLE();
}

void pd_rx_enable_monitoring(void) {
	/* clear comparator external interrupt */
	EXTI->PR = PD_COMP_PIN;		// Pending register
	/* enable comparator external interrupt */
	EXTI->IMR |= PD_COMP_PIN;
}

void pd_rx_disable_monitoring(void) {
	/* disable comparator external interrupt */
	EXTI->IMR &= ~ PD_COMP_PIN;
	/* clear comparator external interrupt */
	EXTI->PR = PD_COMP_PIN;
}

uint32_t pd_rx_started(void) {
	/* is the sampling timer running ? */
	return TIM3->CR1 & 1;	// RM0091 436, CEN == 1 ??
}

void pd_rx_complete() {
	/* stop stampling TIM2 */
	TIM3->CR1 &= ~1; // CEN = 0;
	// Disable DMA
	DMA1_Channel2->CCR = 0;
	DMA1_Channel3->CCR = 0;
}

void pd_rx_start() {
	// Comparator GPIO -> Alternate function mode (for TIM3_CH4)
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PD_COMP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);

	// Disable DMA
	DMA1_Channel2->CCR = 0;
	DMA1_Channel3->CCR = 0;
	// Clear ISR
	DMA1->IFCR = 0xFF0;

	// Priority very high, MSIZE=16, PSIZE=16, Memory increment mode
	DMA1_Channel2->CCR = 0x3580;
	DMA1_Channel2->CNDTR = PD_BIT_LEN;
	DMA1_Channel2->CPAR = &(TIM3->CCR3);
	DMA1_Channel2->CMAR = (uint32_t)raw_samples_falling;

	DMA1_Channel3->CCR = 0x3580;
	DMA1_Channel3->CNDTR = PD_BIT_LEN;
	DMA1_Channel3->CPAR = &(TIM3->CCR4);
	DMA1_Channel3->CMAR = (uint32_t)raw_samples_rising;

	/* Flush data in write buffer so that DMA can get the lastest data */
	asm volatile("dsb;");



	// Enable DMA
	DMA1_Channel2->CCR |= 0x01;
	DMA1_Channel3->CCR |= 0x01;

	TIM3->EGR = 0x0001;	// UG = 1
	TIM3->SR = 0; // Clear flags
	TIM3->CR1 |= 1; // Enable Timer
}

void pd_rx_handler(void) {
	int next_idx;

	// See RM0091 page 214 & EXTI_PR
	if(__HAL_GPIO_EXTI_GET_IT(PD_COMP_PIN) != RESET) {
		rx_edge_ts[rx_edge_ts_idx] = timestamp_get();
		next_idx = (rx_edge_ts_idx == PD_RX_TRANSITION_COUNT - 1) ?
					0 : rx_edge_ts_idx + 1;

		/*
		 * If we have seen enough edges in a certain amount of
		 * time, then trigger RX start.
		 */
		if ((rx_edge_ts[rx_edge_ts_idx] -
		     rx_edge_ts[next_idx])
		     < PD_RX_TRANSITION_WINDOW) {
			/*
			 * ignore the comparator IRQ until we are done
			 * with current message
			 */
			pd_rx_disable_monitoring();

			/* start sampling */
			pd_rx_start();

			/* trigger the analysis in the task */
			return; // pd_rx_event(i);
		} else {
			/* do not trigger RX start, just clear int */
			__HAL_GPIO_EXTI_CLEAR_IT(PD_COMP_PIN);
		}
		rx_edge_ts_idx = next_idx;
	}
}

int pd_find_preamble(void) {
	uint16_t* rising = raw_samples_rising;		// DMA1_CH3, TIM3_CH4
	// raw_samples_falling DMA1_CH2, TIM3_CH3

	// Wait for two captures
	while (((PD_BIT_LEN - DMA1_Channel2->CNDTR < 3)	|| (PD_BIT_LEN - DMA1_Channel3->CNDTR < 3))
			&& !(TIM3->SR & 4));
	if (TIM3->SR & 4) {
		return PD_RX_ERR_TIMEOUT;
	}

	if (rising[2] > raw_samples_falling[2]) {
		rising -= 1; // Make sure *rising < * falling, See PD3.0 Spec. page 78
	}

	uint32_t all = 0;
	for (uint32_t bit = 2; bit < 60; bit++) { //
		/* wait if the bit is not received yet ... */
		while (((PD_BIT_LEN - DMA1_Channel2->CNDTR < bit + 1) || (PD_BIT_LEN - DMA1_Channel3->CNDTR < bit + 1))
				&& !(TIM3->SR & 4));
		if (TIM3->SR & 4) {
			return PD_RX_ERR_TIMEOUT-1;
		}

		all = (all >> 1) | (rising[bit] - raw_samples_falling[bit - 1] <= 30 ? 1 << 31 : 0);

		all = (all >> 1) | (raw_samples_falling[bit] - rising[bit] <= 30 ? 1 << 31 : 0);

		if (all == 0x36db6db6) {
			return bit - 1;
		} else if (all == 0xF33F3F3F){
			return PD_RX_ERR_HARD_RESET; /* got HARD-RESET */
		} else if (all == 0x3c7fe0ff) {
			return PD_RX_ERR_CABLE_RESET; /* got CABLE-RESET */
		}

		/*
		Explanation to the magic numbers:
		1. The timer and comparator setup determines that, raw_samples[] contains number which is proportional to the width of pulses
		2. "all = (all >> 1) | (cnt <= PERIOD_THRESHOLD ? 1 << 31 : 0);" gives 1 for narrow pulses and 0 for wide pulses
		3. According to BMC encoding (refer to USD PD specification),
			a data 1 is a transition in the middle of a frame
			while a data 0 doesn't transit from L->H or H-L during the frame.
		4. So the decoding rule becomes, read "all" as a 32-bit binary number,
			a 0 corresponds to actual data bit 0 and two adjacent 1 in "all" correspond to actual data bit 1.

			0x36db6db6  should be SYNC-1
			0011 0110 1101 1011 0110 1101 1011 0110‬
			001  01 0 1 01  01  01 0 10 1  01  01 0

			0010101010101010101010
			| [Preamble---------->
			|
			Return value, end of preamble



			0xF33F3F3F  got HARD-RESET
			‭1111 0011 0011 1111 0011 1111 0011 1111‬
			1 1  001  001  1 1  001  1 1  001  1 1

			11001 00111 00111 00111
			RST-2 RST-1 RST-1 RST-1



			0x3c7fe0ff  got CABLE-RESET
			‭0011 1100 0111 1111 1110 0000 1111 1111‬
			001  1 00 01 1  1 1  1 0 0000 1 1  1 1
			00110 00111 11000 00111 1
			Sync3 RST-1 Sync1 RST-1 | Last bit of preamble

			See also:
			USB PD specification 3.0:
			Table 5-1 4b5b Symbol Encoding Table
			Table 5-5 SOP ordered set
			Table 5-11 Hard Reset ordered set
			Table 5-12 Cable Reset ordered set
		*/
	}

	return PD_RX_ERR_INVAL;
}

void pd_rx_process(void) {
	int isr = TIM3->SR;
	int ret = pd_find_preamble();
	pd_rx_complete();
	while(1);
}
