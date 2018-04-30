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
	TIM3->CCR2 = (12000000 / 1000) * USB_PD_RX_TMOUT_US / 1000;	// Channel 2 - Timeout

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

void pd_rx_start() {
	// Comparator GPIO -> Alternate function mode (for TIM3_CH4)
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PD_COMP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);

	// Disable DMA
	DMA1_Channel2->CCR &= ~0x01;
	DMA1_Channel3->CCR &= ~0x01;
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

	TIM3->EGR = 0x0001;	// UG = 1
	TIM3->SR = 0; // Clear flags
	TIM3->CR1 |= 1; // Enable Timer

	// Enable DMA
	DMA1_Channel2->CCR |= 0x01;
	DMA1_Channel3->CCR |= 0x01;
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

void pd_rx_process(void) {
	uint32_t all = 0;
	for (uint32_t bit = 1; bit < PD_BIT_LEN - 1; bit++) {
		/* wait if the bit is not received yet ... */
		if (PD_BIT_LEN - DMA1_Channel3->CNDTR < bit + 1) {
			while ((PD_BIT_LEN - DMA1_Channel3->CNDTR < bit + 1) &&
				!(TIM3->SR & 4)) // CC1lF The counter value has been captured in TIMx_CCR1 register
				;
			if (TIM3->SR & 4) {
					all = -1;
						return;
					}
				}

		if (bit == 5) {
			all = DMA1_Channel3->CNDTR;
			return;
		}
	}
}
