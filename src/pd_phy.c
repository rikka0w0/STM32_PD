#include "pd_phy.h"
#include "stm32f0xx_hal.h"

#define PD_CC1_PIN GPIO_PIN_2
#define PD_CC2_PIN GPIO_PIN_3
#define PD_CC_COMP GPIO_PIN_4
#define PD_CC_GPIO GPIOA

void pd_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Comparator GPIO
	GPIO_InitStruct.Pin = PD_CC_COMP;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP; // GPIO_NOPULL
	HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);
	pd_rx_disable_monitoring();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	pd_rx_enable_monitoring();
}

void pd_rx_enable_monitoring() {
	/* clear comparator external interrupt */
	EXTI->PR = PD_CC_COMP;		// Pending register
	/* enable comparator external interrupt */
	EXTI->IMR |= PD_CC_COMP;
}

void pd_rx_disable_monitoring() {
	/* disable comparator external interrupt */
	EXTI->IMR &= ~ PD_CC_COMP;
	/* clear comparator external interrupt */
	EXTI->PR = PD_CC_COMP;
}

void pd_rx_handler(void) {
	// See RM0091 page 214 & EXTI_PR
	if(__HAL_GPIO_EXTI_GET_IT(PD_CC_COMP) != RESET) {
		// Clear the flag
		HAL_GPIO_EXTI_Callback(PD_CC_COMP);
	}
}
