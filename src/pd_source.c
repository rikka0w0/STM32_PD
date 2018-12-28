#include <stddef.h>
#include "platform.h"
#include "tcpm.h"
#include "pd.h"

/* Voltage indexes for the PDOs */
enum volt_idx {
	PDO_IDX_5V  = 0,
	PDO_IDX_9V  = 1,
	PDO_IDX_12V  = 2,
	PDO_IDX_15V  = 3,
	PDO_IDX_20V  = 4,
	/* TODO: add PPS support */
	PDO_IDX_COUNT
};
#define PDO_FIXED_FLAGS_EXT (PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP | PDO_FIXED_COMM_CAP | PDO_FIXED_EXTERNAL)

/* PDOs */
const uint32_t pd_src_pdo[] = {
	[PDO_IDX_5V]  = PDO_FIXED(5000,  3000, PDO_FIXED_FLAGS_EXT),
	[PDO_IDX_9V]  = PDO_FIXED(9000,  3000, PDO_FIXED_FLAGS_EXT),
	[PDO_IDX_12V]  = PDO_FIXED(12000,  3000, PDO_FIXED_FLAGS_EXT),
	[PDO_IDX_15V]  = PDO_FIXED(15000,  3000, PDO_FIXED_FLAGS_EXT),
	[PDO_IDX_20V]  = PDO_FIXED(20000,  3000, PDO_FIXED_FLAGS_EXT),
};
const int pd_src_pdo_cnt = PDO_IDX_COUNT;

// 11=12V; 12=9V;
#define PS_GPIO_9V GPIO_PIN_12
#define PS_GPIO_12V GPIO_PIN_11
#define PS_GPIO_15V GPIO_PIN_13
#define PS_GPIO_20V GPIO_PIN_14
#define PS_GPIO_MASK (PS_GPIO_9V | PS_GPIO_12V | PS_GPIO_15V | PS_GPIO_20V)
void pd_power_supply_init() {
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PS_GPIO_MASK;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIOB->ODR &=~ PS_GPIO_MASK;
}

void pd_power_supply_reset(int port){
	// Reset to Safe5V
	GPIOB->ODR &=~ PS_GPIO_MASK;
}

void pd_transition_voltage(int idx) {
	// Reset to Safe5V
	GPIOB->ODR &=~ PS_GPIO_MASK;

	switch (idx-1) {
	case PDO_IDX_9V:
		GPIOB->ODR |= PS_GPIO_9V;
		break;
	case PDO_IDX_12V:
		GPIOB->ODR |= PS_GPIO_9V | PS_GPIO_12V;
		break;
	case PDO_IDX_15V:
		GPIOB->ODR |= PS_GPIO_9V | PS_GPIO_12V | PS_GPIO_15V;
		break;
	case PDO_IDX_20V:
		GPIOB->ODR |= PS_GPIO_9V | PS_GPIO_12V | PS_GPIO_15V | PS_GPIO_20V;
		break;
	default:
		break;
	}
	return;
}
