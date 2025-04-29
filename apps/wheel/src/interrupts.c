#include "peripherals.h"

void DMA1_Channel1_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_adc1);
}

void DMA1_Channel5_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_DMA_IRQHandler(&peripherals->hdma_tim2_ch1);
}

void TIM2_IRQHandler(void) {
  peripherals_t *peripherals = get_peripherals();
  HAL_TIM_IRQHandler(&peripherals->htim2);
}
