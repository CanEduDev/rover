#include "peripherals.h"

#include "error.h"
#include "ports.h"

#define PWM_PSC_1MHZ (72 - 1)

#define USART2_IRQ_PRIORITY 5

static peripherals_t peripherals;

void gpio_init(void);
void tim3_init(void);
void uart2_init(void);

peripherals_t* get_peripherals(void) {
  return &peripherals;
}

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  tim3_init();
  uart2_init();
}

void gpio_init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(VDD_IO_LEVEL_GPIO_PORT, VDD_IO_LEVEL_PIN, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = VDD_IO_LEVEL_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VDD_IO_LEVEL_GPIO_PORT, &gpio_init);

  HAL_GPIO_WritePin(VDD_IO_LEVEL_GPIO_PORT, VDD_IO_LEVEL_PIN, GPIO_PIN_SET);
}

void tim3_init(void) {
  TIM_HandleTypeDef* htim3 = &peripherals.htim3;
  TIM_ClockConfigTypeDef clock_source_config = {0};
  TIM_OC_InitTypeDef config_oc = {0};

  htim3->Instance = TIM3;
  htim3->Init.Prescaler = PWM_PSC_1MHZ;
  htim3->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3->Init.Period =
      1000;  // NOLINT (*-magic-numbers) // Dummy value, must be >0
  htim3->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3->Init.RepetitionCounter = 0;
  htim3->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(htim3) != HAL_OK) {
    error();
  }

  clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim3, &clock_source_config) != HAL_OK) {
    error();
  }

  if (HAL_TIM_PWM_Init(htim3) != HAL_OK) {
    error();
  }

  config_oc.OCMode = TIM_OCMODE_PWM1;
  config_oc.Pulse = 0;
  config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  config_oc.OCFastMode = TIM_OCFAST_DISABLE;
  config_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  config_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim3, &config_oc, TIM_CHANNEL_4) != HAL_OK) {
    error();
  }

  GPIO_InitTypeDef gpio_init = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  gpio_init.Pin = BUZZER_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(BUZZER_GPIO_PORT, &gpio_init);
}

void uart2_init(void) {
  UART_HandleTypeDef* huart2 = &peripherals.huart2;
  huart2->Instance = USART2;
  huart2->Init.BaudRate = 100000;                // NOLINT
  huart2->Init.WordLength = UART_WORDLENGTH_9B;  // 8 data bits, 1 parity bit
  huart2->Init.StopBits = UART_STOPBITS_2;
  huart2->Init.Parity = UART_PARITY_EVEN;
  huart2->Init.Mode = UART_MODE_RX;
  huart2->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2->Init.OverSampling = UART_OVERSAMPLING_16;
  huart2->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart2->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(huart2) != HAL_OK) {
    error();
  }
}

/**
 * @brief CAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN) {
    can_msp_init();
  }
}

/**
 * @brief CAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN) {
    can_msp_deinit();
  }
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_init();

  } else if (hspi->Instance == SPI2) {
    spi2_msp_init();
  }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    spi1_msp_deinit();

  } else if (hspi->Instance == SPI2) {
    spi2_msp_deinit();
  }
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM3) {
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM3) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    uart1_msp_init();

  } else if (huart->Instance == USART2) {
    GPIO_InitTypeDef gpio_init = {0};
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    gpio_init.Pin = UART2_RX_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(UART2_GPIO_PORT, &gpio_init);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, USART2_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    uart1_msp_deinit();
  } else if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(UART2_GPIO_PORT, UART2_RX_PIN);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}
