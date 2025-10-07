#include "peripherals.h"

#include "error.h"
#include "ports.h"

#define TIM2_IRQ_PRIORITY 5
#define DMA1_Channel1_IRQ_PRIORITY 5
#define DMA1_Channel5_IRQ_PRIORITY 5

static peripherals_t peripherals;

void gpio_init(void);
void dma_init(void);
void adc1_init(void);
void i2c1_init(void);
void tim2_init(void);

peripherals_t* get_peripherals(void) {
  return &peripherals;
}

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  dma_init();
  i2c1_init();
  tim2_init();
}

void i2c1_init(void) {
  I2C_HandleTypeDef* hi2c1 = &peripherals.hi2c1;
  hi2c1->Instance = I2C1;
  hi2c1->Init.Timing = 0x10808DD3;  // NOLINT
  hi2c1->Init.OwnAddress1 = 0;
  hi2c1->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1->Init.OwnAddress2 = 0;
  hi2c1->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c1) != HAL_OK) {
    error();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    error();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(hi2c1, 0) != HAL_OK) {
    error();
  }
}

void tim2_init(void) {
  TIM_HandleTypeDef* htim2 = &peripherals.htim2;

  htim2->Instance = TIM2;
  htim2->Init.Prescaler = 0;
  htim2->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2->Init.Period = UINT32_MAX;
  htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim2) != HAL_OK) {
    error();
  }

  TIM_ClockConfigTypeDef clock_source_config = {
      .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
  };
  if (HAL_TIM_ConfigClockSource(htim2, &clock_source_config) != HAL_OK) {
    error();
  }

  if (HAL_TIM_IC_Init(htim2) != HAL_OK) {
    error();
  }

  TIM_MasterConfigTypeDef master_config;
  master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim2, &master_config) != HAL_OK) {
    error();
  }

  TIM_SlaveConfigTypeDef slave_config;
  slave_config.SlaveMode = TIM_SLAVEMODE_RESET;
  slave_config.InputTrigger = TIM_TS_TI1FP1;
  slave_config.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  slave_config.TriggerPrescaler = TIM_ICPSC_DIV1;
  slave_config.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(htim2, &slave_config) != HAL_OK) {
    error();
  }

  TIM_IC_InitTypeDef ic_config = {0};
  ic_config.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  ic_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic_config.ICPrescaler = TIM_ICPSC_DIV1;
  ic_config.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(htim2, &ic_config, TIM_CHANNEL_1) != HAL_OK) {
    error();
  }

  HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, DMA1_Channel5_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

void gpio_init(void) {
  GPIO_InitTypeDef gpio_init = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(DEBUG_LED_GPIO_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);

  gpio_init.Pin = DEBUG_LED_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_PORT, &gpio_init);
}

void tim2_dma_init(TIM_HandleTypeDef* htim) {
  DMA_HandleTypeDef* hdma_tim2_ch1 = &peripherals.hdma_tim2_ch1;
  hdma_tim2_ch1->Instance = DMA1_Channel5;
  hdma_tim2_ch1->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_tim2_ch1->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1->Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1->Init.Mode = DMA_NORMAL;
  hdma_tim2_ch1->Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(hdma_tim2_ch1) != HAL_OK) {
    error();
  }

  __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], *hdma_tim2_ch1);
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
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
  GPIO_InitTypeDef gpio_init = {0};
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    gpio_init.Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C1_SCL_GPIO_PORT, &gpio_init);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1) {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(I2C1_SCL_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN);
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
  GPIO_InitTypeDef gpio_init = {0};
  if (htim_base->Instance == TIM2) {
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init.Pin = SENSOR_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(SENSOR_GPIO_PORT, &gpio_init);

    tim2_dma_init(htim_base);
  }
}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM2) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_GPIO_DeInit(SENSOR_GPIO_PORT, SENSOR_PIN);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
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
  }
}
