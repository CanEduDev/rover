#include "peripherals.h"

#include "adc.h"
#include "error.h"
#include "ports.h"

#define DMA1_Channel1_IRQ_PRIORITY 5
#define DMA2_Channel1_IRQ_PRIORITY 5

// Some definitions for PWM
#define PWM_PSC_1MHZ (72 - 1)
#define PWM_PERIOD_50HZ (20000 - 1)
#define PWM_MID_POS_PULSE 1500  // 1500 microseconds

static peripherals_t peripherals;

static uint32_t hal_rcc_adc12_clk_enabled = 0;

void gpio_init(void);
void dma_init(void);
void adc1_init(void);
void adc2_init(void);
void i2c1_init(void);
void i2c3_init(void);
void spi3_init(void);
void tim1_init(void);

peripherals_t* get_peripherals(void) {
  return &peripherals;
}

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  dma_init();  // Must be called before ADC init functions.
  adc1_init();
  adc2_init();
  i2c1_init();
  i2c3_init();
  spi3_init();
  tim1_init();
}

void adc1_init(void) {
  ADC_HandleTypeDef* hadc1 = &peripherals.hadc1;
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef config;

  /** Common config
   */
  hadc1->Instance = ADC1;
  hadc1->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1->Init.Resolution = ADC_RESOLUTION_12B;
  hadc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1->Init.ContinuousConvMode = ENABLE;
  hadc1->Init.DiscontinuousConvMode = DISABLE;
  hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1->Init.NbrOfConversion = ADC1_NUM_CHANNELS;
  hadc1->Init.DMAContinuousRequests = DISABLE;
  hadc1->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1->Init.LowPowerAutoWait = DISABLE;
  hadc1->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc1) != HAL_OK) {
    error();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_1;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_3;
  config.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_4;
  config.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(hadc1, &config) != HAL_OK) {
    error();
  }

  if (HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    error();
  }
}

void adc2_init(void) {
  ADC_HandleTypeDef* hadc2 = &peripherals.hadc2;
  ADC_ChannelConfTypeDef config;

  /** Common config
   */
  hadc2->Instance = ADC2;
  hadc2->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2->Init.Resolution = ADC_RESOLUTION_12B;
  hadc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2->Init.ContinuousConvMode = ENABLE;
  hadc2->Init.DiscontinuousConvMode = DISABLE;
  hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2->Init.NbrOfConversion = ADC2_NUM_CHANNELS;
  hadc2->Init.DMAContinuousRequests = DISABLE;
  hadc2->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2->Init.LowPowerAutoWait = DISABLE;
  hadc2->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc2) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_1;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  /** Configure Regular Channel
   */
  config.Channel = ADC_CHANNEL_2;
  config.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(hadc2, &config) != HAL_OK) {
    error();
  }

  if (HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    error();
  }
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

void i2c3_init(void) {
  I2C_HandleTypeDef* hi2c3 = &peripherals.hi2c3;
  hi2c3->Instance = I2C3;
  hi2c3->Init.Timing = 0x10808DD3;  // NOLINT
  hi2c3->Init.OwnAddress1 = 0;
  hi2c3->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3->Init.OwnAddress2 = 0;
  hi2c3->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c3) != HAL_OK) {
    error();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    error();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(hi2c3, 0) != HAL_OK) {
    error();
  }
}

void spi3_init(void) {
  SPI_HandleTypeDef* hspi3 = &peripherals.hspi3;
  hspi3->Instance = SPI3;
  hspi3->Init.Mode = SPI_MODE_MASTER;
  hspi3->Init.Direction = SPI_DIRECTION_2LINES;
  hspi3->Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3->Init.NSS = SPI_NSS_SOFT;
  hspi3->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3->Init.CRCPolynomial = 7;  // NOLINT
  hspi3->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(hspi3) != HAL_OK) {
    error();
  }
}

void tim1_init(void) {
  TIM_HandleTypeDef* htim1 = &peripherals.htim1;
  TIM_ClockConfigTypeDef clock_source_config;
  TIM_OC_InitTypeDef config_oc;

  htim1->Instance = TIM1;
  htim1->Init.Prescaler = PWM_PSC_1MHZ;
  htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1->Init.Period = PWM_PERIOD_50HZ;
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.RepetitionCounter = 0;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(htim1) != HAL_OK) {
    error();
  }

  clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim1, &clock_source_config) != HAL_OK) {
    error();
  }

  if (HAL_TIM_PWM_Init(htim1) != HAL_OK) {
    error();
  }

  config_oc.OCMode = TIM_OCMODE_PWM1;
  config_oc.Pulse = PWM_MID_POS_PULSE;
  config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  config_oc.OCFastMode = TIM_OCFAST_DISABLE;
  config_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  config_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim1, &config_oc, TIM_CHANNEL_4) != HAL_OK) {
    error();
  }

  GPIO_InitTypeDef gpio_init;
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /**TIM1 GPIO Configuration
  PC3     ------> TIM1_CH4
  */
  gpio_init.Pin = SERVO_PWM_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(SERVO_PWM_GPIO_PORT, &gpio_init);
}

void dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, DMA1_Channel1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, DMA2_Channel1_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}

void gpio_init(void) {
  GPIO_InitTypeDef gpio_init;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(DEBUG_LED_GPIO_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_RESET);

  gpio_init.Pin = DEBUG_LED_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_PORT, &gpio_init);

  /*Configure GPIO pin : SPI3_NSS_PIN */
  gpio_init.Pin = SPI3_NSS_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_NSS_GPIO_PORT, &gpio_init);
}

/* ADC1 DMA Init */
void adc1_dma_init(ADC_HandleTypeDef* hadc) {
  DMA_HandleTypeDef* hdma_adc1 = &peripherals.hdma_adc1;
  hdma_adc1->Instance = DMA1_Channel1;
  hdma_adc1->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1->Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1->Init.Mode = DMA_NORMAL;
  hdma_adc1->Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(hdma_adc1) != HAL_OK) {
    error();
  }
  __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc1);
}

/* ADC2 DMA Init */
void adc2_dma_init(ADC_HandleTypeDef* hadc) {
  DMA_HandleTypeDef* hdma_adc2 = &peripherals.hdma_adc2;
  hdma_adc2->Instance = DMA2_Channel1;
  hdma_adc2->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2->Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc2->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2->Init.Mode = DMA_NORMAL;
  hdma_adc2->Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(hdma_adc2) != HAL_OK) {
    error();
  }

  __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc2);
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef gpio_init;
  if (hadc->Instance == ADC1) {
    /* Peripheral clock enable */
    hal_rcc_adc12_clk_enabled++;
    if (hal_rcc_adc12_clk_enabled == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    gpio_init.Pin = SENSOR_PIN | SERVO_CURRENT_PIN | BAT_VOLTAGE_PIN;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SENSOR_GPIO_PORT, &gpio_init);
    adc1_dma_init(hadc);

  } else if (hadc->Instance == ADC2) {
    /* Peripheral clock enable */
    hal_rcc_adc12_clk_enabled++;
    if (hal_rcc_adc12_clk_enabled == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    */
    gpio_init.Pin = VCC_SERVO_VOLTAGE_PIN;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VCC_SERVO_VOLTAGE_GPIO_PORT, &gpio_init);
    adc2_dma_init(hadc);
  }
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    /* Peripheral clock disable */
    hal_rcc_adc12_clk_enabled--;
    if (hal_rcc_adc12_clk_enabled == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(SENSOR_GPIO_PORT,
                    SENSOR_PIN | SERVO_CURRENT_PIN | BAT_VOLTAGE_PIN);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

  } else if (hadc->Instance == ADC2) {
    /* Peripheral clock disable */
    hal_rcc_adc12_clk_enabled--;
    if (hal_rcc_adc12_clk_enabled == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    */
    HAL_GPIO_DeInit(VCC_SERVO_VOLTAGE_GPIO_PORT, VCC_SERVO_VOLTAGE_PIN);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
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
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
  GPIO_InitTypeDef gpio_init;
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

  } else if (hi2c->Instance == I2C3) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    gpio_init.Pin = I2C3_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF3_I2C3;
    HAL_GPIO_Init(I2C3_SDA_PORT, &gpio_init);

    gpio_init.Pin = I2C3_SCL_PIN;
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF3_I2C3;
    HAL_GPIO_Init(I2C3_SCL_PORT, &gpio_init);

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
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

  } else if (hi2c->Instance == I2C3) {
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(I2C3_SDA_PORT, I2C3_SDA_PIN);

    HAL_GPIO_DeInit(I2C3_SCL_PORT, I2C3_SCL_PIN);
  }
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  GPIO_InitTypeDef gpio_init;
  if (hspi->Instance == SPI1) {
    spi1_msp_init();

  } else if (hspi->Instance == SPI2) {
    spi2_msp_init();

  } else if (hspi->Instance == SPI3) {
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    gpio_init.Pin = SPI3_SCK_PIN | SPI3_MISO_PIN | SPI3_MOSI_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(SPI3_SCK_GPIO_PORT, &gpio_init);
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

  } else if (hspi->Instance == SPI3) {
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(SPI3_SCK_GPIO_PORT,
                    SPI3_SCK_PIN | SPI3_MISO_PIN | SPI3_MOSI_PIN);
  }
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM1) {
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM1) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
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
