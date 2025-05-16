#include "peripherals.h"

#include "error.h"
#include "ports.h"

static peripherals_t peripherals;

void gpio_init(void);
void spi3_init(void);

peripherals_t* get_peripherals(void) {
  return &peripherals;
}

void peripherals_init(void) {
  common_peripherals_init();
  peripherals.common_peripherals = get_common_peripherals();

  gpio_init();
  spi3_init();
}

void gpio_init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init = {0};

  HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI3_NSS_PIN */
  gpio_init.Pin = SPI3_NSS_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_NSS_GPIO_PORT, &gpio_init);
}

void spi3_init(void) {
  SPI_HandleTypeDef* hspi3 = &peripherals.hspi3;
  hspi3->Instance = SPI3;
  hspi3->Init.Mode = SPI_MODE_MASTER;
  hspi3->Init.Direction = SPI_DIRECTION_2LINES;
  hspi3->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3->Init.NSS = SPI_NSS_SOFT;
  hspi3->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3->Init.FirstBit = SPI_FIRSTBIT_MSB;
  if (HAL_SPI_Init(hspi3) != HAL_OK) {
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
  } else if (hspi->Instance == SPI3) {
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitTypeDef gpio_init = {0};
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
