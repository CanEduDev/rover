#include <stdio.h>

#include "peripherals.h"
#include "ports.h"

// STM32Common
#include "clock.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3xx_hal_spi.h"
#include "task.h"

static StackType_t task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t task_buf;

void task_fn(void *unused);

int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  system_clock_init();

  // Initialize all configured peripherals
  peripherals_init();

  xTaskCreateStatic(task_fn, "main", configMINIMAL_STACK_SIZE, NULL,
                    LOWEST_TASK_PRIORITY, task_stack, &task_buf);

  printf("Starting application...\r\n");

  vTaskStartScheduler();

  while (1) {
  }
}

void task_fn(void *unused) {
  (void)unused;

  SPI_HandleTypeDef *spi = &get_peripherals()->hspi3;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint8_t tx = 1;
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, &tx, 1, pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_SET);

    for (int i = 0; i < 10; i++) {
      __NOP();
    }

    uint8_t rx = 0;
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Receive(spi, &rx, 1, pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_PORT, SPI3_NSS_PIN, GPIO_PIN_SET);

    printf("got %x\r\n", rx);
  }
}
