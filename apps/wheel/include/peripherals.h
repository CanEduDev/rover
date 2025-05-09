/*******************************************************************************
 * @file peripherals.h
 *
 * Provides peripheral initialization functions.
 ******************************************************************************/
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common-peripherals.h"

typedef struct {
  // Provided by CPU board
  common_peripherals_t *common_peripherals;

  DMA_HandleTypeDef hdma_tim2_ch1;
  I2C_HandleTypeDef hi2c1;
  TIM_HandleTypeDef htim2;
} peripherals_t;

peripherals_t *get_peripherals(void);

void peripherals_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
