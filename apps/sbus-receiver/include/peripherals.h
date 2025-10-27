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

  TIM_HandleTypeDef htim2;
  UART_HandleTypeDef huart2;
} peripherals_t;

peripherals_t *get_peripherals(void);

void peripherals_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
