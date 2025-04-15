/*******************************************************************************
 * @file ports.h
 *
 * Contains preprocessor definitions for the various GPIO ports.
 ******************************************************************************/
#ifndef PORTS_H
#define PORTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

#define LIGHT_1_PIN GPIO_PIN_4
#define LIGHT_1_GPIO_PORT GPIOC

#define LIGHT_2_PIN GPIO_PIN_5
#define LIGHT_2_GPIO_PORT GPIOC

#define LIGHT_3_PIN GPIO_PIN_0
#define LIGHT_3_GPIO_PORT GPIOB

#define LIGHT_4_PIN GPIO_PIN_1
#define LIGHT_4_GPIO_PORT GPIOB

#define VDD_IO_LEVEL_PIN GPIO_PIN_2
#define VDD_IO_LEVEL_GPIO_PORT GPIOB

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
