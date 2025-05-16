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

#define SPI3_SCK_PIN GPIO_PIN_10
#define SPI3_SCK_GPIO_PORT GPIOC

#define SPI3_MISO_PIN GPIO_PIN_11
#define SPI3_MISO_GPIO_PORT GPIOC

#define SPI3_MOSI_PIN GPIO_PIN_12
#define SPI3_MISO_GPIO_PORT GPIOC

#define SPI3_NSS_PIN GPIO_PIN_2
#define SPI3_NSS_GPIO_PORT GPIOD

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
