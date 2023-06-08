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

#define SWITCH4_PIN2_Pin GPIO_PIN_13
#define SWITCH4_PIN2_GPIO_Port GPIOC
#define SWITCH2_PIN1_Pin GPIO_PIN_15
#define SWITCH2_PIN1_GPIO_Port GPIOC
#define GPIO0_Pin GPIO_PIN_0
#define GPIO0_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_1
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_2
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_3
#define GPIO3_GPIO_Port GPIOC
#define AN1_Pin GPIO_PIN_0
#define AN1_GPIO_Port GPIOA
#define AN2_Pin GPIO_PIN_1
#define AN2_GPIO_Port GPIOA
#define AN3_Pin GPIO_PIN_2
#define AN3_GPIO_Port GPIOA
#define AN4_Pin GPIO_PIN_3
#define AN4_GPIO_Port GPIOA
#define SWITCH3_PIN1_Pin GPIO_PIN_4
#define SWITCH3_PIN1_GPIO_Port GPIOA
#define SWITCH3_PIN2_Pin GPIO_PIN_5
#define SWITCH3_PIN2_GPIO_Port GPIOA
#define SWITCH4_PIN1_Pin GPIO_PIN_6
#define SWITCH4_PIN1_GPIO_Port GPIOA
#define GPIO_PWRON_1_Pin GPIO_PIN_4
#define GPIO_PWRON_1_GPIO_Port GPIOC
#define GPIO_PWRON_2_Pin GPIO_PIN_5
#define GPIO_PWRON_2_GPIO_Port GPIOC
#define GPIO_PWRON_3_Pin GPIO_PIN_0
#define GPIO_PWRON_3_GPIO_Port GPIOB
#define GPIO_PWRON_4_Pin GPIO_PIN_1
#define GPIO_PWRON_4_GPIO_Port GPIOB
#define VDD_IO_LEVEL_Pin GPIO_PIN_2
#define VDD_IO_LEVEL_GPIO_Port GPIOB
#define SWITCH1_PIN1_Pin GPIO_PIN_10
#define SWITCH1_PIN1_GPIO_Port GPIOB
#define SWITCH1_PIN2_Pin GPIO_PIN_11
#define SWITCH1_PIN2_GPIO_Port GPIOB
#define SWITCH2_PIN2_Pin GPIO_PIN_6
#define SWITCH2_PIN2_GPIO_Port GPIOC
#define SPI3_NSS_Pin GPIO_PIN_2
#define SPI3_NSS_GPIO_Port GPIOD

#ifdef __cplusplus
}
#endif

#endif /* PORTS_H */
