#include <stdio.h>
#include <string.h>

#include "ck-data.h"
#include "error.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define WHEEL_SPEED_TASK_STACK_SIZE (2 * configMINIMAL_STACK_SIZE)
static TaskHandle_t wheel_speed_task;
static StaticTask_t wheel_speed_task_buf;
static StackType_t wheel_speed_task_stack[WHEEL_SPEED_TASK_STACK_SIZE];

void measure_wheel_speed(void* unused);
uint32_t calculate_measure_delay(uint32_t rps);

#define TIM2_FREQ_HZ (72 * 1000 * 1000)
#define DEFAULT_WHEEL_DIAMETER_M 0.16F
#define DEFAULT_COG_COUNT 45

typedef struct {
  uint32_t cog_count;
  float wheel_diameter_m;
} wheel_speed_t;

static wheel_speed_t ws_params = {
    .wheel_diameter_m = DEFAULT_WHEEL_DIAMETER_M,
    .cog_count = DEFAULT_COG_COUNT,
};

void init_wheel_speed_task(uint8_t priority) {
  wheel_speed_task = xTaskCreateStatic(
      measure_wheel_speed, "wheel speed", WHEEL_SPEED_TASK_STACK_SIZE, NULL,
      priority, wheel_speed_task_stack, &wheel_speed_task_buf);
}

// 8 bytes in page
// bytes 0-3: number of cogs in wheel (uint32_t)
// bytes 4-7: wheel diameter in meters (float)
int process_set_wheel_parameters_letter(const ck_letter_t* letter) {
  ck_data_t* ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_wheel_parameters_folder->dlc) {
    return APP_NOT_OK;
  }

  uint32_t cog_count = 0;
  memcpy(&cog_count, letter->page.lines, sizeof(cog_count));
  if (cog_count != 0) {
    ws_params.cog_count = cog_count;
  }

  float wheel_diameter_m = 0;
  memcpy(&wheel_diameter_m, &letter->page.lines[4], sizeof(wheel_diameter_m));

  if (wheel_diameter_m <= 0) {
    return APP_NOT_OK;
  }

  if (wheel_diameter_m > 0) {
    ws_params.wheel_diameter_m = wheel_diameter_m;
  }

  return APP_OK;
}

void measure_wheel_speed(void* unused) {
  (void)unused;

  peripherals_t* peripherals = get_peripherals();

  const uint32_t max_measure_time_ms = 100;
  const float pi = 3.14159F;
  const float mps_to_kph = 3.6F;
  const float rps_to_rpm = 60;

  ck_data_t* ck_data = get_ck_data();

  const size_t ic_value_bufsize = 10;
  uint32_t ic_value_buf[ic_value_bufsize];

  while (1) {
    memset(ic_value_buf, 0, sizeof(ic_value_buf));
    HAL_TIM_IC_Start_DMA(&peripherals->htim2, TIM_CHANNEL_1, ic_value_buf,
                         ic_value_bufsize);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(max_measure_time_ms));
    HAL_TIM_IC_Stop_DMA(&peripherals->htim2, TIM_CHANNEL_1);

    // Gather samples
    float rps = 0;
    uint8_t samples = 0;
    for (size_t i = 0; i < ic_value_bufsize; i++) {
      if (ic_value_buf[i] > 0) {
        samples++;
        rps += TIM2_FREQ_HZ /
               ((float)ic_value_buf[i] * (float)ws_params.cog_count);
      }
    }

    // Average
    if (samples > 0) {
      rps /= (float)samples;
    }

    float rpm = rps * rps_to_rpm;
    float speed = pi * ws_params.wheel_diameter_m * rps * mps_to_kph;

    memcpy(ck_data->wheel_speed_page->lines, &rpm, sizeof(float));
    memcpy(&ck_data->wheel_speed_page->lines[4], &speed, sizeof(float));
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(wheel_speed_task, &higher_priority_task_woken);
  }
}
