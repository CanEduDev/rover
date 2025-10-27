#include "buzzer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ck-data.h"
#include "error.h"
#include "peripherals.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#define TIMER_FREQ 1000000
#define BUZZER_QUEUE_SIZE 8

typedef struct {
  uint16_t frequency_hz;  // Accepts frequency > 0
  uint16_t duration_ms;   // Accepts duration > 10 ms
  uint16_t volume;
} buzzer_sound_t;

static StaticTimer_t tone_timer_buf;
static TimerHandle_t tone_timer;

static StaticQueue_t buzzer_queue_buf;
static QueueHandle_t buzzer_queue;
static uint8_t buzzer_queue_storage[BUZZER_QUEUE_SIZE * sizeof(buzzer_sound_t)];

static TaskHandle_t buzzer_task;
static StaticTask_t buzzer_task_buf;
static StackType_t buzzer_task_stack[configMINIMAL_STACK_SIZE];

static void set_buzzer_tone(uint16_t frequency_hz);
static void set_buzzer_volume(uint16_t volume);
static void tone_timer_callback(TimerHandle_t timer);
static void buzzer_task_func(void *unused);
static void buzzer_play_sound(const buzzer_sound_t *sound);
static int buzzer_init(void);

void init_buzzer_task(uint8_t priority) {
  if (buzzer_init() != APP_OK) {
    printf("error: failed to initialize buzzer\r\n");
    error();
  }

  buzzer_task =
      xTaskCreateStatic(buzzer_task_func, "buzzer", configMINIMAL_STACK_SIZE,
                        NULL, priority, buzzer_task_stack, &buzzer_task_buf);
}

// 6 bytes in page
//
// bytes 0-1: Sound frequency in Hz (uint16_t)
// bytes 2-3: Sound duration in ms (uint16_t)
// bytes 4-5: Sound volume (uint16_t)
int process_buzzer_sound_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->buzzer_sound_folder->dlc) {
    return APP_NOT_OK;
  }

  uint16_t frequency = 0;
  uint16_t duration = 0;
  uint16_t volume = 0;

  memcpy(&frequency, &letter->page.lines[0], sizeof(frequency));
  memcpy(&duration, &letter->page.lines[2], sizeof(duration));
  memcpy(&volume, &letter->page.lines[4], sizeof(volume));

  buzzer_sound_t sound = {
      .frequency_hz = frequency,
      .duration_ms = duration,
      .volume = volume,
  };

  if (xQueueSend(buzzer_queue, &sound, pdMS_TO_TICKS(1)) != pdPASS) {
    printf("error: buzzer queue full, dropping sound\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

static int buzzer_init(void) {
  peripherals_t *peripherals = get_peripherals();

  if (HAL_TIM_PWM_Start(&peripherals->htim2, TIM_CHANNEL_1) != HAL_OK) {
    printf("error: failed to start buzzer timer\r\n");
    return APP_NOT_OK;
  }

  tone_timer = xTimerCreateStatic("tone timer", pdMS_TO_TICKS(1000), pdFALSE,
                                  NULL, tone_timer_callback, &tone_timer_buf);

  buzzer_queue = xQueueCreateStatic(BUZZER_QUEUE_SIZE, sizeof(buzzer_sound_t),
                                    buzzer_queue_storage, &buzzer_queue_buf);
  if (buzzer_queue == NULL) {
    printf("error: failed to create buzzer queue\r\n");
    return APP_NOT_OK;
  }

  return APP_OK;
}

static void buzzer_task_func(void *unused) {
  (void)unused;

  buzzer_sound_t sound;

  for (;;) {
    if (xQueueReceive(buzzer_queue, &sound, portMAX_DELAY) == pdPASS) {
      buzzer_play_sound(&sound);
    }
  }
}

static void buzzer_play_sound(const buzzer_sound_t *sound) {
  if (sound->frequency_hz > 0) {
    set_buzzer_tone(sound->frequency_hz);
    set_buzzer_volume(sound->volume);
  }

  if (sound->duration_ms > 0) {
    xTimerChangePeriod(tone_timer, pdMS_TO_TICKS(sound->duration_ms),
                       portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }

  set_buzzer_volume(0);  // Reset default state
}

static void tone_timer_callback(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(buzzer_task);
}

static void set_buzzer_tone(uint16_t frequency_hz) {
  peripherals_t *peripherals = get_peripherals();
  __HAL_TIM_SET_AUTORELOAD(&peripherals->htim2,
                           (TIMER_FREQ / frequency_hz) - 1);
}

static void set_buzzer_volume(uint16_t volume) {
  peripherals_t *peripherals = get_peripherals();
  // Limit to 90% duty cycle
  const uint32_t max_volume =
      (__HAL_TIM_GET_AUTORELOAD(&peripherals->htim2) / 10) * 9;
  if (volume > max_volume) {
    volume = max_volume;
  }
  __HAL_TIM_SET_COMPARE(&peripherals->htim2, TIM_CHANNEL_1, volume);
}
