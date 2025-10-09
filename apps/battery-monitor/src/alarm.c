#include "alarm.h"

#include <stdio.h>
#include <string.h>

#include "ck-data.h"

// CK
#include "ck-types.h"
#include "mayor.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

#define ALARM_TIMER_PERIOD_MS 1000

static StaticTimer_t alarm_timer_buf;
static TimerHandle_t alarm_timer;
static alarm_t alarm_state;

static void alarm_timer_callback(TimerHandle_t timer);
static void send_buzzer_message(void);

void alarm_init(void) {
  alarm_timer =
      xTimerCreateStatic("alarm timer", pdMS_TO_TICKS(ALARM_TIMER_PERIOD_MS),
                         pdTRUE,  // Auto reload timer
                         NULL,    // Timer ID, unused
                         alarm_timer_callback, &alarm_timer_buf);

  // Initialize alarm state
  alarm_state.volume = DEFAULT_ALARM_VOLUME;
  alarm_state.frequency = DEFAULT_ALARM_FREQUENCY;
  alarm_state.duration = DEFAULT_ALARM_DURATION;
}

void alarm_start(void) {
  if (xTimerIsTimerActive(alarm_timer)) {
    return;
  }
  xTimerStart(alarm_timer, portMAX_DELAY);
}

void alarm_stop(void) {
  if (!xTimerIsTimerActive(alarm_timer)) {
    return;
  }
  xTimerStop(alarm_timer, portMAX_DELAY);
}

static void alarm_timer_callback(TimerHandle_t timer) {
  (void)timer;
  send_buzzer_message();
}

static void send_buzzer_message(void) {
  ck_data_t *ck_data = get_ck_data();

  // Create buzzer sound data (6 bytes total)
  uint16_t frequency = alarm_state.frequency;
  uint16_t duration = alarm_state.duration;
  uint16_t volume = alarm_state.volume;

  // Pack the data into the alarm page (little endian)
  memcpy(&ck_data->alarm_page->lines[0], &frequency, sizeof(frequency));
  memcpy(&ck_data->alarm_page->lines[2], &duration, sizeof(duration));
  memcpy(&ck_data->alarm_page->lines[4], &volume, sizeof(volume));

  // Send the document using the alarm folder
  ck_err_t ret = ck_send_document(ck_data->alarm_folder->folder_no);
  if (ret != CK_OK) {
    printf("error: failed to send buzzer message: %d\r\n", ret);
  }
}
