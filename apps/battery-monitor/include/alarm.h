#ifndef ALARM_H
#define ALARM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_ALARM_VOLUME 100
#define DEFAULT_ALARM_FREQUENCY 2000
#define DEFAULT_ALARM_DURATION 200

typedef struct {
  uint16_t volume;
  uint16_t frequency;
  uint16_t duration;
} alarm_t;

void alarm_init(void);
void alarm_start(void);
void alarm_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ALARM_H */
