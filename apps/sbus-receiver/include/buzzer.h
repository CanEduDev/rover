#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ck-types.h"

void init_buzzer_task(uint8_t priority);

int process_buzzer_sound_letter(const ck_letter_t *letter);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
