#include <stdio.h>

#include "json.h"
#include "jsondb.h"
#include "servo.h"

void set_reverse_setting(json_object_t *settings);
void set_position_sensor_setting(json_object_t *settings);

void settings_init(void) {
  json_object_t *json = get_jsondb();
  json_object_t *settings = json_get_object("settings", json);
  if (!settings) {
    printf("note: no settings stored\r\n");
    return;
  }

  set_reverse_setting(settings);
  set_position_sensor_setting(settings);
}

void set_reverse_setting(json_object_t *settings) {
  json_object_t *reverse = json_get_object("reverse", settings);
  if (!reverse) {
    return;
  }

  if (reverse->type != JSON_BOOL) {
    printf("note: invalid reverse setting, using default\r\n");
    return;
  }

  servo_state_t *s = get_servo_state();
  s->reverse = reverse->value->boolean;
}

void set_position_sensor_setting(json_object_t *settings) {
  json_object_t *position_enabled =
      json_get_object("position_enabled", settings);
  if (!position_enabled) {
    return;
  }

  if (position_enabled->type != JSON_BOOL) {
    printf("note: invalid position_enabled setting, using default\r\n");
    return;
  }

  servo_state_t *s = get_servo_state();
  s->position_enabled = position_enabled->value->boolean;
}
