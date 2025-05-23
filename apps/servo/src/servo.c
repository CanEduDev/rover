#include "servo.h"

#include <math.h>

#include "error.h"
#include "failsafe.h"
#include "potentiometer.h"
#include "pwm.h"

// y = kx + m. 500 µs pulse corresponds to 45 degree movement,
// yielding k = 500/45. m is the neutral position.
static const float k_angle_to_pulse = 500 / 45.0F;
static const float m_angle_to_pulse = PWM_NEUTRAL_PULSE_MUS;

static servo_state_t servo;

void update_voltage_controller(void);
bool is_servo_voltage_stable(void);

// Helpers
static int32_t angle_to_pulse(float angle);
static float pulse_to_angle(int32_t pulse);
static float subtrim_pulse_to_angle(int32_t pulse);
static int32_t reverse_pulse(int32_t pulse);

servo_state_t* get_servo_state(void) {
  return &servo;
}

void servo_init(void) {
  servo.target_voltage = DEFAULT_SERVO_VOLTAGE_MV;
  servo.voltage = 0;
  servo.current = 0;
  servo.steering_angle = 0;
  servo.steering_pulse = PWM_NEUTRAL_PULSE_MUS;
  servo.position_enabled = false;
  servo.position = 0;
  servo.reverse = false;

  init_potentiometers();
  write_sensor_potentiometer(POTENTIOMETER_SENSOR_DEFAULT);
  write_servo_potentiometer(POTENTIOMETER_SERVO_DEFAULT);

  failsafe_init();
  failsafe_on();

  pwm_init();
  pwm_set_pulse(servo.steering_pulse);
}

void update_servo_state(adc_reading_t* adc_reading) {
  servo.voltage = adc_to_servo_voltage(adc_reading->adc2_buf[0]);
  servo.current = adc_to_servo_current(adc_reading->adc1_buf[1]);

  float subtrim_angle = subtrim_pulse_to_angle(pwm_get_subtrim());
  float angle = adc_to_servo_position(adc_reading->adc1_buf[0]);
  servo.position = angle + subtrim_angle;

  update_voltage_controller();
  pwm_set_pulse(servo.steering_pulse);
}

void update_voltage_controller(void) {
  if (is_servo_voltage_stable()) {
    return;
  }

  uint8_t current_pot_value = 0;
  if (read_servo_potentiometer(&current_pot_value) != APP_OK) {
    return;
  }

  int16_t next_pot_value = current_pot_value;

  if (servo.voltage > servo.target_voltage && next_pot_value < UINT8_MAX) {
    next_pot_value++;
  }

  if (servo.voltage < servo.target_voltage && next_pot_value > 0) {
    next_pot_value--;
  }

  write_servo_potentiometer(next_pot_value);
}

bool is_servo_voltage_stable(void) {
  const uint8_t accepted_error = 10;
  int32_t max_voltage = (int32_t)servo.target_voltage + accepted_error;
  int32_t min_voltage = (int32_t)servo.target_voltage - accepted_error;

  return servo.voltage < max_voltage && servo.voltage > min_voltage;
}

int update_servo_pulse(int32_t pulse) {
  if (pulse > PWM_MAX_PULSE || pulse < PWM_MIN_PULSE) {
    return APP_NOT_OK;
  }

  if (servo.reverse) {
    pulse = reverse_pulse(pulse);
  }

  servo.steering_pulse = pulse;
  servo.steering_angle = pulse_to_angle(pulse);

  failsafe_refresh();

  return APP_OK;
}

int update_servo_angle(float angle) {
  const float min_angle = -90;
  const float max_angle = 90;

  if (angle > max_angle || angle < min_angle) {
    return APP_NOT_OK;
  }

  if (servo.reverse) {
    angle = -angle;
  }

  servo.steering_angle = angle;
  servo.steering_pulse = angle_to_pulse(angle);

  failsafe_refresh();

  return APP_OK;
}

static int32_t angle_to_pulse(float angle) {
  return (int32_t)roundf((angle * k_angle_to_pulse) + m_angle_to_pulse);
}

static float pulse_to_angle(int32_t pulse) {
  return ((float)(pulse)-m_angle_to_pulse) / k_angle_to_pulse;
}

static float subtrim_pulse_to_angle(int32_t pulse) {
  return (float)pulse / k_angle_to_pulse;
}

static int32_t reverse_pulse(int32_t pulse) {
  return (2 * PWM_NEUTRAL_PULSE_MUS) - pulse;
}
