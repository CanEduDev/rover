#include "servo-internal.h"
#include "servo.h"

// STM32Common
#include "error.h"

// Testing
#include "fff.h"
#include "test.h"

DEFINE_FFF_GLOBALS

// NOLINTBEGIN
FAKE_VALUE_FUNC(int, read_servo_potentiometer, uint8_t*)
FAKE_VALUE_FUNC(int, write_servo_potentiometer, uint8_t)
// NOLINTEND

#define FFF_FAKES_LIST(FAKE)     \
  FAKE(read_servo_potentiometer) \
  FAKE(write_servo_potentiometer)

void reset_fakes(void) {
  FFF_FAKES_LIST(RESET_FAKE);
  FFF_RESET_HISTORY();
}

void test_update_voltage_controller_stable_voltage(void);
void test_update_voltage_controller_increase_voltage(void);
void test_update_voltage_controller_decrease_voltage(void);

// Helpers
int read_servo_potentiometer_returns_almost_max(uint8_t* pot_value);
int read_servo_potentiometer_returns_max(uint8_t* pot_value);
int read_servo_potentiometer_returns_almost_min(uint8_t* pot_value);
int read_servo_potentiometer_returns_min(uint8_t* pot_value);

int main(void) {
  test_update_voltage_controller_stable_voltage();
  test_update_voltage_controller_increase_voltage();
  test_update_voltage_controller_decrease_voltage();
}

void setup_test(void) {
  servo_init();

  // Reset fakes after setup to simplify test code
  reset_fakes();
}

void test_update_voltage_controller_stable_voltage(void) {
  setup_test();

  servo_state_t* servo_state = get_servo_state();
  const uint16_t target_voltage = 7400;
  const uint16_t actual_voltage = target_voltage;
  servo_state->target_voltage = target_voltage;
  servo_state->voltage = actual_voltage;

  update_voltage_controller();

  ASSERT(read_servo_potentiometer_fake.call_count == 0, "expected: 0, got: %u",
         read_servo_potentiometer_fake.call_count);
  ASSERT(write_servo_potentiometer_fake.call_count == 0, "expected: 0, got: %u",
         write_servo_potentiometer_fake.call_count);
}

void test_update_voltage_controller_increase_voltage(void) {
  setup_test();
  servo_state_t* servo_state = get_servo_state();
  const uint16_t target_voltage = 7400;
  const uint16_t actual_voltage = 6000;
  servo_state->target_voltage = target_voltage;
  servo_state->voltage = actual_voltage;

  // Lower pot value corresponds to a higher voltage
  int (*custom_fakes[])(uint8_t*) = {
      read_servo_potentiometer_returns_almost_min,
      read_servo_potentiometer_returns_min,
  };
  SET_CUSTOM_FAKE_SEQ(read_servo_potentiometer, custom_fakes, 2);

  update_voltage_controller();

  ASSERT(read_servo_potentiometer_fake.call_count == 1, "expected: 1, got: %u",
         read_servo_potentiometer_fake.call_count);
  ASSERT(write_servo_potentiometer_fake.call_count == 1, "expected: 1, got: %u",
         write_servo_potentiometer_fake.call_count);

  ASSERT(write_servo_potentiometer_fake.arg0_val == 0, "expected: 0, got: %u",
         write_servo_potentiometer_fake.arg0_val);

  update_voltage_controller();

  ASSERT(read_servo_potentiometer_fake.call_count == 2, "expected: 2, got: %u",
         read_servo_potentiometer_fake.call_count);
  ASSERT(write_servo_potentiometer_fake.call_count == 2, "expected: 2, got: %u",
         write_servo_potentiometer_fake.call_count);

  // Shouldn't try to decrease below 0 even though the voltage is still
  // lower than the target votlage.
  ASSERT(write_servo_potentiometer_fake.arg0_val == 0, "expected: 0, got: %u",
         write_servo_potentiometer_fake.arg0_val);
}

void test_update_voltage_controller_decrease_voltage(void) {
  setup_test();
  servo_state_t* servo_state = get_servo_state();
  const uint16_t target_voltage = 7400;
  const uint16_t actual_voltage = 8000;
  servo_state->target_voltage = target_voltage;
  servo_state->voltage = actual_voltage;

  // Higher pot value corresponds to a lower voltage
  int (*custom_fakes[])(uint8_t*) = {
      read_servo_potentiometer_returns_almost_max,
      read_servo_potentiometer_returns_max,
  };
  SET_CUSTOM_FAKE_SEQ(read_servo_potentiometer, custom_fakes, 2);

  update_voltage_controller();

  ASSERT(write_servo_potentiometer_fake.call_count == 1, "expected: 1, got: %u",
         write_servo_potentiometer_fake.call_count);

  ASSERT(write_servo_potentiometer_fake.arg0_val == UINT8_MAX,
         "expected: %u, got: %u", UINT8_MAX,
         write_servo_potentiometer_fake.arg0_val);

  update_voltage_controller();

  ASSERT(write_servo_potentiometer_fake.call_count == 2, "expected: 2, got: %u",
         write_servo_potentiometer_fake.call_count);

  // Shouldn't try to increase above UINT8_MAX even though the voltage is still
  // higher than the target votlage.
  ASSERT(write_servo_potentiometer_fake.arg0_val == UINT8_MAX,
         "expected: %u, got: %u", UINT8_MAX,
         write_servo_potentiometer_fake.arg0_val);
}

int read_servo_potentiometer_returns_almost_max(uint8_t* pot_value) {
  *pot_value = UINT8_MAX - 1;
  return APP_OK;
}

int read_servo_potentiometer_returns_max(uint8_t* pot_value) {
  *pot_value = UINT8_MAX;
  return APP_OK;
}

int read_servo_potentiometer_returns_almost_min(uint8_t* pot_value) {
  *pot_value = 1;
  return APP_OK;
}

int read_servo_potentiometer_returns_min(uint8_t* pot_value) {
  *pot_value = 0;
  return APP_OK;
}
