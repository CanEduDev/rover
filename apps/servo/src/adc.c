#include "adc.h"

#include "lt6106.h"
#include "math.h"
#include "voltage-divider.h"

#define ADC_REF_VOLTAGE_MV 3300
#define ADC_RESOLUTION_12BIT ((1 << 12) - 1)

#define ADC_REF_ANGLE_DEGREES 360

static float adc_value_to_voltage(uint16_t adc_value);
static float adc_value_to_angle(uint16_t adc_value);

void adc_average_samples(adc_reading_t *average,
                         const volatile adc_samples_t *samples) {
  uint32_t sum_adc1[ADC1_NUM_CHANNELS] = {0};
  uint32_t sum_adc2[ADC2_NUM_CHANNELS] = {0};

  for (uint32_t i = 0; i < ADC_NUM_SAMPLES * ADC1_NUM_CHANNELS; i++) {
    sum_adc1[i % ADC1_NUM_CHANNELS] += samples->adc1_buf[i];
  }

  for (uint8_t j = 0; j < ADC1_NUM_CHANNELS; j++) {
    average->adc1_buf[j] = sum_adc1[j] / ADC_NUM_SAMPLES;
  }

  for (uint32_t i = 0; i < ADC_NUM_SAMPLES * ADC2_NUM_CHANNELS; i++) {
    sum_adc2[i % ADC2_NUM_CHANNELS] += samples->adc2_buf[i];
  }

  for (uint8_t j = 0; j < ADC2_NUM_CHANNELS; j++) {
    average->adc2_buf[j] = sum_adc2[j] / ADC_NUM_SAMPLES;
  }
}

/* Convert the measured servo position sensor value to an angle.
 * Neutral is 180 deg, so we use it as base offset. Outputs -180 to 180 degrees.
 */
float adc_to_servo_position(uint16_t adc_value) {
  const float neutral_offset = 180.0F;
  float measured_angle = adc_value_to_angle(adc_value);
  float angle = measured_angle - neutral_offset;
  return angle;
}

uint16_t adc_to_servo_current(uint16_t adc_value) {
  const lt6106_current_sensor_t sensor = {
      .r_in = 51,
      .r_out = 5100,
      .r_sense = 0.002F,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  const float i_sense = lt6106_sense_current(measured_output_voltage, &sensor);
  return (uint16_t)roundf(i_sense);
}

/* To get battery voltage we need to add the voltage drop of the Schottky diode
 * which is around 300 mV.
 */
uint16_t adc_to_battery_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 47000,
      .r2 = 6200,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  const float battery_voltage =
      reverse_voltage_division(measured_output_voltage, &divider) + 300;
  return (uint16_t)roundf(battery_voltage);
}

uint16_t adc_to_servo_voltage(uint16_t adc_value) {
  const voltage_divider_t divider = {
      .r1 = 39000,
      .r2 = 15000,
  };
  float measured_output_voltage = adc_value_to_voltage(adc_value);
  float servo_voltage =
      reverse_voltage_division(measured_output_voltage, &divider);
  return (uint16_t)roundf(servo_voltage);
}

// Returns voltage in mV.
static float adc_value_to_voltage(uint16_t adc_value) {
  return ADC_REF_VOLTAGE_MV * adc_value / (float)ADC_RESOLUTION_12BIT;
}

static float adc_value_to_angle(uint16_t adc_value) {
  return ADC_REF_ANGLE_DEGREES * adc_value / (float)ADC_RESOLUTION_12BIT;
}
