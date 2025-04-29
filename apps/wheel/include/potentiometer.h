#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#define POTENTIOMETER_SENSOR_SUPPLY 80    // Gives 5.2V
#define POTENTIOMETER_SENSOR_MEASURE 185  // Gives 3.3V

int init_potentiometers(void);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
