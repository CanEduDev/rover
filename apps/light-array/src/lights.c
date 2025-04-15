#include "lights.h"

#include "ck-data.h"
#include "error.h"
#include "ports.h"

// 4 bytes in page
//
// byte 0: Light 1 state, 0=OFF, else ON
// byte 1: Light 2 state, 0=OFF, else ON
// byte 2: Light 3 state, 0=OFF, else ON
// byte 3: Light 4 state, 0=OFF, else ON
int process_light_state_letter(const ck_letter_t *letter) {
  ck_data_t *ck_data = get_ck_data();
  if (letter->page.line_count != ck_data->set_light_state_folder->dlc) {
    return APP_NOT_OK;
  }

  if (letter->page.lines[0] > 0) {
    HAL_GPIO_WritePin(LIGHT_1_GPIO_PORT, LIGHT_1_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LIGHT_1_GPIO_PORT, LIGHT_1_PIN, GPIO_PIN_RESET);
  }

  if (letter->page.lines[1] > 0) {
    HAL_GPIO_WritePin(LIGHT_2_GPIO_PORT, LIGHT_2_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LIGHT_2_GPIO_PORT, LIGHT_2_PIN, GPIO_PIN_RESET);
  }

  if (letter->page.lines[2] > 0) {
    HAL_GPIO_WritePin(LIGHT_3_GPIO_PORT, LIGHT_3_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LIGHT_3_GPIO_PORT, LIGHT_3_PIN, GPIO_PIN_RESET);
  }

  if (letter->page.lines[3] > 0) {
    HAL_GPIO_WritePin(LIGHT_4_GPIO_PORT, LIGHT_4_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LIGHT_4_GPIO_PORT, LIGHT_4_PIN, GPIO_PIN_RESET);
  }

  return APP_OK;
}
