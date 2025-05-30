#ifndef CK_RX_LETTERS_H
#define CK_RX_LETTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

int process_jumper_config_letter(const ck_letter_t *letter);
int process_set_reg_out_voltage_letter(const ck_letter_t *letter);
int process_output_on_off_letter(const ck_letter_t *letter);
int process_report_freq_letter(const ck_letter_t *letter);
int process_low_voltage_cutoff_letter(const ck_letter_t *letter);
int process_vbat_out_overcurrent_threshold_letter(const ck_letter_t *letter);
int process_reg_out_overcurrent_threshold_letter(const ck_letter_t *letter);
int process_cell_calibration_letter(const ck_letter_t *letter);

#ifdef __cplusplus
}
#endif

#endif /* CK_RX_LETTERS_H */
