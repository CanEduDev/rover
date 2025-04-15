#ifndef CK_DATA_H
#define CK_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

#define CK_DATA_TX_PAGE_COUNT 0
#define CK_DATA_TX_DOC_COUNT 0
#define CK_DATA_LIST_COUNT 2
#define CK_DATA_TX_FOLDER_COUNT 0
#define CK_DATA_RX_FOLDER_COUNT 1
#define CK_DATA_FOLDER_COUNT \
  (2 + CK_DATA_TX_FOLDER_COUNT + CK_DATA_RX_FOLDER_COUNT)

typedef struct {
  ck_list_t lists[CK_DATA_LIST_COUNT];
  ck_folder_t folders[CK_DATA_FOLDER_COUNT];

  // Convenience pointers
  ck_list_t *tx_list;
  ck_list_t *rx_list;

  // Receive
  ck_folder_t *set_light_state_folder;

} ck_data_t;

void ck_data_init(void);
ck_data_t *get_ck_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_DATA_H */
