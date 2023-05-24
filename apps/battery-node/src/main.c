#include <stdio.h>
#include <string.h>

#include "app.h"
#include "clock.h"
#include "error.h"
#include "flash.h"
#include "mayor.h"
#include "peripherals.h"
#include "ports.h"
#include "postmaster-hal.h"
#include "postmaster.h"
#include "print.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

// Hardware
static peripherals_t *peripherals;

// FreeRTOS
#define BATTERY_MONITOR_TASK_PERIOD 200
#define POWER_MEASURE_TASK_PERIOD 100
#define LOWEST_TASK_PRIORITY 24

static TaskHandle_t battery_monitor_task;
static StaticTask_t battery_monitor_buf;
static StackType_t battery_monitor_stack[configMINIMAL_STACK_SIZE];
void battery_monitor(void *unused);

static TaskHandle_t power_measure_task;
static StaticTask_t power_measure_buf;
static StackType_t power_measure_stack[configMINIMAL_STACK_SIZE];
void power_measure(void *unused);

// CAN Kingdom process received letters task
static TaskHandle_t proc_letter_task;
static StaticTask_t proc_letter_buf;
static StackType_t proc_letter_stack[configMINIMAL_STACK_SIZE];
void proc_letter(void *unused);

void task_init(void);

// CAN Kingdom data
#define PAGE_COUNT 4
#define DOC_COUNT 3
#define LIST_COUNT 2
#define FOLDER_COUNT 5
static ck_page_t pages[PAGE_COUNT];
static ck_document_t docs[DOC_COUNT];
static ck_list_t lists[LIST_COUNT];
static ck_folder_t folders[FOLDER_COUNT];

// Convenience pointers
static ck_list_t *tx_list = &lists[0];
static ck_list_t *rx_list = &lists[1];

static ck_page_t *cell_page0 = &pages[0];
static ck_page_t *cell_page1 = &pages[1];
static ck_page_t *reg_out_current_page = &pages[2];
static ck_page_t *vbat_out_current_page = &pages[3];

static ck_document_t *cell_doc = &docs[0];
static ck_document_t *reg_out_current_doc = &docs[1];
static ck_document_t *vbat_out_current_doc = &docs[2];

static ck_folder_t *cell_folder = &folders[2];
static ck_folder_t *reg_out_current_folder = &folders[3];
static ck_folder_t *vbat_out_current_folder = &folders[4];

void mayor_init(void);

// Application
#define CAN_BASE_ID 0x100
#define LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD 10

static BatteryNodeState batteryNodeState;
// Set to 1 by GPIO external interrupt on the OVER_CURRENT pin.
static uint8_t OverCurrentFault = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  if (flash_init() != APP_OK) {
    error();
  }

  system_clock_init();

  // Initialize all configured peripherals
  peripherals = get_peripherals();
  gpio_init();
  dma_init();
  can_init();
  uart1_init();
  spi1_init();
  adc1_init();
  adc2_init();
  i2c1_init();

  mayor_init();

  task_init();

  print(&peripherals->huart1, "Starting application...\r\n");

  // Start scheduler
  vTaskStartScheduler();

  // We should never get here as control is now taken by the scheduler.
  while (1) {
  }
}

ck_err_t set_action_mode(ck_action_mode_t mode) {
  (void)mode;
  return CK_OK;
}

ck_err_t set_city_mode(ck_city_mode_t mode) {
  (void)mode;
  return CK_OK;
}

void mayor_init(void) {
  // Set up the pages
  // NOLINTBEGIN(*-magic-numbers)
  cell_page0->line_count = 7;
  cell_page0->lines[0] = 0;
  // Pagination
  cell_page1->line_count = 7;
  cell_page1->lines[0] = 1;
  reg_out_current_page->line_count = 2;
  vbat_out_current_page->line_count = 4;
  // NOLINTEND(*-magic-numbers)

  // Set up the documents
  cell_doc->direction = CK_DIRECTION_TRANSMIT;
  cell_doc->page_count = 2;
  cell_doc->pages[0] = cell_page0;
  cell_doc->pages[1] = cell_page1;

  reg_out_current_doc->direction = CK_DIRECTION_TRANSMIT;
  reg_out_current_doc->page_count = 1;
  reg_out_current_doc->pages[0] = reg_out_current_page;

  vbat_out_current_doc->direction = CK_DIRECTION_TRANSMIT;
  vbat_out_current_doc->page_count = 1;
  vbat_out_current_doc->pages[0] = vbat_out_current_page;

  // Set up the lists
  rx_list->type = CK_LIST_DOCUMENT;
  rx_list->direction = CK_DIRECTION_RECEIVE;
  rx_list->list_no = 0;
  rx_list->record_count = 1;  // Only 1 slot, for the king's doc.

  tx_list->type = CK_LIST_DOCUMENT;
  tx_list->direction = CK_DIRECTION_TRANSMIT;
  tx_list->list_no = 0;
  // We have 3 documents, and CK needs 1 slot for the mayor's doc.
  tx_list->record_count = 4;
  tx_list->records[1] = cell_doc;
  tx_list->records[2] = reg_out_current_doc;
  tx_list->records[3] = vbat_out_current_doc;

  // Set up the folders
  cell_folder->direction = CK_DIRECTION_TRANSMIT;
  cell_folder->doc_list_no = 0;
  cell_folder->doc_no = 1;  // 0 reserved by mayor's doc
  cell_folder->enable = true;
  cell_folder->folder_no = 2;
  // 3 cells per page + 1 byte for pagination.
  cell_folder->dlc = 3 * sizeof(batteryNodeState.cells[0]) + 1;

  reg_out_current_folder->direction = CK_DIRECTION_TRANSMIT;
  reg_out_current_folder->doc_list_no = 0;
  reg_out_current_folder->doc_no = 2;
  reg_out_current_folder->enable = true;
  reg_out_current_folder->folder_no = 3;
  reg_out_current_folder->dlc = sizeof(batteryNodeState.regOutCurrent);

  vbat_out_current_folder->direction = CK_DIRECTION_TRANSMIT;
  vbat_out_current_folder->doc_list_no = 0;
  vbat_out_current_folder->doc_no = 3;
  vbat_out_current_folder->enable = true;
  vbat_out_current_folder->folder_no = 4;
  vbat_out_current_folder->dlc = sizeof(batteryNodeState.vbatOutCurrent);

  // TODO: Set envelopes using king's pages.
  cell_folder->envelope_count = 1;
  cell_folder->envelopes[0].envelope_no = CAN_BASE_ID;
  cell_folder->envelopes[0].enable = true;
  reg_out_current_folder->envelope_count = 1;
  reg_out_current_folder->envelopes[0].envelope_no = CAN_BASE_ID + 1;
  reg_out_current_folder->envelopes[0].enable = true;
  vbat_out_current_folder->envelope_count = 1;
  vbat_out_current_folder->envelopes[0].envelope_no = CAN_BASE_ID + 2;
  vbat_out_current_folder->envelopes[0].enable = true;

  peripherals_t *peripherals = get_peripherals();
  postmaster_init(&(peripherals->hcan));  // Set up the postmaster

  ck_mayor_t mayor = {
      .ean_no = 123,       // NOLINT(*-magic-numbers)
      .serial_no = 456,    // NOLINT(*-magic-numbers)
      .city_address = 13,  // NOLINT(*-magic-numbers)
      .base_no = 25,       // NOLINT(*-magic-numbers)
      .set_action_mode = set_action_mode,
      .set_city_mode = set_city_mode,
      .folder_count = 5,  // NOLINT(*-magic-numbers)
      .folders = folders,
      .list_count = 2,
      .lists = lists,
  };

  if (ck_mayor_init(&mayor) != CK_OK) {
    error();
  }
}

void task_init(void) {
  battery_monitor_task = xTaskCreateStatic(
      battery_monitor, "battery monitor", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 2, battery_monitor_stack, &battery_monitor_buf);

  power_measure_task = xTaskCreateStatic(
      power_measure, "power measure", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY + 1, power_measure_stack, &power_measure_buf);

  proc_letter_task = xTaskCreateStatic(
      proc_letter, "process letter", configMINIMAL_STACK_SIZE, NULL,
      LOWEST_TASK_PRIORITY, proc_letter_stack, &proc_letter_buf);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(power_measure_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin != OVER_CURRENT_Pin) {
    return;
  }
  OverCurrentFault = 1;
}

void battery_monitor_timer(TimerHandle_t xTimer) {
  (void)xTimer;
  xTaskNotifyGive(battery_monitor_task);
}

void power_measure_timer(TimerHandle_t xTimer) {
  (void)xTimer;
  xTaskNotifyGive(power_measure_task);
}

void power_measure(void *unused) {
  (void)unused;

  StaticTimer_t timer_buf;
  TimerHandle_t xTimer = xTimerCreateStatic(
      "power measure timer", pdMS_TO_TICKS(POWER_MEASURE_TASK_PERIOD),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      power_measure_timer, &timer_buf);

  ADCReading adcBuf;

  xTimerStart(xTimer, portMAX_DELAY);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Start both ADCs
    HAL_ADC_Start_DMA(&peripherals->hadc1, (uint32_t *)adcBuf.adc1Buf,
                      sizeof(adcBuf.adc1Buf) / sizeof(uint16_t));
    HAL_ADC_Start_DMA(&peripherals->hadc2, (uint32_t *)adcBuf.adc2Buf,
                      sizeof(adcBuf.adc2Buf) / sizeof(uint16_t));

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for DMA

    ParseADCValues(&adcBuf, &batteryNodeState);

    // Copy cells 0,1,2 to page 0 of cell doc, cells 3,4,5 to page 1.
    memcpy(&cell_page0->lines[1], &batteryNodeState.cells[0], cell_folder->dlc);
    memcpy(&cell_page1->lines[1], &batteryNodeState.cells[3], cell_folder->dlc);

    memcpy(reg_out_current_page->lines, &batteryNodeState.regOutCurrent,
           reg_out_current_folder->dlc);

    memcpy(vbat_out_current_page->lines, &batteryNodeState.vbatOutCurrent,
           vbat_out_current_folder->dlc);

    // Send the documents
    if (ck_send_document(cell_folder->folder_no) != CK_OK) {
      print(&peripherals->huart1, "failed to send doc.\r\n");
    }
    if (ck_send_document(reg_out_current_folder->folder_no) != CK_OK) {
      print(&peripherals->huart1, "failed to send doc.\r\n");
    }
    if (ck_send_document(vbat_out_current_folder->folder_no) != CK_OK) {
      print(&peripherals->huart1, "failed to send doc.\r\n");
    }
  }
}

void battery_monitor(void *unused) {
  (void)unused;

  SetJumperConfig(ALL_ON);

  ConfigureVoltageRegulator(&peripherals->hi2c1, POT_IVRA_DEFAULT);
  HAL_GPIO_WritePin(REG_PWR_ON_GPIO_Port, REG_PWR_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_SET);

  StaticTimer_t timer_buf;
  TimerHandle_t xTimer = xTimerCreateStatic(
      "battery monitor timer", pdMS_TO_TICKS(BATTERY_MONITOR_TASK_PERIOD),
      pdTRUE,  // Auto reload timer
      NULL,    // Timer ID, unused
      battery_monitor_timer, &timer_buf);

  BatteryCharge charge = CHARGE_100_PERCENT;
  uint8_t lowPowerReportCount = 0;

  xTimerStart(xTimer, portMAX_DELAY);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for task activation

    // Don't try to update charge state after low voltage cutoff
    // or over current fault.
    if (lowPowerReportCount > LOW_VOLTAGE_CUTOFF_REPORT_THRESHOLD ||
        OverCurrentFault != 0) {
      // Turn off the power outputs to reduce the battery power drain.
      HAL_GPIO_WritePin(REG_PWR_ON_GPIO_Port, REG_PWR_ON_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(POWER_OFF_GPIO_Port, POWER_OFF_Pin, GPIO_PIN_RESET);
      // Blink LEDs red to show user something is wrong.
      BlinkLEDsRed();
      continue;
    }

    charge = ReadBatteryCharge(&batteryNodeState);
    lowPowerReportCount += SetChargeStateLED(&charge);
  }
}

// Convert HAL CAN frame to ck_letter_t.
ck_letter_t frame_to_letter(CAN_RxHeaderTypeDef *header, uint8_t *data) {
  ck_letter_t letter;
  letter.envelope.is_remote = header->RTR;
  letter.envelope.has_extended_id = header->IDE;
  if (letter.envelope.has_extended_id) {
    letter.envelope.envelope_no = header->ExtId;
  } else {
    letter.envelope.envelope_no = header->StdId;
  }
  letter.page.line_count = header->DLC;
  memcpy(letter.page.lines, data, header->DLC);
  return letter;
}

// Deactivate interrupt, then signal task. Let the task reactivate the
// interrupt.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(proc_letter_task, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void dispatch_letter(ck_letter_t *letter) { (void)letter; }

void proc_letter(void *unused) {
  (void)unused;

  // This will go on bus using silent mode.
  ck_err_t ret = ck_set_comm_mode(CK_COMM_MODE_COMMUNICATE);
  if (ret != CK_OK) {
    print(&peripherals->huart1, "Error setting comm mode.\r\n");
    error();
  }

  CAN_RxHeaderTypeDef header;
  uint8_t data[CK_CAN_MAX_DLC];
  ck_letter_t letter;

  for (;;) {
    if (HAL_CAN_ActivateNotification(&peripherals->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      print(&peripherals->huart1, "Error activating interrupt.\r\n");
      error();
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Process all messages
    while (HAL_CAN_GetRxMessage(&peripherals->hcan, CAN_RX_FIFO0, &header,
                                data) == HAL_OK) {
      letter = frame_to_letter(&header, data);
      dispatch_letter(&letter);
    }
  }
}
