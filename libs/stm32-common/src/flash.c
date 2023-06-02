#include "flash.h"

#include <stdbool.h>
#include <string.h>

#include "error.h"
#include "stm32f3xx_hal.h"

#define FLASH_ERASE_OK 0xFFFFFFFF  // Specified by STM32 HAL

int flash_erase(void) {
  // 1 page = 2KB
  FLASH_EraseInitTypeDef erase_conf = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = FLASH_RW_START,
      .NbPages = 1,
  };
  HAL_FLASH_Unlock();
  uint32_t err = 0;
  HAL_FLASHEx_Erase(&erase_conf, &err);
  HAL_FLASH_Lock();
  if (err != FLASH_ERASE_OK) {
    return APP_NOT_OK;
  }
  return APP_OK;
}

int flash_read(uint32_t addr, void *data, size_t len) {
  // Check if within writeable memory bounds
  if (addr < FLASH_RW_START) {
    return APP_NOT_OK;
  }
  if (addr + len > FLASH_RW_END) {
    return APP_NOT_OK;
  }
  memcpy(data, (const void *)addr, len);
  return APP_OK;
}

int flash_write(uint32_t addr, const void *data, size_t len) {
  // Check if within writeable memory bounds
  if (addr < FLASH_RW_START) {
    return APP_NOT_OK;
  }
  if (addr + len > FLASH_RW_END) {
    return APP_NOT_OK;
  }
  // Check if data is word-aligned
  if (len % 4 > 0) {
    return APP_NOT_OK;
  }

  // Erase before write
  if (flash_erase() != APP_OK) {
    return APP_NOT_OK;
  }

  const size_t word_length = 4;
  uint32_t word = 0;
  HAL_FLASH_Unlock();
  bool flash_failed = false;
  for (size_t i = 0; i < len; i += word_length) {
    memcpy(&word, (uint8_t *)data + i, word_length);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, (uint64_t)word) !=
        HAL_OK) {
      flash_failed = true;
      break;
    }
  }
  HAL_FLASH_Lock();

  if (flash_failed) {
    return APP_NOT_OK;
  }

  return APP_OK;
}
