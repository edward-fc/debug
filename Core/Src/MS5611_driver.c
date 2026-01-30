/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Evan Madurai
    Created on: 15 December 2023
    Description: Driver file for the Barometer module MS561101BA03
   (https://www.te.com/usa-en/product-MS560702BA03-50.html)
*/

#include "MS5611_driver.h"

#include <stdint.h>

#include "cmsis_os.h"
#include "main.h"

// min OSR by default
// static uint8_t pressAddr  = PRESSURE_OSR_256;
// static uint8_t tempAddr   = TEMP_OSR_256;
// static uint32_t convDelay = CONVERSION_OSR_256;

PROM_data ms5611_prom_data;

static SPI_HandleTypeDef *ms5611_spi = NULL;

extern osMutexId_t spi1MutexHandle;

#define MS5611_SPI_TIMEOUT_MS      50U
#define MS5611_CONVERSION_DELAY_MS 12U

static inline void ms5611_cs_low(void) {
  HAL_GPIO_WritePin(CS_BARO_GPIO_Port, CS_BARO_Pin, GPIO_PIN_RESET);
}

static inline void ms5611_cs_high(void) {
  HAL_GPIO_WritePin(CS_BARO_GPIO_Port, CS_BARO_Pin, GPIO_PIN_SET);
}

static uint8_t ms5611_spi_cmd(uint8_t cmd) {
  HAL_StatusTypeDef status;

  ms5611_cs_low();
  status = HAL_SPI_Transmit(ms5611_spi, &cmd, 1, MS5611_SPI_TIMEOUT_MS);
  ms5611_cs_high();

  return (status == HAL_OK) ? 0U : 1U;
}

static uint8_t ms5611_read_adc24(uint32_t *out)
{
  uint8_t tx[4] = { MS5611_CMD_READ_ADC, 0x00, 0x00, 0x00 };
  uint8_t rx[4] = {0};

  ms5611_cs_low();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(ms5611_spi, tx, rx, sizeof(tx), MS5611_SPI_TIMEOUT_MS);
  ms5611_cs_high();

  if (st != HAL_OK) return 1U;

  *out = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];
  return 0U;
}

static uint8_t ms5611_read_prom16(uint8_t prom_cmd, uint16_t *out)
{
  uint8_t tx[3] = { prom_cmd, 0x00, 0x00 };
  uint8_t rx[3] = { 0 };

  ms5611_cs_low();
  HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(ms5611_spi, tx, rx, sizeof(tx), MS5611_SPI_TIMEOUT_MS);
  ms5611_cs_high();

  if (st != HAL_OK) return 1U;

  // rx[0] is garbage during command phase; data is in rx[1], rx[2]
  *out = (uint16_t)((rx[1] << 8) | rx[2]);
  return 0U;
}

static int32_t calculate_pressure(uint32_t D1, uint32_t D2, M5611_data *data);
static uint8_t ms5611_read_raw_locked(uint32_t *D1, uint32_t *D2);

/**
  @brief Init MS5611 Barometer driver
  @param spi Selected SPI
  @warning Return code not implemented
  @return Error code
*/
uint8_t MS5611_init(SPI_HandleTypeDef *spi) {
  if (spi == NULL) {
    return 1U;
  }

  ms5611_spi = spi;
  ms5611_cs_high();

  if (ms5611_spi_cmd(MS5611_CMD_RESET) != 0U) {
    return 2U;
  }

  osDelay(3);

  return MS5611_read_PROM();
}

/* ---------------------------------- Private Functions
 * ---------------------------------- */

uint16_t PROM_values[8];

/**
  @brief Read PROM data from Barometer into ms5611_prom_data structure
  @warning Error code not implemented
  @return Error code
*/
uint8_t MS5611_read_PROM(void)
{
  uint16_t *prom_ptr = (uint16_t *)&ms5611_prom_data;

  if (ms5611_spi == NULL) {
    return 1U;
  }

  for (int i = 0; i < 8; i++) {
    uint16_t val = 0;

    if (ms5611_read_prom16(MS5611_CMD_READ_PROM(i), &val) != 0U) {
      return 2U;
    }

    *(prom_ptr + i) = val;
    osDelay(10);
  }

  return 0U;
}

/**
  @brief Get pressure and temperature data off MS5611 sensor and place into
  MS5611_data struct
  @param data Structure for Barometer data to be placed in
  @warning Error code not implemented
  @return Error code
*/
uint8_t MS5611_get_data(M5611_data *data) {
  uint32_t D1;
  uint32_t D2;
  osStatus_t lock_status;

  if (data == NULL || ms5611_spi == NULL || spi1MutexHandle == NULL) {
    return 1U;
  }

  lock_status = osMutexAcquire(spi1MutexHandle, osWaitForever);
  if (lock_status != osOK) {
    return 2U;
  }

  if (ms5611_read_raw_locked(&D1, &D2) != 0U) {
    osMutexRelease(spi1MutexHandle);
    return 3U;
  }

  (void)calculate_pressure(D1, D2, data);

  osMutexRelease(spi1MutexHandle);

  return 0U;
}

uint8_t MS5611_read_raw(uint32_t *D1, uint32_t *D2) {
  osStatus_t lock_status;

  if (D1 == NULL || D2 == NULL || ms5611_spi == NULL || spi1MutexHandle == NULL) {
    return 1U;
  }

  lock_status = osMutexAcquire(spi1MutexHandle, osWaitForever);
  if (lock_status != osOK) {
    return 2U;
  }

  if (ms5611_read_raw_locked(D1, D2) != 0U) {
    osMutexRelease(spi1MutexHandle);
    return 3U;
  }

  osMutexRelease(spi1MutexHandle);

  return 0U;
}

void MS5611_get_prom(PROM_data *out) {
  if (out == NULL) {
    return;
  }

  *out = ms5611_prom_data;
}

static uint8_t ms5611_read_raw_locked(uint32_t *D1, uint32_t *D2) {
  uint8_t cmd;

  cmd = MS5611_CMD_CONVERT_D2;
  if (ms5611_spi_cmd(cmd) != 0U) {
    return 1U;
  }

  osDelay(MS5611_CONVERSION_DELAY_MS);

  if (ms5611_read_adc24(D2) != 0U) return 2U;

  cmd = MS5611_CMD_CONVERT_D1;
  if (ms5611_spi_cmd(cmd) != 0U) {
    return 3U;
  }

  osDelay(MS5611_CONVERSION_DELAY_MS);

  if (ms5611_read_adc24(D1) != 0U) return 4U;

  return 0U;
}

static int32_t calculate_pressure(uint32_t D1, uint32_t D2, M5611_data* data) {
  int32_t dT = ((int32_t)D2) - ((int32_t)ms5611_prom_data.T_REF << 8);
  int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * ms5611_prom_data.TEMPSENS) / (1LL << 23));

  int64_t OFF = ((int64_t)ms5611_prom_data.OFF << 16) + ((int64_t)ms5611_prom_data.TCO * dT >> 7);
  int64_t SENS = ((int64_t)ms5611_prom_data.SENS << 15) + ((int64_t)ms5611_prom_data.TCS * dT >> 8);

  // Second order temperature compensation
  int64_t OFF2, SENS2;
  int32_t T2;
  if (TEMP < 2000) {
    T2 = (int64_t)(dT * dT) >> 31;
    OFF2 = (int64_t)(5 * (((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >> 1);
    SENS2 = (int64_t)(5 * (int64_t)(((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >> 2);
    if (TEMP < -1500) {
      OFF2 = OFF2 + (int64_t)(7 * ((TEMP + 1500) * (TEMP + 1500)));
      SENS2 = SENS2 + (int64_t)(11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1);
    }
  } else {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }

  TEMP = TEMP - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  int32_t PRESSURE = (int32_t)((((int64_t)D1 * SENS >> 21) - OFF) >> 15);

  data->temp = TEMP;
  data->pressure = PRESSURE;
  return 0;
}
