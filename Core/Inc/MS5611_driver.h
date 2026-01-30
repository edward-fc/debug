/*
    Leeds University Rocketry Organisation - LURA
    Description: header file for the Barometer module MS561101BA03
*/
#ifndef MS5611_DRIVER_H
#define MS5611_DRIVER_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

/* MS5611 commands */
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_READ_ADC     0x00
#define MS5611_CMD_CONVERT_D1   0x40  /* Pressure, OSR=256 */
#define MS5611_CMD_CONVERT_D2   0x50  /* Temperature, OSR=256 */
#define MS5611_CMD_READ_PROM(i) (0xA0 | ((i) << 1))

typedef struct {
  uint16_t C0;       /* factory data */
  uint16_t SENS;     /* C1 */
  uint16_t OFF;      /* C2 */
  uint16_t TCS;      /* C3 */
  uint16_t TCO;      /* C4 */
  uint16_t T_REF;    /* C5 */
  uint16_t TEMPSENS; /* C6 */
  uint16_t CRC_code; /* C7 */
} PROM_data;

typedef struct {
  int32_t temp;      /* centi-degC */
  int32_t pressure;  /* Pa */
} M5611_data;

uint8_t MS5611_init(SPI_HandleTypeDef *hspi);
uint8_t MS5611_read_PROM(void);
uint8_t MS5611_get_data(M5611_data *data);
uint8_t MS5611_read_raw(uint32_t *D1, uint32_t *D2);
void MS5611_get_prom(PROM_data *out);

#endif /* MS5611_DRIVER_H */
