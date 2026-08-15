#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define BMP280_I2C_ADDRESS_LOW   (0x76U << 1)
#define BMP280_I2C_ADDRESS_HIGH  (0x77U << 1)

typedef struct
{
  I2C_HandleTypeDef *i2c;
  uint16_t address;
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
  int32_t t_fine;
} BMP280_Handle;

HAL_StatusTypeDef BMP280_Init(BMP280_Handle *sensor, I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef BMP280_Read(BMP280_Handle *sensor,
                              int32_t *temperature_centi_c,
                              uint32_t *pressure_pa);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
