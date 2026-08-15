#ifndef AHT20_H
#define AHT20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define AHT20_I2C_ADDRESS  (0x38U << 1)

typedef enum
{
  AHT20_STATUS_OK = 0,
  AHT20_STATUS_BUSY,
  AHT20_STATUS_ERROR
} AHT20_Status;

typedef struct
{
  I2C_HandleTypeDef *i2c;
} AHT20_Handle;

HAL_StatusTypeDef AHT20_Init(AHT20_Handle *sensor, I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef AHT20_StartMeasurement(AHT20_Handle *sensor);
AHT20_Status AHT20_ReadMeasurement(AHT20_Handle *sensor,
                                   int32_t *temperature_milli_c,
                                   uint32_t *humidity_milli_percent);

#ifdef __cplusplus
}
#endif

#endif /* AHT20_H */
