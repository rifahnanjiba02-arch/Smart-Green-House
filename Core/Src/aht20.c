#include "aht20.h"

#define AHT20_TIMEOUT_MS          20U
#define AHT20_STATUS_COMMAND      0x71U
#define AHT20_INITIALIZE_COMMAND  0xBEU
#define AHT20_MEASURE_COMMAND     0xACU
#define AHT20_SOFT_RESET_COMMAND  0xBAU
#define AHT20_STATUS_BUSY_BIT     0x80U
#define AHT20_STATUS_CAL_BIT      0x08U

static HAL_StatusTypeDef AHT20_ReadStatus(AHT20_Handle *sensor,
                                          uint8_t *status)
{
  uint8_t command = AHT20_STATUS_COMMAND;

  if (HAL_I2C_Master_Transmit(sensor->i2c, AHT20_I2C_ADDRESS, &command, 1U,
                              AHT20_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Master_Receive(sensor->i2c, AHT20_I2C_ADDRESS, status, 1U,
                                AHT20_TIMEOUT_MS);
}

static uint8_t AHT20_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFFU;
  uint8_t byte_index;
  uint8_t bit_index;

  for (byte_index = 0U; byte_index < length; byte_index++)
  {
    crc ^= data[byte_index];
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ 0x31U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

HAL_StatusTypeDef AHT20_Init(AHT20_Handle *sensor, I2C_HandleTypeDef *i2c)
{
  uint8_t command[3];
  uint8_t status;

  if ((sensor == NULL) || (i2c == NULL))
  {
    return HAL_ERROR;
  }

  sensor->i2c = i2c;
  HAL_Delay(40U);

  if (HAL_I2C_IsDeviceReady(i2c, AHT20_I2C_ADDRESS, 2U,
                            AHT20_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AHT20_ReadStatus(sensor, &status) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((status & AHT20_STATUS_CAL_BIT) == 0U)
  {
    command[0] = AHT20_INITIALIZE_COMMAND;
    command[1] = 0x08U;
    command[2] = 0x00U;
    if (HAL_I2C_Master_Transmit(i2c, AHT20_I2C_ADDRESS, command, 3U,
                                AHT20_TIMEOUT_MS) != HAL_OK)
    {
      return HAL_ERROR;
    }

    HAL_Delay(10U);
    if ((AHT20_ReadStatus(sensor, &status) != HAL_OK) ||
        ((status & AHT20_STATUS_CAL_BIT) == 0U))
    {
      command[0] = AHT20_SOFT_RESET_COMMAND;
      (void)HAL_I2C_Master_Transmit(i2c, AHT20_I2C_ADDRESS, command, 1U,
                                    AHT20_TIMEOUT_MS);
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef AHT20_StartMeasurement(AHT20_Handle *sensor)
{
  uint8_t command[3];

  if ((sensor == NULL) || (sensor->i2c == NULL))
  {
    return HAL_ERROR;
  }

  command[0] = AHT20_MEASURE_COMMAND;
  command[1] = 0x33U;
  command[2] = 0x00U;

  return HAL_I2C_Master_Transmit(sensor->i2c, AHT20_I2C_ADDRESS, command, 3U,
                                 AHT20_TIMEOUT_MS);
}

AHT20_Status AHT20_ReadMeasurement(AHT20_Handle *sensor,
                                   int32_t *temperature_milli_c,
                                   uint32_t *humidity_milli_percent)
{
  uint8_t data[7];
  uint32_t raw_humidity;
  uint32_t raw_temperature;

  if ((sensor == NULL) || (sensor->i2c == NULL) ||
      (temperature_milli_c == NULL) || (humidity_milli_percent == NULL))
  {
    return AHT20_STATUS_ERROR;
  }

  if (HAL_I2C_Master_Receive(sensor->i2c, AHT20_I2C_ADDRESS, data, 7U,
                             AHT20_TIMEOUT_MS) != HAL_OK)
  {
    return AHT20_STATUS_ERROR;
  }

  if ((data[0] & AHT20_STATUS_BUSY_BIT) != 0U)
  {
    return AHT20_STATUS_BUSY;
  }

  if (AHT20_Crc8(data, 6U) != data[6])
  {
    return AHT20_STATUS_ERROR;
  }

  raw_humidity = ((uint32_t)data[1] << 12U) |
                 ((uint32_t)data[2] << 4U) |
                 ((uint32_t)data[3] >> 4U);
  raw_temperature = (((uint32_t)data[3] & 0x0FU) << 16U) |
                    ((uint32_t)data[4] << 8U) |
                    (uint32_t)data[5];

  *humidity_milli_percent =
      (uint32_t)(((uint64_t)raw_humidity * 100000ULL) >> 20U);
  *temperature_milli_c =
      (int32_t)(((int64_t)raw_temperature * 200000LL) >> 20U) - 50000;

  return AHT20_STATUS_OK;
}
