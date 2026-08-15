#include "bmp280.h"

#define BMP280_TIMEOUT_MS      20U
#define BMP280_REG_ID          0xD0U
#define BMP280_REG_RESET       0xE0U
#define BMP280_REG_CALIB       0x88U
#define BMP280_REG_CONFIG      0xF5U
#define BMP280_REG_CTRL_MEAS   0xF4U
#define BMP280_REG_DATA        0xF7U
#define BMP280_CHIP_ID         0x58U
#define BMP280_RESET_VALUE     0xB6U

static uint16_t BMP280_U16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t BMP280_S16(const uint8_t *data)
{
  return (int16_t)BMP280_U16(data);
}

static HAL_StatusTypeDef BMP280_ReadRegister(BMP280_Handle *sensor,
                                             uint8_t reg,
                                             uint8_t *data,
                                             uint16_t length)
{
  return HAL_I2C_Mem_Read(sensor->i2c, sensor->address, reg,
                          I2C_MEMADD_SIZE_8BIT, data, length,
                          BMP280_TIMEOUT_MS);
}

static HAL_StatusTypeDef BMP280_WriteRegister(BMP280_Handle *sensor,
                                              uint8_t reg,
                                              uint8_t value)
{
  return HAL_I2C_Mem_Write(sensor->i2c, sensor->address, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           BMP280_TIMEOUT_MS);
}

HAL_StatusTypeDef BMP280_Init(BMP280_Handle *sensor, I2C_HandleTypeDef *i2c)
{
  uint8_t id;
  uint8_t calibration[24];

  if ((sensor == NULL) || (i2c == NULL))
  {
    return HAL_ERROR;
  }

  sensor->i2c = i2c;
  sensor->address = BMP280_I2C_ADDRESS_LOW;
  if (HAL_I2C_IsDeviceReady(i2c, sensor->address, 2U,
                            BMP280_TIMEOUT_MS) != HAL_OK)
  {
    sensor->address = BMP280_I2C_ADDRESS_HIGH;
    if (HAL_I2C_IsDeviceReady(i2c, sensor->address, 2U,
                              BMP280_TIMEOUT_MS) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if ((BMP280_ReadRegister(sensor, BMP280_REG_ID, &id, 1U) != HAL_OK) ||
      (id != BMP280_CHIP_ID))
  {
    return HAL_ERROR;
  }

  if (BMP280_WriteRegister(sensor, BMP280_REG_RESET,
                           BMP280_RESET_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(3U);

  if (BMP280_ReadRegister(sensor, BMP280_REG_CALIB, calibration,
                          sizeof(calibration)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  sensor->dig_t1 = BMP280_U16(&calibration[0]);
  sensor->dig_t2 = BMP280_S16(&calibration[2]);
  sensor->dig_t3 = BMP280_S16(&calibration[4]);
  sensor->dig_p1 = BMP280_U16(&calibration[6]);
  sensor->dig_p2 = BMP280_S16(&calibration[8]);
  sensor->dig_p3 = BMP280_S16(&calibration[10]);
  sensor->dig_p4 = BMP280_S16(&calibration[12]);
  sensor->dig_p5 = BMP280_S16(&calibration[14]);
  sensor->dig_p6 = BMP280_S16(&calibration[16]);
  sensor->dig_p7 = BMP280_S16(&calibration[18]);
  sensor->dig_p8 = BMP280_S16(&calibration[20]);
  sensor->dig_p9 = BMP280_S16(&calibration[22]);

  if (sensor->dig_p1 == 0U)
  {
    return HAL_ERROR;
  }

  /* One-second standby, no filter, normal mode, temperature/pressure x1. */
  if (BMP280_WriteRegister(sensor, BMP280_REG_CONFIG, 0xA0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return BMP280_WriteRegister(sensor, BMP280_REG_CTRL_MEAS, 0x27U);
}

HAL_StatusTypeDef BMP280_Read(BMP280_Handle *sensor,
                              int32_t *temperature_centi_c,
                              uint32_t *pressure_pa)
{
  uint8_t data[6];
  int32_t raw_temperature;
  int32_t raw_pressure;
  int32_t var1_t;
  int32_t var2_t;
  int64_t var1_p;
  int64_t var2_p;
  int64_t pressure;

  if ((sensor == NULL) || (sensor->i2c == NULL) ||
      (temperature_centi_c == NULL) || (pressure_pa == NULL))
  {
    return HAL_ERROR;
  }

  if (BMP280_ReadRegister(sensor, BMP280_REG_DATA, data,
                          sizeof(data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  raw_pressure = ((int32_t)data[0] << 12U) |
                 ((int32_t)data[1] << 4U) |
                 ((int32_t)data[2] >> 4U);
  raw_temperature = ((int32_t)data[3] << 12U) |
                    ((int32_t)data[4] << 4U) |
                    ((int32_t)data[5] >> 4U);

  var1_t = ((((raw_temperature >> 3) -
              ((int32_t)sensor->dig_t1 << 1)))*
            (int32_t)sensor->dig_t2) >> 11;
  var2_t = (((((raw_temperature >> 4) - (int32_t)sensor->dig_t1) *
               ((raw_temperature >> 4) - (int32_t)sensor->dig_t1)) >> 12) *
            (int32_t)sensor->dig_t3) >> 14;
  sensor->t_fine = var1_t + var2_t;
  *temperature_centi_c = (sensor->t_fine * 5 + 128) >> 8;

  var1_p = (int64_t)sensor->t_fine - 128000LL;
  var2_p = var1_p * var1_p * (int64_t)sensor->dig_p6;
  var2_p += (var1_p * (int64_t)sensor->dig_p5) * 131072LL;
  var2_p += (int64_t)sensor->dig_p4 * (1LL << 35);
  var1_p = ((var1_p * var1_p * (int64_t)sensor->dig_p3) >> 8) +
           ((var1_p * (int64_t)sensor->dig_p2) * 4096LL);
  var1_p = (((1LL << 47) + var1_p) * (int64_t)sensor->dig_p1) >> 33;

  if (var1_p == 0LL)
  {
    return HAL_ERROR;
  }

  pressure = 1048576LL - (int64_t)raw_pressure;
  pressure = (((pressure << 31) - var2_p) * 3125LL) / var1_p;
  var1_p = ((int64_t)sensor->dig_p9 * (pressure >> 13) *
            (pressure >> 13)) >> 25;
  var2_p = ((int64_t)sensor->dig_p8 * pressure) >> 19;
  pressure = ((pressure + var1_p + var2_p) >> 8) +
             ((int64_t)sensor->dig_p7 * 16LL);

  if (pressure < 0LL)
  {
    return HAL_ERROR;
  }

  *pressure_pa = (uint32_t)(pressure >> 8);
  return HAL_OK;
}
