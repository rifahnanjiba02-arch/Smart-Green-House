#ifndef GREENHOUSE_H
#define GREENHOUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef enum
{
  GREENHOUSE_TEST_BUZZER = 0,
  GREENHOUSE_TEST_LIGHT,
  GREENHOUSE_TEST_SOIL,
  GREENHOUSE_TEST_AHT20,
  GREENHOUSE_TEST_BMP280,
  GREENHOUSE_TEST_OLED,
  GREENHOUSE_TEST_SERVO,
  GREENHOUSE_TEST_FAN,
  GREENHOUSE_TEST_PUMP,
  GREENHOUSE_TEST_FULL
} GreenhouseTestMode;

/*
 * Zero means "not measured" for every field below. Automatic actions which
 * depend on an unset value remain disabled and the OLED reports the problem.
 */
typedef struct
{
  uint16_t soil_dry_adc;
  uint16_t soil_wet_adc;
  uint16_t light_dark_adc;
  uint16_t light_bright_adc;
  uint16_t servo_closed_pulse_us;
  uint16_t servo_open_pulse_us;
  uint32_t servo_move_time_ms;
  uint32_t pump_run_time_ms;
} GreenhouseConfig;

HAL_StatusTypeDef Greenhouse_Init(ADC_HandleTypeDef *adc,
                                  I2C_HandleTypeDef *i2c,
                                  TIM_HandleTypeDef *pwm_timer,
                                  const GreenhouseConfig *config,
                                  GreenhouseTestMode test_mode);
void Greenhouse_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* GREENHOUSE_H */
