#include "greenhouse.h"

#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"

#include <stdio.h>
#include <string.h>

#define BUZZER_PORT                    GPIOA
#define BUZZER_PIN                     GPIO_PIN_8
#define PUMP_PORT                      GPIOA
#define PUMP_PIN                       GPIO_PIN_9
#define FAN_PORT                       GPIOA
#define FAN_PIN                        GPIO_PIN_10

#define SENSOR_INTERVAL_MS             2000U
#define SENSOR_RETRY_INTERVAL_MS       10000U
#define AHT20_MEASUREMENT_TIME_MS      85U
#define AHT20_MEASUREMENT_TIMEOUT_MS   250U
#define DISPLAY_INTERVAL_MS            2500U
#define READY_MESSAGE_TIME_MS          2000U
#define SOAK_TIME_MS                   20000U
#define LOW_LIGHT_TIME_MS              30000U
#define MAX_WATERING_ATTEMPTS          3U
#define ADC_TIMEOUT_MS                 10U

#define FAN_ON_TEMP_MC                 30000
#define FAN_OFF_TEMP_MC                28000
#define FAN_ON_HUMIDITY_MPCT           80000U
#define FAN_OFF_HUMIDITY_MPCT          75000U
#define WINDOW_OPEN_TEMP_MC            32000
#define WINDOW_CLOSE_TEMP_MC           29000
#define WINDOW_OPEN_HUMIDITY_MPCT      85000U
#define WINDOW_CLOSE_HUMIDITY_MPCT     78000U
#define CRITICAL_TEMP_MC               40000
#define PUMP_START_PERCENT             30
#define WATERING_COMPLETE_PERCENT      45
#define LOW_LIGHT_PERCENT              20
#define LOW_LIGHT_CLEAR_PERCENT        30

typedef enum
{
  WINDOW_UNKNOWN = 0,
  WINDOW_CLOSED,
  WINDOW_OPEN
} WindowState;

typedef enum
{
  WATER_IDLE = 0,
  WATER_PUMPING,
  WATER_SOAKING,
  WATER_FAILED
} WaterState;

typedef struct
{
  ADC_HandleTypeDef *adc;
  I2C_HandleTypeDef *i2c;
  TIM_HandleTypeDef *pwm_timer;
  GreenhouseConfig config;
  GreenhouseTestMode test_mode;
  AHT20_Handle aht20;
  BMP280_Handle bmp280;
  SSD1306_Handle oled;

  int32_t temperature_mc;
  uint32_t humidity_mpct;
  uint32_t pressure_pa;
  uint16_t soil_adc;
  uint16_t light_adc;
  int16_t soil_percent;
  int16_t light_percent;

  uint8_t aht20_initialized;
  uint8_t bmp280_initialized;
  uint8_t adc_ready;
  uint8_t aht20_pending;
  uint8_t aht20_valid;
  uint8_t bmp280_valid;
  uint8_t soil_valid;
  uint8_t light_valid;
  uint8_t oled_valid;
  uint8_t fan_on;
  uint8_t pump_on;
  uint8_t servo_moving;
  uint8_t ventilation_requested;
  uint8_t critical_temperature;
  uint8_t low_light_timer_active;
  uint8_t low_light_warning;
  uint8_t startup_complete;
  uint8_t ready_message_active;
  uint8_t display_page;
  uint8_t watering_attempts;
  uint8_t buzzer_on;
  uint8_t buzzer_beeps_remaining;
  uint8_t previous_sensor_alarm;
  uint8_t test_output_on;

  WindowState window_state;
  WindowState servo_target;
  WaterState water_state;

  uint32_t sensor_tick;
  uint32_t aht20_tick;
  uint32_t aht20_retry_tick;
  uint32_t bmp280_retry_tick;
  uint32_t servo_tick;
  uint32_t water_tick;
  uint32_t low_light_tick;
  uint32_t display_tick;
  uint32_t ready_tick;
  uint32_t buzzer_tick;
  uint32_t test_tick;
} GreenhouseContext;

static GreenhouseContext greenhouse;

static uint8_t TimeReached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t Elapsed(uint32_t now, uint32_t start, uint32_t duration)
{
  return ((uint32_t)(now - start) >= duration) ? 1U : 0U;
}

static uint8_t AdcCalibrationValid(uint16_t first, uint16_t second)
{
  return ((first != 0U) && (second != 0U) && (first != second)) ? 1U : 0U;
}

static uint8_t ServoConfigurationValid(void)
{
  uint32_t period;

  period = __HAL_TIM_GET_AUTORELOAD(greenhouse.pwm_timer) + 1U;
  return ((greenhouse.config.servo_closed_pulse_us != 0U) &&
          (greenhouse.config.servo_open_pulse_us != 0U) &&
          (greenhouse.config.servo_closed_pulse_us !=
           greenhouse.config.servo_open_pulse_us) &&
          (greenhouse.config.servo_closed_pulse_us <= period) &&
          (greenhouse.config.servo_open_pulse_us <= period) &&
          (greenhouse.config.servo_move_time_ms != 0U)) ? 1U : 0U;
}

static int16_t ConvertPercent(uint16_t raw, uint16_t zero,
                              uint16_t full)
{
  int32_t value;
  int32_t denominator;

  denominator = (int32_t)full - (int32_t)zero;
  if (denominator == 0)
  {
    return 0;
  }

  value = (((int32_t)raw - (int32_t)zero) * 100) / denominator;
  if (value < 0)
  {
    value = 0;
  }
  else if (value > 100)
  {
    value = 100;
  }

  return (int16_t)value;
}

static HAL_StatusTypeDef ADC_ReadChannel(uint32_t channel, uint16_t *value)
{
  ADC_ChannelConfTypeDef channel_config = {0};
  HAL_StatusTypeDef status;

  if (greenhouse.adc_ready == 0U)
  {
    return HAL_ERROR;
  }

  channel_config.Channel = channel;
  channel_config.Rank = ADC_REGULAR_RANK_1;
  channel_config.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

  status = HAL_ADC_ConfigChannel(greenhouse.adc, &channel_config);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_Start(greenhouse.adc);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_PollForConversion(greenhouse.adc, ADC_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    *value = (uint16_t)HAL_ADC_GetValue(greenhouse.adc);
  }
  (void)HAL_ADC_Stop(greenhouse.adc);
  return status;
}

static void Fan_Set(uint8_t on)
{
  if (on != 0U)
  {
    if (greenhouse.servo_moving != 0U)
    {
      on = 0U;
    }
    else
    {
      HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
      greenhouse.pump_on = 0U;
    }
  }

  HAL_GPIO_WritePin(FAN_PORT, FAN_PIN,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  greenhouse.fan_on = on;
}

static void Pump_Set(uint8_t on)
{
  if ((on != 0U) && ((greenhouse.fan_on != 0U) ||
                     (greenhouse.servo_moving != 0U)))
  {
    on = 0U;
  }

  HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  greenhouse.pump_on = on;
}

static void Buzzer_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  greenhouse.buzzer_on = on;
}

static void Buzzer_Request(uint8_t beep_count, uint32_t now)
{
  if ((greenhouse.critical_temperature == 0U) &&
      (greenhouse.water_state != WATER_FAILED) &&
      (beep_count > greenhouse.buzzer_beeps_remaining))
  {
    greenhouse.buzzer_beeps_remaining = beep_count;
    if (greenhouse.buzzer_on == 0U)
    {
      greenhouse.buzzer_tick = now;
    }
  }
}

static void Buzzer_Process(uint32_t now)
{
  if ((greenhouse.critical_temperature != 0U) ||
      (greenhouse.water_state == WATER_FAILED))
  {
    Buzzer_Set(((uint32_t)(now % 1000U) < 150U) ? 1U : 0U);
    return;
  }

  if (greenhouse.buzzer_beeps_remaining == 0U)
  {
    Buzzer_Set(0U);
    return;
  }

  if (TimeReached(now, greenhouse.buzzer_tick) == 0U)
  {
    return;
  }

  if (greenhouse.buzzer_on != 0U)
  {
    Buzzer_Set(0U);
    greenhouse.buzzer_beeps_remaining--;
  }
  else
  {
    Buzzer_Set(1U);
  }
  greenhouse.buzzer_tick = now + 120U;
}

static uint8_t Servo_Start(WindowState target, uint32_t now)
{
  uint16_t pulse;

  if ((greenhouse.servo_moving != 0U) ||
      (ServoConfigurationValid() == 0U))
  {
    return 0U;
  }

  Fan_Set(0U);
  Pump_Set(0U);
  pulse = (target == WINDOW_OPEN) ?
          greenhouse.config.servo_open_pulse_us :
          greenhouse.config.servo_closed_pulse_us;
  __HAL_TIM_SET_COMPARE(greenhouse.pwm_timer, TIM_CHANNEL_1, pulse);
  if (HAL_TIM_PWM_Start(greenhouse.pwm_timer, TIM_CHANNEL_1) != HAL_OK)
  {
    return 0U;
  }

  greenhouse.servo_target = target;
  greenhouse.servo_moving = 1U;
  greenhouse.servo_tick = now;
  return 1U;
}

static void Servo_Process(uint32_t now)
{
  if ((greenhouse.servo_moving != 0U) &&
      (Elapsed(now, greenhouse.servo_tick,
               greenhouse.config.servo_move_time_ms) != 0U))
  {
    (void)HAL_TIM_PWM_Stop(greenhouse.pwm_timer, TIM_CHANNEL_1);
    greenhouse.servo_moving = 0U;
    greenhouse.window_state = greenhouse.servo_target;
    if (greenhouse.startup_complete == 0U)
    {
      greenhouse.startup_complete = 1U;
      greenhouse.ready_message_active = 1U;
      greenhouse.ready_tick = now;
    }
  }
}

static void Watering_StopAndReset(void)
{
  Pump_Set(0U);
  greenhouse.water_state = WATER_IDLE;
  greenhouse.watering_attempts = 0U;
}

static void Watering_StartAttempt(uint32_t now)
{
  if ((greenhouse.config.pump_run_time_ms == 0U) ||
      (greenhouse.fan_on != 0U) || (greenhouse.servo_moving != 0U))
  {
    return;
  }

  greenhouse.watering_attempts++;
  greenhouse.water_state = WATER_PUMPING;
  greenhouse.water_tick = now;
  Pump_Set(1U);
  if (greenhouse.pump_on != 0U)
  {
    Buzzer_Request(1U, now);
  }
  else
  {
    greenhouse.water_state = WATER_IDLE;
  }
}

static void Watering_ProcessTimers(uint32_t now)
{
  if ((greenhouse.water_state == WATER_PUMPING) &&
      (Elapsed(now, greenhouse.water_tick,
               greenhouse.config.pump_run_time_ms) != 0U))
  {
    Pump_Set(0U);
    greenhouse.water_state = WATER_SOAKING;
    greenhouse.water_tick = now;
  }
}

static void Watering_ProcessAllowed(uint32_t now)
{
  if ((greenhouse.soil_valid == 0U) ||
      (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                           greenhouse.config.soil_wet_adc) == 0U))
  {
    Watering_StopAndReset();
    return;
  }

  if (greenhouse.soil_percent >= WATERING_COMPLETE_PERCENT)
  {
    Watering_StopAndReset();
    return;
  }

  switch (greenhouse.water_state)
  {
    case WATER_IDLE:
      if (greenhouse.soil_percent < PUMP_START_PERCENT)
      {
        Watering_StartAttempt(now);
      }
      break;

    case WATER_PUMPING:
      break;

    case WATER_SOAKING:
      if (Elapsed(now, greenhouse.water_tick, SOAK_TIME_MS) != 0U)
      {
        if (greenhouse.watering_attempts >= MAX_WATERING_ATTEMPTS)
        {
          greenhouse.water_state = WATER_FAILED;
        }
        else
        {
          Watering_StartAttempt(now);
        }
      }
      break;

    case WATER_FAILED:
    default:
      Pump_Set(0U);
      break;
  }
}

static void Sensors_StartCycle(uint32_t now)
{
  int32_t bmp_temperature;

  greenhouse.sensor_tick = now;

  greenhouse.light_valid =
      (ADC_ReadChannel(ADC_CHANNEL_0, &greenhouse.light_adc) == HAL_OK) ?
      1U : 0U;
  greenhouse.soil_valid =
      (ADC_ReadChannel(ADC_CHANNEL_1, &greenhouse.soil_adc) == HAL_OK) ?
      1U : 0U;

  if ((greenhouse.light_valid != 0U) &&
      (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                           greenhouse.config.light_bright_adc) != 0U))
  {
    greenhouse.light_percent = ConvertPercent(
        greenhouse.light_adc, greenhouse.config.light_dark_adc,
        greenhouse.config.light_bright_adc);
  }

  if ((greenhouse.soil_valid != 0U) &&
      (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                           greenhouse.config.soil_wet_adc) != 0U))
  {
    greenhouse.soil_percent = ConvertPercent(
        greenhouse.soil_adc, greenhouse.config.soil_dry_adc,
        greenhouse.config.soil_wet_adc);
  }

  if ((greenhouse.bmp280_initialized == 0U) &&
      (Elapsed(now, greenhouse.bmp280_retry_tick,
               SENSOR_RETRY_INTERVAL_MS) != 0U))
  {
    greenhouse.bmp280_retry_tick = now;
    greenhouse.bmp280_initialized =
        (BMP280_Init(&greenhouse.bmp280, greenhouse.i2c) == HAL_OK) ?
        1U : 0U;
  }

  if (greenhouse.bmp280_initialized != 0U)
  {
    greenhouse.bmp280_valid =
        (BMP280_Read(&greenhouse.bmp280, &bmp_temperature,
                     &greenhouse.pressure_pa) == HAL_OK) ? 1U : 0U;
    if (greenhouse.bmp280_valid == 0U)
    {
      greenhouse.bmp280_initialized = 0U;
      greenhouse.bmp280_retry_tick = now;
    }
  }

  if ((greenhouse.aht20_initialized == 0U) &&
      (Elapsed(now, greenhouse.aht20_retry_tick,
               SENSOR_RETRY_INTERVAL_MS) != 0U))
  {
    greenhouse.aht20_retry_tick = now;
    greenhouse.aht20_initialized =
        (AHT20_Init(&greenhouse.aht20, greenhouse.i2c) == HAL_OK) ?
        1U : 0U;
  }

  if (greenhouse.aht20_initialized != 0U)
  {
    if (AHT20_StartMeasurement(&greenhouse.aht20) == HAL_OK)
    {
      greenhouse.aht20_pending = 1U;
      greenhouse.aht20_tick = now;
    }
    else
    {
      greenhouse.aht20_valid = 0U;
      greenhouse.aht20_initialized = 0U;
      greenhouse.aht20_retry_tick = now;
    }
  }
  else
  {
    greenhouse.aht20_valid = 0U;
  }
}

static void Sensors_ProcessAht20(uint32_t now)
{
  AHT20_Status status;

  if ((greenhouse.aht20_pending == 0U) ||
      (Elapsed(now, greenhouse.aht20_tick,
               AHT20_MEASUREMENT_TIME_MS) == 0U))
  {
    return;
  }

  status = AHT20_ReadMeasurement(&greenhouse.aht20,
                                 &greenhouse.temperature_mc,
                                 &greenhouse.humidity_mpct);
  if (status == AHT20_STATUS_OK)
  {
    greenhouse.aht20_valid = 1U;
    greenhouse.aht20_pending = 0U;
  }
  else if ((status == AHT20_STATUS_ERROR) ||
           (Elapsed(now, greenhouse.aht20_tick,
                    AHT20_MEASUREMENT_TIMEOUT_MS) != 0U))
  {
    greenhouse.aht20_valid = 0U;
    greenhouse.aht20_pending = 0U;
    greenhouse.aht20_initialized = 0U;
    greenhouse.aht20_retry_tick = now;
  }
}

static void LowLight_Process(uint32_t now)
{
  if ((greenhouse.light_valid == 0U) ||
      (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                           greenhouse.config.light_bright_adc) == 0U))
  {
    greenhouse.low_light_timer_active = 0U;
    greenhouse.low_light_warning = 0U;
    return;
  }

  if (greenhouse.light_percent < LOW_LIGHT_PERCENT)
  {
    if (greenhouse.low_light_timer_active == 0U)
    {
      greenhouse.low_light_timer_active = 1U;
      greenhouse.low_light_tick = now;
    }
    else if ((greenhouse.low_light_warning == 0U) &&
             (Elapsed(now, greenhouse.low_light_tick,
                      LOW_LIGHT_TIME_MS) != 0U))
    {
      greenhouse.low_light_warning = 1U;
      Buzzer_Request(2U, now);
    }
  }
  else if (greenhouse.light_percent >= LOW_LIGHT_CLEAR_PERCENT)
  {
    greenhouse.low_light_timer_active = 0U;
    greenhouse.low_light_warning = 0U;
  }
}

static WindowState RequiredWindowState(void)
{
  if (greenhouse.aht20_valid == 0U)
  {
    return WINDOW_OPEN;
  }

  if ((greenhouse.temperature_mc >= WINDOW_OPEN_TEMP_MC) ||
      (greenhouse.humidity_mpct >= WINDOW_OPEN_HUMIDITY_MPCT))
  {
    return WINDOW_OPEN;
  }

  if ((greenhouse.temperature_mc <= WINDOW_CLOSE_TEMP_MC) &&
      (greenhouse.humidity_mpct <= WINDOW_CLOSE_HUMIDITY_MPCT))
  {
    return WINDOW_CLOSED;
  }

  return greenhouse.window_state;
}

static void FullControl_Process(uint32_t now)
{
  WindowState required_window;

  greenhouse.critical_temperature =
      ((greenhouse.aht20_valid != 0U) &&
       (greenhouse.temperature_mc >= CRITICAL_TEMP_MC)) ? 1U : 0U;

  if (greenhouse.aht20_valid != 0U)
  {
    if ((greenhouse.temperature_mc >= FAN_ON_TEMP_MC) ||
        (greenhouse.humidity_mpct >= FAN_ON_HUMIDITY_MPCT))
    {
      greenhouse.ventilation_requested = 1U;
    }
    else if ((greenhouse.temperature_mc <= FAN_OFF_TEMP_MC) &&
             (greenhouse.humidity_mpct <= FAN_OFF_HUMIDITY_MPCT))
    {
      greenhouse.ventilation_requested = 0U;
    }
  }
  else
  {
    greenhouse.ventilation_requested = 1U;
  }

  if (greenhouse.startup_complete == 0U)
  {
    Fan_Set(0U);
    Pump_Set(0U);
    return;
  }

  required_window = RequiredWindowState();
  if ((greenhouse.servo_moving == 0U) &&
      (required_window != WINDOW_UNKNOWN) &&
      (required_window != greenhouse.window_state))
  {
    if (Servo_Start(required_window, now) != 0U)
    {
      return;
    }
  }

  if (greenhouse.servo_moving != 0U)
  {
    Fan_Set(0U);
    Pump_Set(0U);
    return;
  }

  if ((greenhouse.critical_temperature != 0U) ||
      (greenhouse.aht20_valid == 0U) ||
      (greenhouse.ventilation_requested != 0U))
  {
    if (greenhouse.water_state == WATER_PUMPING)
    {
      Watering_StopAndReset();
    }
    else
    {
      Pump_Set(0U);
    }
    Fan_Set(1U);
    return;
  }

  Fan_Set(0U);
  Watering_ProcessAllowed(now);
}

static void FormatTemperature(char *text, size_t size)
{
  uint32_t magnitude;
  char sign;

  sign = ' ';
  if (greenhouse.temperature_mc < 0)
  {
    sign = '-';
    magnitude = (uint32_t)(-(int64_t)greenhouse.temperature_mc);
  }
  else
  {
    magnitude = (uint32_t)greenhouse.temperature_mc;
  }

  (void)snprintf(text, size, "T:%c%lu.%luC H:%lu%%", sign,
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)((magnitude % 1000U) / 100U),
                 (unsigned long)(greenhouse.humidity_mpct / 1000U));
}

static void DisplayWarningScreen(const char *line0, const char *line2,
                                 const char *line4)
{
  SSD1306_Clear(&greenhouse.oled);
  SSD1306_WriteLine(&greenhouse.oled, 0U, line0);
  SSD1306_WriteLine(&greenhouse.oled, 2U, line2);
  SSD1306_WriteLine(&greenhouse.oled, 4U, line4);
}

static void Display_Process(uint32_t now)
{
  char line[24];
  const char *window_text;

  if ((greenhouse.oled_valid == 0U) ||
      (Elapsed(now, greenhouse.display_tick, DISPLAY_INTERVAL_MS) == 0U))
  {
    return;
  }
  greenhouse.display_tick = now;

  if (greenhouse.ready_message_active != 0U)
  {
    if (Elapsed(now, greenhouse.ready_tick, READY_MESSAGE_TIME_MS) == 0U)
    {
      DisplayWarningScreen("SYSTEM READY", "MONITORING ACTIVE", "");
      (void)SSD1306_Update(&greenhouse.oled);
      return;
    }
    greenhouse.ready_message_active = 0U;
  }

  if (greenhouse.test_mode == GREENHOUSE_TEST_OLED)
  {
    DisplayWarningScreen("OLED TEST", "128 X 64 I2C", "DISPLAY OK");
  }
  else if (greenhouse.critical_temperature != 0U)
  {
    DisplayWarningScreen("HIGH TEMP ALERT", "PUMP OFF", "VENTILATING");
  }
  else if (greenhouse.aht20_valid == 0U)
  {
    DisplayWarningScreen("AHT20 ERROR", "PUMP OFF", "SAFE VENTILATION");
  }
  else if (greenhouse.soil_valid == 0U)
  {
    DisplayWarningScreen("SOIL SENSOR ERROR", "PUMP DISABLED", "");
  }
  else if (greenhouse.light_valid == 0U)
  {
    DisplayWarningScreen("LIGHT SENSOR ERROR", "CONTROL CONTINUES", "");
  }
  else if (greenhouse.water_state == WATER_FAILED)
  {
    DisplayWarningScreen("CHECK WATER/SOIL", "3 ATTEMPTS FAILED", "PUMP OFF");
  }
  else
  {
    SSD1306_Clear(&greenhouse.oled);
    if (greenhouse.display_page == 0U)
    {
      FormatTemperature(line, sizeof(line));
      SSD1306_WriteLine(&greenhouse.oled, 0U, line);
      if (greenhouse.bmp280_valid != 0U)
      {
        (void)snprintf(line, sizeof(line), "PRESSURE:%lu HPA",
                       (unsigned long)(greenhouse.pressure_pa / 100U));
      }
      else
      {
        (void)snprintf(line, sizeof(line), "PRESSURE ERROR");
      }
      SSD1306_WriteLine(&greenhouse.oled, 2U, line);

      if (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                              greenhouse.config.light_bright_adc) != 0U)
      {
        (void)snprintf(line, sizeof(line), "LIGHT:%d%%",
                       (int)greenhouse.light_percent);
      }
      else
      {
        (void)snprintf(line, sizeof(line), "SET LIGHT CAL");
      }
      SSD1306_WriteLine(&greenhouse.oled, 4U, line);
      SSD1306_WriteLine(&greenhouse.oled, 6U,
                        (greenhouse.low_light_warning != 0U) ?
                        "LOW LIGHT" : "SYSTEM MONITORING");
    }
    else
    {
      if (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                              greenhouse.config.soil_wet_adc) != 0U)
      {
        (void)snprintf(line, sizeof(line), "SOIL:%d%%",
                       (int)greenhouse.soil_percent);
      }
      else
      {
        (void)snprintf(line, sizeof(line), "SET SOIL CAL");
      }
      SSD1306_WriteLine(&greenhouse.oled, 0U, line);
      (void)snprintf(line, sizeof(line), "FAN:%s PUMP:%s",
                     (greenhouse.fan_on != 0U) ? "ON" : "OFF",
                     (greenhouse.pump_on != 0U) ? "ON" : "OFF");
      SSD1306_WriteLine(&greenhouse.oled, 2U, line);
      window_text = (greenhouse.window_state == WINDOW_OPEN) ? "OPEN" :
                    ((greenhouse.window_state == WINDOW_CLOSED) ? "CLOSED" :
                     "UNKNOWN");
      (void)snprintf(line, sizeof(line), "WINDOW:%s", window_text);
      SSD1306_WriteLine(&greenhouse.oled, 4U, line);

      if (ServoConfigurationValid() == 0U)
      {
        SSD1306_WriteLine(&greenhouse.oled, 6U, "SET SERVO CAL");
      }
      else if (greenhouse.config.pump_run_time_ms == 0U)
      {
        SSD1306_WriteLine(&greenhouse.oled, 6U, "SET PUMP TIME");
      }
      else
      {
        SSD1306_WriteLine(&greenhouse.oled, 6U, "CONTROL ACTIVE");
      }
    }
    greenhouse.display_page ^= 1U;
  }

  if (SSD1306_Update(&greenhouse.oled) != HAL_OK)
  {
    greenhouse.oled_valid = 0U;
  }
}

static void TestMode_Process(uint32_t now)
{
  Fan_Set(0U);
  Pump_Set(0U);
  greenhouse.critical_temperature = 0U;

  switch (greenhouse.test_mode)
  {
    case GREENHOUSE_TEST_BUZZER:
      if (Elapsed(now, greenhouse.test_tick, 2000U) != 0U)
      {
        greenhouse.test_tick = now;
        Buzzer_Request(1U, now);
      }
      break;

    case GREENHOUSE_TEST_SERVO:
      if ((greenhouse.servo_moving == 0U) &&
          (Elapsed(now, greenhouse.test_tick, 2000U) != 0U))
      {
        greenhouse.test_tick = now;
        (void)Servo_Start((greenhouse.window_state == WINDOW_OPEN) ?
                          WINDOW_CLOSED : WINDOW_OPEN, now);
      }
      break;

    case GREENHOUSE_TEST_FAN:
      if (Elapsed(now, greenhouse.test_tick, 2000U) != 0U)
      {
        greenhouse.test_tick = now;
        greenhouse.test_output_on ^= 1U;
      }
      Fan_Set(greenhouse.test_output_on);
      break;

    case GREENHOUSE_TEST_PUMP:
      if ((greenhouse.config.pump_run_time_ms != 0U) &&
          (Elapsed(now, greenhouse.test_tick,
                   greenhouse.test_output_on != 0U ?
                   greenhouse.config.pump_run_time_ms : 2000U) != 0U))
      {
        greenhouse.test_tick = now;
        greenhouse.test_output_on ^= 1U;
      }
      Pump_Set(greenhouse.test_output_on);
      break;

    case GREENHOUSE_TEST_LIGHT:
    case GREENHOUSE_TEST_SOIL:
    case GREENHOUSE_TEST_AHT20:
    case GREENHOUSE_TEST_BMP280:
    case GREENHOUSE_TEST_OLED:
    default:
      break;
  }
}

HAL_StatusTypeDef Greenhouse_Init(ADC_HandleTypeDef *adc,
                                  I2C_HandleTypeDef *i2c,
                                  TIM_HandleTypeDef *pwm_timer,
                                  const GreenhouseConfig *config,
                                  GreenhouseTestMode test_mode)
{
  uint32_t now;

  if ((adc == NULL) || (i2c == NULL) || (pwm_timer == NULL) ||
      (config == NULL))
  {
    return HAL_ERROR;
  }

  memset(&greenhouse, 0, sizeof(greenhouse));
  greenhouse.adc = adc;
  greenhouse.i2c = i2c;
  greenhouse.pwm_timer = pwm_timer;
  greenhouse.config = *config;
  greenhouse.test_mode = test_mode;
  greenhouse.window_state = WINDOW_UNKNOWN;
  greenhouse.water_state = WATER_IDLE;
  greenhouse.aht20.i2c = i2c;
  greenhouse.bmp280.i2c = i2c;
  greenhouse.oled.i2c = i2c;

  Fan_Set(0U);
  Pump_Set(0U);
  Buzzer_Set(0U);
  (void)HAL_TIM_PWM_Stop(pwm_timer, TIM_CHANNEL_1);
  greenhouse.adc_ready =
      (HAL_ADCEx_Calibration_Start(adc) == HAL_OK) ? 1U : 0U;

  greenhouse.aht20_initialized =
      (AHT20_Init(&greenhouse.aht20, i2c) == HAL_OK) ? 1U : 0U;
  greenhouse.bmp280_initialized =
      (BMP280_Init(&greenhouse.bmp280, i2c) == HAL_OK) ? 1U : 0U;
  greenhouse.oled_valid =
      (SSD1306_Init(&greenhouse.oled, i2c) == HAL_OK) ? 1U : 0U;

  now = HAL_GetTick();
  greenhouse.sensor_tick = now - SENSOR_INTERVAL_MS;
  greenhouse.aht20_retry_tick = now;
  greenhouse.bmp280_retry_tick = now;
  greenhouse.display_tick = now - DISPLAY_INTERVAL_MS;
  greenhouse.test_tick = now;

  if (test_mode == GREENHOUSE_TEST_FULL)
  {
    if (Servo_Start(WINDOW_CLOSED, now) == 0U)
    {
      greenhouse.startup_complete = 1U;
      greenhouse.ready_message_active = 1U;
      greenhouse.ready_tick = now;
    }
  }
  else
  {
    greenhouse.startup_complete = 1U;
  }

  if (greenhouse.oled_valid != 0U)
  {
    DisplayWarningScreen("SYSTEM STARTING", "SENSORS INITIALIZED", "");
    (void)SSD1306_Update(&greenhouse.oled);
  }

  return HAL_OK;
}

void Greenhouse_Process(void)
{
  uint32_t now;
  uint8_t sensor_alarm;

  now = HAL_GetTick();
  Servo_Process(now);
  Watering_ProcessTimers(now);
  Sensors_ProcessAht20(now);

  if (Elapsed(now, greenhouse.sensor_tick, SENSOR_INTERVAL_MS) != 0U)
  {
    Sensors_StartCycle(now);
  }

  LowLight_Process(now);

  sensor_alarm = ((greenhouse.aht20_valid == 0U) ||
                  (greenhouse.soil_valid == 0U)) ? 1U : 0U;
  if ((sensor_alarm != 0U) && (greenhouse.previous_sensor_alarm == 0U))
  {
    Buzzer_Request(3U, now);
  }
  greenhouse.previous_sensor_alarm = sensor_alarm;

  if (greenhouse.test_mode == GREENHOUSE_TEST_FULL)
  {
    FullControl_Process(now);
  }
  else
  {
    TestMode_Process(now);
  }

  Buzzer_Process(now);
  Display_Process(now);
}
