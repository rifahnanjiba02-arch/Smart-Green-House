#include "greenhouse.h"

#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"

#include <stdio.h>
#include <string.h>

#define BUZZER_PORT GPIOA
#define BUZZER_PIN GPIO_PIN_8
#define PUMP_PORT GPIOA
#define PUMP_PIN GPIO_PIN_9
#define FAN_PORT GPIOA
#define FAN_PIN GPIO_PIN_10

#define SENSOR_INTERVAL_MS 2000U
#define TEST_SENSOR_INTERVAL_MS 500U
#define RETRY_INTERVAL_MS 10000U
#define AHT20_READ_DELAY_MS 85U
#define AHT20_TIMEOUT_MS 250U
#define DISPLAY_INTERVAL_MS 2500U
#define TEST_DISPLAY_INTERVAL_MS 500U
#define STARTUP_MESSAGE_MS 500U
#define READY_MESSAGE_MS 2000U
#define SOAK_TIME_MS 20000U
#define LOW_LIGHT_TIME_MS 30000U
#define MAX_WATERING_ATTEMPTS 3U
#define ADC_TIMEOUT_MS 10U
#define ADC_MAX_VALUE 4095U

#define FAN_ON_TEMP_MC 30000
#define FAN_OFF_TEMP_MC 28000
#define FAN_ON_HUMIDITY_MPCT 80000U
#define FAN_OFF_HUMIDITY_MPCT 75000U
#define WINDOW_OPEN_TEMP_MC 32000
#define WINDOW_CLOSE_TEMP_MC 29000
#define WINDOW_OPEN_HUMIDITY_MPCT 85000U
#define WINDOW_CLOSE_HUMIDITY_MPCT 78000U
#define CRITICAL_TEMP_MC 40000
#define PUMP_START_PERCENT 30
#define WATERING_COMPLETE_PERCENT 45
#define LOW_LIGHT_PERCENT 20
#define LOW_LIGHT_CLEAR_PERCENT 30

typedef enum
{
  SENSOR_NOT_ATTEMPTED = 0,
  SENSOR_PENDING,
  SENSOR_VALID,
  SENSOR_ERROR
} SensorState;

typedef enum
{
  STARTUP_STARTING = 0,
  STARTUP_INITIALIZING_SENSORS,
  STARTUP_WAITING_FOR_FIRST_READINGS,
  STARTUP_CLOSING_WINDOW,
  STARTUP_READY,
  STARTUP_CONFIG_REQUIRED
} StartupState;

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

  SensorState aht20_state;
  SensorState bmp280_state;
  SensorState soil_state;
  SensorState light_state;
  StartupState startup_state;
  WindowState window_state;
  WindowState servo_target;
  WaterState water_state;

  uint8_t aht20_initialized;
  uint8_t aht20_has_reading;
  uint8_t bmp280_initialized;
  uint8_t aht20_attempt_complete;
  uint8_t bmp280_attempt_complete;
  uint8_t soil_attempt_complete;
  uint8_t light_attempt_complete;
  uint8_t adc_ready;
  uint8_t peripherals_ready;
  uint8_t sensor_initialization_done;
  uint8_t oled_valid;
  uint8_t fan_on;
  uint8_t pump_on;
  uint8_t servo_moving;
  uint8_t servo_start_failed;
  uint8_t ventilation_requested;
  uint8_t critical_temperature;
  uint8_t low_light_timer_active;
  uint8_t low_light_warning;
  uint8_t ready_message_active;
  uint8_t display_page;
  uint8_t warning_turn;
  uint8_t warning_index;
  uint8_t watering_attempts;
  uint8_t buzzer_on;
  uint8_t buzzer_beeps_remaining;
  uint8_t previous_sensor_alarm;
  uint8_t test_output_on;

  uint32_t sensor_tick;
  uint32_t aht20_tick;
  uint32_t aht20_retry_tick;
  uint32_t bmp280_retry_tick;
  uint32_t oled_retry_tick;
  uint32_t servo_tick;
  uint32_t servo_retry_tick;
  uint32_t water_tick;
  uint32_t soil_sample_tick;
  uint32_t low_light_tick;
  uint32_t display_tick;
  uint32_t startup_tick;
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

static uint8_t IsSensorTest(void)
{
  return ((greenhouse.test_mode == GREENHOUSE_TEST_LIGHT) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_SOIL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_AHT20) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_BMP280)) ? 1U : 0U;
}

static uint8_t Aht20Selected(void)
{
  return ((greenhouse.test_mode == GREENHOUSE_TEST_FULL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_AHT20)) ? 1U : 0U;
}

static uint8_t Bmp280Selected(void)
{
  return ((greenhouse.test_mode == GREENHOUSE_TEST_FULL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_BMP280)) ? 1U : 0U;
}

static uint8_t SoilSelected(void)
{
  return ((greenhouse.test_mode == GREENHOUSE_TEST_FULL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_SOIL)) ? 1U : 0U;
}

static uint8_t LightSelected(void)
{
  return ((greenhouse.test_mode == GREENHOUSE_TEST_FULL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_LIGHT)) ? 1U : 0U;
}

static uint32_t SensorInterval(void)
{
  return (IsSensorTest() != 0U) ? TEST_SENSOR_INTERVAL_MS :
                                 SENSOR_INTERVAL_MS;
}

static uint32_t DisplayInterval(void)
{
  return (IsSensorTest() != 0U) ? TEST_DISPLAY_INTERVAL_MS :
                                 DISPLAY_INTERVAL_MS;
}

static uint8_t AdcCalibrationValid(uint16_t first, uint16_t second)
{
  return ((first != 0U) && (second != 0U) && (first != second) &&
          (first <= ADC_MAX_VALUE) && (second <= ADC_MAX_VALUE)) ? 1U : 0U;
}

static uint8_t ServoConfigurationValid(void)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(greenhouse.pwm_timer) + 1U;

  return ((greenhouse.config.servo_closed_pulse_us != 0U) &&
          (greenhouse.config.servo_open_pulse_us != 0U) &&
          (greenhouse.config.servo_closed_pulse_us !=
           greenhouse.config.servo_open_pulse_us) &&
          (greenhouse.config.servo_closed_pulse_us <= period) &&
          (greenhouse.config.servo_open_pulse_us <= period) &&
          (greenhouse.config.servo_move_time_ms != 0U)) ? 1U : 0U;
}

static uint8_t Aht20ReadingAvailable(void)
{
  return ((greenhouse.aht20_state == SENSOR_VALID) ||
          ((greenhouse.aht20_state == SENSOR_PENDING) &&
           (greenhouse.aht20_has_reading != 0U))) ? 1U : 0U;
}

static int16_t ConvertPercent(uint16_t raw, uint16_t zero, uint16_t full)
{
  int32_t value = (((int32_t)raw - (int32_t)zero) * 100) /
                  ((int32_t)full - (int32_t)zero);

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

  if ((greenhouse.adc_ready == 0U) || (value == NULL))
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

static void Buzzer_Request(uint8_t count, uint32_t now)
{
  if ((greenhouse.critical_temperature == 0U) &&
      (greenhouse.water_state != WATER_FAILED) &&
      (count > greenhouse.buzzer_beeps_remaining))
  {
    greenhouse.buzzer_beeps_remaining = count;
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
    Buzzer_Set(((now % 1000U) < 150U) ? 1U : 0U);
  }
  else if (greenhouse.buzzer_beeps_remaining == 0U)
  {
    Buzzer_Set(0U);
  }
  else if (TimeReached(now, greenhouse.buzzer_tick) != 0U)
  {
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
}

static void Watering_Reset(void)
{
  Pump_Set(0U);
  greenhouse.water_state = WATER_IDLE;
  greenhouse.watering_attempts = 0U;
}

static void Watering_Abort(void)
{
  Pump_Set(0U);
  if (greenhouse.water_state != WATER_FAILED)
  {
    greenhouse.water_state = WATER_IDLE;
    greenhouse.watering_attempts = 0U;
  }
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
  Watering_Abort();
  pulse = (target == WINDOW_OPEN) ? greenhouse.config.servo_open_pulse_us :
                                   greenhouse.config.servo_closed_pulse_us;
  __HAL_TIM_SET_COMPARE(greenhouse.pwm_timer, TIM_CHANNEL_1, pulse);
  if (HAL_TIM_PWM_Start(greenhouse.pwm_timer, TIM_CHANNEL_1) != HAL_OK)
  {
    greenhouse.servo_start_failed = 1U;
    greenhouse.servo_retry_tick = now;
    return 0U;
  }
  greenhouse.servo_start_failed = 0U;
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
  }
}

static void Soil_Read(uint32_t now)
{
  greenhouse.soil_attempt_complete = 1U;
  if (ADC_ReadChannel(ADC_CHANNEL_1, &greenhouse.soil_adc) == HAL_OK)
  {
    greenhouse.soil_state = SENSOR_VALID;
    greenhouse.soil_sample_tick = now;
    if (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                            greenhouse.config.soil_wet_adc) != 0U)
    {
      greenhouse.soil_percent = ConvertPercent(
          greenhouse.soil_adc, greenhouse.config.soil_dry_adc,
          greenhouse.config.soil_wet_adc);
    }
  }
  else
  {
    greenhouse.soil_state = SENSOR_ERROR;
  }
}

static void Light_Read(void)
{
  greenhouse.light_attempt_complete = 1U;
  if (ADC_ReadChannel(ADC_CHANNEL_0, &greenhouse.light_adc) == HAL_OK)
  {
    greenhouse.light_state = SENSOR_VALID;
    if (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                            greenhouse.config.light_bright_adc) != 0U)
    {
      greenhouse.light_percent = ConvertPercent(
          greenhouse.light_adc, greenhouse.config.light_dark_adc,
          greenhouse.config.light_bright_adc);
    }
  }
  else
  {
    greenhouse.light_state = SENSOR_ERROR;
  }
}

static void Watering_Start(uint32_t now)
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

static void Watering_Timers(uint32_t now)
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

static void Watering_Process(uint32_t now)
{
  if ((greenhouse.soil_state != SENSOR_VALID) ||
      (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                           greenhouse.config.soil_wet_adc) == 0U) ||
      (greenhouse.config.pump_run_time_ms == 0U))
  {
    Watering_Reset();
    return;
  }
  if (greenhouse.soil_percent >= WATERING_COMPLETE_PERCENT)
  {
    Watering_Reset();
    return;
  }

  if ((greenhouse.water_state == WATER_IDLE) &&
      (greenhouse.soil_percent < PUMP_START_PERCENT))
  {
    Watering_Start(now);
  }
  else if ((greenhouse.water_state == WATER_SOAKING) &&
           (Elapsed(now, greenhouse.water_tick, SOAK_TIME_MS) != 0U))
  {
    if (TimeReached(greenhouse.soil_sample_tick,
                    greenhouse.water_tick + SOAK_TIME_MS) == 0U)
    {
      Soil_Read(now);
    }
    if (greenhouse.soil_state != SENSOR_VALID)
    {
      Watering_Reset();
    }
    else if (greenhouse.soil_percent >= WATERING_COMPLETE_PERCENT)
    {
      Watering_Reset();
    }
    else if (greenhouse.watering_attempts >= MAX_WATERING_ATTEMPTS)
    {
      Pump_Set(0U);
      greenhouse.water_state = WATER_FAILED;
    }
    else
    {
      Watering_Start(now);
    }
  }
  else if (greenhouse.water_state == WATER_FAILED)
  {
    Pump_Set(0U);
  }
}

static void Sensors_Initialize(uint32_t now)
{
  if (Aht20Selected() != 0U)
  {
    greenhouse.aht20_initialized =
        (AHT20_Init(&greenhouse.aht20, greenhouse.i2c) == HAL_OK) ? 1U : 0U;
    if (greenhouse.aht20_initialized == 0U)
    {
      greenhouse.aht20_state = SENSOR_ERROR;
      greenhouse.aht20_has_reading = 0U;
      greenhouse.aht20_attempt_complete = 1U;
      greenhouse.aht20_retry_tick = now;
    }
  }
  if (Bmp280Selected() != 0U)
  {
    greenhouse.bmp280_initialized =
        (BMP280_Init(&greenhouse.bmp280, greenhouse.i2c) == HAL_OK) ? 1U : 0U;
    if (greenhouse.bmp280_initialized == 0U)
    {
      greenhouse.bmp280_state = SENSOR_ERROR;
      greenhouse.bmp280_attempt_complete = 1U;
      greenhouse.bmp280_retry_tick = now;
    }
  }
}

static void Sensors_StartCycle(uint32_t now)
{
  int32_t bmp_temperature;

  greenhouse.sensor_tick = now;
  if (LightSelected() != 0U)
  {
    Light_Read();
  }
  if (SoilSelected() != 0U)
  {
    Soil_Read(now);
  }

  if (Bmp280Selected() != 0U)
  {
    if ((greenhouse.bmp280_initialized == 0U) &&
        ((greenhouse.bmp280_state == SENSOR_NOT_ATTEMPTED) ||
         (Elapsed(now, greenhouse.bmp280_retry_tick, RETRY_INTERVAL_MS) != 0U)))
    {
      greenhouse.bmp280_initialized =
          (BMP280_Init(&greenhouse.bmp280, greenhouse.i2c) == HAL_OK) ? 1U : 0U;
      greenhouse.bmp280_retry_tick = now;
      if (greenhouse.bmp280_initialized == 0U)
      {
        greenhouse.bmp280_state = SENSOR_ERROR;
        greenhouse.bmp280_attempt_complete = 1U;
      }
    }
    if (greenhouse.bmp280_initialized != 0U)
    {
      if (BMP280_Read(&greenhouse.bmp280, &bmp_temperature,
                      &greenhouse.pressure_pa) == HAL_OK)
      {
        greenhouse.bmp280_state = SENSOR_VALID;
        greenhouse.bmp280_attempt_complete = 1U;
      }
      else
      {
        greenhouse.bmp280_state = SENSOR_ERROR;
        greenhouse.bmp280_attempt_complete = 1U;
        greenhouse.bmp280_initialized = 0U;
        greenhouse.bmp280_retry_tick = now;
      }
    }
  }

  if ((Aht20Selected() != 0U) &&
      (greenhouse.aht20_state != SENSOR_PENDING))
  {
    if ((greenhouse.aht20_initialized == 0U) &&
        ((greenhouse.aht20_state == SENSOR_NOT_ATTEMPTED) ||
         (Elapsed(now, greenhouse.aht20_retry_tick, RETRY_INTERVAL_MS) != 0U)))
    {
      greenhouse.aht20_initialized =
          (AHT20_Init(&greenhouse.aht20, greenhouse.i2c) == HAL_OK) ? 1U : 0U;
      greenhouse.aht20_retry_tick = now;
      if (greenhouse.aht20_initialized == 0U)
      {
        greenhouse.aht20_state = SENSOR_ERROR;
        greenhouse.aht20_has_reading = 0U;
        greenhouse.aht20_attempt_complete = 1U;
      }
    }
    if (greenhouse.aht20_initialized != 0U)
    {
      if (AHT20_StartMeasurement(&greenhouse.aht20) == HAL_OK)
      {
        greenhouse.aht20_state = SENSOR_PENDING;
        greenhouse.aht20_tick = now;
      }
      else
      {
        greenhouse.aht20_state = SENSOR_ERROR;
        greenhouse.aht20_has_reading = 0U;
        greenhouse.aht20_attempt_complete = 1U;
        greenhouse.aht20_initialized = 0U;
        greenhouse.aht20_retry_tick = now;
      }
    }
  }
}

static void Sensors_ProcessAht20(uint32_t now)
{
  AHT20_Status status;

  if ((greenhouse.aht20_state != SENSOR_PENDING) ||
      (Elapsed(now, greenhouse.aht20_tick, AHT20_READ_DELAY_MS) == 0U))
  {
    return;
  }
  status = AHT20_ReadMeasurement(&greenhouse.aht20,
                                 &greenhouse.temperature_mc,
                                 &greenhouse.humidity_mpct);
  if (status == AHT20_STATUS_OK)
  {
    greenhouse.aht20_state = SENSOR_VALID;
    greenhouse.aht20_has_reading = 1U;
    greenhouse.aht20_attempt_complete = 1U;
  }
  else if ((status == AHT20_STATUS_ERROR) ||
           (Elapsed(now, greenhouse.aht20_tick, AHT20_TIMEOUT_MS) != 0U))
  {
    greenhouse.aht20_state = SENSOR_ERROR;
    greenhouse.aht20_has_reading = 0U;
    greenhouse.aht20_attempt_complete = 1U;
    greenhouse.aht20_initialized = 0U;
    greenhouse.aht20_retry_tick = now;
  }
}

static void LowLight_Process(uint32_t now)
{
  if ((greenhouse.light_state != SENSOR_VALID) ||
      (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                           greenhouse.config.light_bright_adc) == 0U))
  {
    greenhouse.low_light_timer_active = 0U;
    greenhouse.low_light_warning = 0U;
  }
  else if (greenhouse.light_percent < LOW_LIGHT_PERCENT)
  {
    if (greenhouse.low_light_timer_active == 0U)
    {
      greenhouse.low_light_timer_active = 1U;
      greenhouse.low_light_tick = now;
    }
    else if ((greenhouse.low_light_warning == 0U) &&
             (Elapsed(now, greenhouse.low_light_tick, LOW_LIGHT_TIME_MS) != 0U))
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

static WindowState RequiredWindow(void)
{
  if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    return WINDOW_OPEN;
  }
  if (Aht20ReadingAvailable() == 0U)
  {
    return greenhouse.window_state;
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
      ((Aht20ReadingAvailable() != 0U) &&
       (greenhouse.temperature_mc >= CRITICAL_TEMP_MC)) ? 1U : 0U;
  if (Aht20ReadingAvailable() != 0U)
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
  else if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    greenhouse.ventilation_requested = 1U;
  }

  if ((greenhouse.peripherals_ready == 0U) ||
      ((greenhouse.startup_state != STARTUP_READY) &&
       (greenhouse.startup_state != STARTUP_CONFIG_REQUIRED) &&
       (greenhouse.startup_state != STARTUP_WAITING_FOR_FIRST_READINGS)) ||
      ((greenhouse.startup_state == STARTUP_WAITING_FOR_FIRST_READINGS) &&
       (Aht20ReadingAvailable() == 0U) &&
       (greenhouse.aht20_state != SENSOR_ERROR)))
  {
    Pump_Set(0U);
    Fan_Set(0U);
    return;
  }

  required_window = RequiredWindow();
  if ((greenhouse.startup_state == STARTUP_WAITING_FOR_FIRST_READINGS) &&
      (required_window == WINDOW_CLOSED) &&
      (greenhouse.window_state != WINDOW_CLOSED))
  {
    /* Reserve the first close for the explicit CLOSING_WINDOW phase. */
    required_window = greenhouse.window_state;
  }
  if ((greenhouse.servo_moving == 0U) &&
      (required_window != WINDOW_UNKNOWN) &&
      (required_window != greenhouse.window_state) &&
      (Servo_Start(required_window, now) != 0U))
  {
    return;
  }
  if (greenhouse.servo_moving != 0U)
  {
    Fan_Set(0U);
    Pump_Set(0U);
  }
  else if ((greenhouse.critical_temperature != 0U) ||
           (greenhouse.aht20_state == SENSOR_ERROR) ||
           (greenhouse.ventilation_requested != 0U))
  {
    Watering_Abort();
    Fan_Set(1U);
  }
  else
  {
    Fan_Set(0U);
    Watering_Process(now);
  }
}

static uint8_t AllFirstReadingsValid(void)
{
  return ((((Aht20Selected() == 0U) ||
             (Aht20ReadingAvailable() != 0U)) != 0U) &&
           (((Bmp280Selected() == 0U) ||
             (greenhouse.bmp280_state == SENSOR_VALID)) != 0U) &&
           (((SoilSelected() == 0U) ||
             (greenhouse.soil_state == SENSOR_VALID)) != 0U) &&
           (((LightSelected() == 0U) ||
             (greenhouse.light_state == SENSOR_VALID)) != 0U) &&
           (greenhouse.oled_valid != 0U)) ? 1U : 0U;
}

static uint8_t AllSelectedAttemptsComplete(void)
{
  return ((((Aht20Selected() == 0U) ||
             (greenhouse.aht20_attempt_complete != 0U)) != 0U) &&
           (((Bmp280Selected() == 0U) ||
             (greenhouse.bmp280_attempt_complete != 0U)) != 0U) &&
           (((SoilSelected() == 0U) ||
             (greenhouse.soil_attempt_complete != 0U)) != 0U) &&
           (((LightSelected() == 0U) ||
             (greenhouse.light_attempt_complete != 0U)) != 0U)) ? 1U : 0U;
}

static void DisplayScreen(const char *line0, const char *line2,
                          const char *line4)
{
  SSD1306_Clear(&greenhouse.oled);
  SSD1306_WriteLine(&greenhouse.oled, 0U, line0);
  SSD1306_WriteLine(&greenhouse.oled, 2U, line2);
  SSD1306_WriteLine(&greenhouse.oled, 4U, line4);
}

static HAL_StatusTypeDef DisplayCommit(uint32_t now)
{
  if (SSD1306_Update(&greenhouse.oled) != HAL_OK)
  {
    greenhouse.oled_valid = 0U;
    greenhouse.oled.available = 0U;
    greenhouse.oled_retry_tick = now;
    return HAL_ERROR;
  }
  return HAL_OK;
}

static void DisplayStartup(uint32_t now)
{
  if (greenhouse.oled_valid == 0U)
  {
    return;
  }
  switch (greenhouse.startup_state)
  {
    case STARTUP_STARTING:
      DisplayScreen("SYSTEM STARTING", "", "");
      break;
    case STARTUP_INITIALIZING_SENSORS:
      DisplayScreen("READING SENSORS", "", "");
      break;
    case STARTUP_WAITING_FOR_FIRST_READINGS:
      DisplayScreen("READING SENSORS", "", "");
      break;
    case STARTUP_CLOSING_WINDOW:
      if (greenhouse.servo_start_failed != 0U)
      {
        DisplayScreen("SERVO ERROR", "WINDOW NOT CLOSED", "RETRYING");
      }
      else
      {
        DisplayScreen("CLOSING WINDOW", "", "");
      }
      break;
    case STARTUP_CONFIG_REQUIRED:
      DisplayScreen("CONFIG REQUIRED", "SERVO CAL MISSING", "");
      break;
    case STARTUP_READY:
      DisplayScreen("SYSTEM READY", "MONITORING ACTIVE", "");
      break;
    default:
      DisplayScreen("READING SENSORS", "", "");
      break;
  }
  (void)DisplayCommit(now);
}

static void Startup_Set(StartupState state, uint32_t now)
{
  greenhouse.startup_state = state;
  greenhouse.startup_tick = now;
  greenhouse.display_tick = now;
  DisplayStartup(now);
}

static void Startup_Process(uint32_t now)
{
  switch (greenhouse.startup_state)
  {
    case STARTUP_STARTING:
      if (Elapsed(now, greenhouse.startup_tick, STARTUP_MESSAGE_MS) != 0U)
      {
        Startup_Set(STARTUP_INITIALIZING_SENSORS, now);
      }
      break;
    case STARTUP_INITIALIZING_SENSORS:
      if (greenhouse.sensor_initialization_done == 0U)
      {
        Sensors_Initialize(now);
        greenhouse.sensor_initialization_done = 1U;
      }
      if (Elapsed(now, greenhouse.startup_tick, STARTUP_MESSAGE_MS) != 0U)
      {
        greenhouse.sensor_tick = now - SensorInterval();
        Startup_Set(STARTUP_WAITING_FOR_FIRST_READINGS, now);
      }
      break;
    case STARTUP_WAITING_FOR_FIRST_READINGS:
      if ((greenhouse.test_mode == GREENHOUSE_TEST_FULL) &&
          (ServoConfigurationValid() == 0U))
      {
        Startup_Set(STARTUP_CONFIG_REQUIRED, now);
      }
      else if ((greenhouse.peripherals_ready != 0U) &&
               (AllFirstReadingsValid() != 0U))
      {
        if (greenhouse.test_mode != GREENHOUSE_TEST_FULL)
        {
          greenhouse.ready_message_active = 1U;
          greenhouse.ready_tick = now;
          Startup_Set(STARTUP_READY, now);
        }
        else if (greenhouse.window_state == WINDOW_CLOSED)
        {
          greenhouse.ready_message_active = 1U;
          greenhouse.ready_tick = now;
          Startup_Set(STARTUP_READY, now);
        }
        else
        {
          Startup_Set(STARTUP_CLOSING_WINDOW, now);
          (void)Servo_Start(WINDOW_CLOSED, now);
        }
      }
      break;
    case STARTUP_CLOSING_WINDOW:
      if ((greenhouse.servo_moving == 0U) &&
          (greenhouse.window_state == WINDOW_CLOSED))
      {
        if ((greenhouse.peripherals_ready != 0U) &&
            (AllFirstReadingsValid() != 0U))
        {
          greenhouse.ready_message_active = 1U;
          greenhouse.ready_tick = now;
          Startup_Set(STARTUP_READY, now);
        }
        else
        {
          Startup_Set(STARTUP_WAITING_FOR_FIRST_READINGS, now);
        }
      }
      else if ((greenhouse.servo_moving == 0U) &&
               (Elapsed(now, greenhouse.servo_retry_tick,
                        RETRY_INTERVAL_MS) != 0U))
      {
        (void)Servo_Start(WINDOW_CLOSED, now);
        DisplayStartup(now);
      }
      break;
    default:
      break;
  }
}

static void FormatTemperature(char *line, size_t size)
{
  uint32_t value;

  if ((greenhouse.aht20_state == SENSOR_NOT_ATTEMPTED) ||
      (greenhouse.aht20_state == SENSOR_PENDING))
  {
    (void)snprintf(line, size, "TEMP:WAIT");
  }
  else if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    (void)snprintf(line, size, "TEMP:ERR");
  }
  else
  {
    value = (greenhouse.temperature_mc < 0) ?
            (uint32_t)(-(int64_t)greenhouse.temperature_mc) :
            (uint32_t)greenhouse.temperature_mc;
    (void)snprintf(line, size, "TEMP:%s%lu.%lu C",
                   (greenhouse.temperature_mc < 0) ? "-" : "",
                   (unsigned long)(value / 1000U),
                   (unsigned long)((value % 1000U) / 100U));
  }
}

static void FormatHumidity(char *line, size_t size)
{
  if ((greenhouse.aht20_state == SENSOR_NOT_ATTEMPTED) ||
      (greenhouse.aht20_state == SENSOR_PENDING))
  {
    (void)snprintf(line, size, "HUM:WAIT");
  }
  else if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    (void)snprintf(line, size, "HUM:ERR");
  }
  else
  {
    (void)snprintf(line, size, "HUM:%lu.%lu %%",
                   (unsigned long)(greenhouse.humidity_mpct / 1000U),
                   (unsigned long)((greenhouse.humidity_mpct % 1000U) / 100U));
  }
}

static void FormatPressure(char *line, size_t size)
{
  if ((greenhouse.bmp280_state == SENSOR_NOT_ATTEMPTED) ||
      (greenhouse.bmp280_state == SENSOR_PENDING))
  {
    (void)snprintf(line, size, "PRESS:WAIT");
  }
  else if (greenhouse.bmp280_state == SENSOR_ERROR)
  {
    (void)snprintf(line, size, "PRESS:ERR");
  }
  else
  {
    (void)snprintf(line, size, "PRESS:%lu HPA",
                   (unsigned long)(greenhouse.pressure_pa / 100U));
  }
}

static void FormatLight(char *line, size_t size)
{
  if ((greenhouse.light_state == SENSOR_NOT_ATTEMPTED) ||
      (greenhouse.light_state == SENSOR_PENDING))
  {
    (void)snprintf(line, size, "LIGHT:WAIT");
  }
  else if (greenhouse.light_state == SENSOR_ERROR)
  {
    (void)snprintf(line, size, "LIGHT:ERR");
  }
  else if (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                               greenhouse.config.light_bright_adc) != 0U)
  {
    (void)snprintf(line, size, "LIGHT:%d %%", (int)greenhouse.light_percent);
  }
  else
  {
    (void)snprintf(line, size, "LIGHT RAW:%u", (unsigned int)greenhouse.light_adc);
  }
}

static void FormatSoil(char *line, size_t size)
{
  if ((greenhouse.soil_state == SENSOR_NOT_ATTEMPTED) ||
      (greenhouse.soil_state == SENSOR_PENDING))
  {
    (void)snprintf(line, size, "SOIL:WAIT");
  }
  else if (greenhouse.soil_state == SENSOR_ERROR)
  {
    (void)snprintf(line, size, "SOIL:ERR");
  }
  else if (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                               greenhouse.config.soil_wet_adc) != 0U)
  {
    (void)snprintf(line, size, "SOIL:%d %%", (int)greenhouse.soil_percent);
  }
  else
  {
    (void)snprintf(line, size, "SOIL RAW:%u", (unsigned int)greenhouse.soil_adc);
  }
}

static void DisplayStatusLine(void)
{
  const char *status;

  if (greenhouse.critical_temperature != 0U)
  {
    status = "STATUS:HIGH TEMP";
  }
  else if (greenhouse.water_state == WATER_FAILED)
  {
    status = "STATUS:WATER FAILED";
  }
  else if (greenhouse.startup_state == STARTUP_CONFIG_REQUIRED)
  {
    status = "SERVO CAL MISSING";
  }
  else if (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                               greenhouse.config.soil_wet_adc) == 0U)
  {
    status = "SOIL CAL REQUIRED";
  }
  else if (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                               greenhouse.config.light_bright_adc) == 0U)
  {
    status = "LIGHT CAL REQUIRED";
  }
  else if (greenhouse.config.pump_run_time_ms == 0U)
  {
    status = "PUMP TIME MISSING";
  }
  else if (greenhouse.low_light_warning != 0U)
  {
    status = "STATUS:LOW LIGHT";
  }
  else if ((greenhouse.aht20_state == SENSOR_ERROR) ||
           (greenhouse.bmp280_state == SENSOR_ERROR) ||
           (greenhouse.soil_state == SENSOR_ERROR) ||
           (greenhouse.light_state == SENSOR_ERROR))
  {
    status = "STATUS:SENSOR ERROR";
  }
  else
  {
    status = "STATUS:NORMAL";
  }
  SSD1306_WriteLine(&greenhouse.oled, 6U, status);
}

static void DisplayNormalPage(void)
{
  char line[24];
  const char *window_text;

  SSD1306_Clear(&greenhouse.oled);
  if (greenhouse.display_page == 0U)
  {
    FormatTemperature(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 0U, line);
    FormatHumidity(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 2U, line);
    FormatPressure(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 4U, line);
    FormatLight(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 6U, line);
  }
  else
  {
    FormatSoil(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 0U, line);
    (void)snprintf(line, sizeof(line), "FAN:%s PUMP:%s",
                   (greenhouse.fan_on != 0U) ? "ON" : "OFF",
                   (greenhouse.pump_on != 0U) ? "ON" : "OFF");
    SSD1306_WriteLine(&greenhouse.oled, 2U, line);
    if (greenhouse.servo_moving != 0U)
    {
      window_text = (greenhouse.servo_target == WINDOW_OPEN) ? "OPENING" : "CLOSING";
    }
    else
    {
      window_text = (greenhouse.window_state == WINDOW_OPEN) ? "OPEN" :
                    ((greenhouse.window_state == WINDOW_CLOSED) ? "CLOSED" : "UNKNOWN");
    }
    (void)snprintf(line, sizeof(line), "WINDOW:%s", window_text);
    SSD1306_WriteLine(&greenhouse.oled, 4U, line);
    DisplayStatusLine();
  }
  greenhouse.display_page ^= 1U;
}

static uint8_t DisplayWarning(void)
{
  uint8_t offset;
  uint8_t warning;

  if (greenhouse.critical_temperature != 0U)
  {
    DisplayScreen("HIGH TEMP ALERT", "PUMP OFF", "VENTILATING");
    return 1U;
  }
  if (greenhouse.water_state == WATER_FAILED)
  {
    DisplayScreen("WATERING FAILED", "CHECK WATER/SOIL", "PUMP OFF");
    return 1U;
  }

  for (offset = 0U; offset < 11U; offset++)
  {
    warning = (uint8_t)((greenhouse.warning_index + offset) % 11U);
    if ((warning == 1U) && (greenhouse.aht20_state == SENSOR_ERROR))
    {
      DisplayScreen("AHT20 ERROR", "TEMP/HUM UNAVAILABLE", "SAFE VENTILATION");
    }
    else if ((warning == 2U) && (greenhouse.bmp280_state == SENSOR_ERROR))
    {
      DisplayScreen("BMP280 ERROR", "PRESSURE UNAVAILABLE", "CONTROL CONTINUES");
    }
    else if ((warning == 3U) && (greenhouse.soil_state == SENSOR_ERROR))
    {
      DisplayScreen("SOIL ADC ERROR", "PUMP DISABLED", "");
    }
    else if ((warning == 4U) && (greenhouse.light_state == SENSOR_ERROR))
    {
      DisplayScreen("LIGHT ADC ERROR", "LIGHT UNAVAILABLE", "");
    }
    else if ((warning == 6U) && (greenhouse.low_light_warning != 0U))
    {
      DisplayScreen("LOW LIGHT", "LIGHT BELOW 20%", "");
    }
    else if ((warning == 7U) &&
             (greenhouse.startup_state == STARTUP_CONFIG_REQUIRED))
    {
      DisplayScreen("CONFIG REQUIRED", "SERVO CAL MISSING", "");
    }
    else if ((warning == 8U) && (greenhouse.config.pump_run_time_ms == 0U))
    {
      DisplayScreen("PUMP TIME MISSING", "AUTO WATERING OFF", "");
    }
    else if ((warning == 9U) &&
             (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                                  greenhouse.config.soil_wet_adc) == 0U))
    {
      DisplayScreen("SOIL CAL REQUIRED", "RAW ADC DISPLAYED", "PUMP DISABLED");
    }
    else if ((warning == 10U) &&
             (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                                  greenhouse.config.light_bright_adc) == 0U))
    {
      DisplayScreen("LIGHT CAL REQUIRED", "RAW ADC DISPLAYED", "");
    }
    else
    {
      continue;
    }
    greenhouse.warning_index = (uint8_t)((warning + 1U) % 11U);
    return 1U;
  }
  return 0U;
}

static void Oled_Retry(uint32_t now)
{
  if ((greenhouse.oled_valid == 0U) &&
      (Elapsed(now, greenhouse.oled_retry_tick, RETRY_INTERVAL_MS) != 0U))
  {
    greenhouse.oled_retry_tick = now;
    if (SSD1306_Init(&greenhouse.oled, greenhouse.i2c) == HAL_OK)
    {
      greenhouse.oled_valid = 1U;
      greenhouse.display_tick = now - DisplayInterval();
      if (greenhouse.startup_state != STARTUP_READY)
      {
        DisplayStartup(now);
      }
    }
  }
}

static void Display_Process(uint32_t now)
{
  char line[24];

  if ((greenhouse.oled_valid == 0U) ||
      (Elapsed(now, greenhouse.display_tick, DisplayInterval()) == 0U))
  {
    return;
  }
  greenhouse.display_tick = now;
  if ((greenhouse.startup_state != STARTUP_READY) &&
      (greenhouse.startup_state != STARTUP_CONFIG_REQUIRED) &&
      ((greenhouse.startup_state != STARTUP_WAITING_FOR_FIRST_READINGS) ||
       (AllSelectedAttemptsComplete() == 0U)))
  {
    DisplayStartup(now);
    return;
  }
  if ((greenhouse.ready_message_active != 0U) &&
      (Elapsed(now, greenhouse.ready_tick, READY_MESSAGE_MS) == 0U))
  {
    DisplayScreen("SYSTEM READY", "MONITORING ACTIVE", "");
  }
  else
  {
    greenhouse.ready_message_active = 0U;
    if (greenhouse.test_mode == GREENHOUSE_TEST_LIGHT)
    {
      SSD1306_Clear(&greenhouse.oled);
      SSD1306_WriteLine(&greenhouse.oled, 0U, "LIGHT TEST");
      FormatLight(line, sizeof(line));
      SSD1306_WriteLine(&greenhouse.oled, 2U, line);
      if ((greenhouse.light_state == SENSOR_VALID) &&
          (AdcCalibrationValid(greenhouse.config.light_dark_adc,
                               greenhouse.config.light_bright_adc) == 0U))
      {
        SSD1306_WriteLine(&greenhouse.oled, 4U, "LIGHT CAL REQUIRED");
      }
    }
    else if (greenhouse.test_mode == GREENHOUSE_TEST_SOIL)
    {
      SSD1306_Clear(&greenhouse.oled);
      SSD1306_WriteLine(&greenhouse.oled, 0U, "SOIL TEST");
      FormatSoil(line, sizeof(line));
      SSD1306_WriteLine(&greenhouse.oled, 2U, line);
      if ((greenhouse.soil_state == SENSOR_VALID) &&
          (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                               greenhouse.config.soil_wet_adc) == 0U))
      {
        SSD1306_WriteLine(&greenhouse.oled, 4U, "SOIL CAL REQUIRED");
      }
    }
    else if (greenhouse.test_mode == GREENHOUSE_TEST_AHT20)
    {
      SSD1306_Clear(&greenhouse.oled);
      SSD1306_WriteLine(&greenhouse.oled, 0U, "AHT20 TEST");
      FormatTemperature(line, sizeof(line));
      SSD1306_WriteLine(&greenhouse.oled, 2U, line);
      FormatHumidity(line, sizeof(line));
      SSD1306_WriteLine(&greenhouse.oled, 4U, line);
    }
    else if (greenhouse.test_mode == GREENHOUSE_TEST_BMP280)
    {
      SSD1306_Clear(&greenhouse.oled);
      SSD1306_WriteLine(&greenhouse.oled, 0U, "BMP280 TEST");
      FormatPressure(line, sizeof(line));
      SSD1306_WriteLine(&greenhouse.oled, 2U, line);
    }
    else if (greenhouse.test_mode == GREENHOUSE_TEST_OLED)
    {
      DisplayScreen("OLED TEST", "128 X 64 I2C", "DISPLAY OK");
    }
    else if ((greenhouse.warning_turn == 0U) && (DisplayWarning() != 0U))
    {
      /* An alert may replace one interval; the next two are sensor pages. */
      greenhouse.warning_turn = 1U;
    }
    else
    {
      DisplayNormalPage();
      if (greenhouse.warning_turn == 1U)
      {
        greenhouse.warning_turn = 2U;
      }
      else
      {
        greenhouse.warning_turn = 0U;
      }
    }
  }
  (void)DisplayCommit(now);
}

static void TestMode_Process(uint32_t now)
{
  Fan_Set(0U);
  Pump_Set(0U);
  greenhouse.critical_temperature = 0U;
  if ((greenhouse.test_mode == GREENHOUSE_TEST_BUZZER) &&
      (Elapsed(now, greenhouse.test_tick, 2000U) != 0U))
  {
    greenhouse.test_tick = now;
    Buzzer_Request(1U, now);
  }
  else if ((greenhouse.test_mode == GREENHOUSE_TEST_SERVO) &&
           (greenhouse.servo_moving == 0U) &&
           (Elapsed(now, greenhouse.test_tick, 2000U) != 0U))
  {
    greenhouse.test_tick = now;
    (void)Servo_Start((greenhouse.window_state == WINDOW_OPEN) ?
                      WINDOW_CLOSED : WINDOW_OPEN, now);
  }
  else if (greenhouse.test_mode == GREENHOUSE_TEST_FAN)
  {
    if (Elapsed(now, greenhouse.test_tick, 2000U) != 0U)
    {
      greenhouse.test_tick = now;
      greenhouse.test_output_on ^= 1U;
    }
    Fan_Set(greenhouse.test_output_on);
  }
  else if (greenhouse.test_mode == GREENHOUSE_TEST_PUMP)
  {
    if (greenhouse.config.pump_run_time_ms == 0U)
    {
      greenhouse.test_output_on = 0U;
    }
    else if (Elapsed(now, greenhouse.test_tick,
                     (greenhouse.test_output_on != 0U) ?
                     greenhouse.config.pump_run_time_ms : 2000U) != 0U)
    {
      greenhouse.test_tick = now;
      greenhouse.test_output_on ^= 1U;
    }
    Pump_Set(greenhouse.test_output_on);
  }
}

HAL_StatusTypeDef Greenhouse_Init(ADC_HandleTypeDef *adc,
                                  I2C_HandleTypeDef *i2c,
                                  TIM_HandleTypeDef *pwm_timer,
                                  const GreenhouseConfig *config,
                                  GreenhouseTestMode test_mode)
{
  uint32_t now;
  uint8_t i2c_ready;
  uint8_t pwm_ready;

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
  greenhouse.startup_state = STARTUP_STARTING;
  greenhouse.window_state = WINDOW_UNKNOWN;
  greenhouse.aht20.i2c = i2c;
  greenhouse.bmp280.i2c = i2c;
  greenhouse.oled.i2c = i2c;

  Fan_Set(0U);
  Pump_Set(0U);
  Buzzer_Set(0U);
  (void)HAL_TIM_PWM_Stop(pwm_timer, TIM_CHANNEL_1);
  greenhouse.adc_ready =
      (HAL_ADCEx_Calibration_Start(adc) == HAL_OK) ? 1U : 0U;
  i2c_ready = (HAL_I2C_GetState(i2c) == HAL_I2C_STATE_READY) ? 1U : 0U;
  pwm_ready = (HAL_TIM_Base_GetState(pwm_timer) == HAL_TIM_STATE_READY) ? 1U : 0U;
  greenhouse.peripherals_ready =
      ((greenhouse.adc_ready != 0U) && (i2c_ready != 0U) &&
       (pwm_ready != 0U)) ? 1U : 0U;

  now = HAL_GetTick();
  greenhouse.sensor_tick = now;
  greenhouse.aht20_retry_tick = now;
  greenhouse.bmp280_retry_tick = now;
  greenhouse.oled_retry_tick = now;
  greenhouse.servo_retry_tick = now;
  greenhouse.display_tick = now;
  greenhouse.startup_tick = now;
  greenhouse.test_tick = now;
  if ((i2c_ready != 0U) &&
      (SSD1306_Init(&greenhouse.oled, i2c) == HAL_OK))
  {
    greenhouse.oled_valid = 1U;
    DisplayStartup(now);
  }
  return HAL_OK;
}

void Greenhouse_Process(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t acquisition_enabled;
  uint8_t sensor_alarm;

  Oled_Retry(now);
  Servo_Process(now);
  Watering_Timers(now);
  Sensors_ProcessAht20(now);
  acquisition_enabled =
      ((greenhouse.startup_state == STARTUP_WAITING_FOR_FIRST_READINGS) ||
       (greenhouse.startup_state == STARTUP_CLOSING_WINDOW) ||
       (greenhouse.startup_state == STARTUP_READY) ||
       (greenhouse.startup_state == STARTUP_CONFIG_REQUIRED)) ? 1U : 0U;
  if ((acquisition_enabled != 0U) &&
      (Elapsed(now, greenhouse.sensor_tick, SensorInterval()) != 0U))
  {
    Sensors_StartCycle(now);
  }
  Startup_Process(now);
  if (greenhouse.test_mode == GREENHOUSE_TEST_FULL)
  {
    LowLight_Process(now);
  }

  sensor_alarm =
      ((greenhouse.test_mode == GREENHOUSE_TEST_FULL) &&
       ((greenhouse.aht20_state == SENSOR_ERROR) ||
        (greenhouse.bmp280_state == SENSOR_ERROR) ||
        (greenhouse.soil_state == SENSOR_ERROR) ||
        (greenhouse.light_state == SENSOR_ERROR))) ? 1U : 0U;
  if ((sensor_alarm != 0U) && (greenhouse.previous_sensor_alarm == 0U))
  {
    greenhouse.ready_message_active = 0U;
    Buzzer_Request(3U, now);
  }
  greenhouse.previous_sensor_alarm = sensor_alarm;

  if (greenhouse.test_mode == GREENHOUSE_TEST_FULL)
  {
    FullControl_Process(now);
  }
  else if (greenhouse.startup_state == STARTUP_READY)
  {
    TestMode_Process(now);
  }
  Buzzer_Process(now);
  Display_Process(now);
}
