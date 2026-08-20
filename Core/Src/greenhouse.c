#include "greenhouse.h"

#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"

#include <string.h>

#define BUZZER_PORT GPIOA
#define BUZZER_PIN GPIO_PIN_8
#define PUMP_PORT GPIOA
#define PUMP_PIN GPIO_PIN_9
#define FAN_PORT GPIOA
#define FAN_PIN GPIO_PIN_10
#define I2C_RECOVERY_PORT GPIOB
#define I2C_RECOVERY_SCL_PIN GPIO_PIN_6
#define I2C_RECOVERY_SDA_PIN GPIO_PIN_7

/* Both fan and pump use active-high low-side N-MOSFET drivers. */
#define FAN_ON_LEVEL GPIO_PIN_SET
#define FAN_OFF_LEVEL GPIO_PIN_RESET
#define PUMP_ON_LEVEL GPIO_PIN_SET
#define PUMP_OFF_LEVEL GPIO_PIN_RESET
#define SENSOR_INTERVAL_MS 500U
#define TEST_SENSOR_INTERVAL_MS 500U
#define RETRY_INTERVAL_MS 2000U
#define AHT20_READ_DELAY_MS 85U
#define AHT20_TIMEOUT_MS 250U
#define DISPLAY_INTERVAL_MS 1000U
#define TEST_DISPLAY_INTERVAL_MS 500U
#define TEST_OUTPUT_INTERVAL_MS 1000U
#define OUTPUT_ARM_DELAY_MS 3000U
#define SOAK_TIME_MS 3000U
#define LOW_LIGHT_TIME_MS 3000U
#define LOW_LIGHT_BEEP_ON_MS 250U
#define LOW_LIGHT_BEEP_OFF_MS 180U
#define LOW_LIGHT_REPEAT_MS 5000U
#define LOW_LIGHT_BEEP_COUNT 2U
#define MAX_WATERING_ATTEMPTS 3U
#define ADC_TIMEOUT_MS 10U
#define ADC_SAMPLE_COUNT 8U
#define ADC_MAX_VALUE 4095U

#define FAN_ON_TEMP_MC 32000
#define FAN_OFF_TEMP_MC 31000
#define FAN_ON_HUMIDITY_MPCT 82000U
#define FAN_OFF_HUMIDITY_MPCT 80000U
#define WINDOW_OPEN_TEMP_MC 30000
#define WINDOW_CLOSE_TEMP_MC 29000
#define WINDOW_OPEN_HUMIDITY_MPCT 78000U
#define WINDOW_CLOSE_HUMIDITY_MPCT 75000U
#define CRITICAL_TEMP_MC 40000
#define PUMP_START_PERCENT 30
#define PUMP_CLEAR_PERCENT 35
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
  uint8_t enabled;
  uint8_t pending;
  uint32_t tick;
} OutputGate;

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
  WindowState window_state;
  WindowState servo_target;
  WaterState water_state;

  uint8_t aht20_initialized;
  uint8_t aht20_has_reading;
  uint8_t bmp280_initialized;
  uint8_t adc_ready;
  uint8_t oled_valid;
  OutputGate light_output;
  OutputGate soil_output;
  OutputGate climate_output;
  uint8_t fan_on;
  uint8_t pump_on;
  uint8_t servo_moving;
  uint8_t servo_start_failed;
  uint8_t ventilation_requested;
  uint8_t critical_temperature;
  uint8_t low_light_timer_active;
  uint8_t low_light_warning;
  uint8_t low_light_beeps_remaining;
  uint8_t display_page;
  uint8_t watering_attempts;
  uint8_t buzzer_on;
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
  uint32_t low_light_buzzer_tick;
  uint32_t low_light_repeat_tick;
  uint32_t display_tick;
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

static size_t TextEnd(const char *text, size_t size)
{
  size_t position = 0U;

  while ((position < size) && (text[position] != '\0'))
  {
    position++;
  }
  return position;
}

static void TextSet(char *text, size_t size, const char *value)
{
  size_t position = 0U;

  if (size == 0U)
  {
    return;
  }
  while ((position + 1U < size) && (value[position] != '\0'))
  {
    text[position] = value[position];
    position++;
  }
  text[position] = '\0';
}

static void TextAppend(char *text, size_t size, const char *value)
{
  size_t position = TextEnd(text, size);
  size_t source = 0U;

  if (position >= size)
  {
    return;
  }
  while ((position + 1U < size) && (value[source] != '\0'))
  {
    text[position++] = value[source++];
  }
  text[position] = '\0';
}

static void TextAppendUnsigned(char *text, size_t size, uint32_t value)
{
  char digits[10];
  uint8_t count = 0U;
  size_t position = TextEnd(text, size);

  do
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while ((value != 0U) && (count < sizeof(digits)));

  while ((count > 0U) && (position + 1U < size))
  {
    text[position++] = digits[--count];
  }
  if (position < size)
  {
    text[position] = '\0';
  }
}

static uint8_t IsSensorTest(void)
{
  return ((greenhouse.test_mode == GREENHOUSE_TEST_LIGHT) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_SOIL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_SOIL_PUMP) ||
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
          (greenhouse.test_mode == GREENHOUSE_TEST_SOIL) ||
          (greenhouse.test_mode == GREENHOUSE_TEST_SOIL_PUMP)) ? 1U : 0U;
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

static void OutputGate_Update(OutputGate *gate, uint8_t reading_available,
                              uint32_t now)
{
  if (gate->enabled != 0U)
  {
    return;
  }
  if (reading_available == 0U)
  {
    gate->pending = 0U;
    return;
  }
  if (gate->pending == 0U)
  {
    gate->pending = 1U;
    gate->tick = now;
    return;
  }
  if (Elapsed(now, gate->tick, OUTPUT_ARM_DELAY_MS) != 0U)
  {
    gate->enabled = 1U;
    gate->pending = 0U;
  }
}

static void OutputGates_Process(uint32_t now)
{
  /* Low-light already has its own confirmation timer.  Arm it as soon as
     the light ADC has one valid reading instead of adding a second delay. */
  if (greenhouse.light_state == SENSOR_VALID)
  {
    greenhouse.light_output.enabled = 1U;
    greenhouse.light_output.pending = 0U;
  }
  else if (greenhouse.light_output.enabled == 0U)
  {
    greenhouse.light_output.pending = 0U;
  }

  OutputGate_Update(&greenhouse.soil_output,
                    (greenhouse.soil_state == SENSOR_VALID) ? 1U : 0U,
                    now);
  OutputGate_Update(&greenhouse.climate_output,
                    Aht20ReadingAvailable(), now);
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
  uint32_t sum = 0U;
  uint16_t sample;
  uint8_t index;

  if ((greenhouse.adc_ready == 0U) || (value == NULL))
  {
    return HAL_ERROR;
  }

  channel_config.Channel = channel;
  channel_config.Rank = ADC_REGULAR_RANK_1;
  channel_config.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(greenhouse.adc, &channel_config) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Discard the first conversion after switching channels, then average. */
  for (index = 0U; index <= ADC_SAMPLE_COUNT; index++)
  {
    if (HAL_ADC_Start(greenhouse.adc) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (HAL_ADC_PollForConversion(greenhouse.adc, ADC_TIMEOUT_MS) != HAL_OK)
    {
      (void)HAL_ADC_Stop(greenhouse.adc);
      return HAL_ERROR;
    }
    sample = (uint16_t)HAL_ADC_GetValue(greenhouse.adc);
    (void)HAL_ADC_Stop(greenhouse.adc);
    if (index != 0U)
    {
      sum += sample;
    }
  }

  *value = (uint16_t)((sum + (ADC_SAMPLE_COUNT / 2U)) /
                      ADC_SAMPLE_COUNT);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_RecoverBus(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t pulse;

  (void)HAL_I2C_DeInit(greenhouse.i2c);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);
  gpio.Pin = I2C_RECOVERY_SCL_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_RECOVERY_PORT, &gpio);

  gpio.Pin = I2C_RECOVERY_SDA_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(I2C_RECOVERY_PORT, &gpio);
  HAL_Delay(1U);

  for (pulse = 0U; pulse < 9U; pulse++)
  {
    if (HAL_GPIO_ReadPin(I2C_RECOVERY_PORT,
                        I2C_RECOVERY_SDA_PIN) == GPIO_PIN_SET)
    {
      break;
    }
    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL_PIN,
                      GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL_PIN,
                      GPIO_PIN_SET);
    HAL_Delay(1U);
  }

  /* Generate a STOP before restoring the peripheral alternate function. */
  gpio.Pin = I2C_RECOVERY_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(I2C_RECOVERY_PORT, &gpio);
  HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SDA_PIN, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL_PIN, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SDA_PIN, GPIO_PIN_SET);
  HAL_Delay(1U);

  return HAL_I2C_Init(greenhouse.i2c);
}

static void Fan_Set(uint8_t on)
{
  /* Fan has the lowest actuator priority and never changes pump state. */
  if ((on != 0U) && ((greenhouse.pump_on != 0U) ||
                     (greenhouse.servo_moving != 0U)))
  {
    on = 0U;
  }
  HAL_GPIO_WritePin(FAN_PORT, FAN_PIN,
                    (on != 0U) ? FAN_ON_LEVEL : FAN_OFF_LEVEL);
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
                    (on != 0U) ? PUMP_ON_LEVEL : PUMP_OFF_LEVEL);
  greenhouse.pump_on = on;
}

static void Buzzer_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  greenhouse.buzzer_on = on;
}

static void LowLight_BuzzerStop(void)
{
  greenhouse.low_light_beeps_remaining = 0U;
  Buzzer_Set(0U);
}

static void LowLight_BuzzerStart(uint32_t now)
{
  greenhouse.low_light_beeps_remaining = LOW_LIGHT_BEEP_COUNT;
  greenhouse.low_light_buzzer_tick = now;
  Buzzer_Set(1U);
}

static void LowLight_BuzzerProcess(uint32_t now)
{
  if (greenhouse.low_light_warning == 0U)
  {
    LowLight_BuzzerStop();
    return;
  }

  if (greenhouse.low_light_beeps_remaining == 0U)
  {
    if (Elapsed(now, greenhouse.low_light_repeat_tick,
                LOW_LIGHT_REPEAT_MS) != 0U)
    {
      LowLight_BuzzerStart(now);
    }
    return;
  }

  if ((greenhouse.buzzer_on != 0U) &&
      (Elapsed(now, greenhouse.low_light_buzzer_tick,
               LOW_LIGHT_BEEP_ON_MS) != 0U))
  {
    Buzzer_Set(0U);
    greenhouse.low_light_beeps_remaining--;
    greenhouse.low_light_buzzer_tick = now;
    if (greenhouse.low_light_beeps_remaining == 0U)
    {
      greenhouse.low_light_repeat_tick = now;
    }
  }
  else if ((greenhouse.buzzer_on == 0U) &&
           (Elapsed(now, greenhouse.low_light_buzzer_tick,
                    LOW_LIGHT_BEEP_OFF_MS) != 0U))
  {
    Buzzer_Set(1U);
    greenhouse.low_light_buzzer_tick = now;
  }
}


static void Watering_Reset(void)
{
  Pump_Set(0U);
  greenhouse.water_state = WATER_IDLE;
  greenhouse.watering_attempts = 0U;
}


static uint8_t Servo_Start(WindowState target, uint32_t now)
{
  uint16_t pulse;

  /* A servo move never interrupts watering; it waits for the pump. */
  if ((greenhouse.servo_moving != 0U) ||
      (greenhouse.pump_on != 0U) ||
      (ServoConfigurationValid() == 0U))
  {
    return 0U;
  }
  if ((greenhouse.servo_start_failed != 0U) &&
      (Elapsed(now, greenhouse.servo_retry_tick,
               RETRY_INTERVAL_MS) == 0U))
  {
    return 0U;
  }
  Fan_Set(0U);
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
  /* Watering gets priority over the fan, but never over a moving servo. */
  if ((greenhouse.soil_state != SENSOR_VALID) ||
      (greenhouse.soil_percent >= PUMP_START_PERCENT) ||
      (greenhouse.config.pump_run_time_ms == 0U) ||
      (greenhouse.servo_moving != 0U))
  {
    return;
  }

  Fan_Set(0U);
  greenhouse.watering_attempts++;
  greenhouse.water_state = WATER_PUMPING;
  greenhouse.water_tick = now;
  Pump_Set(1U);
  if (greenhouse.pump_on == 0U)
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

  /* A failed cycle rearms only after a clearly wet reading. */
  if (greenhouse.water_state == WATER_FAILED)
  {
    Pump_Set(0U);
    if (greenhouse.soil_percent >= PUMP_CLEAR_PERCENT)
    {
      Watering_Reset();
    }
    return;
  }

  if (greenhouse.soil_percent >= PUMP_CLEAR_PERCENT)
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
    else if (greenhouse.soil_percent >= PUMP_START_PERCENT)
    {
      Watering_Reset();
    }
    else if (greenhouse.watering_attempts >= MAX_WATERING_ATTEMPTS)
    {
      Pump_Set(0U);
      greenhouse.water_state = WATER_FAILED;
      greenhouse.water_tick = now;
    }
    else
    {
      Watering_Start(now);
    }
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
      }
    }
    if (greenhouse.bmp280_initialized != 0U)
    {
      if (BMP280_Read(&greenhouse.bmp280, &bmp_temperature,
                      &greenhouse.pressure_pa) == HAL_OK)
      {
        greenhouse.bmp280_state = SENSOR_VALID;
      }
      else
      {
        greenhouse.bmp280_state = SENSOR_ERROR;
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
  }
  else if ((status == AHT20_STATUS_ERROR) ||
           (Elapsed(now, greenhouse.aht20_tick, AHT20_TIMEOUT_MS) != 0U))
  {
    greenhouse.aht20_state = SENSOR_ERROR;
    greenhouse.aht20_has_reading = 0U;
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
    LowLight_BuzzerStop();
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
      LowLight_BuzzerStart(now);
    }
  }
  else if (greenhouse.light_percent >= LOW_LIGHT_CLEAR_PERCENT)
  {
    greenhouse.low_light_timer_active = 0U;
    greenhouse.low_light_warning = 0U;
    LowLight_BuzzerStop();
  }
  else if (greenhouse.low_light_warning == 0U)
  {
    /* The initial delay must be continuously below the trigger threshold. */
    greenhouse.low_light_timer_active = 0U;
  }

  LowLight_BuzzerProcess(now);
}

static WindowState RequiredWindow(void)
{
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

static void Ventilation_Process(uint32_t now)
{
  WindowState required = RequiredWindow();

  if ((ServoConfigurationValid() != 0U) &&
      (required != WINDOW_UNKNOWN) &&
      (required != greenhouse.window_state) &&
      (greenhouse.servo_moving == 0U))
  {
    Fan_Set(0U);
    if (Servo_Start(required, now) != 0U)
    {
      return;
    }
  }

  if ((greenhouse.pump_on != 0U) || (greenhouse.servo_moving != 0U))
  {
    Fan_Set(0U);
    return;
  }

  if (Aht20ReadingAvailable() == 0U)
  {
    greenhouse.ventilation_requested = 0U;
    Fan_Set(0U);
    return;
  }

  if (greenhouse.ventilation_requested != 0U)
  {
    if ((greenhouse.temperature_mc <= FAN_OFF_TEMP_MC) &&
        (greenhouse.humidity_mpct <= FAN_OFF_HUMIDITY_MPCT))
    {
      greenhouse.ventilation_requested = 0U;
    }
  }
  else if ((greenhouse.temperature_mc >= FAN_ON_TEMP_MC) ||
           (greenhouse.humidity_mpct >= FAN_ON_HUMIDITY_MPCT))
  {
    greenhouse.ventilation_requested = 1U;
  }

  Fan_Set(greenhouse.ventilation_requested);
}


static void FullControl_Process(uint32_t now)
{
  greenhouse.critical_temperature =
      ((greenhouse.climate_output.enabled != 0U) &&
       (Aht20ReadingAvailable() != 0U) &&
       (greenhouse.temperature_mc >= CRITICAL_TEMP_MC)) ? 1U : 0U;

  if ((greenhouse.soil_output.enabled != 0U) &&
      (greenhouse.soil_state == SENSOR_VALID) &&
      (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                           greenhouse.config.soil_wet_adc) != 0U) &&
      (greenhouse.config.pump_run_time_ms != 0U))
  {
    Watering_Process(now);
  }
  else
  {
    Watering_Reset();
  }

  if (greenhouse.pump_on != 0U)
  {
    Fan_Set(0U);
    return;
  }

  if (greenhouse.climate_output.enabled != 0U)
  {
    Ventilation_Process(now);
  }
  else
  {
    greenhouse.ventilation_requested = 0U;
    Fan_Set(0U);
  }
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

static void FormatTemperature(char *line, size_t size)
{
  uint32_t value;

  if ((greenhouse.aht20_state == SENSOR_NOT_ATTEMPTED) ||
      ((greenhouse.aht20_state == SENSOR_PENDING) &&
       (greenhouse.aht20_has_reading == 0U)))
  {
    TextSet(line, size, "TEMP:WAIT");
  }
  else if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    TextSet(line, size, "TEMP:ERR");
  }
  else
  {
    value = (greenhouse.temperature_mc < 0) ?
            (uint32_t)(-(int64_t)greenhouse.temperature_mc) :
            (uint32_t)greenhouse.temperature_mc;
    TextSet(line, size,
            (greenhouse.temperature_mc < 0) ? "TEMP:-" : "TEMP:");
    TextAppendUnsigned(line, size, value / 1000U);
    TextAppend(line, size, ".");
    TextAppendUnsigned(line, size, (value % 1000U) / 100U);
    TextAppend(line, size, " C");
  }
}

static void FormatHumidity(char *line, size_t size)
{
  if ((greenhouse.aht20_state == SENSOR_NOT_ATTEMPTED) ||
      ((greenhouse.aht20_state == SENSOR_PENDING) &&
       (greenhouse.aht20_has_reading == 0U)))
  {
    TextSet(line, size, "HUM:WAIT");
  }
  else if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    TextSet(line, size, "HUM:ERR");
  }
  else
  {
    TextSet(line, size, "HUM:");
    TextAppendUnsigned(line, size, greenhouse.humidity_mpct / 1000U);
    TextAppend(line, size, ".");
    TextAppendUnsigned(line, size,
                       (greenhouse.humidity_mpct % 1000U) / 100U);
    TextAppend(line, size, " %");
  }
}

static void FormatPressure(char *line, size_t size)
{
  if ((greenhouse.bmp280_state == SENSOR_NOT_ATTEMPTED) ||
      (greenhouse.bmp280_state == SENSOR_PENDING))
  {
    TextSet(line, size, "PRESS:WAIT");
  }
  else if (greenhouse.bmp280_state == SENSOR_ERROR)
  {
    TextSet(line, size, "PRESS:ERR");
  }
  else
  {
    TextSet(line, size, "PRESS:");
    TextAppendUnsigned(line, size, greenhouse.pressure_pa / 100U);
    TextAppend(line, size, " HPA");
  }
}

static void FormatAnalog(char *line, size_t size, const char *label,
                         SensorState state, uint8_t calibrated,
                         uint16_t raw, int16_t percent)
{
  TextSet(line, size, label);
  if ((state == SENSOR_NOT_ATTEMPTED) || (state == SENSOR_PENDING))
  {
    TextAppend(line, size, "WAIT");
  }
  else if (state == SENSOR_ERROR)
  {
    TextAppend(line, size, "ERR");
  }
  else if (calibrated != 0U)
  {
    TextAppendUnsigned(line, size, (uint32_t)percent);
    TextAppend(line, size, " % R:");
    TextAppendUnsigned(line, size, raw);
  }
  else
  {
    TextAppend(line, size, "RAW ");
    TextAppendUnsigned(line, size, raw);
  }
}

static void FormatLight(char *line, size_t size)
{
  FormatAnalog(line, size, "LIGHT:", greenhouse.light_state,
               AdcCalibrationValid(greenhouse.config.light_dark_adc,
                                   greenhouse.config.light_bright_adc),
               greenhouse.light_adc, greenhouse.light_percent);
}

static void FormatSoil(char *line, size_t size)
{
  FormatAnalog(line, size, "SOIL:", greenhouse.soil_state,
               AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                                   greenhouse.config.soil_wet_adc),
               greenhouse.soil_adc, greenhouse.soil_percent);
}

static void DisplayStatusLine(void)
{
  const char *status;

  if ((greenhouse.light_output.pending != 0U) ||
      (greenhouse.soil_output.pending != 0U) ||
      (greenhouse.climate_output.pending != 0U))
  {
    status = "OUTPUTS ARMING";
  }
  else if ((greenhouse.light_output.enabled == 0U) &&
           (greenhouse.soil_output.enabled == 0U) &&
           (greenhouse.climate_output.enabled == 0U))
  {
    status = "READING SENSORS";
  }
  else if (greenhouse.critical_temperature != 0U)
  {
    status = "HIGH TEMP ALERT";
  }
  else if (greenhouse.water_state == WATER_FAILED)
  {
    status = "CHECK WATER/SOIL";
  }
  else if (greenhouse.aht20_state == SENSOR_ERROR)
  {
    status = "AHT20 ERROR";
  }
  else if (greenhouse.bmp280_state == SENSOR_ERROR)
  {
    status = "BMP280 ERROR";
  }
  else if (greenhouse.soil_state == SENSOR_ERROR)
  {
    status = "SOIL ADC ERROR";
  }
  else if (greenhouse.light_state == SENSOR_ERROR)
  {
    status = "LIGHT ADC ERROR";
  }
  else if (greenhouse.low_light_warning != 0U)
  {
    status = "LOW LIGHT";
  }
  else if (ServoConfigurationValid() == 0U)
  {
    status = "WINDOW CAL MISSING";
  }
  else
  {
    status = "MONITORING";
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
    DisplayStatusLine();
  }
  else
  {
    FormatSoil(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 0U, line);
    FormatLight(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 2U, line);
    TextSet(line, sizeof(line), "FAN:");
    TextAppend(line, sizeof(line),
               (greenhouse.fan_on != 0U) ? "ON" : "OFF");
    TextAppend(line, sizeof(line), " PUMP:");
    if ((greenhouse.config.pump_run_time_ms == 0U) ||
        (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                             greenhouse.config.soil_wet_adc) == 0U))
    {
      TextAppend(line, sizeof(line), "DIS");
    }
    else
    {
      TextAppend(line, sizeof(line),
                 (greenhouse.pump_on != 0U) ? "ON" : "OFF");
    }
    SSD1306_WriteLine(&greenhouse.oled, 4U, line);
    if (ServoConfigurationValid() == 0U)
    {
      window_text = "CAL MISSING";
    }
    else if (greenhouse.servo_start_failed != 0U)
    {
      window_text = "SERVO ERROR";
    }
    else if (greenhouse.servo_moving != 0U)
    {
      window_text = (greenhouse.servo_target == WINDOW_OPEN) ? "OPENING" : "CLOSING";
    }
    else
    {
      window_text = (greenhouse.window_state == WINDOW_OPEN) ? "OPEN" :
                    ((greenhouse.window_state == WINDOW_CLOSED) ? "CLOSED" : "UNKNOWN");
    }
    TextSet(line, sizeof(line), "WINDOW:");
    TextAppend(line, sizeof(line), window_text);
    SSD1306_WriteLine(&greenhouse.oled, 6U, line);
  }
  greenhouse.display_page ^= 1U;
}

static void Oled_Retry(uint32_t now)
{
  if ((greenhouse.oled_valid == 0U) &&
      (Elapsed(now, greenhouse.oled_retry_tick, RETRY_INTERVAL_MS) != 0U))
  {
    greenhouse.oled_retry_tick = now;
    if ((I2C_RecoverBus() == HAL_OK) &&
        (SSD1306_Init(&greenhouse.oled, greenhouse.i2c) == HAL_OK))
    {
      greenhouse.oled_valid = 1U;
      greenhouse.display_tick = now - DisplayInterval();
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
  if (greenhouse.test_mode == GREENHOUSE_TEST_LIGHT)
  {
    SSD1306_Clear(&greenhouse.oled);
    SSD1306_WriteLine(&greenhouse.oled, 0U, "LIGHT TEST");
    FormatLight(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 2U, line);
  }
  else if (greenhouse.test_mode == GREENHOUSE_TEST_SOIL)
  {
    SSD1306_Clear(&greenhouse.oled);
    SSD1306_WriteLine(&greenhouse.oled, 0U, "SOIL TEST");
    FormatSoil(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 2U, line);
  }
  else if (greenhouse.test_mode == GREENHOUSE_TEST_SOIL_PUMP)
  {
    SSD1306_Clear(&greenhouse.oled);
    SSD1306_WriteLine(&greenhouse.oled, 0U, "SOIL + PUMP TEST");
    FormatSoil(line, sizeof(line));
    SSD1306_WriteLine(&greenhouse.oled, 2U, line);

    TextSet(line, sizeof(line), "PUMP:");
    TextAppend(line, sizeof(line),
               (greenhouse.pump_on != 0U) ? "ON" : "OFF");
    TextAppend(line, sizeof(line), " TRY:");
    TextAppendUnsigned(line, sizeof(line), greenhouse.watering_attempts);
    SSD1306_WriteLine(&greenhouse.oled, 4U, line);

    if ((greenhouse.config.pump_run_time_ms == 0U) ||
        (AdcCalibrationValid(greenhouse.config.soil_dry_adc,
                             greenhouse.config.soil_wet_adc) == 0U))
    {
      TextSet(line, sizeof(line), "PUMP TEST DISABLED");
    }
    else if (greenhouse.soil_state != SENSOR_VALID)
    {
      TextSet(line, sizeof(line), "WAITING FOR SOIL");
    }
    else if (greenhouse.water_state == WATER_PUMPING)
    {
      TextSet(line, sizeof(line), "WATERING");
    }
    else if (greenhouse.water_state == WATER_SOAKING)
    {
      TextSet(line, sizeof(line), "SOAKING / RECHECK");
    }
    else if (greenhouse.water_state == WATER_FAILED)
    {
      TextSet(line, sizeof(line), "MAX ATTEMPTS");
    }
    else if (greenhouse.soil_percent < PUMP_START_PERCENT)
    {
      TextSet(line, sizeof(line), "DRY: STARTING");
    }
    else
    {
      TextSet(line, sizeof(line), "MOIST: PUMP OFF");
    }
    SSD1306_WriteLine(&greenhouse.oled, 6U, line);
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
  else
  {
    DisplayNormalPage();
  }
  (void)DisplayCommit(now);
}

static void TestMode_Process(uint32_t now)
{
  Fan_Set(0U);
  greenhouse.critical_temperature = 0U;

  if (greenhouse.test_mode == GREENHOUSE_TEST_SOIL_PUMP)
  {
    Buzzer_Set(0U);
    Watering_Process(now);
    return;
  }

  Pump_Set(0U);
  if ((greenhouse.test_mode == GREENHOUSE_TEST_BUZZER) &&
      (Elapsed(now, greenhouse.test_tick, TEST_OUTPUT_INTERVAL_MS) != 0U))
  {
    greenhouse.test_tick = now;
    greenhouse.test_output_on ^= 1U;
    Buzzer_Set(greenhouse.test_output_on);
  }
  else if ((greenhouse.test_mode == GREENHOUSE_TEST_SERVO) &&
           (greenhouse.servo_moving == 0U) &&
           (Elapsed(now, greenhouse.test_tick, TEST_OUTPUT_INTERVAL_MS) != 0U))
  {
    greenhouse.test_tick = now;
    (void)Servo_Start((greenhouse.window_state == WINDOW_OPEN) ?
                      WINDOW_CLOSED : WINDOW_OPEN, now);
  }
  else if (greenhouse.test_mode == GREENHOUSE_TEST_FAN)
  {
    if (Elapsed(now, greenhouse.test_tick, TEST_OUTPUT_INTERVAL_MS) != 0U)
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
                     greenhouse.config.pump_run_time_ms : TEST_OUTPUT_INTERVAL_MS) != 0U)
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
  greenhouse.aht20.i2c = i2c;
  greenhouse.bmp280.i2c = i2c;
  greenhouse.oled.i2c = i2c;

  Fan_Set(0U);
  Pump_Set(0U);
  Buzzer_Set(0U);
  (void)HAL_TIM_PWM_Stop(pwm_timer, TIM_CHANNEL_1);
  greenhouse.adc_ready =
      (HAL_ADCEx_Calibration_Start(adc) == HAL_OK) ? 1U : 0U;

  now = HAL_GetTick();
  Sensors_Initialize(now);
  /* Take ADC/BMP readings and start the first AHT20 conversion before any
     automatic actuator is allowed to run. */
  Sensors_StartCycle(now);
  greenhouse.sensor_tick = now;
  greenhouse.aht20_retry_tick = now;
  greenhouse.bmp280_retry_tick = now;
  greenhouse.oled_retry_tick = now;
  greenhouse.servo_retry_tick = now;
  greenhouse.display_tick = now - DisplayInterval();
  greenhouse.test_tick = now;
  if (SSD1306_Init(&greenhouse.oled, i2c) == HAL_OK)
  {
    greenhouse.oled_valid = 1U;
  }
  /* Full mode now controls the window automatically; test mode cycles it. */
  return HAL_OK;
}

void Greenhouse_Process(void)
{
  uint32_t now = HAL_GetTick();

  /* Only actuator timers are conditional.  Sensor, buzzer, and OLED work
     continues every loop regardless of which actuator is active. */
  if (greenhouse.test_mode == GREENHOUSE_TEST_FULL)
  {
    if (greenhouse.climate_output.enabled != 0U)
    {
      Servo_Process(now);
    }
    if (greenhouse.soil_output.enabled != 0U)
    {
      Watering_Timers(now);
    }
  }
  else
  {
    Servo_Process(now);
    Watering_Timers(now);
  }

  Sensors_ProcessAht20(now);
  if (Elapsed(now, greenhouse.sensor_tick, SensorInterval()) != 0U)
  {
    Sensors_StartCycle(now);
  }
  Oled_Retry(now);

  if (greenhouse.test_mode == GREENHOUSE_TEST_FULL)
  {
    OutputGates_Process(now);
    if (greenhouse.light_output.enabled != 0U)
    {
      LowLight_Process(now);
    }
    else
    {
      greenhouse.low_light_timer_active = 0U;
      greenhouse.low_light_warning = 0U;
      LowLight_BuzzerStop();
    }
    FullControl_Process(now);
  }
  else
  {
    TestMode_Process(now);
  }
  Display_Process(now);
}
