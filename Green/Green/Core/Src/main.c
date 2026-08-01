/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "fonts.h"
	#include "ssd1306.h"
	#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===================== TEMT6000 Light Sensor ===================== */
uint32_t lightADC = 0;
uint32_t lightmV = 0;
float lightVolt = 0.0f;

uint32_t TEMT6000_Read(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return adcValue;
}

/* ===================== BMP280 Sensor (Pressure + Temperature) ===================== */
#define BMP280_I2C       hi2c1
#define BMP280_ADDR_76   0xEC  // 0x76 << 1 (SDO → GND)
#define BMP280_ADDR_77   0xEE  // 0x77 << 1 (SDO → VCC)
uint16_t BMP280_ADDRESS  = 0xEC;  // Will be auto-detected at init

/* BMP280 Register addresses */
#define BMP280_ID_REG         0xD0
#define BMP280_RESET_REG      0xE0
#define BMP280_CTRL_MEAS_REG  0xF4
#define BMP280_CONFIG_REG     0xF5
#define BMP280_PRESS_MSB_REG  0xF7

/* Oversampling settings */
#define OSRS_OFF   0x00
#define OSRS_1     0x01
#define OSRS_2     0x02
#define OSRS_4     0x03
#define OSRS_8     0x04
#define OSRS_16    0x05

/* BMP280 Mode */
#define MODE_SLEEP   0x00
#define MODE_FORCED  0x01
#define MODE_NORMAL  0x03

/* BMP280 Standby time */
#define T_SB_0p5   0x00
#define T_SB_62p5  0x01
#define T_SB_125   0x02
#define T_SB_250   0x03
#define T_SB_500   0x04
#define T_SB_1000  0x05

/* BMP280 IIR Filter */
#define IIR_OFF  0x00
#define IIR_2    0x01
#define IIR_4    0x02
#define IIR_8    0x03
#define IIR_16   0x04

/* BMP280 Calibration coefficients (temperature + pressure only) */
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

int32_t t_fine;
int32_t tRaw, pRaw;
uint8_t bmpChipID;
char strCopy[20];

/* --- Read BMP280 trimming/calibration parameters --- */
static int BMP280_TrimRead(void)
{
	uint8_t trimdata[26];
	// Read temperature + pressure calibration (0x88 to 0xA1, 26 bytes)
	if (HAL_I2C_Mem_Read(&BMP280_I2C, BMP280_ADDRESS, 0x88, 1, trimdata, 26, HAL_MAX_DELAY) != HAL_OK)
		return 1;

	dig_T1 = (uint16_t)(trimdata[1] << 8 | trimdata[0]);
	dig_T2 = (int16_t)(trimdata[3] << 8 | trimdata[2]);
	dig_T3 = (int16_t)(trimdata[5] << 8 | trimdata[4]);

	dig_P1 = (uint16_t)(trimdata[7] << 8 | trimdata[6]);
	dig_P2 = (int16_t)(trimdata[9] << 8 | trimdata[8]);
	dig_P3 = (int16_t)(trimdata[11] << 8 | trimdata[10]);
	dig_P4 = (int16_t)(trimdata[13] << 8 | trimdata[12]);
	dig_P5 = (int16_t)(trimdata[15] << 8 | trimdata[14]);
	dig_P6 = (int16_t)(trimdata[17] << 8 | trimdata[16]);
	dig_P7 = (int16_t)(trimdata[19] << 8 | trimdata[18]);
	dig_P8 = (int16_t)(trimdata[21] << 8 | trimdata[20]);
	dig_P9 = (int16_t)(trimdata[23] << 8 | trimdata[22]);

	return 0;
}

/* --- Read raw sensor data (temperature + pressure, no humidity) --- */
static int BMP280_ReadRaw(void)
{
	uint8_t RawData[6];

	// Read chip ID once for diagnostics (displayed on OLED)
	HAL_I2C_Mem_Read(&BMP280_I2C, BMP280_ADDRESS, BMP280_ID_REG, 1, &bmpChipID, 1, 1000);

	// Read pressure + temperature raw data (no chip ID gate — if I2C works, read the data)
	if (HAL_I2C_Mem_Read(&BMP280_I2C, BMP280_ADDRESS, BMP280_PRESS_MSB_REG, 1, RawData, 6, HAL_MAX_DELAY) != HAL_OK)
		return 1;

	pRaw = (RawData[0]<<12)|(RawData[1]<<4)|(RawData[2]>>4);
	tRaw = (RawData[3]<<12)|(RawData[4]<<4)|(RawData[5]>>4);
	return 0;
}

/* --- Temperature compensation (identical to BME280 datasheet) --- */
static int32_t BMP280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

/* --- Pressure compensation (64-bit, identical to BME280 datasheet) --- */
static uint32_t BMP280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0) return 0;
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

/* --- Measure BMP280 (temperature + pressure only) --- */
void BMP280_Measure(float *temperature, float *pressure)
{
	if (BMP280_ReadRaw() == 0)
	{
		// Always run temperature compensation first (sets t_fine for pressure)
		*temperature = (BMP280_compensate_T_int32(tRaw)) / 100.0f;

		// Run pressure compensation
		uint32_t pComp = BMP280_compensate_P_int64(pRaw);
		*pressure = (float)pComp / 256.0f;
	}
	else
	{
		*temperature = 0;
		*pressure    = 0;
	}
}

/* --- Configure BMP280 (no humidity register unlike BME280) --- */
int BMP280_Config(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	uint8_t datatowrite = 0;

	// Auto-detect BMP280 I2C address (try 0x76 first, then 0x77)
	if (HAL_I2C_IsDeviceReady(&BMP280_I2C, BMP280_ADDR_76, 3, 100) == HAL_OK)
		BMP280_ADDRESS = BMP280_ADDR_76;
	else if (HAL_I2C_IsDeviceReady(&BMP280_I2C, BMP280_ADDR_77, 3, 100) == HAL_OK)
		BMP280_ADDRESS = BMP280_ADDR_77;
	else
		return 9;  // BMP280 not found at either address

	// Reset first, then read calibration from clean state
	datatowrite = 0xB6;
	if (HAL_I2C_Mem_Write(&BMP280_I2C, BMP280_ADDRESS, BMP280_RESET_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
		return 2;
	HAL_Delay(100);

	// Read calibration data after reset
	if (BMP280_TrimRead() != 0) return 1;

	// Standby time and IIR filter (must be written in sleep mode)
	datatowrite = (t_sb << 5) | (filter << 2);
	if (HAL_I2C_Mem_Write(&BMP280_I2C, BMP280_ADDRESS, BMP280_CONFIG_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
		return 3;
	HAL_Delay(10);

	// Temperature + Pressure oversampling and mode (this starts measurements)
	datatowrite = (osrs_t << 5) | (osrs_p << 2) | mode;
	if (HAL_I2C_Mem_Write(&BMP280_I2C, BMP280_ADDRESS, BMP280_CTRL_MEAS_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
		return 5;
	HAL_Delay(100);  // Wait for first measurement cycle

	return 0;
}

/* ===================== AHT20 Sensor (Humidity + Temperature) ===================== */
#define AHT20_I2C       hi2c1
#define AHT20_ADDRESS   0x70  // 0x38 << 1

/* AHT20 Commands */
#define AHT20_CMD_INIT      0xBE
#define AHT20_CMD_TRIGGER   0xAC
#define AHT20_CMD_SOFTRESET 0xBA

/* --- Initialize AHT20 --- */
int AHT20_Init(void)
{
	HAL_Delay(40);  // Wait 40ms after power-on

	// Read status byte
	uint8_t status = 0;
	if (HAL_I2C_Master_Receive(&AHT20_I2C, AHT20_ADDRESS, &status, 1, 1000) != HAL_OK)
		return 1;

	// Check if calibrated (bit 3)
	if (!(status & 0x08))
	{
		// Send calibration command: [0xBE, 0x08, 0x00]
		uint8_t cmd[3] = {AHT20_CMD_INIT, 0x08, 0x00};
		if (HAL_I2C_Master_Transmit(&AHT20_I2C, AHT20_ADDRESS, cmd, 3, 1000) != HAL_OK)
			return 2;
		HAL_Delay(10);
	}

	return 0;
}

/* --- Read AHT20 humidity and temperature --- */
int AHT20_Read(float *humidity, float *temperature)
{
	// Send trigger measurement command: [0xAC, 0x33, 0x00]
	uint8_t cmd[3] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
	if (HAL_I2C_Master_Transmit(&AHT20_I2C, AHT20_ADDRESS, cmd, 3, 1000) != HAL_OK)
		return 1;

	HAL_Delay(80);  // Wait for measurement to complete

	// Read 7 bytes: [status, hum[19:12], hum[11:4], hum[3:0]|temp[19:16], temp[15:8], temp[7:0], CRC]
	uint8_t data[7];
	if (HAL_I2C_Master_Receive(&AHT20_I2C, AHT20_ADDRESS, data, 7, 1000) != HAL_OK)
		return 2;

	// Check if measurement is complete (bit 7 of status should be 0)
	if (data[0] & 0x80)
		return 3;  // Still busy

	// Extract raw humidity (20 bits)
	uint32_t rawH = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);

	// Extract raw temperature (20 bits)
	uint32_t rawT = (((uint32_t)(data[3] & 0x0F)) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];

	// Convert to physical values
	*humidity    = ((float)rawH / 1048576.0f) * 100.0f;
	*temperature = ((float)rawT / 1048576.0f) * 200.0f - 50.0f;

	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();

  /* Start PWM for Servo on TIM2 CH2 (PA1) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* Sensor variables */
  float Temperature = 0, Pressure = 0, Humidity = 0;

  /* Servo motor variables */
  #define SERVO_CLOSED  500   // 0.5ms pulse = 0 degrees (window closed)
  #define SERVO_OPEN    2500  // 2.5ms pulse = 180 degrees (window open)
  #define TEMP_THRESHOLD 25.0f
  uint16_t currentServoPos = SERVO_CLOSED;
  uint8_t windowOpen = 0;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CLOSED); // Start closed

  /* Configure BMP280: Temp x2, Pres x16, Normal mode, 0.5ms standby, IIR 16 */
  int bmpStatus = BMP280_Config(OSRS_2, OSRS_16, MODE_NORMAL, T_SB_0p5, IIR_16);
  if (bmpStatus != 0)
  {
	  SSD1306_GotoXY(0, 0);
	  SSD1306_Puts("BMP280 ERROR", &Font_11x18, 1);
	  SSD1306_UpdateScreen();
	  HAL_Delay(2000);  // Show error for 2 seconds so user can see it
  }

  /* Initialize AHT20 humidity sensor */
  if (AHT20_Init() != 0)
  {
	  SSD1306_GotoXY(0, 0);
	  SSD1306_Puts("AHT20 ERROR", &Font_11x18, 1);
	  SSD1306_UpdateScreen();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* ---- Read TEMT6000 Light Sensor ---- */
	  lightADC = TEMT6000_Read();
	  lightmV = (lightADC * 3300) / 4095;
	  lightVolt = (float)lightmV / 1000.0f;

	  /* ---- Read BMP280 Sensor (Temperature + Pressure) ---- */
	  float bmpTemp = 0;
	  BMP280_Measure(&bmpTemp, &Pressure);

	  /* ---- Read AHT20 Sensor (Humidity + Temperature) ---- */
	  if (AHT20_Read(&Humidity, &Temperature) != 0)
	  {
		  Temperature = bmpTemp;  // Fallback to BMP280 temperature
	  }

	  /* ---- Servo Motor: Automatic Window Control ---- */
	  if (Temperature > TEMP_THRESHOLD && !windowOpen)
	  {
		  // Smoothly open window (0° → 180°)
		  for (uint16_t pos = currentServoPos; pos <= SERVO_OPEN; pos += 10)
		  {
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pos);
			  HAL_Delay(3);
		  }
		  currentServoPos = SERVO_OPEN;
		  windowOpen = 1;
	  }
	  else if (Temperature <= TEMP_THRESHOLD && windowOpen)
	  {
		  // Smoothly close window (180° → 0°)
		  for (int16_t pos = currentServoPos; pos >= SERVO_CLOSED; pos -= 10)
		  {
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)pos);
			  HAL_Delay(3);
		  }
		  currentServoPos = SERVO_CLOSED;
		  windowOpen = 0;
	  }

	  /* ---- Display on OLED ---- */
	  SSD1306_Fill(0);  // Clear screen

	  // Line 1 (Y=0): Temperature in Celsius
	  sprintf(strCopy,"T:%.2f C", Temperature);
	  SSD1306_GotoXY(0, 0);
	  SSD1306_Puts(strCopy, &Font_7x10, 1);

	  // Line 2 (Y=12): Humidity in %RH
	  sprintf(strCopy,"H:%.2f %%", Humidity);
	  SSD1306_GotoXY(0, 12);
	  SSD1306_Puts(strCopy, &Font_7x10, 1);

	  // Line 3 (Y=24): Pressure in hPa
	  sprintf(strCopy,"P:%.1f hPa", Pressure / 100.0f);
	  SSD1306_GotoXY(0, 24);
	  SSD1306_Puts(strCopy, &Font_7x10, 1);

	  // Line 4 (Y=36): Light level (TEMT6000)
	  sprintf(strCopy,"L:%lumV %.2fV", lightmV, lightVolt);
	  SSD1306_GotoXY(0, 36);
	  SSD1306_Puts(strCopy, &Font_7x10, 1);

	  // Line 5 (Y=50): Window status
	  sprintf(strCopy,"Win:%s", windowOpen ? "OPEN" : "CLOSED");
	  SSD1306_GotoXY(0, 50);
	  SSD1306_Puts(strCopy, &Font_7x10, 1);

	  SSD1306_UpdateScreen();

	  HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  // Max sampling time for TEMT6000 accuracy
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);  // Run ADC self-calibration for accuracy
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
