/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef huart2;
#define INA219_ADDRESS (0x40<<1) // I2C address
#define CONFIG_REG  0x00
#define SHUNT_VOLTAGE_REG 0x01
#define BUS_VOLTAGE_REG 0x02
#define POWER_REG 0x03
#define CURRENT_REG 0x04
#define CALIBRATION_REG 0x05

#define INA219_CONFIG 0x399F // register value
#define R_SHUNT 0.1f // shunt resistor 0.1O
#define MAX_EXPECTED_CURRENT 3.2f
#define CURRENT_LSB (MAX_EXPECTED_CURRENT / 32767.0f)
#define POWER_LSB (20.0f * CURRENT_LSB)
uint16_t INA219_CALIBRATION_VALUE;

#define I_TARGET  0.3f    // constant current
#define V_TARGET  4.2f    // target voltage 4.2v
#define TEMP_LIMIT 45.0f  // temperature limit

int pwm_duty = 0;    // initialise  duty cycle
uint32_t temp_counter = 0;
float temperature = 25.0f;
int in_cv_mode = 0; // flag
int first_run = 1; // first loop



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart2 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

void I2C_Scan() {
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 100) == HAL_OK) {
					  if (first_run == 1) {
            printf("Found I2C device at 0x%X\n", addr);
						}
        }
    }
}


void INA219_WriteRegister(uint8_t reg, uint16_t value) {
    uint8_t data[2] = { (value >> 8) & 0xFF, value & 0xFF };
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("I2C Write Error at Register 0x%02X!\n", reg);
    } else {
				if (first_run == 1) {
        printf("I2C Write Success at Register 0x%02X: 0x%04X\n", reg, value);
				}
    }
    HAL_Delay(10); // 
}

uint16_t INA219_ReadRegister(uint8_t reg) {
    uint8_t data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("I2C Read Error!\n");
        return 0;
    }
		
    HAL_Delay(10); // 
		
    uint16_t result = (data[0] << 8) | data[1];
		if (first_run == 1) {
    printf("I2C Read Success at Register 0x%02X: 0x%04X\n", reg, result);
		}
    return result;
}

uint16_t INA219_ReadRegister_Quiet(uint8_t reg) {
		uint8_t data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
     HAL_Delay(10); // 
		
    uint16_t result = (data[0] << 8) | data[1];
		return result;
}

void INA219_Init(void) {
	
	  HAL_StatusTypeDef ret;
    
    // 
    ret = HAL_I2C_IsDeviceReady(&hi2c1, INA219_ADDRESS, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("ERROR: INA219 not detected on I2C bus!\n");
        return;
    } else {
        printf("SUCCESS: INA219 device detected at 0x%X!\n", INA219_ADDRESS);
    }
		
		
    float cal_float = 0.04096f / (CURRENT_LSB * R_SHUNT);
    INA219_CALIBRATION_VALUE = (uint16_t)(cal_float + 0.5f);
    
    INA219_WriteRegister(CONFIG_REG, INA219_CONFIG);
    HAL_Delay(5);
    
    INA219_WriteRegister(CALIBRATION_REG, INA219_CALIBRATION_VALUE);
    HAL_Delay(10);
    
    uint16_t cal_reg = INA219_ReadRegister(CALIBRATION_REG);
		uint16_t config_reg = INA219_ReadRegister(CONFIG_REG);
    printf("Config Register Readback: 0x%04X\n", config_reg);
    
    if (config_reg == 0) {
        printf("ERROR: INA219 Configuration Register is 0, I2C WRITE FAILED!\n");
    }

    printf("INA219 Initialized, Calibration Register: %u\n", cal_reg);

}

float INA219_ReadCurrent(void) {
    int16_t rawShuntVoltage = (int16_t)INA219_ReadRegister(SHUNT_VOLTAGE_REG);
    float shuntVoltage = rawShuntVoltage * 0.01f; // mV
    float current_A = (shuntVoltage / R_SHUNT) / 1000.0f; // A
    printf("Raw Shunt Voltage: %d, Shunt Voltage: %.3f mV, Current: %.3f A\n", rawShuntVoltage, shuntVoltage, current_A);
    return current_A;
}

float INA219_ReadCurrent_Register(void) {
    int16_t rawCurrent = (int16_t)INA219_ReadRegister(CURRENT_REG);
    float current_A_2 = rawCurrent * CURRENT_LSB;
		if (first_run == 1) {
				printf("Current Register Raw: %d, Current: %.3f A\n", rawCurrent, current_A_2);
		}
    return current_A_2;
}


float INA219_ReadBusVoltage(void) {
    uint16_t rawBusVoltage = INA219_ReadRegister(BUS_VOLTAGE_REG);
		if (first_run == 1) {
        printf("Raw Bus Voltage Register: 0x%04X\n", rawBusVoltage);
    }
    return (rawBusVoltage >> 3) * 0.004f; // change to voltage (V)
	

}

float INA219_ReadCurrent_Quiet(void) {
    int16_t rawShuntVoltage = (int16_t)INA219_ReadRegister_Quiet(SHUNT_VOLTAGE_REG);
    float shuntVoltage = rawShuntVoltage * 0.01f; // mV
    float current_A = (shuntVoltage / R_SHUNT) / 1000.0f; // A
    return current_A;
}



void Read_ADC_All(float *V_batt, float *temperature_new)
{
	
		uint32_t adc_vbatt_sum = 0;
    uint32_t adc_temp_sum = 0;
    uint32_t adc_vref_sum = 0;
	  for (int i = 0; i < 30; i++) {
        // === V_batt(PA0) ===
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_vbatt_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(1);

        // === temp MCP9700(PA1) ===
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_temp_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(1);

        // ===  Vrefint ===
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_vref_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(1);
    }
	  float adc_vbatt_avg = adc_vbatt_sum / 30.0f;
    float adc_temp_avg = adc_temp_sum / 30.0f;
    float adc_vref_avg = adc_vref_sum / 30.0f;
		
		
	  uint16_t vrefint_cal = *VREFINT_CAL_ADDR;
	  float Vdda = 3.3f * ((float)vrefint_cal / adc_vref_avg);
		
	  float Vout = ((float)adc_temp_avg / 4095.0f) * Vdda;
	  float Temperature = (Vout - 0.5f) / 0.1f; // degree

    float Vadc_batt = 3.3f * adc_vbatt_avg / 4095.0f;
    *V_batt = Vadc_batt * (4.8f / 3.3f); // ????
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	I2C_Scan();
	INA219_Init();
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
				float current_sum = 0;
				for (int i = 0; i < 150; i++) {
				float i_now = INA219_ReadCurrent_Quiet();

				current_sum += i_now;
				HAL_Delay(200); 
				}
				float current = (current_sum / 150.0f)- 0.015f;
				float current_2 = INA219_ReadCurrent_Register();
				float bus_voltage = INA219_ReadBusVoltage(); // read busboltage
				float temperature_new = 0.0f;
				float V_batt = 0.0f;
				float temperature = 0.0f;
				Read_ADC_All(&V_batt, &temperature_new);
				temperature = temperature_new;
				float v_batt_est = bus_voltage - (current * 20.122f);



			if (first_run == 1) {
					printf("-------------------------\n");
					printf("battery Voltage: %.3f V, Current: %.3f A\n", V_batt, current);
					printf("reg_Current: %.3f A\n", current_2);
					printf("Calibration Register Value: %u\n", INA219_CALIBRATION_VALUE);
					printf("PWM Duty: %d,", pwm_duty);
					printf("-------------------------\n");
				
					first_run = 0;
			} else {
					printf("[V_batt: %.3f V]  [I: %.3f A]  [Temp: %.2f C ]  [PWM: %d] [%s Mode]\n",
								 V_batt, current, temperature_new, pwm_duty,
									in_cv_mode ? "CV" : "CC");
			}

			if (first_run == 1) {
					printf("Temperature (Previous): %.2f C\n", temperature);
			}

			// 20s renew temperature
			if (temp_counter >= 60) // 20*500ms = 10s
			{
					temperature = temperature_new;
					temp_counter = 0; //
			}
			else
			{
					temp_counter++;
			}
			
			
			// temperature protection
					if (temperature > TEMP_LIMIT)
			{
					printf("Temperature too high! Stopping charging!\n");
					printf("Duty cycle is 0!\n");
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // PWM duty cycle set to 0
					continue;
			}
			
			//CC-CV charging 
			if (!in_cv_mode && V_batt < V_TARGET)
			{
				printf("Constant Current Mode (CC Stage)\n");
					// CC stage
					if (current < I_TARGET)
					{
							pwm_duty -= 50; // decrease duty cycle
							if (pwm_duty > __HAL_TIM_GET_AUTORELOAD(&htim1)) 
							if (pwm_duty < 0) pwm_duty = 0; 
					}
					else if (current > I_TARGET)
					{
							pwm_duty += 50; //increase duty cycle
							if (pwm_duty > __HAL_TIM_GET_AUTORELOAD(&htim1)) 
									pwm_duty = __HAL_TIM_GET_AUTORELOAD(&htim1);// set limit for max value duty cycle
					}
			}
			else
			{
					if (!in_cv_mode) {
					printf("Voltage reached target! Constant voltage stage!\n");
					in_cv_mode = 1;
					}
					printf("Constant Voltage Mode (CV Stage)\n");
					// CV stage
					if (current > 0.03f)
					{
							if (pwm_duty<= __HAL_TIM_GET_AUTORELOAD(&htim1))
								pwm_duty += 50;
							else
								pwm_duty = __HAL_TIM_GET_AUTORELOAD(&htim1);
					}
			}
    
			// renew duty cycle
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);

			printf("PWM Duty: %d\n", pwm_duty);

			HAL_Delay(120000); // 120s - 2minutes
	}
    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

#ifdef  USE_FULL_ASSERT
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
