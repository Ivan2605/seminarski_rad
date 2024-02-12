/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t red_flag = 0;
uint8_t x = 0;

void delay (uint16_t time)/*definiranje funkcije za delay u mikrosekundama  */
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

void Display_Temp (float Temp)/* definiranje funkcije za prikaz temperature */
{
	char str[20] = {0}; /* niz znakova duljine 20 znakova*/
	lcd_put_cur(0, 0);  /*postavljnanje u prvi stupac i redak pozivom funkcije iz i2c-lcd.c */

	sprintf (str, "Temp:- %.2f ", Temp);
	lcd_send_string(str);/*šalje niz znakova str */
	lcd_send_data('C');
}

void Display_Rh (float Rh)/*Definiranje funkicje za prikaz vlage  */
{
	char str[20] = {0};
	lcd_put_cur(1, 0);/*drugi stupac drugi redak */

	sprintf (str, "Vlaga:- %.2f ", Rh);
	lcd_send_string(str);
	lcd_send_data('%');
}

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, VLAGA, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0; //odgovor senzora

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)/* definiranje funckicje za postavljane pina kao izlaz*/
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

void DHT11_Start (void)
{

	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // postavi pin kao izlaz

	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);
	delay (20);
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);
	delay (18000);

	Set_Pin_Input(DHT11_PORT, DHT11_PIN);// postavi pin kao ulaz

}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))// ako senzor šalje nisko stanje
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) // ako senzor šalje visoko stanje
		Response = 1;
		else Response = -1; // greška
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // kada senzor šalje nisko vraća vrijednost response

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // ako senzor šalje nisko stanje
		delay (50);
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
		{
			i&= ~(1<<(7-j));   // postavlja određeni bit varijable u nulu
		}
		else i|= (1<<(7-j));  // postavlja određeni bit varijable u jedinicu
	}
	return i;
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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);

  lcd_init();

 /* HAL_Delay(3000);
  lcd_clear ();

  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  lcd_clear ();
	  lcd_send_string(" Prikaz vlage i");
	  lcd_put_cur(1, 0);
	  lcd_send_string("  temperature ->");

	  HAL_Delay(2000);

	  DHT11_Start();
	  Presence = DHT11_Check_Response();
	  Rh_byte1 = DHT11_Read ();
	  Rh_byte2 = DHT11_Read ();
	  Temp_byte1 = DHT11_Read ();
	  Temp_byte2 = DHT11_Read ();
	  SUM = DHT11_Read();


	  HAL_Delay(3000);


       if( Temp_byte1>= 24){
	  	 		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	  	 		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
       }
       if(Temp_byte1 < 24){
	  	 		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
	  	 		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);

       }



       if(red_flag == 1 &&  __HAL_TIM_GET_COUNTER(&htim7) == htim7.Init.Period ){



	           while(x<5 ){


		           lcd_clear ();
		           DHT11_Start();
		           Presence = DHT11_Check_Response();
		           Rh_byte1 = DHT11_Read ();
		           Rh_byte2 = DHT11_Read ();
		           Temp_byte1 = DHT11_Read ();
		           Temp_byte2 = DHT11_Read ();
		           SUM = DHT11_Read();

		           TEMP = Temp_byte1;
		           VLAGA = Rh_byte1;
		           Temperature = (float) TEMP;
		           Humidity = (float) VLAGA;
                   Display_Temp(Temperature);
                   Display_Rh(Humidity);



                   HAL_Delay(3000);
                    x++;
	           }


    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_TIM_Base_Start(&htim7);
	red_flag = 1;
	x = 0;
}
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
