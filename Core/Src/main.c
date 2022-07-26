/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <gyro.h>
#include <FXOS8700CQ.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

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
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

//Initialisierung der Variablen, die vom Interrupt gesetzt werden

//volatile, damit sie vom Compiler nicht wegoptimiert werden
volatile bool Tara = false; //für den Tara-Interrupt
volatile bool LEDistAN = true;//Für den TIM16 Interrupt, der die LEDs blinken lässt wenn Auslenkung < 5°


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



	//Initialisierung der Sensor-Rohdaten-Variablen für Gyroskop, Beschleunigungssensor und Magnetometer

	//Gyro
	int16_t x_axis, y_axis, z_axis;

	//Accelerometer
	int16_t x_axis_Acc, y_axis_Acc, z_axis_Acc;

	//Magnetometer
	int16_t x_axis_Mag, y_axis_Mag, z_axis_Mag;

	//Initialisiere TARA-Ausrichtung und setze sie auf Null

	double TaraHeading = 0;

	//Diese Variable misst die Abweichung vom Nullpunkt in Tausendstel Grad
	int16_t Abweichung_milliGrad;



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
  MX_TIM3_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

//starte Timer
  //PWM-Timer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);


//LEDS aus
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);


//Initialisiere die Sensoren

  bool Gyro_Init_Check;
  bool FXOS8700CQ_Init_Check;

  Gyro_Init_Check = InitialisiereGyro();


  FXOS8700CQ_Init_Check = InitialisiereFXOS8700CQ();


  if (Gyro_Init_Check == true){
	  //I2C-Kommunikation funktioniert
	  //blinke grüne LED 3x
	  uint8_t x;
	  for (x = 0; x < 3; ++x){

		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,511);
		  HAL_Delay(100);
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
		  HAL_Delay(100);

		    }

  }

  if (FXOS8700CQ_Init_Check == true){
	  //I2C-Kommunikation funktioniert
	  //blinke blaue LED 3x
	  uint8_t x;
	  for (x = 0; x < 3; ++x){


		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,511);
		  HAL_Delay(100);
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
		  HAL_Delay(100);
		    }

  }

  //Funktion, umd die Ausrichtung des Sensors zu bestimmen

  double BerechneAusrichtung(int16_t *x_axis_Mag, int16_t *y_axis_Mag, double *TaraHeading){

	  //Skalarprodukt von Taravektor [x_Tara_Mag y_Tara_Mag z_Tara_Mag]^T und Nordvektor [x_axis_Mag y_axis_Mag z_axis_Mag]^T

	  //int32_t skalarprod = (*x_Tara_Mag) * (*x_axis_Mag) + (*y_Tara_Mag) * (*y_axis_Mag) + (*z_Tara_Mag) * (*z_axis_Mag);
	  //double BetragTaraVec = sqrt((*x_Tara_Mag) * (*x_Tara_Mag) + (*y_Tara_Mag) * (*y_Tara_Mag) + (*z_Tara_Mag) * (*z_Tara_Mag));
	  //double BetragMagVec = sqrt((*x_axis_Mag) * (*x_axis_Mag) + (*y_axis_Mag) * (*y_axis_Mag) + (*z_axis_Mag) * (*z_axis_Mag));
	   //Winkel zwischen TaraVec und MagVec
	  //Abweichung = acos(skalarprod/(BetragTaraVec*BetragMagVec))*(180/M_PI);
	  double Azimuth = 90 - atan2((double)*y_axis_Mag, (double)*x_axis_Mag) * 180 / M_PI;
	  //Berechne Abweichung von per blauem Knopf gewählten Nullpunt.
	  //TaraHeading ist mit 0 initialisiert, d.h. vor dem ersten Drücken des blauen Knopfes ist Abweichung = Azimuth
	  double Abweichung = Azimuth - *TaraHeading;
	  //konvertiert in int für abs-Funktion
	  int16_t Abweichung_milliGrad = (int16_t)(Abweichung * 1000);
	  return Abweichung_milliGrad;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1){
	  //Lese Sensorwerte aus
	  gyroWerteAuslesen(&x_axis, &y_axis, &z_axis);

 	  FXOS8700CQWerteAuslesen(&x_axis_Mag, &y_axis_Mag, &z_axis_Mag, &x_axis_Acc, &y_axis_Acc, &z_axis_Acc);


	  //Prüfe, ob blauer Knopf gedrückt wurde
	  if (Tara == true){
		  //speichere aktuelle Abweichung von Norden
		  TaraHeading = 90 - atan2 ((double)y_axis_Mag, (double)x_axis_Mag) * 180 / M_PI;
		  Tara = false;
	  }
	  Abweichung_milliGrad = BerechneAusrichtung(&x_axis_Mag, &y_axis_Mag, &TaraHeading);

//Hier der Code, wenn die LEDs blinken sollen weil die Abweichung kleiner als 5° ist.

	  if (abs(Abweichung_milliGrad) <= 5000){


		  HAL_TIM_Base_Start_IT(&htim16);//starte den Timer, der für das Blinken der LEDs da ist, in Interrupt-Modus

		  if (z_axis >= 0){
			  // z-Achse wird GEGEN Uhrzeigersinn gedreht -> grüne LED muss blinken, blaue LED aus
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0); //blau aus
			  if (LEDistAN == true){
				  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,511); //grün an
			  }else{
				  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0); //grün aus
			  }
		  }else{
			  	  // z-Achse wird IM Uhrzeigersinn gedreht -> blaue LED muss blinken, grüne LED aus
				  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0); //grün aus
				  if (LEDistAN == true){
					  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,511); //blau an
				  }else{
					  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0); //blau aus
				  }
			}
		  }else{
			  HAL_TIM_Base_Stop_IT(&htim16);
			  //stoppe den Blink-Timer
		  }


//Hier der Code, wenn die LEDs dauerleuchten sollen, da die Abweichung >5° ist.

	  if (z_axis >= 0 && abs(Abweichung_milliGrad) > 5000){
	  		  // z-Achse wird GEGEN Uhrzeigersinn gedreht
	  		  int16_t z_axis_Max = 0x7FFF; //maximaler Wert eines 16-bit signed int
	  		  int16_t z = (z_axis*511)/z_axis_Max; //511 ist in der Konfiguration von Tim3 die Zahl, bis zu der gezählt wird.

	  		  //setzt Pulsweite für grüne LEDauf berechneten %-Wert
	  		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,z);
	  		  //setzt Pulsweite für blaue LED auf 0
	  		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);

	  	  }else if (z_axis <= 0 && abs(Abweichung_milliGrad) > 5000){
	  		  // z-Achse wird IM Uhrzeigersinn gedreht
	  		  int16_t z_axis_Min = -0x8000;	//minimaler Wert eines 16-bit signed int
	  		  int16_t z = (z_axis*511)/z_axis_Min;
	  		  //setzt Pulsweite für blaue LEDauf berechneten %-Wert
	  		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,z);
	  		  //setzt Pulsweite für grüne LED auf 0
	  		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }//Ende while-Schleife
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */


  /*HAL_I2C_Slave_Transmit(hi2c, pData, Size, Timeout) */
  /*LL_I2C_SetSlaveAddr*/



  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 512-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 250-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
// Interrupt Funktionen
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
     if(GPIO_Pin == GPIO_PIN_0) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	//kurzer Interrupt setzt Tara Wert auf true, damit dann in der while Schleife die aktuelle Ausrichtung gespeichert werden kann
    	Tara = true;
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim16){ //Prüfe, ob es tatsächlich Timer 16 war, der den Interrupt ausgelöst hat

		//jetzt wird der LEDistAN Wert umgeschaltet; entweder von Wahr auf Falsch oder von Falsch auf Wahr.
		//Bei jedem Durchlauf des Counters wird somit ein Interrupt ausgelöst. In der Hauptschleife kann somit entweder Die LED an- oder ausgeschaltet werden,
		//je nachdem welchen Wert LEDistAN annimmt.
		if (LEDistAN == true){

			LEDistAN = false;
		}else{
			LEDistAN = true;
		}
		 //
	}
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
