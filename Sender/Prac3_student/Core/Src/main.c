/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
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
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t prev_millis = 0;
uint32_t curr_millis = 0;
uint32_t delay_t = 500; // Initialise delay to 500ms

uint32_t data = 0b11011011;


uint32_t data;
uint32_t message;


char bin_number[13];


uint32_t adc_val;
volatile uint32_t last_button_press_time = 0;

volatile uint32_t last_button2_press_time = 0;


#define True 1
#define False 0


volatile int transmit = False;

volatile int done_sending = False;

#define LINE_ONE 0x80 // Address of the first char in line 1 
#define LINE_TWO 0xC0 // Address of the first char in line 2
void setLCD1(char *str);
void setLCD2(char *str);

char* Dec2RadixI(int, int);
void decimalToBinaryString(uint16_t, char*);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void writeLCD(char *char_in);
uint32_t pollADC(void);
uint32_t ADCtoCCR(uint32_t adc_val);
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
  MX_ADC_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  init_LCD();

  // PWM setup
  uint32_t CCR = 0;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start PWM on TIM3 Channel 3

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int send = False;

  uint32_t adc_value;

  char bin_number[13];

  char bin_number2[13] = "xxxxxxxxxxxx";

  volatile int index = 0;

  volatile int setup = 0;

  volatile int ready = False;

  adc_value = pollADC();



  
  
  //data = 0b11111111;
  while (1)
    {
    // Toggle LED0

    HAL_GPIO_TogglePin(GPIOB, LED7_Pin);

    //send = HAL_GPIO_ReadPin(GPIOB, LED7_Pin);

    send = True;

    if (done_sending)
    {
      delay_t = 100;
      send = HAL_GPIO_ReadPin(GPIOB, LED7_Pin);
      HAL_GPIO_WritePin(GPIOB, LED5_Pin, send);
    }


    if (transmit)
    {
      //send = ~send;

      //int to_send = data & 0b1;

      if (ready && index < 13)
      {

        char bit_to_send_char = bin_number[index];

        int to_send = bit_to_send_char - 48;

        if (send)
        {

          //int to_send = data & 0b1;
          //data = data >> 1;

          if (to_send)
            HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
          else
            HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);

          bin_number2[index] = bin_number[index];

          ++index;

          //++adc_value;
          //char lcd_display_string[16];
          //sprintf(lcd_display_string, "%lu", to_send);
          //writeLCD(lcd_display_string);

        }
        else
        {
          HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
        }
        // HAL_GPIO_TogglePin(GPIOB, LED5_Pin);

        // ADC to LCD; TODO: Read POT1 value and write to LCD



        //sprintf(lcd_display_string, "%lu", to_send);
        //sprintf(lcd_display_string, "S: %s", data);
        //writeLCD(lcd_display_string);
        writeLCD(bin_number2);



        //char bin_number[13];

        decimalToBinaryString(adc_value, bin_number);

        //char* ace = Dec2RadixI(asa, 2);

        //char * bin_number = Dec2RadixI(asa, 2);



        //sprintf(bin_number, "0b", Dec2RadixI((int)adc_val, 2) );
        setLCD2(bin_number);


        // Update PWM value; TODO: Get CRR
        CCR= ADCtoCCR(adc_value);

        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CCR);
      }

      else
      {

        ++setup;
        if (setup == 3)
        {
          ready = True;
          //HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
        }
        if (setup < 3)
        {
          char lcd_display_string[16];
          sprintf(lcd_display_string, "Setup: %d/8", setup);
          writeLCD(lcd_display_string);

        }
        else
        {
          char lcd_display_string[16] = "Sending Done";
          writeLCD(lcd_display_string);
          setLCD2(bin_number2);
          //delay_t = 150;

          int to_send = HAL_GPIO_ReadPin(GPIOB, LED7_Pin);
          if (to_send)
            HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
          else
            HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
        }
      }

    }
    else
    {

      // ADC to LCD; TODO: Read POT1 value and write to LCD

      adc_value = pollADC();

      char lcd_display_string[16];
      //sprintf(lcd_display_string, "%lu", to_send);
      sprintf(lcd_display_string, "Polling: %d", adc_value);
      writeLCD(lcd_display_string);


        index = 0;

        setup = 0;

        ready = False;

        for (int i = 0; i < 13; i++) {
          bin_number2[i] = 'x';
        }



      //char bin_number[13];

      decimalToBinaryString(adc_value, bin_number);

      //char* ace = Dec2RadixI(asa, 2);

      //char * bin_number = Dec2RadixI(asa, 2);



      //sprintf(bin_number, "0b", Dec2RadixI((int)adc_val, 2) );
      setLCD2(bin_number);


      // Update PWM value; TODO: Get CRR
      CCR= ADCtoCCR(adc_value);

      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CCR);
    }

    // Wait for delay ms
    HAL_Delay (delay_t);
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

void decimalToBinaryString(uint16_t decimal, char* binaryString) {
    for (int i = 11; i >= 0; i--) {
        binaryString[11 - i] = ((decimal >> i) & 1) + '0';
    }
    binaryString[12] = '\0'; // Null-terminate the string
}


void setLCD1(char *str)
{
    lcd_command(LINE_ONE);        // set cursor to the first line
    lcd_putstring("                "); // clear the first line
    lcd_command(LINE_ONE);      // set cursor to beginning of first line
    lcd_putstring(str);           // display the given string
}

void setLCD2(char *str)
{
    lcd_command(LINE_TWO);
    lcd_putstring("                "); // clear the first line
    lcd_command(LINE_TWO);      // set cursor to beginning of first line
    lcd_putstring(str);
}


void send_data_DEPRECATED(int send, int data, uint32_t CCR)
{

  while(1)
  {
    // Toggle LED0
    //send = ~send;
    HAL_GPIO_TogglePin(GPIOB, LED7_Pin);

    int to_send = data & 0b1;

    if (send)
    {

      //int to_send = data & 0b1;
      data = data >> 1;

      if (to_send)
        HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
      else
        HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);

      //++adc_value;
      //char lcd_display_string[16];
      //sprintf(lcd_display_string, "%lu", to_send);
      //writeLCD(lcd_display_string);

    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
    }
    // HAL_GPIO_TogglePin(GPIOB, LED5_Pin);

    // ADC to LCD; TODO: Read POT1 value and write to LCD

    uint32_t adc_value = pollADC();

    char lcd_display_string[16];
    //sprintf(lcd_display_string, "%lu", to_send);
    sprintf(lcd_display_string, "%d :: %d :: %d", to_send, adc_value, data);
    writeLCD(lcd_display_string);

  }


}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */
  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  ADC1->CR |= ADC_CR_ADCAL;
  while(ADC1->CR & ADC_CR_ADCAL);			// Calibrate the ADC
  ADC1->CR |= (1 << 0);						// Enable ADC
  while((ADC1->ISR & (1 << 0)) == 0);		// Wait for ADC ready
  /* USER CODE END ADC_Init 2 */

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
  htim3.Init.Period = 47999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);

  /**/
  LL_GPIO_SetPinPull(Button1_GPIO_Port, Button1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button1_GPIO_Port, Button1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);


  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Add code to switch LED7 delay frequency
	// Check if the interrupt is triggered by EXTI line 0 (PA0)
	 if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
		    // Clear the EXTI line 0 pending bit
		    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

		    // Get the current time
		    uint32_t current_time = HAL_GetTick();

		    // Calculate the time difference since the last button press
		    uint32_t time_difference = current_time - last_button_press_time;

		    // Check if the time difference is greater than a debounce threshold (e.g., 100 ms)
		    if (time_difference > 100) {
		        // Toggle the delay frequency between 1 Hz and 2 Hz

            //data = adc_val;

            data = pollADC();

            //decimalToBinaryString(data, message);

            //transmit = True;

            transmit = True;

            //send_data(0b0, data, 0);


            HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);






		        //if (delay_t == 1000) {
		        //    delay_t = 500;  // Change to 2 Hz (500 ms period)
		        //} else {
		        //    delay_t = 1000; // Change to 1 Hz (1000 ms period)
		        //}

		        // Update the last button press time
		        last_button_press_time = current_time;
		    }
		}
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) {
      // Clear the EXTI line 0 pending bit
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

      // Get the current time
      uint32_t current_time = HAL_GetTick();

      // Calculate the time difference since the last button press
      uint32_t time_difference = current_time - last_button2_press_time;

      // Check if the time difference is greater than a debounce threshold (e.g., 100 ms)
      if (time_difference > 100) {

        HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);


        //done_sending = True;
        if (transmit == False)
        {
          done_sending = True;
        }
        transmit = False;

        HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);


        last_button2_press_time = current_time;

      }
    }
  HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags

}

// TODO: Complete the writeLCD function
void writeLCD(char *char_in){
    delay(3000);
	lcd_command(CLEAR);
	lcd_putstring(char_in);  // Display the input string on the LCD
}

// Get ADC value
uint32_t pollADC(void){
  // TODO: Complete function body to get ADC val
	uint32_t val = 0;

	// Start ADC conversion
	HAL_ADC_Start(&hadc);

	// Wait for conversion to complete
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
	  // Read ADC value
	  val = HAL_ADC_GetValue(&hadc);
    }

	// Stop ADC conversion
	HAL_ADC_Stop(&hadc);


	return val;
}

// Calculate PWM CCR value
uint32_t ADCtoCCR(uint32_t adc_val){
  // TODO: Calculate CCR val using an appropriate equation
  uint32_t val = 0;

  uint32_t arr_value = 47999;  // ARR for 1 kHz frequency
  val = (adc_val * arr_value) / 4095;

  return val;
}

void ADC1_COMP_IRQHandler(void)
{
	adc_val = HAL_ADC_GetValue(&hadc); // read adc value
	HAL_ADC_IRQHandler(&hadc); //Clear flags
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