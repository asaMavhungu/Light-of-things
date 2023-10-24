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


#define True 1
#define False 0

uint16_t startup_stage = True;
uint16_t end_stage = False;
uint16_t recieving_stage = False;
uint16_t receiving_end_stage = False;
uint16_t data;
uint32_t checkpoint = 0;
char bin_number2[13] = "xxxxxxxxxxxx";
int index = 0;
int threshold = 18;

/* USER CODE BEGIN PV */
uint32_t prev_millis = 0;
uint32_t prevOn_millis = 0;
uint32_t ldrPreviousState = 0;
uint16_t stoppedSending = 0;
uint16_t readStopingTime =0;
uint32_t checkpointPreviousTime = 0;
//uint16_t index = 0;
uint32_t ldrHigh = 0;
uint8_t receivedData = 0;
uint16_t reachedEnd =0;
uint32_t requiredTime = 250; // 200 ms
int16_t startedReceiving = 0;
volatile uint32_t delay_t = 500; // Initialise delay to 500ms
uint32_t adc_val;
uint16_t lux = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void writeLCD(char *char_in);
int binaryToDecimal(char *binary);
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
  char lux_str[20];
  char the_lux[20] = "one";
  delay_t = 100;
  int ticks = 0;
  while (1)
  {

    //HAL_GPIO_TogglePin(GPIOB, LED7_Pin);

    
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc,20);
    lux = HAL_ADC_GetValue(&hadc); //read ldr

    

    //HAL_GPIO_TogglePin(GPIOB, LED7_Pin);

    if (startup_stage)
    {


      //sprintf(lux_str, "liux is, %d", lux);
      sprintf(lux_str, "Ready 2 receive");
      writeLCD(lux_str);
      
      if (lux>threshold)
      {

        uint32_t currentTime = HAL_GetTick();
        uint32_t timePassed = currentTime-prev_millis; //check how much time have passed since led on

        //sprintf(lux_str, "tik is, %d", ticks);
        sprintf(lux_str, "Ready 2 receive");
        writeLCD(lux_str);
        ++ticks;

        
        if (ticks >= 8 ) // This is to turn on recieving
        { //check if 1 sec have passed since led on, (starting bit) then start transmission
          
          startup_stage = False;
          recieving_stage = True;
          
          checkpoint++ ;
          delay_t = 550;
          HAL_Delay(250);
        }

      }
      else
      {
        if (ticks >=4 && ticks <8)
        {

          startup_stage = False;
          end_stage = True;
          
          sprintf(lux_str, "Checkpoint , %d", checkpoint);
          writeLCD(lux_str);
        }

        ticks = 0;
      }

    }


    if (end_stage)
    {
      
      startup_stage = False;
      recieving_stage = True;
      receiving_end_stage = True;
      end_stage = False;
      delay_t = 550;
      HAL_Delay(175);


    }

    if (recieving_stage)
    {
      //writeLCD(bin_number2);

      //HAL_Delay(500);
      HAL_GPIO_TogglePin(GPIOB, LED7_Pin);


      if (lux>threshold)
      {
        data = True;
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
      }
      else
      {
        data = False;
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
      }
        

      if (index!=0)
      {
        bin_number2[index-1] = (data+48);
      }


      ++index;

      writeLCD(bin_number2);

      if (index >=13)
      {
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
        
        delay_t = 100;
        if (receiving_end_stage )
        {
          HAL_Delay(5000);
          int actual_checkpoint = binaryToDecimal(bin_number2);
          sprintf(lux_str, "Actual: %d", actual_checkpoint);
          writeLCD(lux_str);
          
          lcd_command(LINE_TWO);
          sprintf(lux_str, "Received: %d",checkpoint);
          lcd_putstring(lux_str);

          //writeLCD("finished");

          
          if (actual_checkpoint!=checkpoint)
          {
            uint16_t ledFlash= 0;
            
            while (ledFlash<5000)
            {
              HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
              HAL_Delay(200); //flash led on and off for 1 sec with 200ms delat inbetween
              HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
              HAL_Delay(200);
              ledFlash += 200;
            }

          }

          else if(actual_checkpoint==checkpoint)
          {
            HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
            HAL_Delay(5000);
          }

          HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
          
          HAL_Delay(9999999);
        }
        else
        {

          recieving_stage = False;
          startup_stage = True;
        }
        index = 0;
        for (int i=0;i<12;i++)
        {
          bin_number2[i] = 'x';
        }
        HAL_Delay(5000);

      }
        
    }



    HAL_Delay(delay_t);




    /*

    // Ready stage
    if (!stoppedSending && !reachedEnd && !startedReceiving) //transimission haven't started
    { 
      sprintf(lux_str, "Ready 2 receive");
      writeLCD(lux_str);
    }

    if (lux>9 && !stoppedSending && !reachedEnd)
    {
      uint32_t currentTime = HAL_GetTick();
      uint32_t timePassed = currentTime-prev_millis; //check how much time have passed since led on


      if (timePassed>=1000 && !startedReceiving) // This is to turn on recieving
      { //check if 1 sec have passed since led on, (starting bit) then start transmission
        startup_stage = True;
        //startedReceiving = True;
        prev_millis = currentTime;
      }
    }
    else if (lux<9 && startup_stage && !stoppedSending && !reachedEnd)
    {
      uint32_t currentTime = HAL_GetTick();
      uint32_t timePassed = currentTime-prev_millis; //check how much time have passed since led on

      startup_stage = False;
      recieving_stage = True;

    }

    if (recieving_stage)
    {

      if (lux > 9)
      {
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
        // wait 500ms
      }
      else
      {
        // write a 0
      }


    }

    

    if (lux>9 && !stoppedSending && !reachedEnd)
    { 
      uint32_t currentTime = HAL_GetTick();
      uint32_t timePassed = currentTime-prev_millis; //check how much time have passed since led on


      if (timePassed>=1000 && !startedReceiving) // This is to turn on recieving
      { //check if 1 sec have passed since led on, (starting bit) then start transmission
        startedReceiving = True;
        prev_millis = currentTime;
      }


      else if (startedReceiving)
      {  //if transmission of data in progress, turn led if ldr detected light
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
        sprintf(lux_str, "Receiving data %d", lux);
        writeLCD(lux_str);
        ldrHigh = HAL_GetTick() - ldrPreviousState; //calculate for how long led on
      }

    }

    else
    {
      HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET); //turn off led if no light detectected
      if (ldrHigh>100 && !reachedEnd)
      {  //read how long led was on, if on for 200 ms data transmission comes to an end
        if (ldrHigh<=200)
        {
          //HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
          //startedReceiving = j0;
          if (!stoppedSending)
          { //check at what time transmission stopped then record it, will be used when calculating checkpoint
            readStopingTime = HAL_GetTick() -200;
          }
          stoppedSending = 1;
          HAL_Delay(800); //sender is flashing for 1sec to represent end of transmission, do nothing during this
        }
      }
      ldrPreviousState=HAL_GetTick();
    }



    if(stoppedSending && !reachedEnd)
    { //if finished receiving data but still not received checkpoint
      uint32_t difference = readStopingTime-prev_millis;
      checkpoint = difference/500;  //calculate number of bits received/ checkpoint
      uint32_t checkpointTimeDifference = HAL_GetTick()-checkpointPreviousTime;
      if (checkpointTimeDifference>=500)
      { //checkpoint is received every 500ms check if this time has passed
        if (lux>9)
        { //check if led on when receiving checkpoint
          receivedData |= (1 << index); //detect number checkpoint received from sender, since received as binary. Converted to decimal 
        }
        index++;
        checkpointPreviousTime = HAL_GetTick();
        sprintf(lux_str, "Received %d", checkpoint); //display received number of bits
        writeLCD(lux_str);
      }
      if (checkpoint == receivedData && index>=8)
      { //if finished receiving all 8 bit represented checkpoint from sender and match(correct data)
        sprintf(lux_str,"Correct data"); //display validation of correct data
        writeLCD(lux_str);
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
        HAL_Delay(1000); //turn on led for 1 sec since correct data
        HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
      }
      else if (checkpoint != receivedData && index>=8)
      { //if finished receiving all 8 bit represented checkpoint from sender and incorrect data
        sprintf(lux_str,"Expected: %d", receivedData);// display when incorrect data, showing expected data
        writeLCD(lux_str);
        uint16_t ledFlash= 0;
        while (ledFlash<1000){
          HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
          HAL_Delay(200); //flash led on and off for 1 sec with 200ms delat inbetween
          HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
          HAL_Delay(200);
          ledFlash += 200;
        }
        
      }
      if(index>=8)
      { //end of transmission with checkpoint received
        reachedEnd = 1;
        writeLCD("Tranmission end");
      } 
      
    }
     //sprintf(lux_str, "%d", lux);
     //HAL_Delay (250);
   
    }
  */
  }
  
  /* USER CODE END 3 */
}


int binaryToDecimal(char *binary) {
    int decimal = 0;
    //int length = strlen(binary);

    for (int i = 1; i < 12; i++) {
        if (binary[i] == '1') {
            decimal = (decimal << 1) | 1;
        } else if (binary[i] == '0') {
            decimal = decimal << 1;
        } else {
            // Handle invalid characters if needed
            //printf("Invalid character in the binary string: %c\n", binary[i]);
            return -1;
        }
    }

    //decimal = decimal >> 1;

    return decimal;
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

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Add code to switch LED7 delay frequency
	
  
	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}

// TODO: Complete the writeLCD function
void writeLCD(char *char_in){
  delay(3000);
	lcd_command(CLEAR);
  lcd_putstring(char_in);

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
