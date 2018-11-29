/**
******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************


*/




/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure;
uint16_t Tim3_PrescalerValue;
GPIO_InitTypeDef   GPIO_InitStruct;

__IO uint16_t Tim3_CCR;	// compare register for tim3
double factor = 1;	// speed factor
double factorTrue = 1;
char fString[6];    // LCD display buffer

uint16_t dirBool = 1;		// 1 = CW, 0 = CCW
uint16_t stepBool =1;		// 1 = full, 0 = half
typedef enum state{s1,s2,s3,s4,s5,s6,s7,s8}state;
state steps = s1;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void TIM3_Config(void);
void TIM3_OC_Config(void);
void PINS_Config(void);

//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	HAL_Init();
	

	SystemClock_Config();   //sysclock is 4 MHz
  
	HAL_InitTick(0x0000); // set systick's priority to the highest.

	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);


	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);  
	
	TIM3_Config();
	Tim3_CCR=5833;		// calculated
	TIM3_OC_Config();
	
	PINS_Config();
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)"yeet");	
	
 	
  while (1)
  {/*
		
					switch (steps){
						case s1:
								if (dirBool == 1){		// if clockwise
										if (stepBool == 1){		// and full step
													steps = s3;
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
										}
										else{		// if half step
													steps = s2;
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
										}
								}
								else{								// if ccw
										if (stepBool == 1){		// and full step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
													steps = s7;
										}
										else{		// if half step
													steps = s8;
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
										}
								}
								break;
						case s2:
  							if (dirBool == 1){		// if clockwise
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);		
										steps = s3;				// can only go to s3 on both full and half step
								}
								else{								// if ccw
										steps = s1;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
								}
								break;
						case s3:
  							if (dirBool == 1){		// if clockwise
										if (stepBool == 1){		// and full step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
													steps = s5;
										}
										else{		// if half step
													steps = s4;
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
										}
								}
								else{								// if ccw
										if (stepBool == 1){		// and full step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
													steps = s1;
										}
										else{		// if half step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
													steps = s2;
										}
								}
								break;
						case s4:
  							if (dirBool == 1){		// if clockwise
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
										steps = s5;				// can only go one step on both full and half step
								}
								else{								// if ccw
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
										steps = s3;
								}
								break;
						case s5:
  							if (dirBool == 1){		// if clockwise
										if (stepBool == 1){		// and full step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
													steps = s7;
										}
										else{		// if half step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
													steps = s6;
										}
								}
								else{								// if ccw
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
										if (stepBool == 1){		// and full step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
													steps = s3;
										}
										else{		// if half step
													steps = s4;
										}
								}
								break;
						case s6:
  							if (dirBool == 1){		// if clockwise
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
										steps = s7;				// can only go one step on both full and half step
								}
								else{								// if ccw
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
										steps = s5;
								}
								break;
						case s7:
  							if (dirBool == 1){		// if clockwise
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
										if (stepBool == 1){		// and full step
													HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
													steps = s1;
										}
										else{		// if half step
													steps = s8;
										}
								}
								else{								// if ccw
										if (stepBool == 1){		// and full step
													steps = s5;
										}
										else{		// if half step
													steps = s6;
										}
								}
								break;
						case s8:
  							if (dirBool == 1){		// if clockwise
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
										steps = s1;				// can only go one step on both full and half step
								}
								else{								// if ccw
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
										steps = s7;
								}
								break;

					}*/
			
	} //end of while 1
	

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* The following clock configuration sets the Clock configuration sets after System reset                */
  /* It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig */
  /* and to be eventually adapted to new clock configuration                                               */

  /* MSI is enabled after System reset at 4Mhz, PLL not used */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  /* Set 0 Wait State flash latency for 4Mhz */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz??

void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    Tim3 is of 16 bits. Timer 2..7 is on APB1.
	
		Since the clock source is MSI, and the clock range is RCC_MSIRANGE_6, SystemCoreClock=4Mhz.
	
		Since RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1, HCLK=4Mhz.
	
		Since RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1, PCLK1=4MHz, and TIM3CLK=4Mhz.
		(if the APB1 prescaler is not 1, the timer clock frequency will be 2 times of APB1 frequency)
	
		 that is, for current RCC config, the the Timer3 frequency=SystemCoreClock.
	
		To get TIM3's counter clock at 10 KHz, for example, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	
		i.e: Prescaler = (SystemCoreClock /10 KHz) - 1
       
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	/* we want 1 kHz, base block is 4MHz
	 sub into formula 4000000/(PSC+1) = 10kHz
	 so PSC = 399	*/
	 
	Tim3_PrescalerValue = (uint16_t) 399;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
 
  Tim3_Handle.Init.Period = 100000;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
}

void TIM3_OC_Config(void)
{
		Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim3_OCInitStructure.Pulse=Tim3_CCR;
		Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim3_Handle); // if the TIM3 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}

void PINS_Config(void)
{
		
			__HAL_RCC_GPIOE_CLK_ENABLE();
	
			GPIO_InitStruct.Pin = GPIO_PIN_10;			// A1 = IN 1
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
			GPIO_InitStruct.Pin = GPIO_PIN_12;			// A2 = IN 2
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
			GPIO_InitStruct.Pin = GPIO_PIN_14;			// B1 = IN 4
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
			GPIO_InitStruct.Pin = GPIO_PIN_15;			// B2 = IN 3
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);		// so that they start opposite
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);

}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button	
				
							if (stepBool==1){
									stepBool = 0;
							}
							else{
									stepBool = 1;
							}
							
							break;	
			case GPIO_PIN_1:     //left button
							
							if (dirBool==1){
									dirBool = 0;
							}
							else{
									dirBool = 1;
							}


							break;
			case GPIO_PIN_2:    //right button						 

							break;
			case GPIO_PIN_3:    //up button
				
							BSP_LCD_GLASS_Clear();
							factor = factor-0.1;			// increase speed by 10% each time press up				
							factorTrue = factorTrue+0.1;
							sprintf(fString, "%.1f", factorTrue);
							BSP_LCD_GLASS_DisplayString((uint8_t*)fString);
							__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_1, Tim3_CCR*factor);
			
							break;
			
			
			case GPIO_PIN_5:    //down button			
				
							BSP_LCD_GLASS_Clear();
							factor = factor+0.1;			// increase speed by 10% each time press up				
							factorTrue = factorTrue-0.1;
							sprintf(fString, "%.1f", factorTrue);
							BSP_LCD_GLASS_DisplayString((uint8_t*)fString);
							__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_1, Tim3_CCR*factor);	
							
							break;
			default://
						//default
						break;
	  } 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
	//if ((*htim).Instance==TIM3)    //since only one timer, this line is actually not needed
	//	BSP_LED_Toggle(LED5);
	
}

// 10    14
// 12    15


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
			switch (steps){			// different states
						case s1:			// + +
								if (dirBool ==1){
										steps = s2;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
								}
								else{
										steps = s8;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
								}
								break;
						case s2:			// + OFF
								if (dirBool ==1){ // if cw
										if (stepBool ==1){	// if full step go to OFF +
												steps = s8;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	
										}
										else{				// cw half step go to + -
												steps = s3;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
										}
								}
								else{			// if ccw
										if (stepBool ==1){	// if full step go to OFF -
												steps = s4;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);	
										}
										else{				// ccw half step go to ++
												steps = s1;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
										}
								}
								break;
						case s3:			// + -
								if (dirBool ==1){	// if cw, go to OFF -
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
										steps = s4;
								}
								else{			// go to + OFF
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
										steps = s2;
								}
								break;
						case s4:			// OFF -
								if (dirBool ==1){ // if cw 
										if (stepBool ==1){	// if full step go to + OFF
												steps = s2;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	
										}
										else{				// cw half step
												steps = s5;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
										}
								}
								else{			// if ccw
										if (stepBool ==1){	// if full step go to - OFF
												steps = s6;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);	
										}
										else{				// ccw half step
												steps = s3;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
										}
								}
								break;
						case s5:			// - -
								if (dirBool ==1){	// go to - OFF
										steps = s6;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
								}
								else{						// go to OFF -
										steps = s4;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
								}
								break;
						case s6:	// - OFF
								if (dirBool ==1){ // if cw
										if (stepBool ==1){	// if full step go to OFF -
												steps = s4;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);	
										}
										else{				// cw half step go to -+
												steps = s7;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
										}
								}
								else{			// if ccw
										if (stepBool ==1){	// if full step go to OFF +
												steps = s8;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	
										}
										else{				// ccw half step go to - -
												steps = s5;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
										}
								}
								break;
						case s7:
								if (dirBool ==1){
										steps = s8;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
								}
								else{
										steps = s6;
										HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
								}
								break;
						case s8:
								if (dirBool ==1){ // if cw
										if (stepBool ==1){	// if full step
												steps = s6;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);	
										}
										else{				// cw half step
												steps = s1;
													
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
										}
								}
								else{			// if ccw
										if (stepBool ==1){	// if full step
												steps = s2;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);	
										}
										else{				// ccw half step
												steps = s7;
												HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
										}
								}
								break;
					}
			__HAL_TIM_SET_COUNTER(htim, 0x0000);  
}
 


static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
