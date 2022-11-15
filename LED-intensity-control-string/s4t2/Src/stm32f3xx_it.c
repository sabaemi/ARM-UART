/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
int i=0;
char a;
int b=0;
int position = 0;
extern int tv;
extern int iv;
extern int t;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 break and TIM15 interrupts.
*/
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
* @brief This function handles TIM1 update and TIM16 interrupts.
*/
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
* @brief This function handles TIM1 trigger, commutation and TIM17 interrupts.
*/
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	extern unsigned char data[1];
  extern unsigned char buffer[100];
	iv=0;
	tv=0;
	t=0;
			if(data[0] != '\r'){
					buffer[position] = data[0];
					buffer[position+1] = '\0';
				  position++;
			}
			else if(data[0] == '\r'){
				
				while(buffer[i]!=' '){
					char bb=buffer[i];
				int ib=bb-'0';
					iv=(iv*10)+ib;
					i++;
				}
				
				i++;
				
				while(buffer[i]!=' '){
					char bb=buffer[i];
				int ib=bb-'0';
					tv=(tv*10)+ib;
					i++;
				}
				
				i++;
				
				while(buffer[i]!='\0'){
					char bb=buffer[i];
				int ib=bb-'0';
					t=(t*10)+ib;
					i++;
				}
			
if(iv==tv){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, iv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, iv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, iv);
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, iv);
		
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, iv);	
}			

		else if(iv<=tv){
float tt=(t*1000)/(tv-iv);			
	while(iv<tv+1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, iv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, iv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, iv);
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, iv);
		
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, iv);

		HAL_Delay(tt);
		iv++;
	}			
}			

else{		
	float tt=(t*1000)/(iv-tv);
	while(iv>tv-1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, iv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, iv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, iv);
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, iv);
		
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, iv);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, iv);

		HAL_Delay(tt);
		iv--;
	}
}
				i=0;
				position=0;		
			}	
			HAL_UART_Receive_IT(&huart2,data,sizeof(data));
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
