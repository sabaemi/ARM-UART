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
#include "main.h"
extern unsigned char data[8];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
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
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	if(x%2==0)
HAL_TIM_Base_Start_IT(&htim4);
	if(x%2==1){

		//HAL_UART_IRQHandler(&huart2);
		//extern unsigned char data[8];
		if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)){
			unsigned char data[8]="1 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)){
			unsigned char data[8]="2 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)){
			unsigned char data[8]="3 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)){
			unsigned char data[8]="4 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)){
			unsigned char data[8]="5 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)){
			unsigned char data[8]="6 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)){
			unsigned char data[8]="7 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)){
			unsigned char data[8]="8 \n";
		  HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		}
		//HAL_UART_Transmit(&huart2,data,sizeof(data),1000);
		HAL_TIM_Base_Stop_IT(&htim4);
	}
	x++;
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
		if(counter%8==0){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15))
				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15);
	}
		if(counter%8==1){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
	}
		if(counter%8==2){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
	}
		if(counter%8==3){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
	}
		if(counter%8==4){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12);
	}
		if(counter%8==5){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13);
	}
		if(counter%8==6){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
	}
		if(counter%8==7){
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15);
	}
		counter++;
  /* USER CODE END TIM4_IRQn 1 */
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
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
