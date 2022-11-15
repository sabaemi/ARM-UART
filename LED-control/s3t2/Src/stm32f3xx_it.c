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
int position = 0;
extern int count;
extern int on;
extern int led;
extern int r;
//extern unsigned char data[1];
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
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
if(i<=count){
	if(on==1){
		if(led==1){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,1);
		}
		else if(led==2){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,1);
		}
		else if(led==3){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,1);
		}
		else if(led==4){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,1);
		}
		else if(led==5){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,1);
		}
		else if(led==6){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,1);
		}
		else if(led==7){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
		}
		else if(led==8){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,1);
		}
	}
	else if(on==0){
		if(led==1){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,0);
		}
		else if(led==2){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,0);
		}
		else if(led==3){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,0);
		}
		else if(led==4){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,0);
		}
		else if(led==5){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,0);
		}
		else if(led==6){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,0);
		}
		else if(led==7){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,0);
		}
		else if(led==8){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,0);
		}
	}
}
i++;
if(r==1){
	if(led==1)
		led=9;
	led--;
}
else if(r==0){
	if(led==8)
		led=0;
	led++;
}
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
extern unsigned char data[1];
extern unsigned char buffer[100];
		HAL_TIM_Base_Stop_IT(&htim4);
		HAL_UART_Receive_IT(&huart2,data,sizeof(data));
	
			if(data[0] != '\r'){
					buffer[position] = data[0];
					buffer[position+1] = '\0';
				  position++;
			}
			else if(data[0] == '\r'){
				if(buffer[0]=='1')led=1;
				if(buffer[0]=='2')led=2;
				if(buffer[0]=='3')led=3;
				if(buffer[0]=='4')led=4;
				if(buffer[0]=='5')led=5;
				if(buffer[0]=='6')led=6;
				if(buffer[0]=='7')led=7;
				if(buffer[0]=='8')led=8;
				   if(buffer[2]=='l'){
						 r=0;
					 }
					 else if(buffer[2]=='r'){
						 r=1;
					 }
				if(buffer[6]==' '){
					
				if(buffer[7]=='1')count=1;
				if(buffer[7]=='2')count=2;
				if(buffer[7]=='3')count=3;
				if(buffer[7]=='4')count=4;
				if(buffer[7]=='5')count=5;
				if(buffer[7]=='6')count=6;
				if(buffer[7]=='7')count=7;
				if(buffer[7]=='8')count=8;
					if(buffer[8]==' '){
						if(buffer[10]=='n')
							on=1;
						if(buffer[10]=='f')
							on=0;
						}
				}
				if(buffer[7]==' '){
					
				if(buffer[8]=='1')count=1;
				if(buffer[8]=='2')count=2;
				if(buffer[8]=='3')count=3;
				if(buffer[8]=='4')count=4;
				if(buffer[8]=='5')count=5;
				if(buffer[8]=='6')count=6;
				if(buffer[8]=='7')count=7;
				if(buffer[8]=='8')count=8;
					
					if(buffer[9]==' '){
						if(buffer[11]=='n')
							on=1;
						if(buffer[11]=='f')
							on=0;
						}
				}
				i=0;
				position=0;
				HAL_TIM_Base_Start_IT(&htim4);			
			}	
			HAL_UART_Receive_IT(&huart2,data,sizeof(data));

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
