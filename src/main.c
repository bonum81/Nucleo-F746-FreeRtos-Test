/*!****************************************************************************
 * @file				main.c
 * @author			IMS - Illarionov Maksim
 * @version		
 * @date				25.06.2019
 * @copyright	
 * @brief		
 */

/*!****************************************************************************
 * Include
 */

#include "stm32f7xx.h"                  
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

 /*!****************************************************************************
* Memory
*/


xSemaphoreHandle 	xSemaphore;						

char byte_rx[16];														// буфер для приема по USART3				
char byte_tx[16];														// буфер для отправки по USART3	
uint8_t uart3_rx_bit;												// счетчик для приема строк по USART3




/*!****************************************************************************
* Functions
*/


int main(void)
{
	
	Init_RCC();																																																			//  тактирование				
	Init_LEDs();																																																		//  инициализация ледов
	Init_USART3();																																																	//	инициализация USART3
	vSemaphoreCreateBinary (xSemaphore);																																									
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	xTaskCreate(vTaskLedRed, 			"vTaskLedRed" , 			configMINIMAL_STACK_SIZE, 		NULL, tskIDLE_PRIORITY+0, NULL);	
	xTaskCreate(vTaskLedBlue, 		"vTaskLedBlue" , 				configMINIMAL_STACK_SIZE, 		NULL, tskIDLE_PRIORITY+5, NULL);	
	
	vTaskStartScheduler();																																													//	старт планировщика
}



/*!****************************************************************************
* FreeRtos функции
*/

void vTaskLedRed(void * pvParameters)
{
	uint8_t i;
	while(1)
	{
			LED_RED_ON;
			vTaskDelay( 100 / portTICK_RATE_MS );
			LED_RED_OFF;
			vTaskDelay( 100 / portTICK_RATE_MS );
	}
	
	vTaskDelete(NULL);
}

void vTaskLedBlue(void * pvParameters)
{
	uint8_t i;
	double k = 36.00;
	char str[12];
	while(1)
	{
		xSemaphoreTake(xSemaphore, portMAX_DELAY);									// взятие семофора 
		for (i	=	0;	i	<	20;	i++)
		{
			LED_BLUE_ON;
			USART3_send_srt("vTaskLedBlue run.\r\n");
			vTaskDelay( 100 / portTICK_RATE_MS );
			LED_BLUE_OFF;
			vTaskDelay( 100 / portTICK_RATE_MS );
		}
	}
	
	vTaskDelete(NULL);
}


/*!****************************************************************************
* Функции инициализации переферии
*/

void Init_RCC(void)
{
	
	RCC->CR			|= RCC_CR_HSION;																							
	while (!(RCC->CR & RCC_CR_HSIRDY));
	RCC->CFGR		|=	RCC_CFGR_SW_HSI;
	RCC->CFGR		|= 	RCC_CFGR_SWS_HSI;
	RCC->CFGR		|= 	RCC_CFGR_HPRE_1;
	RCC->CFGR		|= 	RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR		|= 	RCC_CFGR_PPRE1_DIV1;
	
	FLASH->ACR 	|= 	FLASH_ACR_LATENCY_5WS;
}

void Init_LEDs(void)
{
	RCC->AHB1ENR 		|= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER 		|= GPIO_MODER_MODER7_0			| GPIO_MODER_MODER14_0 			| GPIO_MODER_MODER0_0;
	GPIOB->OSPEEDR 	|= GPIO_OSPEEDER_OSPEEDR7_0	| GPIO_OSPEEDER_OSPEEDR14_0 | GPIO_OSPEEDER_OSPEEDR0_0;
}



void Init_GPIO(void)
{
	RCC->AHB1ENR 		|= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER 		|= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER14_1;
	GPIOB->OSPEEDR 	|= GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR14_0;
	GPIOB->PUPDR 		|= GPIO_PUPDR_PUPDR7_1;
	GPIOB->BSRR 		|= GPIO_BSRR_BS_7;
}


void Init_USART3(void)
{
		RCC->APB1ENR	|=	RCC_APB1ENR_USART3EN;							//	Такты на USART3
		RCC->AHB1ENR	|=	RCC_AHB1ENR_GPIODEN;							// 	Такты на порт D
	
		//PD8 - TX
		GPIOD->MODER |= GPIO_MODER_MODER8_1;            		//	MODER 10 альтернативная функция вкл
		GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR8;               	//	PUPDR 00	
		GPIOD->OTYPER &= ~GPIO_OTYPER_OT_8;               	//	OTYPER 0 => PD8 - AF PP
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;      			//	OSPEED 11 
		GPIOD->AFR[1] |= 0x0000077;                       	//	AF8, AF9
	
		//PD9 - RX
		GPIOD->MODER |= GPIO_MODER_MODER9_1;               	//	MODER 10 альтернативная функция вкл
		GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR9;               	//	PUPDR 0 
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;   				//	OSPEED 11 
	
		USART3->BRR = 0x00000683;                          	// 	BRR = (f(APB1)+ 4800)/9600, f(APB1) == 18 MHz
		USART3->CR1 |= USART_CR1_UE;                      	//	Вкл USART3
		USART3->CR1 |= USART_CR1_TE;                       	//	Вкл передачи USART3
		USART3->CR1 |= USART_CR1_RE;                       	//	Вкл приема USART3
		USART3->CR1	|= USART_CR1_M_1;

		USART3->CR1 |= USART_CR1_RXNEIE;                  	//	RX прерывание по приему
		
		NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_send_byte (char byte)
{
   while(!(USART3->ISR & USART_ISR_TC));
   USART3->TDR = byte;
}

void USART3_send_srt(char * string)																																						// ф-ция отправки строк в usart3
{	
	 uint8_t i=0;
	 while(string[i])
	 {
		USART3_send_byte(string[i]);
		i++;
	 }
}


/*!****************************************************************************
* Обработчики прерываний
*/


void USART3_IRQHandler(void)
{
	char uart_data;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	 if (USART3->ISR & USART_ISR_RXNE)
		{
				USART3->TDR 					= USART3->RDR;
				uart_data 						= USART3->RDR;
				byte_rx[uart3_rx_bit]	=	USART3->RDR;
				uart3_rx_bit++;
				if(uart_data == '\r')
				{
					LED_RED_OFF;
					LED_BLUE_OFF;
					LED_GREEN_OFF;
					
					if(strcmp(byte_rx, "push\r") == 0)
					{
						xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);															// отдаем семафор
					}
					else
					{
						USART3_send_srt("\n"); 																																		// если незнаем команду, просто выводим обратно что получили
						USART3_send_srt("String: ");
						USART3_send_srt(byte_rx); 
					}
					
					memset(byte_rx, 0, sizeof(byte_rx));
					uart3_rx_bit	=	0;
					USART3_send_srt("\n");
				}
		}
}


