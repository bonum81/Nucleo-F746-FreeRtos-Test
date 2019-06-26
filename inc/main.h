/*!****************************************************************************
 * @file			main.h
 * @author		IMS - Illarionov Maksim
 * @version		
 * @date		25.06.2019
 * @copyright	
 * @brief		
 */

#ifndef __MAIN
#define __MAIN


/*!****************************************************************************
 * Include
 */
#include <stdint.h>
 


/*!****************************************************************************
* User define
*/
#define 		LED_BLUE_ON						GPIOB->ODR = GPIO_BSRR_BS_7	
#define 		LED_RED_ON						GPIOB->ODR = GPIO_BSRR_BS_14				
#define 		LED_GREEN_ON					GPIOB->ODR = GPIO_BSRR_BS_0

#define 		LED_BLUE_OFF					GPIOB->ODR &= ~GPIO_BSRR_BS_7	
#define 		LED_RED_OFF						GPIOB->ODR &= ~GPIO_BSRR_BS_14				
#define 		LED_GREEN_OFF					GPIOB->ODR &= ~GPIO_BSRR_BS_0
/*!****************************************************************************
 * Typedefs
 */




/*!****************************************************************************
* External variables
*/



/*!****************************************************************************
* Prototypes for the functions
*/
void Init_RCC(void);
void Init_LEDs(void);
void Init_GPIO(void);
void Init_USART3(void);
void USART3_send_byte(char byte);
void USART3_send_srt(char * string);



void vTaskLedRed(void * pvParameters);
void vTaskLedBlue(void * pvParameters);















#endif
