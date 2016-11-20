/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "stm32l1xx.h"

#define USB_SUPPORT

//#define USART_SUPPORT

void printf2(const char *format, ...);

/*****************************************************************************************************************//*
 * To enable Direct Memory Access for SPI set this option to (1)
 * This option will increase speed of spi transactions but it will use an extra RAM memory buffer
 */
#define DMA_ENABLE	(0)

/*****************************************************************************************************************//*
**/
#if (DMA_ENABLE == 1)
 #define writetospi		writetospi_dma
 #define readfromspi	readfromspi_dma
 void dma_init(void);
#else

 extern int writetospi_serial( uint16_t headerLength,
 			   	    const uint8_t *headerBuffer,
 					uint32_t bodylength,
 					const uint8_t *bodyBuffer
 				  );

 extern int readfromspi_serial( uint16_t	headerLength,
 			    	 const uint8_t *headerBuffer,
 					 uint32_t readlength,
 					 uint8_t *readBuffer );

 #define writetospi		writetospi_serial
 #define readfromspi	readfromspi_serial
#endif

typedef enum
{
    LED_PB6,
    LED_PB7,
    LED_PC8,
    LED_PC9,
    LED_ALL,
    LEDn
} led_t;

#define SPIy_PRESCALER				SPI_BaudRatePrescaler_128

#define SPIy						SPI2   // ancien spi lcd
#define SPIy_GPIO					GPIOB
#define SPIy_CS						GPIO_Pin_12
#define SPIy_CS_GPIO				GPIOC
#define SPIy_SCK					GPIO_Pin_13
#define SPIy_MISO					GPIO_Pin_14
#define SPIy_MOSI					GPIO_Pin_15

#define LCD_RW						GPIO_Pin_10
#define LCD_RS						GPIO_Pin_11

#define SPIx_PRESCALER				SPI_BaudRatePrescaler_8

#define SPIx						SPI1   // spi dm1000
#define SPIx_GPIO					GPIOB
#define SPIx_CS						GPIO_Pin_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_Pin_5
#define SPIx_SCK_GPIO				GPIOA
#define SPIx_MISO					GPIO_Pin_11
#define SPIx_MISO_GPIO				GPIOB
#define SPIx_MOSI					GPIO_Pin_12
#define SPIx_MOSI_GPIO				GPIOB

#define DW1000_RSTn					GPIO_Pin_0
#define DW1000_RSTn_GPIO			GPIOA

#define DECARSTIRQ                  GPIO_Pin_0
#define DECARSTIRQ_GPIO             GPIOA
#define DECARSTIRQ_EXTI             EXTI_Line0
#define DECARSTIRQ_EXTI_PORT        EXTI_PortSourceGPIOA
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource0
#define DECARSTIRQ_EXTI_IRQn        EXTI0_IRQn

#define DECAIRQ                     GPIO_Pin_2
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI_Line2
#define DECAIRQ_EXTI_PORT           EXTI_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource2
#define DECAIRQ_EXTI_IRQn           EXTI2_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE

#define TA_BOOT1                 	GPIO_Pin_2
#define TA_BOOT1_GPIO            	GPIOB

#define TA_RESP_DLY                 GPIO_Pin_0
#define TA_RESP_DLY_GPIO            GPIOC

#define TAG_RESET_GPIO				GPIOA // GPIOC
#define TAG_RESET_GPIO_PIN			GPIO_Pin_14 // GPIO_Pin_11
#define DOOR_GPIO					GPIOC
#define DOOR_GPIO_PIN				GPIO_Pin_12
#define REGISTERING_GPIO			GPIOA // GPIOC
#define REGISTERING_GPIO_PIN		GPIO_Pin_13 // GPIO_Pin_10
#define POT_GPIO					GPIOC // Pris par le LCD , prendre PC4 plus tard, chanel 14
#define POT_GPIO_PIN				GPIO_Pin_0 //GPIO_Pin_4
#define POT_ADC_CHANNEL				ADC_Channel_10 //ADC_Channel_14

#define S1_SWITCH_ON  (1)
#define S1_SWITCH_OFF (0)
//when switch (S1) is 'on' the pin is low
int is_switch_on(uint16_t GPIOpin);

#define port_IS_TAG_pressed()		is_switch_on(TA_SW1_4)
#define port_IS_LONGDLY_pressed()	is_dlybutton_low()

#define USARTx						USART2

#define port_USARTx_busy_sending()	0 //(USART_GetFlagStatus((USARTx),(USART_FLAG_TXE))==(RESET))
#define port_USARTx_no_data()		0 //(USART_GetFlagStatus((USARTx),(USART_FLAG_RXNE))==(RESET))
#define port_USARTx_send_data(x)	  //USART_SendData((USARTx),(uint8_t)(x))
#define port_USARTx_receive_data()	0 //USART_ReceiveData(USARTx)

#define port_SPIx_busy_sending()		(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)			SPI_I2S_SendData((SPIx),(x))
#define port_SPIx_receive_data()		SPI_I2S_ReceiveData(SPIx)
#define port_SPIx_disable()				SPI_Cmd(SPIx,DISABLE)
#define port_SPIx_enable()              SPI_Cmd(SPIx,ENABLE)
#define port_SPIx_set_chip_select()		GPIO_SetBits(SPIx_CS_GPIO,SPIx_CS)
#define port_SPIx_clear_chip_select()	GPIO_ResetBits(SPIx_CS_GPIO,SPIx_CS)

#define port_SPIy_busy_sending()		(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIy_no_data()				(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIy_send_data(x)			SPI_I2S_SendData((SPIy),(x))
#define port_SPIy_receive_data()		SPI_I2S_ReceiveData(SPIy)
#define port_SPIy_disable()				SPI_Cmd(SPIy,DISABLE)
#define port_SPIy_enable()              SPI_Cmd(SPIy,ENABLE)
#define port_SPIy_set_chip_select()		GPIO_SetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_SPIy_clear_chip_select()	GPIO_ResetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_LCD_RS_set()				GPIO_SetBits(SPIy_GPIO,LCD_RS)
#define port_LCD_RS_clear()				GPIO_ResetBits(SPIy_GPIO,LCD_RS)
#define port_LCD_RW_set()				GPIO_SetBits(SPIy_GPIO,LCD_RW)
#define port_LCD_RW_clear()				GPIO_ResetBits(SPIy_GPIO,LCD_RW)

#define port_GET_stack_pointer()		__get_MSP()
#define port_GET_rtc_time()				RTC_GetCounter()
#define port_SET_rtc_time(x)			RTC_SetCounter(x)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

void __weak process_deca_irq(void);
//define LCD functions
#define port_LCD_Clear(x) 											0
#define port_LCD_SetBackColor(x) 									0
#define port_LCD_SetTextColor(x) 									0
#define port_LCD_DisplayString(line, column, font_index, buffer) 	0
int is_IRQ_enabled(void);

int is_button_low(uint16_t GPIOpin);
int is_button_reset_on(void);

#define is_button_high(x)			0
void led_on(led_t led);
void led_off(led_t led);
#define gpio_set(x)				0
#define gpio_reset(x)				0
#define is_gpio_out_low(x)			0
#define is_gpio_out_high(x)			0
			
int peripherals_init(void);
void spi_peripheral_init(void);

void SPI_ChangeRate(uint16_t scalingfactor);
void SPI_ConfigFastRate(uint16_t scalingfactor);

unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()

void reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);
void process_dwRSTn_irq(void) ;

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
