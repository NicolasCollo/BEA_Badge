/*! ----------------------------------------------------------------------------
 * @file	port.c
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

#include "compiler.h"
#include "port.h"

#define rcc_init(x)					RCC_Configuration(x)
#define systick_init(x)				SysTick_Configuration(x)
#define rtc_init(x)					RTC_Configuration(x)
#define interrupt_init(x)			NVIC_Configuration(x)
#define usart_init(x)				USART_Configuration(x)
#define spi_init(x)					SPI_Configuration(x)
#define gpio_init(x)				GPIO_Configuration(x)
#define ethernet_init(x)			No_Configuration(x)
#define fs_init(x)					No_Configuration(x)
#define usb_init(x)					No_Configuration(x)
#define lcd_init(x)					No_Configuration(x)
#define touch_screen_init(x)		No_Configuration(x)
#define adc_init(x)					ADC_Configuration(x)

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

int No_Configuration(void)
{
	return -1;
}

unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

int SysTick_Configuration(void)
{
	if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);

	return 0;
}

void RTC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC */
	PWR_RTCAccessCmd(ENABLE);

	/* LSI used as RTC source clock */
	/* The RTC Clock may varies due to LSI frequency dispersion. */
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	//EXTI configuration ******************************************************
	EXTI_ClearITPendingBit(EXTI_Line20);
	EXTI_InitStructure.EXTI_Line = EXTI_Line20;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable the RTC Wakeup Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	RTC_SetWakeUpCounter(0x0);

	/* Enable the RTC Wakeup Interrupt */
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	/* Enable Wakeup Counter */
	RTC_WakeUpCmd(ENABLE);
	PWR_RTCAccessCmd(ENABLE);
}


int NVIC_DisableDECAIRQ(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_NOIRQ;
	EXTI_Init(&EXTI_InitStructure);

	return 0;
}


int NVIC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable GPIO used as DECA IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = DECAIRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_DOWN;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
	GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);

	/* Connect EXTI Line to GPIO Pin */
	SYSCFG_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);
	//GPIO_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DECAIRQ_EXTI_USEIRQ;

	NVIC_Init(&NVIC_InitStructure);

	//Enable the RTC Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	return 0;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));

  enablestatus =  EXTI->IMR & EXTI_Line;
  if (enablestatus != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

int RCC_Configuration(void)
{
	RCC_ClocksTypeDef RCC_ClockFreq;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable LSI */
	RCC_LSICmd(ENABLE);

	/* Enable HSI */
	RCC_HSICmd(ENABLE);

	/* Wait until HSI oscillator is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);


	/* Enable MSI */
	//RCC_MSICmd(ENABLE);


	/* Choix de la valeur de MSI 65 kHz car max 128 KHz en LowPowerRun*/
	//RCC_MSIRangeConfig(RCC_MSIRange_6);
	//RCC_MSIRangeConfig(RCC_MSIRange_6);


	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd(ENABLE);

	/* Select MSI as system clock source */
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_MSI);

	/* Wait till MSI is used as system clock source */
	//while (RCC_GetSYSCLKSource() != 0x00){}

	RCC_GetClocksFreq(&RCC_ClockFreq);

	/****************************************************************/
	/* HSI = up to 16MHz,
	HCLK=32kHz, PCLK2=32kHz, PCLK1=32kHz 						*/
	/****************************************************************/
	/* Flash 2 wait state */
	FLASH_SetLatency(FLASH_Latency_1);
	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div2);
	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);
	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div1);
	/*  ADCCLK = PCLK2/4 */
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div1;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* Configure PLL *********************************************************/
	/* PLL configuration: PLLCLK = (HSI / 4) * 8 = 32 MHz */
	RCC_PLLConfig(RCC_PLLSource_HSI, RCC_PLLMul_8, RCC_PLLDiv_4);

	/* Enable PLL */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{

	}
	/* Select MSI as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// wait till sys clock source is pll
		while (RCC_GetSYSCLKSource() != 12){}
		RCC_GetClocksFreq(&RCC_ClockFreq);
	/* Enable SPI1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable SPI2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable GPIOs clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD,ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP | RCC_APB1Periph_LCD | RCC_APB1Periph_PWR,ENABLE);

	return 0;
}

int USART_Configuration(void)
{
	return 0;
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

void SPI_ConfigFastRate(uint16_t scalingfactor)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_I2S_DeInit(SPIx);

	// SPIx Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx, &SPI_InitStructure);

	// Enable SPIx
	SPI_Cmd(SPIx, ENABLE);
}

int SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	SPI_I2S_DeInit(SPIx);

	// SPIx Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_PRESCALER;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx, &SPI_InitStructure);

/*	GPIO_InitStructure.GPIO_Pin = SPIx_SCK | SPIx_MOSI | SPIx_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); */

	// SPIx SCK pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_SCK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(SPIx_SCK_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(SPIx_SCK_GPIO,GPIO_PinSource5,GPIO_AF_SPI1); //SCLK

	// SPIx MOSI pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SPIx_MOSI_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(SPIx_MOSI_GPIO,GPIO_PinSource12,GPIO_AF_SPI1); //MOSI

	// SPIx MISO pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(SPIx_MISO_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(SPIx_MISO_GPIO,GPIO_PinSource11,GPIO_AF_SPI1); //MISO

	// SPIx CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);

	// Disable SPIx SS Output
	SPI_SSOutputCmd(SPIx, DISABLE);

	// Enable SPIx
	SPI_Cmd(SPIx, ENABLE);

	// Set CS high
	GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS);

    return 0;
}


int SPI2_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	SPI_I2S_DeInit(SPIy);

	// SPIy Mode setup
	//SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIy_PRESCALER;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIy, &SPI_InitStructure);

	// SPIy SCK and MOSI pin setup
	GPIO_InitStructure.GPIO_Pin = SPIy_SCK | SPIy_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	// SPIy MISO pin setup
	GPIO_InitStructure.GPIO_Pin = SPIy_MISO;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	// SPIy CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPIy_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(SPIy_CS_GPIO, &GPIO_InitStructure);

	// Disable SPIy SS Output
	SPI_SSOutputCmd(SPIy, DISABLE);

	// Enable SPIy
	SPI_Cmd(SPIy, ENABLE);

	// Set CS high
	GPIO_SetBits(SPIy_CS_GPIO, SPIy_CS);

	// LCD_RS pin setup
	GPIO_InitStructure.GPIO_Pin = LCD_RS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	// LCD_RW pin setup
	GPIO_InitStructure.GPIO_Pin = LCD_RW;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    return 0;
}

int GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure all unused GPIO port pins in Analog Input mode (floating input
	* trigger OFF), this will reduce the power consumption and increase the device
	* immunity against EMI/EMC */

	// Enable GPIOs clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |RCC_AHBPeriph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	// Set all GPIO pins as analog inputs (only PC13)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	//GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_Init(GPIOD, &GPIO_InitStructure);
	//GPIO_StructInit(&GPIO_InitStructure)

	// Enable GPIO used for LEDs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    return 0;
}

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	// Sleep(2); delay a remplacer .
}


void setup_DW1000RSTnIRQ(int enable)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	if(enable)
	{
		// Enable GPIO used as DECA IRQ for interrupt
		GPIO_InitStructure.GPIO_Pin = DECARSTIRQ;
		//GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPD;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
		GPIO_Init(DECARSTIRQ_GPIO, &GPIO_InitStructure);

		/* Connect EXTI Line to GPIO Pin */
		SYSCFG_EXTILineConfig(DECARSTIRQ_EXTI_PORT, DECARSTIRQ_EXTI_PIN);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MP IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

		/* Enable and set EXTI Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = DECARSTIRQ_EXTI_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		NVIC_Init(&NVIC_InitStructure);
	}
	else
	{
		//put the pin back to tri-state ... as input
		GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MP IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&EXTI_InitStructure);
	}
}


int ETH_GPIOConfigure(void)
{
	return 0;
}

int is_button_reset_on()
{
	int result = 1;

	if (GPIO_ReadInputDataBit(TAG_RESET_GPIO, TAG_RESET_GPIO_PIN))
		result = 0;

	return result;
}

void led_off (led_t led)
{
	switch (led)
	{
	case LED_PB6:
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		break;
	case LED_PB7:
		GPIO_ResetBits(GPIOB, GPIO_Pin_7);
		break;
	case LED_ALL:
		GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

void led_on (led_t led)
{
	switch (led)
	{
	case LED_PB6:
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		break;
	case LED_PB7:
		GPIO_SetBits(GPIOB, GPIO_Pin_7);
		break;
	case LED_ALL:
		GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}


#ifdef USART_SUPPORT

/**
  * @brief  Configures COM port.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void usartinit(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* USARTx configured as follow:
		  - BaudRate = 115200 baud
		  - Word Length = 8 Bits
		  - One Stop Bit
		  - No parity
		  - Hardware flow control disabled (RTS and CTS signals)
		  - Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200 ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//For EVB1000 -> USART2_REMAP = 0

	/* Enable the USART2 Pins Software Remapping */
	GPIO_PinRemapConfig(GPIO_Remap_USART2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);
}

void USART_putc(char c)
{
	//while(!(USART2->SR & 0x00000040));
	//USART_SendData(USART2,c);
	/* e.g. write a character to the USART */
	USART_SendData(USART2, c);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)	;
}

void USART_puts(const char *s)
{
	int i;
	for(i=0; s[i]!=0; i++)
	{
		USART_putc(s[i]);
	}
}

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
void printf2(const char *format, ...)
{
	va_list list;
	va_start(list, format);

	int len = vsnprintf(0, 0, format, list);
	char *s;

	s = (char *)malloc(len + 1);
	vsprintf(s, format, list);

	USART_puts(s);

	free(s);
	va_end(list);
	return;
}


#endif


int is_IRQ_enabled(void)
{
	return ((   NVIC->ISER[((uint32_t)(DECAIRQ_EXTI_IRQn) >> 5)]
	           & (uint32_t)0x01 << (DECAIRQ_EXTI_IRQn & (uint8_t)0x1F)  ) ? 1 : 0) ;
}

int peripherals_init (void)
{
	rcc_init();
	//rtc_init();
	gpio_init();
	systick_init();
	interrupt_init();
	//usart_init();
	//spi_init();
	//ethernet_init();
	//fs_init();
	//usb_init();
	//lcd_init();
	//touch_screen_init();

	return 0;
}

void spi_peripheral_init()
{
	spi_init(); // appel a la configuration du SPI DM1000

	//initialise SPI2 peripheral for LCD control
	// SPI2_Configuration(); // pas encore defini dans notre cas

	//port_LCD_RS_clear();

	//port_LCD_RW_clear();
}
