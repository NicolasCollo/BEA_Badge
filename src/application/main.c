/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"
#include "stm32l_discovery_lcd.h"
#include "discover_board.h"
#include "stm32l1xx_lcd.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);


#define DWINTERRUPT_EN (1)  //set to 1 when using DW interrupt, set to 0 to poll DW1000 IRQ line

#define SOFTWARE_VER_STRING    "Version 3.05    " //


#define SWS1_SRM_MODE 0x80  //slow ranging mode with PC - response is configurable and > 150 ms
#define SWS1_ANC_MODE 0x08  //anchor mode
#define SWS1_SHF_MODE 0x10	//short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE 0x20	// 64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE 0x40	//channel 5 mode (switch S1-7)

int instance_anchaddr = 0; //0 = 0xDECA020000000001; 1 = 0xDECA020000000002; 2 = 0xDECA020000000003
//NOTE: switches TA_SW1_7 and TA_SW1_8 are used to set tag/anchor address
int dr_mode = 0;
//if instance_mode = TAG_TDOA then the device cannot be selected as anchor
int instance_mode = ANCHOR;
//int instance_mode = TAG;
//int instance_mode = TAG_TDOA;
//int instance_mode = LISTENER;

uint8 s1switch = 0;
int chan, tagaddr, ancaddr;

#define LCD_BUFF_LEN (100)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];

int ranging = 0;
double max_range = 0;
int n = 1000; // Number defined to read the analog value after N cycles.

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};


uint32 inittestapplication(uint8 s1switch);

// Restart and re-configure
void restartinstance(void)
{
    instance_close() ;                          //shut down instance, PHY, SPI close, etc.

    spi_peripheral_init();                      //re initialise SPI...

    inittestapplication(s1switch) ;                     //re-initialise instance/device
} // end restartinstance()

//NOTE: Channel 5 is not supported for the non-discovery mode
int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_64M_MODE)
    {
        mode = mode + 2;
    }
    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 4;
    }

    return mode;
}

uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
    instanceConfig_t instConfig;
    int result;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT MP device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }


    if(s1switch & SWS1_ANC_MODE)
    {
        instance_mode = ANCHOR;
        //led_on(LED_PC6);
    }
    else
    {
        instance_mode = TAG;
        //led_on(LED_PC7);
    }

    instancesetrole(instance_mode) ;     // Set this instance role

    instance_init_s(instance_mode);
    dr_mode = decarangingmode(s1switch);

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;


    instance_config(&instConfig) ;                  // Set operating channel etc

    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

    instance_init_timings();

    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void process_dwRSTn_irq(void)
{
    instance_notify_DW1000_inIDLE(1);
}
void Delay(uint32_t nTime)
{
 // while (TSL_tim_CheckDelay_ms((TSL_tTick_ms_T) nTime, &last_tick_tsl) != TSL_STATUS_OK);
	uint32_t counter=nTime&0xffff;
	    TIM_Cmd(TIM2,ENABLE);
	    TIM_SetCounter(TIM2,counter);
	    while(counter>1)
	    {
	        counter=TIM_GetCounter(TIM2);
	    }
	    TIM_Cmd(TIM2,DISABLE);
}

void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 400;//40000 avant
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 500;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);
}


#if (DWINTERRUPT_EN == 1)
void process_deca_irq(void)
{
    do{

        instance_process_irq(0);

    }while(port_CheckEXT_IRQ() == 1); //while IRQ line active (ARM can only do edge sensitive interrupts)

}
#else
void process_deca_irq(void)
{
	while(dwt_checkIRQ() == 1)
	{
        instance_process_irq(0);
    } //while IRQ line active

}
#endif

void initLCD(void)
{
	LCD_InitTypeDef LCD_InitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;


	  LCD_InitStruct.LCD_Prescaler = LCD_Prescaler_1;
	  LCD_InitStruct.LCD_Divider = LCD_Divider_31;
	  LCD_InitStruct.LCD_Duty = LCD_Duty_1_4;
	  LCD_InitStruct.LCD_Bias = LCD_Bias_1_3;
	  LCD_InitStruct.LCD_VoltageSource = LCD_VoltageSource_Internal;


	  /* Initialize the LCD */
	  LCD_Init(&LCD_InitStruct);

	  LCD_MuxSegmentCmd(ENABLE);

	  /* To set contrast to mean value */
	  LCD_ContrastConfig(LCD_Contrast_Level_4);

	  LCD_DeadTimeConfig(LCD_DeadTime_0);
	  LCD_PulseOnDurationConfig(LCD_PulseOnDuration_4);

	  /* Wait Until the LCD FCR register is synchronized */
	  LCD_WaitForSynchro();

	  /* Enable LCD peripheral */
	  LCD_Cmd(ENABLE);

	  /* Wait Until the LCD is enabled */
	  while(LCD_GetFlagStatus(LCD_FLAG_ENS) == RESET)
	  {
	  }
	  /*!< Wait Until the LCD Booster is ready */
	  while(LCD_GetFlagStatus(LCD_FLAG_RDY) == RESET)
	  {
	  }

	  LCD_BlinkConfig(LCD_BlinkMode_Off,LCD_BlinkFrequency_Div32);
	  LCD_GLASS_Clear();

	  /* Configure Port A LCD Output pins as alternate function */
	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_15;
	  	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  	  GPIO_Init( GPIOA, &GPIO_InitStructure);

	  	/* Select LCD alternate function for Port A LCD Output pins */
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,GPIO_AF_LCD) ;

	  	  /* Configure Port B LCD Output pins as alternate function */
	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	  	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  	  GPIO_Init( GPIOB, &GPIO_InitStructure);

	  	  /* Select LCD alternate function for Port B LCD Output pins */
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_LCD) ;

	  	  /* Configure Port C LCD Output pins as alternate function */
	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 \
	  	                                 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 ;
	  	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  	  GPIO_Init( GPIOC, &GPIO_InitStructure);

	  	  /* Select LCD alternate function for Port B LCD Output pins */
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource0,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource2,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_LCD) ;
	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_LCD) ;
}

/*
 * @brief switch_mask  - bitmask of testing switches (currently 7 switches)
 *        switchbuff[] - switch name to test
 *        *switch_fn[]() - corresponded to switch test function
**/
#define switch_mask   (0x7F)

void setLCDline1(uint8 s1switch)
{
	uint8 command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);

	sprintf((char*)&dataseq[0], "DecaRanging  %02x", s1switch);
	writetoLCD( 40, 1, dataseq); //send some data

	sprintf((char*)&dataseq1[0], "                 ");
	writetoLCD( 16, 1, dataseq1); //send some data
}

#define ADC_CONVERT_RATIO      806    /* (3300mV / 4095)* 1000 input voltage variation from 0 to 3.3V, 12 bits*/
#define RANGE_MAX	30

double readADC(uint8 channel)
{
	uint16 ADC_Data = 0;
	uint32 VDDmV = 0;
	double range_max = 0;

	// Start the conversion
	ADC_SoftwareStartConv(ADC1);
 	// Wait until conversion completion
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	// Get the conversion value
 	ADC_Data = ADC_GetConversionValue(ADC1);
 	VDDmV = ((uint32)ADC_Data*(uint32)ADC_CONVERT_RATIO);
 	range_max = ((double)VDDmV * (double)RANGE_MAX)/((double)3300);

 	return range_max;
}

void enterLowPowerRunMode(void)
{
	/* Select the Voltage Range 2 (1.5V) */
	PWR_VoltageScalingConfig(PWR_VoltageScaling_Range2);

	 /* Wait Until the Voltage Regulator is ready */
	 while(PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET)
	 {
	 }
	 /* Enter RUN LP Mode */
	 PWR_EnterLowPowerRunMode(ENABLE);

	 /* Wait until the system enters RUN LP and the Regulator is in LP mode */
	 while(PWR_GetFlagStatus(PWR_FLAG_REGLP) == RESET)
	 {
	 }
}
/*
 * @fn      main()
 * @brief   main entry point
**/
int main(void)
{
    int i = 0;
    int toggle = 1;
    double range_result = 0;
    double avg_result = 0;
     // 1tag  0 anchor

    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    spi_peripheral_init();
    InitializeTimer();

//    Sleep(1000); //wait for LCD to power on

    initLCD();

    memset(dataseq, 0, LCD_BUFF_LEN);
    memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
    LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED); //send some data
    memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, 16); // Also set at line #26 (Should make this from single value !!!)
    LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED); //send some data

   // Sleep(1000);

    s1switch = 40; //code anchor mode 3 par défaut
  //  s1switch = 32; //code tag mode 3 par défaut

    port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device

    //test EVB1000 - used in EVK1000 production


    //run DecaRanging application

        uint8 dataseq[LCD_BUFF_LEN];
        uint8 command = 0x0;

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
        memset(dataseq, ' ', LCD_BUFF_LEN);
        memcpy(dataseq, (const uint8 *) "DECAWAVE  RANGE", 15);
        LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED);      //writetoLCD( 15, 1, dataseq); //send some data

        led_off(LED_ALL);

  /*      if(inittestapplication(s1switch) == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', LCD_BUFF_LEN);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            return 0; //error
        }
*/
        //sleep for 5 seconds displaying "Decawave" (LEDs only if in ANCHOR mode)

        if(instance_mode == TAG)
        {
        	enterLowPowerRunMode();
        }

        if(instance_mode == ANCHOR)
        {
        	i=1; // 30 normalement
        	while(i--)
        	{
        		if (i & 1) led_off(LED_ALL);
        		else    led_on(LED_ALL);


        	}
        }




        else
        {
        	i=30;
        	while(i--);// Less consumption in TAG mode // sleep avant
        }

        i = 0;
        led_off(LED_ALL);
        command = 0x2 ;  //return cursor home
        LCD_GLASS_Clear();//writetoLCD( 1, 0,  &command);

        memset(dataseq, ' ', LCD_BUFF_LEN);

        if(s1switch & SWS1_ANC_MODE)
        {
            instance_mode = ANCHOR;

            //led_on(LED_PC6);
        }
        else
        {
            instance_mode = TAG;
            //led_on(LED_PC7);
        }

        if(instance_mode == TAG)
        {
            memcpy(&dataseq[2], (const uint8 *) " TAG BLINK  ", 12);

            LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED); //send some data
            sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
            LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED); //send some data

        }
        else
        {
            memcpy(&dataseq[2], (const uint8 *) "  AWAITING  ", 12);
            LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED); //send some data
            memcpy(&dataseq[2], (const uint8 *) "    POLL    ", 12);
            LCD_GLASS_ScrollSentence(dataseq,1,SCROLL_SPEED); //send some data
        }

        command = 0x2 ;  //return cursor home
        LCD_GLASS_Clear();//writetoLCD( 1, 0,  &command);

#if (DWINTERRUPT_EN == 1)
    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting
#endif

    memset(dataseq, ' ', LCD_BUFF_LEN);
    memset(dataseq1, ' ', LCD_BUFF_LEN);

    // main loop
    while(1)
    {
#if (DWINTERRUPT_EN == 0)
    	process_deca_irq(); //poll DW1000 IRQ line when using polling of interrupt line
#endif

        instance_run();

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((instance_data[0].monitor == 1) && ((portGetTickCnt() - instance_data[0].timeofTx) > instance_data[0].finalReplyDelay_ms))
        {
			instance_data[0].wait4ack = 0;

			if(instance_mode == TAG)
			{
				inst_processrxtimeout(&instance_data[0]);
			}
			else //if(instance_mode == ANCHOR)
			{
				dwt_forcetrxoff();	//this will clear all events
				//enable the RX
				instance_data[0].testAppState = TA_RXE_WAIT ;
			}
			instance_data[0].monitor = 0;
        }

        if(instancenewrange())
        {
        	int n, l = 0, aaddr, taddr, txa, rxa, rng, rng_raw;
            ranging = 1;
            // Send the new range information to LCD and/or USB
            range_result = instance_get_idist();
            avg_result = instance_get_adist();

            // Calculate the maximum range every n cycles.
            if(instance_mode == ANCHOR)
            {
            	n--;
            	if(n == 0)
            	{
            		max_range = readADC(POT_ADC_CHANNEL);
            		n = 1000;
            	}
            }


            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  dataseq);

            memset(dataseq, ' ', LCD_BUFF_LEN);
            memset(dataseq1, ' ', LCD_BUFF_LEN);
            sprintf((char*)&dataseq[1], "LAST: %4.2f m", range_result);
            writetoLCD( 40, 1, dataseq); //send some data

            sprintf((char*)&dataseq1[1], "AVG8: %4.2f m", avg_result);

            writetoLCD( 16, 1, dataseq1); //send some data

            l = instance_get_lcount();
            aaddr = instancenewrangeancadd();
            taddr = instancenewrangetagadd();
            txa =  instancetxantdly();
            rxa =  instancerxantdly();
            rng = (int) (range_result*1000);
            rng_raw = (int) (instance_get_idistraw()*1000);

            if(instance_mode == TAG)
            {
            	if(range_result > max_range) // Out of max range
            	{
            		// Code pour faire s'endormir le µP
            	}
            	else // All the conditions matches, data can be sent
            	{
            		n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x t", aaddr, taddr, rng, rng_raw, l, txa, rxa);
            	}
            }
            else
            {
            	if(range_result > max_range) // Out of max range
            	{
                	led_on(LED_PB7); // Red LED means that the anchor is not linked with any tag
                	led_off(LED_PB6);
            		GPIO_WriteBit(DOOR_GPIO, DOOR_GPIO_PIN, Bit_RESET); // Door closed in this case
            	}
            	else
            	{
                	led_on(LED_PB6); // Green LED means that the anchor is linked with one tag
                	led_off(LED_PB7);
            		GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET); // Door opened in this case
            		n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x a", aaddr, taddr, rng, rng_raw, l, txa, rxa);
            	}
            }


        }

        if(ranging == 0)
        {
            if(instance_mode != ANCHOR)
            {
                if(instancesleeping())
                {
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    if(toggle)
                    {
                        toggle = 0;
                        memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
                        writetoLCD( 40, 1, dataseq); //send some data
                        memcpy(&dataseq[0], (const uint8 *) "    RESPONSE    ", 16);
                        writetoLCD( 16, 1, dataseq); //send some data
                    }
                    else
                    {
                        toggle = 1;
                        memcpy(&dataseq[2], (const uint8 *) "   TAG BLINK    ", 16);

                        writetoLCD( 40, 1, dataseq); //send some data
                        sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                        writetoLCD( 16, 1, dataseq); //send some data
                    }
                }

                if(instanceanchorwaiting() == 2)
                {
                    ranging = 1;
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
                    writetoLCD( 40, 1, dataseq); //send some data
                    sprintf((char*)&dataseq[0], "%016llX", instance_get_anchaddr());
                    writetoLCD( 16, 1, dataseq); //send some data
                }
            }
            else //if(instance_mode == ANCHOR)
            {
            	led_on(LED_PB7); // Red LED means that the anchor is not linked with any tag
            	led_off(LED_PB6);
        		GPIO_WriteBit(DOOR_GPIO, DOOR_GPIO_PIN, Bit_RESET); // Door closed in this case

                if(instanceanchorwaiting())
                {
                    toggle+=2;

                    if(toggle > 300000)
                    {
                        dataseq[0] = 0x2 ;  //return cursor home
                        writetoLCD( 1, 0,  dataseq);
                        if(toggle & 0x1)
                        {
                            toggle = 0;
                            memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
                            writetoLCD( 40, 1, dataseq); //send some data
                            memcpy(&dataseq[0], (const uint8 *) "      POLL      ", 16);
                            writetoLCD( 16, 1, dataseq); //send some data
                        }
                        else
                        {
                            toggle = 1;
                            memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", 16);
                            writetoLCD( 40, 1, dataseq); //send some data
                            sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                            writetoLCD( 16, 1, dataseq); //send some data
                        }
                    }

                }
                else if(instanceanchorwaiting() == 2)
                {
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
                    writetoLCD( 40, 1, dataseq); //send some data
                    sprintf((char*)&dataseq[0], "%llX", instance_get_tagaddr());
                    writetoLCD( 16, 1, dataseq); //send some data
                }
            }
        }

    // Reset of tag list in memory if BP reset held 10 s
    int k=2500;
    while(is_button_reset_on() == 1)
    	{
    		k--;

    		if(k % 124 == 0 && k % 248 !=0) // Alternate blink from LEDs during the reset phase
    		{
            	led_on(LED_PB7);
            	led_off(LED_PB6);
    		}
    		else if(k % 124 == 0 && k % 248 ==0)
    		{
            	led_on(LED_PB6);
            	led_off(LED_PB7);
    		}

    		dataseq[0] = 0x2 ;  //return cursor home
    		writetoLCD( 1, 0,  &dataseq[0]);
    		memset(dataseq, ' ', LCD_BUFF_LEN);
    		memcpy(dataseq, (const uint8 *) " RESET MEM ", 12);
    		writetoLCD( 40, 1, dataseq); //send some data
    		memcpy(dataseq, (const uint8 *) "  EN COURS  ", 12);
    		writetoLCD( 40, 1, dataseq); //send some data
    		if(k == 0)
        	{
    			instcleartaglist();

    			dataseq[0] = 0x2 ;  //return cursor home
    			writetoLCD( 1, 0,  &dataseq[0]);
    			memset(dataseq, ' ', LCD_BUFF_LEN);
    			memcpy(dataseq, (const uint8 *) " RESET MEM ", 12);
    			writetoLCD( 40, 1, dataseq); //send some data
    			memcpy(dataseq, (const uint8 *) "    DONE    ", 12);
    			writetoLCD( 40, 1, dataseq); //send some data

    			i=20; // Sleep few seconds displaying that the reset is done
    			while(i--)
    			{
    				sleep(200);
    			}
        	}
    	}


    return 0;
}
}

