/*****************************************************************************
 *   gpio.c:  GPIO C file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.07.20  ver 1.00    Preliminary version, first Release
 *   2009.12.09  ver 1.05    Mod to use mask registers for GPIO writes + inlining (.h)
 *
*****************************************************************************/
//#include "LPC11xx.h"			/* LPC11xx Peripheral Registers */
#include "LPC15xx.h"
#include "main.h"
#include "../gpio.h"
#include "bldc.h"




uint16_t green_timeout = 0;
uint16_t blue_timeout = 0;


/*****************************************************************************
** Function name:		GPIOInit
**
** Descriptions:		Initialize GPIO, install the
**						GPIO interrupt handler
**
** parameters:			None
** Returned value:		true or false, return false if the VIC table
**						is full and GPIO interrupt handler can be
**						installed.
** 
*****************************************************************************/
void GPIOInit(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 13);  // enable IOCON clk register
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 12);  // enable SWN clk register
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 11) | (1 << 18); // enable clock to INMUX and PINT
  /* Enable AHB clock to the GPIO0..2 domain. */
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 14) | (1 << 15) | (1 << 16); // enable GPIO0..2 clock
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 11);  // enable MUX clock

  // enable output pins: Green led, Red led
  LPC_GPIO_PORT->DIR[LED_uC_GREEN_PORT] |= (1 << LED_uC_GREEN_PIN);
  LPC_GPIO_PORT->DIR[LED_uC_RED_PORT] |= (1 << LED_uC_RED_PIN);

  LPC_GPIO_PORT->DIR[1] |= (1<<19) | (1<<20); // hall voltage select outputs 0
  LPC_GPIO_PORT->DIR[1] |= (1<<7) | (1<<8); // hall voltage select outputs 1

  //end switch init, disable pullups
  LPC_GPIO_PORT->DIR[1] &= ~((1<<12) | (1<<13) | (1<<14) | (1<<15));
  LPC_IOCON->PIO[END_SW_0_LO_PORT][END_SW_0_LO_PIN] &= ~(3<<3);
  LPC_IOCON->PIO[END_SW_0_HI_PORT][END_SW_0_HI_PIN] &= ~(3<<3);
  #ifdef END_SW_1_LO_PORT
  LPC_IOCON->PIO[END_SW_1_LO_PORT][END_SW_1_LO_PIN] &= ~(3<<3);
  LPC_IOCON->PIO[END_SW_1_HI_PORT][END_SW_1_HI_PIN] &= ~(3<<3);
  #endif

  LPC_GPIO_PORT->DIR[1] |= (1<<21);  //voltage divider A 
  LPC_GPIO_PORT->DIR[1] |= (1<<22);  //voltage divider B 

  LPC_GPIO_PORT->DIR[1] |= (1<<17) | (1<<24); //output connector->output1, 2

  
//        LPC_SYSCON->CLKOUTSEL[1] = 0x01;
//        LPC_SYSCON->CLKOUTSEL[0] = 0x01;
//        LPC_SYSCON->CLKOUTDIV = 1;
//        LPC_SYSCON->USBCLKDIV = 3; //144MHz / 3 = 48MHz
//        LPC_SWM->PINASSIGN[13] = 57 << 8; clkout

  LPC_SWM->PINASSIGN[7] |= 0xff; // USB

  LPC_SWM->PINENABLE[1] &= ~(1 << 21);        // enable RESET PIN
  LPC_SWM->PINENABLE[1] &= ~(1 << 22);        // enable SWCLK PIN
  LPC_SWM->PINENABLE[1] &= ~(1 << 23);        // enable SWDIO PIN

  return;
}


uint32_t prevent_hang;
void set_LED(uint8_t color, uint8_t state, uint16_t timeout){

  if(color == RED){
    if(state)
      LPC_GPIO_PORT->SET[LED_uC_RED_PORT] |= (1 << LED_uC_RED_PIN);		
    else		
      LPC_GPIO_PORT->CLR[LED_uC_RED_PORT] |= (1 << LED_uC_RED_PIN);
  }
  else if(color == GREEN_uC){  
    if(state)
      LPC_GPIO_PORT->SET[LED_uC_GREEN_PORT] |= (1 << LED_uC_GREEN_PIN);		
    else		
      LPC_GPIO_PORT->CLR[LED_uC_GREEN_PORT] |= (1 << LED_uC_GREEN_PIN); 
  }

#ifdef LED_XBEE_BLUE_PIN

  if(transceiver == XBEE)
    return;
  timeout /= 10; //for Kvark timeout is 10X shorter than in Converter

  if(prevent_hang)
    if(!LoRa_channel_received)
      prevent_hang--;
  else
    LPC_GPIO_PORT->B[LED_XBEE_BLUE_PORT][LED_XBEE_BLUE_PIN] = 1;
  
  if(state == 1){
    if(color == GREEN){
      LPC_GPIO_PORT->B[LED_XBEE_GREEN_PORT][LED_XBEE_GREEN_PIN] = 0;
      green_timeout = timeout;
    }
    else if (color == BLUE){
      LPC_GPIO_PORT->B[LED_XBEE_BLUE_PORT][LED_XBEE_BLUE_PIN] = 0;
      blue_timeout = timeout;
      prevent_hang = 400000;
    } 

  }else{
    if(color == GREEN)
      if(green_timeout){
        green_timeout --;
      } else
        LPC_GPIO_PORT->B[LED_XBEE_GREEN_PORT][LED_XBEE_GREEN_PIN] = 1;

    if(color == BLUE)
      if(blue_timeout){
        blue_timeout --;
      } else
        LPC_GPIO_PORT->B[LED_XBEE_BLUE_PORT][LED_XBEE_BLUE_PIN] = 1;      

  }

#endif

}



/******************************************************************************
**                            End Of File
******************************************************************************/
