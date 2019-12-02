/*****************************************************************************
 *   uart.c:  UART API file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.12.07  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#include "../main.h"
#include "LPC15xx.h"
#include "uart.h"
#include "../gpio.h"
//#include "uart_15xx.h"


// CodeRed - change for CMSIS 1.3
#define SystemFrequency SystemCoreClock

volatile uint8_t UARTTxEmpty0 = 1;
volatile uint8_t UARTTxEmpty1 = 1;
volatile uint8_t UARTTxEmpty2 = 1;
volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t UARTBuffer1[BUFSIZE];
volatile uint8_t UARTBuffer2[BUFSIZE];
volatile int rxTimeout0;
volatile int rxTimeout1;
volatile int rxTimeout2;
volatile uint32_t UARTCount0 = 0;
volatile uint32_t UARTtxCount0;
volatile uint32_t UARTtxCount1;
volatile uint32_t UARTtxCount2;
         uint8_t *BufferTXPtr0;  
volatile uint8_t *BufferTXPtr1;  
volatile uint8_t *BufferTXPtr2;  
volatile uint32_t UARTCount1 = 0;
volatile uint32_t UARTCount2 = 0;
volatile uint32_t UARTCount3 = 0;
volatile uint8_t ModbusState0;
volatile uint8_t ModbusState1;
volatile uint8_t ModbusState2;
uint16_t flow_ctrl_hangup_timer = 0; 
unsigned char uartMode;

/*****************************************************************************
** Function name:   UART_IRQHandler
**
** Descriptions:    UART interrupt handler
**
** parameters:      None
** Returned value:    None
** 
*****************************************************************************/
// RS485
void UART0_IRQHandler(void)
{
  uint32_t intstat = LPC_USART0->INTSTAT;
  uartMode = UART_MODE_RS485;

  // Rx data
  if (intstat & UART_STAT_RXRDY) {

    LPC_GPIO_PORT->CLR[RS485_RTS_PORT] |= (1 << RS485_RTS_PIN);

    if (UARTCount0 == 0)
      rxTimeout0 = 0;

    unsigned char rxData = LPC_USART0->RXDATA;
    if (!(ModbusState0 & MODBUS_DISCARD_PACKET)) {

      UARTBuffer0[UARTCount0] = rxData;
      UARTCount0++;

      if ((LPC_USART0->STAT >> 8) & 1 || (LPC_USART0->STAT >> 13) & 1 || (LPC_USART0->STAT >> 14) & 1 ||
       (LPC_USART0->STAT >> 15) & 1 || (LPC_USART0->STAT >> 16) & 1) {
        ModbusState0 |= MODBUS_DISCARD_PACKET;
      }
      else {
        ModbusState0 |= MODBUS_PACKET_RECIVED;
        rxTimeout0 = 0;
      }
    }
  }


  // Tx ready interrupt
  else if(intstat & UART_STAT_TXRDY) {

    while (~LPC_USART0->STAT & UART_STAT_TXIDLE);
    if(UARTtxCount0){ //character finished transmitting, load next character

      LPC_USART0->TXDATA = *(++BufferTXPtr0);
      UARTtxCount0 --;
    }
    else{   //packet tx finished, clear flow control

      LPC_GPIO_PORT->CLR[RS485_RTS_PORT] |= (1 << RS485_RTS_PIN);
      LPC_USART0->INTENSET |= (1 << 0);       //enable rx interrupt
      LPC_USART0->INTENCLR = UART_STAT_TXRDY; //clear tx interrupt

    }
  }

}
/*****************************************************************************/

void rs485_RTS_timeout(void){
  if(LPC_GPIO_PORT->PIN[RS485_RTS_PORT] & (1 << RS485_RTS_PIN)){  //  RTS - prevent hang-up
      flow_ctrl_hangup_timer++;
      if(flow_ctrl_hangup_timer > 1000){
        flow_ctrl_hangup_timer = 0;
        LPC_GPIO_PORT->CLR[RS485_RTS_PORT] |= (1 << RS485_RTS_PIN);  // clear RTS -> prevent hang-up
      }
    }
    else
      flow_ctrl_hangup_timer = 0;
}

// XBEE
void UART1_IRQHandler(void)
{
  uint32_t intstat = LPC_USART1->INTSTAT;

  uartMode = UART_MODE_XBEE;

  // Rx data
  if (intstat & UART_STAT_RXRDY) {

    if (UARTCount1 == 0)
      rxTimeout1 = 0;

    unsigned char rxData = LPC_USART1->RXDATA;
    if (!(ModbusState1 & MODBUS_DISCARD_PACKET)) {

      UARTBuffer1[UARTCount1] = rxData;
      UARTCount1++;

      if ((LPC_USART1->STAT >> 8) & 1 || (LPC_USART1->STAT >> 13) & 1 || (LPC_USART1->STAT >> 14) & 1 ||
       (LPC_USART1->STAT >> 15) & 1 || (LPC_USART1->STAT >> 16) & 1) {
        ModbusState1 |= MODBUS_DISCARD_PACKET;
      }
      else {
        ModbusState1 |= MODBUS_PACKET_RECIVED;
        rxTimeout1 = 0;
      }
    }
  }


  // Tx ready interrupt
  else if(intstat & UART_STAT_TXRDY) {

    while (~LPC_USART1->STAT & UART_STAT_TXIDLE);
    if(UARTtxCount1){ //character finished transmitting, load next character

      LPC_USART1->TXDATA = *(++BufferTXPtr1);
      UARTtxCount1 --;
    }
    else{   //packet tx finished, clear flow control

      LPC_GPIO_PORT->CLR[XBEE_RTS_PORT] |= (1 << XBEE_RTS_PIN);
      LPC_USART1->INTENSET |= (1 << 0);       //enable rx interrupt
      LPC_USART1->INTENCLR = UART_STAT_TXRDY; //clear tx interrupt

    }
  }

}

/*****************************************************************************/
void UART2_IRQHandler(void)
{
  uint32_t intstat = LPC_USART2->INTSTAT;

  // Rx data
  if (intstat & UART_STAT_RXRDY) {

    if (UARTCount2 == 0)
      rxTimeout2 = 0;

    unsigned char rxData = LPC_USART2->RXDATA;
    if (!(ModbusState2 & MODBUS_DISCARD_PACKET)) {

      UARTBuffer2[UARTCount2] = rxData;
      UARTCount2++;

      if ((LPC_USART2->STAT >> 8) & 1 || (LPC_USART2->STAT >> 13) & 1 || (LPC_USART2->STAT >> 14) & 1 ||
       (LPC_USART2->STAT >> 15) & 1 || (LPC_USART2->STAT >> 16) & 1) {
        ModbusState2 |= MODBUS_DISCARD_PACKET;
      }
      else {
        ModbusState2 |= MODBUS_PACKET_RECIVED;
        rxTimeout2 = 0;
      }
    }
  }

  // Tx ready interrupt
  else if(intstat & UART_STAT_TXRDY) {

    while (~LPC_USART2->STAT & UART_STAT_TXIDLE);
    if(UARTtxCount2){ //character finished transmitting, load next character

      LPC_USART2->TXDATA = *(++BufferTXPtr2);
      UARTtxCount2 --;
    }
    else{   //packet tx finished, clear flow control

      LPC_GPIO_PORT->CLR[RS485_RTS_PORT] |= (1 << RS485_RTS_PIN);
      LPC_USART2->INTENSET |= (1 << 0);       //enable rx interrupt
      LPC_USART2->INTENCLR = UART_STAT_TXRDY; //clear tx interrupt

    }
  }

  return;
}


/*****************************************************************************
** Function name:   UARTInit
**
** Descriptions:    Initialize UART0 port, setup pin select,
**        clock, parity, stop bits, FIFO, etc.
**
** parameters:      UART baudrate
** Returned value:    None
** 
*****************************************************************************/
void UART0Init(uint32_t baudrate)   //RS485
{
  uint32_t Fdiv;
  uint32_t regVal;

  UARTTxEmpty0 = 1;
  UARTCount0 = 0;
  
  NVIC_DisableIRQ(UART0_IRQn);

  NVIC_SetPriority(UART0_IRQn, 2); //mzp: bldc must be able to interrupt this routine

  LPC_SWM->PINASSIGN[0] &= ~(0xFF << 8);
  LPC_SWM->PINASSIGN[0] |= (RS485_RXD << 8);         // enable UART0_RXD_I
  LPC_SWM->PINASSIGN[0] &= ~(0xFF << 0);
  LPC_SWM->PINASSIGN[0] |= (RS485_TXD << 0);         // enable UART0_TXD_O

  LPC_GPIO_PORT->DIR[RS485_RTS_PORT] |= 1<<RS485_RTS_PIN;

  LPC_SYSCON->SYSAHBCLKCTRL1 |= (1 << 17);   // enable UART0 clock

  uint32_t reg;
  reg = LPC_USART0->CFG & ~((0x3 << 2) | (0x3 << 4) | (0x1 << 6));
  LPC_USART0->CFG = reg | ((0x01 << 2) | (0x02 << 4) | (0x00 << 6)); // 8-bit length, Even Parity, one stop bit

  LPC_USART0->CFG |= (1 << 20); // output enable select RS485

  switch (baudrate) {
    case 19200:
      LPC_USART0->BRG = 312;
      break;
    case 115200:
      LPC_USART0->BRG = 50;
      break;
  }

  LPC_USART0->CFG |= 0x01;              // UART0 enable
  LPC_USART0->CTRL &= ~(0x01 << 6);     // TX enable

  /* Enable the UART Interrupt */
 NVIC_EnableIRQ(UART0_IRQn);

  LPC_USART0->INTENSET |= (0x01 << 0);  // RX interrupt enable

  return;
}

/*****************************************************************************/
void UART1Init(uint32_t baudrate)   //ZigBee
{
  uint32_t Fdiv;
  uint32_t regVal;

  UARTTxEmpty1 = 1;
  UARTCount1 = 0;
  
  NVIC_DisableIRQ(UART1_IRQn);

  LPC_SWM->PINASSIGN[1] &= ~(0xFF << 16);
  LPC_SWM->PINASSIGN[1] |= (XBEE_RXD << 16);         // enable UART1_RXD_I
  LPC_SWM->PINASSIGN[1] &= ~(0xFF << 8);
  LPC_SWM->PINASSIGN[1] |= (XBEE_TXD << 8);          // enable UART1_TXD_O
  LPC_SWM->PINASSIGN[1] &= ~(0xFF << 24);
  LPC_SWM->PINASSIGN[1] |= (XBEE_RTS << 24);        // enable UART1_RTS
  LPC_SWM->PINASSIGN[2] &= ~(0xFF << 0);
  LPC_SWM->PINASSIGN[2] |= (XBEE_CTS << 0);          // enable UART1_CTS

  LPC_GPIO_PORT->DIR[XBEE_RTS_PORT] |= 1<<XBEE_RTS_PIN;

  LPC_SYSCON->SYSAHBCLKCTRL1 |= (1 << 18);   // enable UART1 clock
  
  uint32_t reg;
  reg = LPC_USART1->CFG & ~((0x3 << 2) | (0x3 << 4) | (0x1 << 6));
  LPC_USART1->CFG = reg | ((0x01 << 2) | (0x02 << 4) | (0x00 << 6)); // 8-bit length, Even Parity, one stop bit

  LPC_USART1->CFG |= (1 << 20); // output enable select RS485

  switch (baudrate) {
    case 19200:
      LPC_USART1->BRG = 312;
      break;
    case 115200:
      LPC_USART1->BRG = 51;
      break;
  }

  LPC_USART1->CFG |= 0x01;              // UART1 enable
  LPC_USART1->CTRL &= ~(0x01 << 6);     // TX enable

  /* Enable the UART Interrupt */
  NVIC_EnableIRQ(UART1_IRQn);

  LPC_USART1->INTENSET |= (0x01 << 0);  // RX interrupt enable

  return;
}

/*****************************************************************************/
void UART2Init(uint32_t baudrate)
{
//  UARTTxEmpty2 = 1;
  UARTCount2 = 0;
  
  NVIC_DisableIRQ(UART2_IRQn);

  LPC_SYSCON->UARTCLKDIV = 0x1;             // UART clock divided by 16
  LPC_SYSCON->FRGCTRL = 0XFF;               // DIV = 256 if MULT is used

  LPC_SWM->PINASSIGN[2] &= ~(0xFF << 16);
  LPC_SWM->PINASSIGN[2] |= (23 << 16);  // Tx
  LPC_SWM->PINASSIGN[2] &= ~(0xFF << 24);
  LPC_SWM->PINASSIGN[2] |= (22 << 24); // Rx

  LPC_SYSCON->SYSAHBCLKCTRL1 |= (1 << 19);   // enable UART2 clock
  
  uint32_t reg;
  reg = LPC_USART2->CFG & ~((0x3 << 2) | (0x3 << 4) | (0x1 << 6));
  LPC_USART2->CFG = reg | ((0x01 << 2) | (0x02 << 4) | (0x00 << 6)); // 8-bit length, Even Parity, one stop bit

  LPC_USART2->CFG |= (1 << 20); // output enable select RS485

  switch (baudrate) {
    case 19200:
      LPC_USART2->BRG = 312;
      break;
    case 115200:
      if(conv_mode == CONV_MODE_MASTER)
        LPC_USART2->BRG = 50;  //51 not working for sigma
      else 
        LPC_USART2->BRG = 51;
    break;
  }

  LPC_USART2->CFG |= 0x01;              // UART2 enable
  LPC_USART2->CTRL &= ~(0x01 << 6);     // TX enable

  // Enable the UART Interrupt
  NVIC_EnableIRQ(UART2_IRQn);

  LPC_USART2->INTENSET |= (0x01 << 0);  // RX interrupt enable

  return;
}

/*****************************************************************************
** Function name:   UARTSend
**
** Descriptions:    Send a block of data to the UART 0 port based
**        on the data length
**
** parameters:    buffer pointer, and data length
** Returned value:  None
** 
/*****************************************************************************/
void UART1Send(uint8_t *BufferPtr, uint32_t Length)
{
  LPC_GPIO_PORT->SET[XBEE_RTS_PORT] |= (1 << XBEE_RTS_PIN);  // set RTS
  LPC_USART1->INTENCLR |= (1 << 0);    //diasble rx ready interrupt

  while (!(LPC_USART1->STAT & UART_STAT_TXRDY));

  LPC_USART1->TXDATA = *BufferPtr;
  BufferTXPtr1 = BufferPtr;
  UARTtxCount1 = Length - 1;
  LPC_USART1->INTENSET |= (1 << 2);   //enable TX ready interrupt

  return;
}

#ifdef KVARK //************************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
  LPC_GPIO_PORT->SET[RS485_RTS_PORT] |= (1 << RS485_RTS_PIN);  // set RTS
  LPC_USART0->INTENCLR |= (1 << 0);    //diasble rx ready interrupt

  while (!(LPC_USART0->STAT & UART_STAT_TXRDY));

  LPC_USART0->TXDATA = *BufferPtr;
  BufferTXPtr0 = BufferPtr;
  UARTtxCount0 = Length - 1;
  LPC_USART0->INTENSET |= (1 << 2);   //enable TX ready interrupt

  return;
}
#else
#ifdef CONVERTER //********************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
  LPC_GPIO_PORT->SET[RS485_RTS_PORT] |= (1 << RS485_RTS_PIN);  // set RTS
  LPC_USART2->INTENCLR |= (1 << 0);    //diasble rx ready interrupt

  while (!(LPC_USART2->STAT & UART_STAT_TXRDY));

  LPC_USART2->TXDATA = *BufferPtr;
  BufferTXPtr2 = BufferPtr;
  UARTtxCount2 = Length - 1;
  LPC_USART2->INTENSET |= (1 << 2);   //enable TX ready interrupt

  return;
}
#endif //******************************************************************/
#endif

int modbus_newRequest()
{
  if (uartMode == UART_MODE_RS485)
    return (ModbusState0 & MODBUS_PACKET_RECIVED) ? 1:0;
  else if (uartMode == UART_MODE_XBEE)
    return (ModbusState1 & MODBUS_PACKET_RECIVED) ? 1:0;
}
int modbus_newRequest1()
{
  return (ModbusState1 & MODBUS_PACKET_RECIVED) ? 1:0;
}

int modbus_newRequest2()
{
  return (ModbusState2 & MODBUS_PACKET_RECIVED) ? 1:0;
}


int modbus_discard()
{
  if (uartMode == UART_MODE_RS485)
    return (ModbusState0 & MODBUS_DISCARD_PACKET) ? 1:0;
  else if (uartMode == UART_MODE_XBEE)
    return (ModbusState1 & MODBUS_DISCARD_PACKET) ? 1:0;
}

int modbus_discard1()
{
  return (ModbusState1 & MODBUS_DISCARD_PACKET) ? 1:0;
}


int modbus_discard2()
{
  return (ModbusState2 & MODBUS_DISCARD_PACKET) ? 1:0;
}

void modbus_ReqProcessed()
{
  if (uartMode == UART_MODE_RS485) {
    ModbusState0 &= MODBUS_CLEAR_MASK;
    UARTCount0 = 0;
  }
  else if (uartMode == UART_MODE_XBEE) {
    ModbusState1 &= MODBUS_CLEAR_MASK;
    UARTCount1 = 0;
  }

  //uartMode = UART_MODE_NONE;
}

void modbus_ReqProcessed1()
{
  ModbusState1 &= MODBUS_CLEAR_MASK;
  UARTCount1 = 0;
}

void modbus_ReqProcessed2()
{
  ModbusState2 &= MODBUS_CLEAR_MASK;
 // UARTCount2 = 0;
}


void UART0ClearStatus()
{
  LPC_USART0->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}

void UART1ClearStatus()
{
  LPC_USART1->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}


void UART2ClearStatus()
{
  LPC_USART2->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}

/******************************************************************************
**                            End Of File
******************************************************************************/
