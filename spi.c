#include "LPC15xx.h"
#include "../main.h"
#include "../gpio.h"
#include "SX1278.h"
#include "spi.h"


void PIN_INT0_IRQHandler() { //LoRa external interrupt received

  LPC_PINT->IENR &= ~(1<<0);//disable loRa interrupt
  LPC_PINT->IST = 1<<0; //clear edge interrupt

  if (transmission){
    lora_int_stat = TRANSMISSION_FINISHED;
    transmission = 0;
  }
  else{ 
    lora_int_stat = PACKET_RECEIVED;
  }
}  

void LoRa_Interrupt(uint8_t enable_disable){
  if(enable_disable)
    NVIC_EnableIRQ(PIN_INT0_IRQn);
  else
    NVIC_DisableIRQ(PIN_INT0_IRQn);
}


void LoRa_SPI_init(void) { //SPI INIT LPC1549

  LPC_GPIO_PORT->DIR[LORA_RESET_PORT] &= ~(1<<LORA_RESET_PIN);  //Xbee reset High_Z:DEAsert ; Low:asert

  LPC_SWM->PINASSIGN[1] = 0xffffffff;

  LPC_SWM->PINASSIGN[4] &= ~(0xff);
  LPC_SWM->PINASSIGN[4] |= LORA_SS;         // SS

  LPC_SWM->PINASSIGN[3] &= ~(0xff << 8);
  LPC_SWM->PINASSIGN[3] |= LORA_SCK << 8;   // SCK

  LPC_SWM->PINASSIGN[3] &= ~(0xff << 16);
  LPC_SWM->PINASSIGN[3] |= LORA_MOSI << 16; //MOSI

  LPC_SWM->PINASSIGN[3] &= ~(0xff << 24);
  LPC_SWM->PINASSIGN[3] |= LORA_MISO << 24; //MISO


  LPC_GPIO_PORT->DIR[LORA_DIO0_PORT] &= ~(1 << LORA_DIO0_PIN); // DIO0

  LPC_SYSCON->SYSAHBCLKCTRL1 |= 1 << 9; //enable clock to SPI0
  LPC_SYSCON->PRESETCTRL1 &= ~(1 << 9); //De-assert reset to SPI0

  LPC_SPI0->TXCTL |= (1 << 16) | (7 << 24); // SSEL0, 8bit data transfer

  LPC_SPI0->CFG = (1 << 0) | (1 << 2); //SPI enable, master mode
 // LPC_SPI0->DLY =0xffff;

  LPC_SPI0->DIV = 12;

  LPC_GPIO_PORT->DIR[LORA_RESET_PORT] |= (1 << LORA_RESET_PIN);
  LPC_GPIO_PORT->B[LORA_RESET_PORT][LORA_RESET_PIN] |= 1; //Reset pin init


  // PINT IRQ

  LPC_INMUX->PINTSEL[0] = LORA_DIO0; //pin external interrupt

  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 11) | (1 << 18); // enable clock to INMUX and PINT

  LPC_PINT->IENR |= 1<<0; //Enable rising edge interrupt


  set_LED(BLUE, 0, 0);

  LoRa_reset();

}

void LoRa_SPIWrite(uint8_t addr, uint8_t *pcBuffer, uint8_t cNbBytes) {
  __disable_irq();
  while (!(LPC_SPI0->STAT & (1 << 1))) ;
  spi_rx_fifo_clear();
  LPC_SPI0->TXDATCTL = (addr | 0x80)  | (1 << 22) | (7 << 24) ; //Address + write command, RXIGNORE, data length


  for (int i = cNbBytes; i >0 ; i--) {
    while (!(LPC_SPI0->STAT & (1 << 1)));
    
    if(i==1)  LPC_SPI0->TXCTL |=(1 << 20)|(1 << 21); //EOT (deasert SSEL), EOF
  
    LPC_SPI0->TXDAT = *pcBuffer;
    pcBuffer++;
  }

  while (!(LPC_SPI0->STAT & (1 << 1)));

  __enable_irq();
}


uint8_t LoRa_SPIRead(uint8_t addr, uint8_t *pcBuffer, uint8_t cNbBytes) {
  
  spi_rx_fifo_clear();
 
  __disable_irq();
  while (!(LPC_SPI0->STAT & (1 << 1)));
  LPC_SPI0->TXDATCTL = addr | (7 << 24) ; //Address , data length
  while(!(LPC_SPI0->STAT & (1<<0))); //Wait to receive data
  uint8_t dump = LPC_SPI0->RXDAT;

  for (int i = cNbBytes; i >0 ; i--) {
    if(i==1)  LPC_SPI0->TXCTL |= (1 << 20)|(1 << 21); //EOT (deasert SSEL), EOF
   
    LPC_SPI0->TXDAT = 0x00 ;
    while (!(LPC_SPI0->STAT & (1 << 0))); //Wait to receive data  
    *pcBuffer = LPC_SPI0->RXDAT;
    pcBuffer++;
  }
  __enable_irq();
}

void LoRa_reset(void) {
  LPC_GPIO_PORT->B[LORA_RESET_PORT][LORA_RESET_PIN] = 0;
  for (int i = 0; i < 48000; i++);
  LPC_GPIO_PORT->B[LORA_RESET_PORT][LORA_RESET_PIN] = 1;
  for (int i = 0; i < 48000; i++);
}

void spi_rx_fifo_clear(void) {
  uint8_t dump;
  while (LPC_SPI0->STAT & (1 << 0))
    dump = LPC_SPI0->RXDAT;
}