#include "LPC15xx.h"
#include "../main.h"
#include "../gpio.h"
#include "SX1278.h"
#include <string.h>
#include "modbus.h"
#include "uart.h"

LoRa_t OriginalSettings;

volatile uint8_t transmission = 0;
volatile uint8_t lora_int_stat = 0;
uint8_t BindPacket[7];
uint8_t LoRa_id;

LoRa_t module;
LoRa_t original;

uint8_t LoRa_bindMode_slave;
uint8_t LoRa_channel_received;
uint8_t checkRouting;

uint8_t conv_mode = CONV_MODE_SLAVE;
uint8_t tx_settings_buffer[BUFSIZE];
uint8_t LoRa_route[165][MAX_ROUTE_HOPS];
uint8_t LoRa_bindMode_master = 0;
uint8_t rxBufferCopy[BUFSIZE];

void LoRa_TxPacket(uint8_t * txBuffer_, uint8_t length, uint32_t timeout) {

  uint8_t txBuffer[LoRa_MAX_PACKET];
  memcpy((uint8_t*)txBuffer, (uint8_t*)txBuffer_, length );
  uint8_t tmp[5];
  int8_t total_route_number = 0;
  uint8_t pos_id = txBuffer[0];


  if(pos_id > 165 && pos_id < 0xfe)  //invalid id
    return;
  if(length > 200)
    return;

  if (conv_mode == CONV_MODE_MASTER){       //get total_route_number  
    if(pos_id == 0xff && LoRa_route[0][0])
      pos_id = 0x00;                       
    module.routed = 0;
    for(volatile int i=0 ; i<MAX_ROUTE_HOPS; i++){     
        if(LoRa_route[pos_id][i]){
           total_route_number++;
           module.routed = 1;
        }else break;
    }
  }

  if (module.routed) { //Send or return packet through routes

    if (conv_mode == CONV_MODE_SLAVE)
      total_route_number = (rxBufferCopy[2] & 0xf0) >> 4;

    char routedBuffer[LoRa_MAX_PACKET];
    memcpy((char * ) routedBuffer + total_route_number + 3, (char * ) txBuffer, length);

    if (conv_mode == CONV_MODE_MASTER) { //send packet through routers (master)
      for(int i=3 ; i<3+total_route_number ; i++)
        routedBuffer[i] = LoRa_route[pos_id][i-3];
    } else {
      for (int i = 1; i < total_route_number; i++) //return packet through routers (slave)
        routedBuffer[i + 2] = rxBufferCopy[(total_route_number + 2) - i]; //switch router numbers
      routedBuffer[total_route_number + 2] = 100;
    }
    routedBuffer[0] = 0xFE;
    routedBuffer[1] = length;
    routedBuffer[2] = (total_route_number << 4) | 1; //route number
    length += 4 + total_route_number;
    routedBuffer[length - 1] = 0xFE;
    //if(slave_addr == 100)
    memcpy((char * ) txBuffer, (char * ) routedBuffer, length + 4 + total_route_number);
  }

  set_LED(BLUE, 1, 0); //Turn on BLUE TX LED

  LoRa_standby();

  LoRa_EntryTx(length, 8000);

  LoRa_clearIrq();
  tmp[0] = 0x0;
  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1); //RegFifoAddrPtr
  LoRa_SPIWrite(LR_RegFifoTxBaseAddr, tmp, 1);
  LoRa_SPIWrite(LR_RegFifoRxBaseAddr, tmp, 1);
  LoRa_SPIWrite(LR_RegPayloadLength, & length, 1);

  transmission = 1;

  LoRa_SPIWrite(0x00, txBuffer, length); //send first 4byte packet

  tmp[0] = 0x8b;
  LoRa_SPIWrite(LR_RegOpMode, tmp, 1); //Tx Mode
  LPC_PINT->IENR |= 1<<0; 

  //debug_printf("id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x \n" , txBuffer[0], txBuffer[1], txBuffer[30], txBuffer[31], txBuffer[32], txBuffer[33], txBuffer[34], txBuffer[35], txBuffer[8], txBuffer[9], txBuffer[10]);

  return ;

}

void PIN_INT0_IRQHandler() {

  LPC_PINT->IENR &= ~(1<<0);//disable loRa interrupt
  LPC_PINT->IST = 1<<0; //clear edge interrupt

  if (transmission){
    LoRa_clearIrq();
    lora_int_stat = TRANSMISSION_FINISHED;
    transmission = 0;
  }
  else{ 
    lora_int_stat = PACKET_RECEIVED;
  }
}  

void tx_finished(void){

    //for(volatile int i=0 ; i<50000 ; i++);  //this somehow prevents false packets   +  version number is not received...
    set_LED(BLUE, 0, 0); ///Turn off BLUE TX LED
    LoRa_EntryRx(LoRa_MAX_PACKET, 8000);
    transmission = 0;
    lora_int_stat = 0;
    return;
}

void rx_finished(void){ 
  uint8_t tmp[10];
  lora_int_stat = 0;
  LoRa_channel_received = 0;
  set_LED(GREEN, 1, 200); //set led for 200ms

  bool msg_ok = true;   
  // Bad message if CRC info missing.   
  LoRa_SPIRead(LR_RegHopChannel, tmp, 1);   
  if(!(tmp[0] & (1 << 6)))    
    msg_ok = false;

  // Check if PayloadCrcError (manual, pg. 111)   
  // Bad if CRC in error.   
  LoRa_SPIRead(LR_RegIrqFlags, tmp, 1);   
  if(tmp[0] & LoRa_IRQ_PayloadCrcError)   
    msg_ok = false;   
  LoRa_clearIrq();

  memset(module.rxBuffer, 0x00, LoRa_MAX_PACKET);

  spi_rx_fifo_clear();

  LoRa_SPIRead(LR_RegRxNbBytes, (uint8_t * ) & module.packetLength, 1); //Number of received bytes

  // If bad message, skip payload transfer    
  if(!msg_ok) {   
    return;   
  }

  LoRa_SPIRead(LR_RegFifoRxCurrentaddr, tmp, 1);
  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1);

  LoRa_SPIRead(0x00, module.rxBuffer, module.packetLength);

  checkRouting = 1;

  //if(conv_mode == 0)
  //debug_printf("id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x \n" , module.rxBuffer[0], module.rxBuffer[1], module.rxBuffer[2], module.rxBuffer[3], module.rxBuffer[4], module.rxBuffer[5], module.rxBuffer[6], module.rxBuffer[7], module.rxBuffer[8], module.rxBuffer[9], module.rxBuffer[10]);

  tmp[0] = 0x8D;
  LoRa_SPIWrite(LR_RegOpMode, tmp, 1); //Rx Mode  
  LPC_PINT->IENR |= 1<<0;

}

void check_routing(void) {
  checkRouting = 0;
  if (!(module.rxBuffer[0] == 0xFE)) { //No routing
    module.routed = 0;
    module.packetReady = 1;
    return;
  }

  int route_number = module.rxBuffer[2] & 0x0f;
  int total_route_number = (module.rxBuffer[2] & 0xf0) >> 4;
  module.packetLength = module.rxBuffer[1];

  if (module.rxBuffer[2 + route_number] != LoRa_id)
    return;

  if (route_number == total_route_number) { // on destination -> razpakiraj paket
    memcpy((char * ) rxBufferCopy, (char * ) module.rxBuffer, module.packetLength + 4 + total_route_number);
    memmove((char * ) module.rxBuffer, (char * ) module.rxBuffer + 3 + route_number, module.packetLength);
    module.routed = 1;
    module.packetReady = 1;
  }else { //forward to another router
    module.routed = 0;
    module.packetLength += 4 + total_route_number;
    route_number++;
    module.rxBuffer[2] = (total_route_number << 4) | route_number;

    set_tx_flag((char * ) module.rxBuffer, module.packetLength);
  }

}


void LoRa_SPIWrite(uint8_t addr, uint8_t *pcBuffer, uint8_t cNbBytes) {
  spi_rx_fifo_clear();

  __disable_irq();

  while (!(LPC_SPI0->STAT & (1 << 1))) ;
  LPC_SPI0->TXDATCTL = (addr | 0x80)  | (1 << 22) | (7 << 24) ; //Address + write command, SSEL0 EOT, RXIGNORE
  //uint8_t dump = LPC_SPI0->RXDAT;

  for (int i = cNbBytes; i >0 ; i--) {
    while (!(LPC_SPI0->STAT & (1 << 1)));
    
    if(i==1)  LPC_SPI0->TXCTL |=(1 << 20)|(1 << 21);
  
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
  LPC_SPI0->TXDATCTL = addr | (7 << 24) ; //Address , SSEL0, EOT, RXIGNORE
  while(!(LPC_SPI0->STAT & (1<<0))); //Wait to receive data
  uint8_t dump = LPC_SPI0->RXDAT;

  for (int i = cNbBytes; i >0 ; i--) {
    if(i==1)  LPC_SPI0->TXCTL |= (1 << 20)|(1 << 21);
   
    LPC_SPI0->TXDAT = 0x00 ;
    while (!(LPC_SPI0->STAT & (1 << 0))); //Wait to receive data  
    *pcBuffer = LPC_SPI0->RXDAT;
    pcBuffer++;
  }
  __enable_irq();
}

void LoRa_Interrupt(uint8_t enable_disable){
  if(enable_disable)
    NVIC_EnableIRQ(PIN_INT0_IRQn);
  else
    NVIC_DisableIRQ(PIN_INT0_IRQn);
}

void LoRa_standby(void) {
  uint8_t tmp = 0x89;
  LoRa_SPIWrite(LR_RegOpMode, & tmp, 1);
  module.status = STANDBY;
}

void LoRa_sleep(void) {
  char tmp = 0x88;
  LoRa_SPIWrite(LR_RegOpMode, & tmp, 1);
  module.status = SLEEP;
}

void LoRa_entry(void) {
  char tmp = 0x88;
  LoRa_SPIWrite(LR_RegOpMode, & tmp, 1);
}

void LoRa_clearIrq(void) {
  char tmp = 0xFF;
  LoRa_SPIWrite(LR_RegIrqFlags, & tmp, 1);
}

uint8_t LoRa_get_rssi(void){
  uint8_t rssi;
  LoRa_SPIRead(LR_RegPktRssiValue, & rssi, 1);
  return rssi;
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


////////////////////////////////SETUP/////////////////////////////////////

void LoRa_SPI_init(void) { //LPC1549

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

void LoRa_Bind_Mode(uint8_t mode) {

  if (!(LoRa_bindMode_master || LoRa_bindMode_slave)) { // 1st entry

    OriginalSettings.channel = module.channel;
    OriginalSettings.power = module.power;
    OriginalSettings.spFactor = module.spFactor;
    OriginalSettings.LoRa_BW = module.LoRa_BW;

    BindPacket[0] = 0; //slave addr = broadcast
    BindPacket[1] = INTCOM_LORA_SET_SETTINGS;
    BindPacket[2] = module.channel;
    BindPacket[3] = module.power | (module.spFactor << 4) | 0x04;
    BindPacket[4] = module.LoRa_BW;

    int crc_calc2 = modbus_crc((uint8_t * ) BindPacket, 5, CRC_NORMAL);
    BindPacket[5] = crc_calc2 & 0xFF;
    BindPacket[6] = crc_calc2 / 0x100;

  }

  if(mode == MASTER_BY_CHANNEL){
    LoRa_bindMode_master = 1;
    LoRa_bindMode_slave = 0;
    set_LED(BLUE, 1, 0);
    LoRa_config(module.channel, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);    
  }
 else if (mode == MASTER) {
    LoRa_bindMode_master = 1;
    set_LED(BLUE, 1, 0);
    LoRa_config(0, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);
  } else if (mode == SLAVE) {
    LoRa_bindMode_slave = 1;
    set_LED(BLUE, 0, 0);
    set_LED(GREEN, 1, 0);
    LoRa_config(0, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);
  }else if (mode == DISABLE) {
    //for (volatile int i = 0; i < 5000000; i++);
    if (LoRa_bindMode_slave) {
      OriginalSettings.channel = module.channel;
      OriginalSettings.power = module.power;
      OriginalSettings.spFactor = module.spFactor;
      OriginalSettings.LoRa_BW = module.LoRa_BW;
      set_LED(BLUE, 1, 0);
      LoRa_bindMode_slave = 0;
      LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET, RxMode); //Set settings to slave
    } else {
      set_LED(BLUE, 0, 0);
      LoRa_bindMode_master = 0;
      LoRa_config(OriginalSettings.channel, OriginalSettings.power, OriginalSettings.spFactor, OriginalSettings.LoRa_BW, LoRa_MAX_PACKET, RxMode); //set settings to master
      if(conv_mode == CONV_MODE_MASTER){
        memset(tx_settings_buffer, 0x00, 10);
      }
    }
  }else if(mode == BY_CHANNEL){
    LoRa_bindMode_slave = 1;
    set_LED(BLUE, 0, 0);
    set_LED(GREEN, 1, 0);
    OriginalSettings.spFactor = module.spFactor;
    OriginalSettings.LoRa_BW = module.LoRa_BW;
    LoRa_config(module.channel, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);
    module.spFactor = OriginalSettings.spFactor;
    module.LoRa_BW = OriginalSettings.LoRa_BW;    
  }
}

uint8_t LoRa_config(uint8_t channel, uint8_t power, uint8_t spFactor, uint8_t LoRa_BW, uint8_t packetLength, uint8_t mode) {
  uint8_t tmp[10];

  NVIC_DisableIRQ(PIN_INT0_IRQn);

  set_settings_flag = 0;

  module.channel = channel;
  module.power = power;
  module.spFactor = spFactor;
  module.LoRa_BW = LoRa_BW;
  module.packetLength = packetLength;

  LoRa_sleep(); //Change modem mode Must in Sleep mode
  for (volatile int i = 0; i < 720000; i++); //delay 15ms

  LoRa_entry();
  //LoRa_SPIWrite(0x5904); //?? Change digital regulator from 1.6V to 1.47V: see errata note

  if (LoRa_BW == LoRa_BW_125KHZ)
    module.frequency = 0x6C4666 + 0xC00 * channel;

  else if (LoRa_BW == LoRa_BW_250KHZ)
    module.frequency = 0x6C4666 + 0x1800 * channel;

  else if (LoRa_BW == LoRa_BW_500KHZ)
    module.frequency = 0x6C4666 + 0x3000 * channel;

  else if (LoRa_BW == LoRa_BW_62_5KHZ)
    module.frequency = 0x6C4666 + 0x600 * channel;

  tmp[0] = module.frequency >> 16;
  tmp[1] = (module.frequency & 0xff00) >> 8;
  tmp[2] = module.frequency & 0xff;

  LoRa_SPIWrite(LR_RegFrMsb, tmp, 3); //setting  frequency parameter
  LoRa_SPIRead(LR_RegFrMsb, tmp, 3);
  if (!(tmp[0] == (module.frequency >> 16)))
    return NONE;

  //setting base parameter
  tmp[0] = LoRa_Power[power];
  LoRa_SPIWrite(LR_RegPaConfig, tmp, 1); //Setting output power parameter
  tmp[0] = 0x0B;
  LoRa_SPIWrite(LR_RegOcp, tmp, 1); //RegOcp,Close Ocp
  tmp[0] = 0x23;
  LoRa_SPIWrite(LR_RegLna, tmp, 1); //RegLNA,High & LNA Enable
  tmp[0] = 0x04;
  LoRa_SPIWrite(LR_RegModemConfig3, tmp, 1);
  tmp[0] = (LoRa_LoRaBandwidth[LoRa_BW] << 4) + (LoRa_CR << 1) + 0x00;
  LoRa_SPIWrite(LR_RegModemConfig1, tmp, 1); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
  tmp[0] = (spFactor << 4) + (LoRa_CRC << 2) + 0x03;
  LoRa_SPIWrite(LR_RegModemConfig2, tmp, 1); //SFactor &  LNA gain set by the internal AGC loop
  tmp[0] = 0xFF;
  LoRa_SPIWrite(LR_RegSymbTimeoutLsb, tmp, 1); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
  tmp[0] = 0x00;
  LoRa_SPIWrite(LR_RegPreambleMsb, tmp, 1); //RegPreambleMsb
  tmp[0] = 8;
  LoRa_SPIWrite(LR_RegPreambleLsb, tmp, 1); //RegPreambleLsb 8+4=12byte Preamble
  tmp[0] = 0x01;
  LoRa_SPIWrite(REG_LR_DIOMAPPING2, tmp, 1); //RegDioMapping2 DIO5=00, DIO4=01
  module.readBytes = 0;

  tmp[0] = 0xff;

  LoRa_SPIWrite(LR_RegMaxPayloadLength, tmp, 1);

  LoRa_standby(); //Entry standby mode

  NVIC_EnableIRQ(PIN_INT0_IRQn);

  if (mode)
    LoRa_EntryTx(packetLength, 2000);
  else LoRa_EntryRx(packetLength, 2000);

  return LORA;
}

int LoRa_EntryTx(uint8_t length, uint32_t timeout) {
  uint8_t addr;
  uint8_t tmp[10];

  module.packetLength = length;

  tmp[0] = 0x87;
  LoRa_SPIWrite(REG_LR_PADAC, tmp, 1); //Tx for 20dBm
  tmp[0] = 0x00;
  LoRa_SPIWrite(LR_RegHopPeriod, tmp, 1); //RegHopPeriod NO FHSS
  tmp[0] = 0x41;
  LoRa_SPIWrite(REG_LR_DIOMAPPING1, tmp, 1); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
  LoRa_clearIrq();
  tmp[0] = 0xF7;
  LoRa_SPIWrite(LR_RegIrqFlagsMask, tmp, 1); //Open TxDone interrupt
  LoRa_SPIWrite(LR_RegPayloadLength, & length, 1); //RegPayloadLength
  LoRa_SPIRead(LR_RegFifoTxBaseAddr, tmp, 1); //RegFiFoTxBaseAddr
  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1); //RegFifoAddrPtr

  while (1) {
    LoRa_SPIRead(LR_RegPayloadLength, tmp, 1);
    if (tmp[0] == length) {
      module.status = TX;
      return 1;
    }

    if (--timeout == 0) {
      //LoRa_reset();
      //LoRa_config(4, LoRa_POWER_20DBM, LoRa_SF_7, LoRa_BW_500KHZ, 14, TxMode);
      return 0;
    }
  }
}

int LoRa_EntryRx(uint8_t length, uint32_t timeout) {
  uint8_t addr;
  uint8_t tmp[10];

  module.packetLength = length;

  tmp[0] = 0x84;
  LoRa_SPIWrite(REG_LR_PADAC, tmp, 1); //Normal and RX
  tmp[0] = 0xFF;
  LoRa_SPIWrite(LR_RegHopPeriod, tmp, 1); //No FHSS
  tmp[0] = 0x01;
  LoRa_SPIWrite(REG_LR_DIOMAPPING1, tmp, 1); //DIO=00,DIO1=00,DIO2=00, DIO3=01
  tmp[0] = 0x1F;
  LoRa_SPIWrite(LR_RegIrqFlagsMask, tmp, 1); //Open RxDone interrupt & Timeout & PayloadCrcError
  LoRa_clearIrq();
  LoRa_SPIWrite(LR_RegPayloadLength, & length, 1); //Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
  LoRa_SPIRead(LR_RegFifoRxBaseAddr, & addr, 1); //Read RxBaseAddr
  LoRa_SPIWrite(LR_RegFifoAddrPtr, & addr, 1); //RxBaseAddr->FiFoAddrPtr
  tmp[0] = 0x8D;
  LoRa_SPIWrite(LR_RegOpMode, tmp, 1); //Mode//Low Frequency Mode
  //LoRa_SPIWrite(LR_RegOpMode,0x05); //Continuous Rx Mode //High Frequency Mode
  module.readBytes = 0;

  while (1) {
    LoRa_SPIRead(LR_RegModemStat, tmp, 1);
    if ((tmp[0] & 0x04) == 0x04) { //Rx-on going RegModemStat
      module.status = RX;
      return 1;
    }
    if (--timeout == 0) {
      LoRa_reset();
      //LoRa_defaultConfig();
      return 0;
    }
    for (int i = 0; i < 48000; i++);
  }
}