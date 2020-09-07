#include "SX1278.h"
#include <string.h>
#include "../gpio.h"
#include "../main.h"
#include "modbus.h"
#include "uart.h"
#include "spi.h"

//#include "LPC11xx.h"

LoRa_t OriginalSettings;

volatile uint8_t transmission = 0;
volatile uint8_t lora_int_stat = 0;
extern uint8_t tx_packet_buffer[BUFSIZE];
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
  uint8_t total_route_number = 0;
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

  LoRa_standby();
  set_LED(BLUE, 1, 20); //Turn on BLUE TX LED

  LoRa_EntryTx(length, 8000);

  tmp[0] = 0x0;
  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1); //RegFifoAddrPtr
  LoRa_SPIWrite(LR_RegPayloadLength, & length, 1);

  transmission = 1;

  LoRa_SPIWrite(0x00, txBuffer, length); //Send packet

  tmp[0] = 0x8b;
  LoRa_SPIWrite(LR_RegOpMode, tmp, 1); //Tx Mode

  return;

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
  set_LED(GREEN, 1, 40); //set led for 200ms

  bool msg_ok = true;     
  // Bad message if CRC info missing.     
  LoRa_SPIRead(LR_RegHopChannel, tmp, 1);     
  if(!(tmp[0] & (1 << 6)))      
    msg_ok = false;

  // Check if PayloadCrcError (manual, pg. 111)     
  // Bad if CRC in error.     
  LoRa_SPIRead(LR_RegIrqFlags, tmp, 1);  
  if(tmp[0] & (LoRa_IRQ_PayloadCrcError | LoRa_IRQ_RxTimeout) || !(tmp[0] & (LoRa_IRQ_ValidHeader | LoRa_IRQ_RxDone))  )  
    msg_ok = false;     
  LoRa_clearIrq();

  LoRa_SPIRead(LR_RegRxNbBytes, (uint8_t * ) & module.packetLength, 1); //Number of received bytes
 
  // If bad message, skip payload transfer      
  if(!msg_ok) {     
    return;     
  }

  LoRa_SPIRead(LR_RegFifoRxCurrentaddr, tmp, 1);
  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1);

  LoRa_SPIRead(0x00, module.rxBuffer, module.packetLength);
      
//      if(module.rxBuffer[0] == 0x10){
//        debug_printf("id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x" , module.rxBuffer[0], module.rxBuffer[1], module.rxBuffer[2], module.rxBuffer[3], module.rxBuffer[4], module.rxBuffer[5], module.rxBuffer[6], module.rxBuffer[7]);
//        debug_printf("  %d \n" , module.packetLength);
//      }

  checkRouting = 1;

  tmp[0] = 0x8D;
  LoRa_SPIWrite(LR_RegOpMode, tmp, 1); //Rx Mode  
  rxOffline_counter = 10000;

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

void set_tx_flag(uint8_t* tx_buffer, uint8_t length){
  uint32_t tx_timeout = 1000000;
  while(tx_buffered_flag && tx_timeout > 0){
    tx_timeout--;
    if(transmission == 0){
      LoRa_TxPacket((uint8_t *)tx_packet_buffer, tx_packet_length, 8000);
      tx_buffered_flag = 0;
    }
  }
  if(tx_buffered_flag == 0){
    tx_buffered_flag = 1;
    memcpy((char * ) tx_packet_buffer, (char * ) tx_buffer, length);
    tx_packet_length = length;
  }
}

void LoRa_standby(void) {
  uint8_t tmp = 0x89;
  LoRa_SPIWrite(LR_RegOpMode, &tmp, 1);
  module.status = STANDBY;
}

void LoRa_sleep(void) {
  char tmp = 0x88;
  LoRa_SPIWrite(LR_RegOpMode, &tmp, 1);
  module.status = SLEEP;
}

void LoRa_clearIrq(void) {
  char tmp = 0xFF;
  LoRa_SPIWrite(LR_RegIrqFlags, &tmp, 1);
}

uint8_t LoRa_get_rssi(void){
  uint8_t rssi;
  LoRa_SPIRead(LR_RegPktRssiValue, & rssi, 1);
  return rssi;
}          






////////////////////////////////SETUP/////////////////////////////////////




void LoRa_Bind_Mode(uint8_t mode){

  if(!(LoRa_bindMode_master || LoRa_bindMode_slave)){

    OriginalSettings.channel = module.channel;         //Save settings on 1st entry
    OriginalSettings.power = module.power;
    OriginalSettings.spFactor = module.spFactor;
    OriginalSettings.LoRa_BW = module.LoRa_BW;

    BindPacket[0] = 0; //slave addr = broadcast
    BindPacket[1] = MCMD_LORA_SET_SETTINGS;
    BindPacket[2] = module.channel;
    BindPacket[3] = module.power | (module.spFactor << 4) | 0x04;
    BindPacket[4] = module.LoRa_BW;

    int crc_calc2 = modbus_crc((uint8_t *)BindPacket, 5, CRC_NORMAL);
    BindPacket[5] = crc_calc2 & 0xFF;
    BindPacket[6] = crc_calc2 / 0x100;

  }

  if(mode == MASTER_BY_CHANNEL){ //binding with the same channel
    LoRa_bindMode_master = 1;
    LoRa_bindMode_slave = 0;
    set_LED(BLUE, 1, 0);
    LoRa_config(module.channel, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);    
  }
  else if (mode == MASTER) { // binding with channel 0
    LoRa_bindMode_master = 1;
    set_LED(BLUE, 1, 0);
    LoRa_config(0, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);
  } 
  else if (mode == SLAVE) {
    LoRa_bindMode_slave = 1;
    set_LED(BLUE, 0, 0);
    set_LED(GREEN, 1, 2000);
    LoRa_config(0, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);
  }
  else if (mode == DISABLE) { //exit binding mode on slave or master
    if (LoRa_bindMode_slave) {
      if(!LoRa_channel_received){
         module.channel = OriginalSettings.channel;
         module.power = OriginalSettings.power;
         module.spFactor = OriginalSettings.spFactor;
         module.LoRa_BW = OriginalSettings.LoRa_BW;
      }
      set_LED(BLUE, 1, 500);
      LoRa_bindMode_slave = 0;
      LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET, RxMode); //Set settings to slave
    } 
    else { //exit bindmode on master
      set_LED(BLUE, 0, 0);
      LoRa_bindMode_master = 0;
      LoRa_config(OriginalSettings.channel, OriginalSettings.power, OriginalSettings.spFactor, OriginalSettings.LoRa_BW, LoRa_MAX_PACKET, RxMode); //set settings to master
      if(conv_mode == CONV_MODE_MASTER){
        memset(tx_settings_buffer, 0x00, 10);
      }
    }
  }else if(mode == BY_CHANNEL){   //
    LoRa_bindMode_slave = 1;
    set_LED(BLUE, 0, 0);
    set_LED(GREEN, 1, 2000);
    OriginalSettings.spFactor = module.spFactor;
    OriginalSettings.LoRa_BW = module.LoRa_BW;
    LoRa_config(module.channel, LoRa_POWER_20DBM, LoRa_SF_10, LoRa_BW_62_5KHZ, LoRa_MAX_PACKET, RxMode);
    module.spFactor = OriginalSettings.spFactor;
    module.LoRa_BW = OriginalSettings.LoRa_BW;    
  }
}

   
uint8_t LoRa_config(uint8_t channel, uint8_t power, uint8_t spFactor, uint8_t LoRa_BW, uint8_t packetLength, uint8_t mode) {

  uint8_t tmp[10];

  set_settings_flag = 0;
  LoRa_Interrupt(DISABLE_INTERRUPTS);

  module.channel = channel;
  module.power = power;
  module.spFactor = spFactor;
  module.LoRa_BW = LoRa_BW;
  module.packetLength = packetLength;


  LoRa_sleep(); //Change modem mode Must in Sleep mode
  for(volatile int i=0 ; i<720000 ; i++); //delay 15ms

  if(LoRa_BW == LoRa_BW_125KHZ)
    module.frequency = 0x6C4666 + 0xC00 * channel;

  else if(LoRa_BW == LoRa_BW_250KHZ)
    module.frequency = 0x6C4666 + 0x1800 * channel;

  else if(LoRa_BW == LoRa_BW_500KHZ)
    module.frequency = 0x6C4666 + 0x3000 * channel;        

  else if(LoRa_BW == LoRa_BW_62_5KHZ)
    module.frequency = 0x6C4666 + 0x600 * channel;

  tmp[0] = module.frequency >> 16;
  tmp[1] = (module.frequency & 0xff00) >> 8;
  tmp[2] = module.frequency & 0xff;

  LoRa_SPIWrite(LR_RegFrMsb, tmp, 3); //setting  frequency parameter
  LoRa_SPIRead(LR_RegFrMsb, tmp, 3);
  if(!(tmp[0] == (module.frequency >> 16)))
    return 0;

  //setting base parameter
  tmp[0] = LoRa_Power[power];
  LoRa_SPIWrite(LR_RegPaConfig, tmp, 1); //Setting output power parameter
  tmp[0] = 0x0B;
  LoRa_SPIWrite(LR_RegOcp, tmp, 1);     //RegOcp,Close Ocp
  tmp[0] = 0x23;
  LoRa_SPIWrite(LR_RegLna, tmp, 1);   //RegLNA,High & LNA Enable
  tmp[0] = 0x04;
  LoRa_SPIWrite(LR_RegModemConfig3, tmp, 1);
  if (spFactor == 6) {  //SFactor=6
    tmp[0] = ((LoRa_LoRaBandwidth[LoRa_BW] << 4) + (LoRa_CR << 1) + 0x01);
    LoRa_SPIWrite(LR_RegModemConfig1, tmp, 1); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

    tmp[0] = ((spFactor << 4) + (LoRa_CRC << 2) + 0x03);
    LoRa_SPIWrite(LR_RegModemConfig2, tmp, 1);

    tmp[0] = 0x42;
    LoRa_SPIRead(RegPacketConfig2, tmp, 1);
    if(tmp[0] == 0x42)
      return 0;
    tmp[0] |= 0x05;
    LoRa_SPIWrite(RegPacketConfig2, tmp, 1);
    tmp[0] = 0x0C;
    LoRa_SPIWrite(RegSeqConfig2, tmp, 1);
    packetLength = 4;
    module.packetLength = 4;
  } else {
    tmp[0] = (LoRa_LoRaBandwidth[LoRa_BW] << 4) + (LoRa_CR << 1) + 0x00;
    LoRa_SPIWrite(LR_RegModemConfig1, tmp, 1); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
    tmp[0] = (spFactor << 4) + (LoRa_CRC << 2) + 0x03;
    LoRa_SPIWrite(LR_RegModemConfig2, tmp, 1); //SFactor &  LNA gain set by the internal AGC loop
  }
  tmp[0] = 0xFF;
  LoRa_SPIWrite(LR_RegSymbTimeoutLsb, tmp, 1); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
  LoRa_SPIWrite(LR_RegMaxPayloadLength, tmp, 1);
  tmp[0] = 0x00;
  LoRa_SPIWrite(LR_RegPreambleMsb, tmp, 1); //RegPreambleMsb
  tmp[0] = 8;
  LoRa_SPIWrite(LR_RegPreambleLsb, tmp, 1); //RegPreambleLsb 8+4=12byte Preamble
  tmp[0] = 0x00;
  LoRa_SPIWrite(REG_LR_DIOMAPPING2, tmp, 1); //RegDioMapping2 DIO0 = 00- RX_Done
  LoRa_SPIWrite(LR_RegHopPeriod, tmp, 1); //RegHopPeriod NO FHSS
  LoRa_SPIWrite(LR_RegFifoTxBaseAddr, tmp, 1);
  LoRa_SPIWrite(LR_RegFifoRxBaseAddr, tmp, 1);

  tmp[0] = 0x87;
  LoRa_SPIWrite(REG_LR_PADAC, tmp, 1);  //Tx for 20dBm on HP pin

  LoRa_standby(); //Entry standby mode

  LoRa_Interrupt(ENABLE_INTERRUPTS);

  if(mode)
   LoRa_EntryTx(packetLength, 2000);
 else LoRa_EntryRx(packetLength, 2000);

 return 1;
}


int LoRa_EntryTx(uint8_t length, uint32_t timeout) {
  uint8_t tmp[10];

  module.packetLength = length;

  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1); //RegFifoAddrPtr
  tmp[0] = 0x40;
  LoRa_SPIWrite(REG_LR_DIOMAPPING1, tmp, 1); //DIO0=01 TX_Done
  LoRa_clearIrq();
  tmp[0] = 0xF7;
  LoRa_SPIWrite(LR_RegIrqFlagsMask, tmp, 1); //Open TxDone interrupt
  LoRa_SPIWrite(LR_RegPayloadLength, &length, 1); //RegPayloadLength

  while (1) {
    LoRa_SPIRead(LR_RegPayloadLength, tmp, 1);
    if (tmp[0] == length) {
      module.status = TX;
      return 1;
    }

    if (--timeout == 0) {
      return 0;
    }
  }
}

int LoRa_EntryRx(uint8_t length, uint32_t timeout) {
  uint8_t tmp[10];

  module.packetLength = length;

  tmp[0] = 0x00;
  LoRa_SPIWrite(REG_LR_DIOMAPPING1, tmp, 1);//DIO0=00 RX_Done
  LoRa_SPIWrite(LR_RegFifoAddrPtr, tmp, 1); //RxBaseAddr->FiFoAddrPtr
  tmp[0] = 0x0F;
  LoRa_SPIWrite(LR_RegIrqFlagsMask, tmp, 1);//Open RxDone, Timeout, ValidHeader, CRC error
  LoRa_clearIrq();
  LoRa_SPIWrite(LR_RegPayloadLength, &length, 1);//Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
  tmp[0] = 0x8D;
  LoRa_SPIWrite(LR_RegOpMode, tmp, 1);  //Mode//Low Frequency Mode, receiver mode

  while (1) {
    LoRa_SPIRead(LR_RegModemStat, tmp, 1);
    if ((tmp[0] & 0x04) == 0x04) {  //Rx-on going RegModemStat
      module.status = RX;
      return 1;
    }
    if (--timeout == 0) {
      LoRa_reset();
      //LoRa_defaultConfig();
      return 0;
    }
    for(int i=0 ; i<48000 ; i++);
  }
}