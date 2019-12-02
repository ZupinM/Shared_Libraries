/**************************************************************************
 *    "Pico" solar positioner software
 *
 *    filename: modbus.c  
 *    pcb: tiv29B
 *
 *    Copyright(C) 2011, Sat Control d.o.o.
 *    All rights reserved.
**************************************************************************/

#include "LPC15xx.h"
#include "modbus.h"
#include "uart.h"
#include "config.h"
#include "../gpio.h"
#include <string.h>
#include "SX1278.h"
#include "aes.h"
#include "../main.h"
#include "../eeprom.h"

/////


unsigned int crc_calc1;
unsigned int crc_calc2;
unsigned int number_TX_bytes1;
unsigned int number_TX_bytes2;

extern volatile uint32_t UARTCount0;
extern volatile uint32_t UARTCount1;
extern volatile uint32_t UARTCount2;
extern volatile uint8_t UARTBuffer0[BUFSIZE];
extern volatile uint8_t UARTBuffer1[BUFSIZE];
extern volatile uint8_t UARTBuffer2[BUFSIZE];
uint8_t UARTBuffer2_long[BUFSIZE_LONG];
char upgradeCount = 0;

extern  uint32_t tx_packet_length;
extern uint8_t tx_packet_buffer[BUFSIZE];
extern uint8_t number_TX_settings_bytes;
extern uint8_t  tx_setting_route;
extern uint8_t tx_settings_flag;
extern uint16_t online_timeouts[165];
extern uint8_t tx_settings_buffer[BUFSIZE];

/* parameters */

extern unsigned char slave_addr;        // slave address on RS485
long long int online_slaves;


extern unsigned int SN[4];        // vsebujejo serijske stevilke

extern unsigned int modbus_cnt1;        // steje cas zadnjega modbus ukaza
extern unsigned int modbus_cnt2;        // steje cas zadnjega modbus ukaza
extern unsigned int modbus_timeout;     // timeout, ko ni MODBUS komunikacije [sekunde]
extern unsigned int modbus_timeout_delay;
extern unsigned int crc_errors;

/* flags registers */
extern unsigned int flags;

extern const Version swVersion;
extern volatile uint8_t UARTTxEmpty0;
extern volatile uint8_t UARTTxEmpty1;
extern volatile uint8_t UARTTxEmpty2;
extern volatile int rxTimeout0;


extern unsigned int reset_status;
extern volatile unsigned int start_count;

volatile int slaveCommandTimeout;
volatile int slaveCommandTimeout;
unsigned int xbLength;
char xbData[BUFSIZE];
unsigned int xbSendPacketPrepare(char *pchData, unsigned int uiLength);
unsigned int xbReceivePacketRestore(char *pchBuffer);

extern uint8_t routeOrders[MAX_SLAVE_ADDR+1][MAX_ROUTE_HOPS];

#define ftoint(val) (*((unsigned int *)(unsigned int) & (val))) 
#define fsendval(val) (((ftoint(val) << 24) & 0xff000000) | ((ftoint(val) << 8) & 0xff0000) | ((ftoint(val) >> 24) & 0xff) | ((ftoint(val) >> 8) & 0xff00)) 

/***********************************************************
  MODBUS COMMANDS
************************************************************/
char commShort = 1;
char commBuff1;
extern volatile uint8_t UARTBuffer[BUFSIZE];
uint8_t writePacket1[0x80];
uint8_t writePacket2[0x80];
#define MODE_NORMAL 0
#define MODE_BOOT 1
#define MODE_UPGRADE 2
#define MODE_ERROR 3

unsigned int addrPrev = 0;
// from Sigma to converter(slave) via ZigBee / LoRa
////////////////////////////////////////////////////
void modbus_cmd1() {

  uint32_t UARTCount;
  unsigned int dataLength = 0;
  char mode = MODE_NORMAL;
  unsigned int crcUpgradeCode;

  if(transceiver == LORA)
    memcpy((uint8_t *)UARTBuffer1, (uint8_t *)module.rxBuffer, BUFSIZE);

  if (UARTBuffer1[0] == 0x7E || transceiver == LORA) {

    if(transceiver == XBEE) {

      dataLength = xbReceivePacketRestore((char *)UARTBuffer1);
    }
    else if(transceiver == LORA)
      dataLength = module.packetLength;

    // if second part of write packet is not true
    if(upgradeCount && dataLength != 0x93) {
      UARTCount1 = 0;
      return;
    }

    memcpy((uint8_t *)UARTBuffer, (uint8_t *)UARTBuffer1, BUFSIZE);
    if (dataLength == -1) {
      UARTCount1 = 0;
      return;
    }
    else if(dataLength == -2) {
      if(crc_errors / 0x10000 == 0xFFFF)
       crc_errors = crc_errors % 0x10000; // reset counter, only keep lower 4 bytes (CRC erors)

      crc_errors = crc_errors + 0x10000;  // increase checksum error - upper 4 bytes
      UARTCount1 = 0;
      return;
    }
    UARTCount = dataLength;
  }
  else {
    UARTCount1 = 0;
    return;
  }

  // check normal packet
  crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount, CRC_NORMAL);

  // not for slave and not for positioner
  if(crc_calc1 == 0 && (UARTBuffer1[1] == INTCOM_LORA_GET_ROUTE || (UARTBuffer1[1] == INTCOM_LORA_SET_ROUTE))) {
    UARTCount1 = 0;
    return;
  }

  // check boot mode the upgrade mode
  if(crc_calc1 != 0) {
    memcpy((uint8_t *)UARTBuffer1, (uint8_t *)UARTBuffer, BUFSIZE);   
    // check boot packet
    crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount, CRC_BOOT);

    if(crc_calc1 != 0) {
      memcpy((uint8_t *)UARTBuffer1, (uint8_t *)UARTBuffer, BUFSIZE);   
      // check upgrade
      crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount, CRC_UPGRADE_NANOD);
      if(crc_calc1 == 0)
        crcUpgradeCode = CRC_UPGRADE_NANOD;
      else {
        crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount, CRC_UPGRADE_KVARK);
        if(crc_calc1 == 0)
          crcUpgradeCode = CRC_UPGRADE_KVARK;
      }
      

      // error
      if(crc_calc1 != 0) {
        mode = MODE_ERROR;

//        UARTBuffer1[2] = 1; // error
//        number_TX_bytes1 = 3;
//        goto TX;
          UARTCount1 = 0;
          return;
      }
      // upgrade processing
      else {
        mode = MODE_UPGRADE;

        if(UARTCount == 0x93 && isOnlineDevice(UARTBuffer1[0])) { // write command

          if(decryptData((char *)&UARTBuffer1[1], UARTCount - 3) == 0) { // decrypt

            unsigned int addr = 0, size = 0; 
            addr += (UARTBuffer1[2] * 0x1000000);
            addr += (UARTBuffer1[3] * 0x10000);
            addr += (UARTBuffer1[4] * 0x100);
            addr += UARTBuffer1[5];
            size += (UARTBuffer1[6] * 0x100);
            size += UARTBuffer1[7];
            if(addr % 0x100 == 0x0 && size == 0x80) { //first part

              addrPrev = addr;
              upgradeCount = 1;
              memcpy((uint8_t *)writePacket1, (uint8_t *)(UARTBuffer1 + 8), 0x80);
              UARTBuffer1[2] = MACK_OK; // OK
              number_TX_bytes1 = 3;
              goto TX;
            }
            else if(addr % 0x100 == 0x80 && size == 0x80) { // second part

              if(addrPrev != addr - 0x80) {
                UARTBuffer1[2] = MACK_UNRECOGNIZED_CMD; // ERR
                number_TX_bytes1 = 3;
                addrPrev = 0;
                goto TX;
              }
              addrPrev = 0;

              UARTCount += 0x80;
              memcpy((uint8_t *)writePacket2, (uint8_t *)(UARTBuffer1 + 8), 0x80);
              memset(UARTBuffer2_long, 0xff, BUFSIZE_LONG);
              memcpy((uint8_t *)(UARTBuffer2_long), (uint8_t *)(UARTBuffer1), 5);
              memcpy((uint8_t *)(UARTBuffer2_long + 0x08), (uint8_t *)writePacket1, 0x80);
              memcpy((uint8_t *)(UARTBuffer2_long + 0x88), (uint8_t *)writePacket2, 0x80);
              UARTBuffer2_long[5] = 0x0;   // round address to 0x100
              UARTBuffer2_long[6] = 0x01;  // length 256
              UARTBuffer2_long[7] = 0x0;

              number_TX_bytes1 = 264;

              number_TX_bytes1 = encryptData((char *)&UARTBuffer2_long[1], number_TX_bytes1 - 1);
              crc_calc1 = modbus_crc((uint8_t *)UARTBuffer2_long, ++number_TX_bytes1, crcUpgradeCode);
              UARTBuffer2_long[number_TX_bytes1++] = crc_calc1 & 0xFF;
              UARTBuffer2_long[number_TX_bytes1++] = crc_calc1 / 0x100;

              if(transceiver == XBEE)
                xbLength = xbSendPacketPrepare((char *)UARTBuffer1, number_TX_bytes1);
            }
          }
        }
      } // end upgrade processing
    }
    else {
      mode = MODE_BOOT;
    }
  }
  // normal mode processing
  else if(((UARTBuffer1[0] == slave_addr ||                       // slaveID or broadcast(0x0) for SN setting, set settings
            UARTBuffer1[0] == 0x00 ||
            UARTBuffer1[1] == INTCOM_LORA_SET_ID_BY_SN ||
            UARTBuffer1[1] == INTCOM_LORA_SET_SETTINGS ||
            UARTBuffer1[1] == INTCOM_LORA_GET_RSSI ||
              (UARTBuffer1[0] == 0xFF &&                          // broadcast(0xFF) for SN getting
              UARTBuffer1[1] == INTCOM_CONV_GET_SN_BY_ID)) &&
          transceiver == LORA) ||                               // << LoRa condition
        transceiver == XBEE) {                                  // ZigBee condition
    
    mode = MODE_NORMAL;

    switch (UARTBuffer1[1]) {
      case CMD_RUN_GET_LOADER_VER: {
        unsigned short ver = getVersionB();
        UARTBuffer1[2] = MACK_OK;
        UARTBuffer1[3] = ver / 0x100;
        UARTBuffer1[4] = ver % 0x100;
        number_TX_bytes1 = 5;
        goto TX;
        break;
      }
      case CMD_RUN_GET_VERSION: {

        unsigned short ver = getVersion();
        UARTBuffer1[2] = MACK_OK;
        UARTBuffer1[3] = ver / 0x100;
        UARTBuffer1[4] = ver % 0x100;
        number_TX_bytes1 = 5;
        goto TX;
        break;
      }
      case CMD_RUN_GET_VOLTAGE: {
        float voltage = gpio_U();
        number_TX_bytes1 = mcmd_read_float(voltage, (char *)UARTBuffer1);
        goto TX;
        break;
      }
      case CMD_ZB2RS_RESET: { 
        if (UARTBuffer1[0] == slave_addr || transceiver == XBEE) {
          flags |= (1 << reset_it);   
          reset_status = RESET_MANUAL;
          UARTBuffer1[2] = MACK_OK;
          number_TX_bytes1 = 3;
          goto TX;
        }
        break;    
      }

      case INTCOM_LORA_SET_SETTINGS: {

        if(!(UARTBuffer1[0] == 0x0 || UARTBuffer1[0] == 0xFF || UARTBuffer1[0] == slave_addr)) {
          UARTCount1 = 0;
          number_TX_bytes1 = 0;
          return;
        }

        if(UARTBuffer1[2] != 0xff) 
          module.channel = UARTBuffer1[2];
        module.power = UARTBuffer1[3] & 0x03;
        module.spFactor = (UARTBuffer1[3] & 0xf0) >> 4;
        module.LoRa_BW =  UARTBuffer1[4]; 

#if (STORAGE_TYPE == STORAGE_FLASH)
        flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
        eeprom_write(SYS_VARS_EE); 
#endif

        if(UARTBuffer1[3] & 0x04){
          LoRa_channel_received = 1;
          UARTBuffer1[2] = LoRa_get_rssi();

          number_TX_bytes1 = 3;
          goto TX;                                  
        }

        LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET,  RxMode); //Set settings to slave
        //timeout_master_check = 400000;
        if(UARTBuffer1[0] < 0xff){
          UARTBuffer1[2] = MACK_OK;
          number_TX_bytes1 = 3;
          goto TX;                     
        }

        break;
      } 

        case INTCOM_LORA_SET_ID_BY_SN: {

          int SNrx[4];  
          int updateSuccess = 0;

          if(UARTBuffer1[0] == slave_addr){
            slave_addr = UARTBuffer1[18];
            updateSuccess = 1;
            //UARTBuffer1[2] = MACK_OK;
          }else/* if(UARTBuffer1[0] == 0xff || UARTBuffer1[0] == 0x00)*/{

            SNrx[0]  = UARTBuffer1[2] << 24;
            SNrx[0] |= UARTBuffer1[3] << 16;
            SNrx[0] |= UARTBuffer1[4] << 8;
            SNrx[0] |= UARTBuffer1[5];

            SNrx[1]  = UARTBuffer1[6] << 24;
            SNrx[1] |= UARTBuffer1[7] << 16;
            SNrx[1] |= UARTBuffer1[8] << 8;
            SNrx[1] |= UARTBuffer1[9];

            SNrx[2]  = UARTBuffer1[10] << 24;
            SNrx[2] |= UARTBuffer1[11] << 16;
            SNrx[2] |= UARTBuffer1[12] << 8;
            SNrx[2] |= UARTBuffer1[13];

            SNrx[3]  = UARTBuffer1[14] << 24;
            SNrx[3] |= UARTBuffer1[15] << 16;
            SNrx[3] |= UARTBuffer1[16] << 8;
            SNrx[3] |= UARTBuffer1[17];

            if(SNrx[0] == SN[0] && SNrx[1] == SN[1] && SNrx[2] == SN[2] && SNrx[3] == SN[3]){
              slave_addr = UARTBuffer1[18];
              updateSuccess = 1;
            }
          }
          LoRa_id = slave_addr;
          if(updateSuccess){
              // save data
#if     (STORAGE_TYPE == STORAGE_FLASH)
                flash_write(SYS_VARS_ADDR);                   //save parameters
#elif   (STORAGE_TYPE == STORAGE_EEPROM)
                eeprom_write(SYS_VARS_EE); 
#endif    

              UARTBuffer1[2] = available_positioners[0];
              UARTBuffer1[3] = available_positioners[1]; 
              UARTBuffer1[4] = available_positioners[2];
              UARTBuffer1[5] = available_positioners[3];
              UARTBuffer1[6] = available_positioners[4];
              UARTBuffer1[7] = available_positioners[5];
              UARTBuffer1[8] = available_positioners[6];
              UARTBuffer1[9] = available_positioners[7];   

              number_TX_bytes1 = 10;
              goto TX; 
            }else{
              UARTCount1  = 0;
              number_TX_bytes1 = 0;
              return;
            }
          }       

          case INTCOM_CONV_GET_SN_BY_ID: {

            UARTBuffer1[2] = (SN[0] >> 24) & 0xff;
            UARTBuffer1[3] = (SN[0] >> 16) & 0xff;
            UARTBuffer1[4] = (SN[0] >> 8)  & 0xff;  
            UARTBuffer1[5] =  SN[0]        & 0xff;  

            UARTBuffer1[6] = (SN[1] >> 24) & 0xff;
            UARTBuffer1[7] = (SN[1] >> 16) & 0xff;
            UARTBuffer1[8] = (SN[1] >> 8)  & 0xff;  
            UARTBuffer1[9] = SN[1]         & 0xff; 

            UARTBuffer1[10] = (SN[2] >> 24) & 0xff;
            UARTBuffer1[11] = (SN[2] >> 16) & 0xff;
            UARTBuffer1[12] = (SN[2] >> 8)  & 0xff;  
            UARTBuffer1[13] =  SN[2]        & 0xff; 

            UARTBuffer1[14] = (SN[3] >> 24) & 0xff;
            UARTBuffer1[15] = (SN[3] >> 16) & 0xff;
            UARTBuffer1[16] = (SN[3] >> 8)  & 0xff;  
            UARTBuffer1[17] =  SN[3]        & 0xff;

            number_TX_bytes1 = 18;
            goto TX; 
          } 
                  
          case INTCOM_LORA_GET_RSSI: {    
            if(UARTBuffer1[0] == slave_addr){   
              UARTBuffer1[2] = LoRa_get_rssi();
              number_TX_bytes1 = 3;
              goto TX;
            }
          }

        }
      }

  // send to positioner
      if(upgradeCount == 1) {
        // upgrade write command
        UART2Send((uint8_t *)UARTBuffer2_long, number_TX_bytes1);
        upgradeCount = 0;

      } else {
        memcpy((char *)UARTBuffer2, (char *)UARTBuffer1, BUFSIZE);
        UART2Send((uint8_t *)UARTBuffer2, UARTCount);
      }
        UARTCount1 = 0;
        number_TX_bytes1 = 0;
        return;

        TX:
        // TX upgrade (write) answer
        if(mode == MODE_UPGRADE && UARTCount == 0x93) {

          // send back to sigma
          number_TX_bytes1 = encryptData((char *)&UARTBuffer1[1], number_TX_bytes1 - 1);
          crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, ++number_TX_bytes1, CRC_BOOT);
          UARTBuffer1[number_TX_bytes1++] = crc_calc1 & 0xFF;
          UARTBuffer1[number_TX_bytes1++] = crc_calc1 / 0x100;

          if(transceiver == LORA){
            set_tx_flag((char *)UARTBuffer1, number_TX_bytes1);
          }
          else if(transceiver == XBEE){

            xbLength = xbSendPacketPrepare((char *)UARTBuffer1, number_TX_bytes1);    
            UART1Send( (uint8_t *)(&xbData[0]), xbLength);
          }

          UARTCount = 0;
          number_TX_bytes1 = 0;
        }
        // TX normal answer
        else if((mode == MODE_NORMAL &&                       // normal mode
                (UARTBuffer1[1] == CMD_RUN_GET_LOADER_VER ||    // get boot version
                  UARTBuffer1[1] == CMD_RUN_GET_VERSION ||      // get version
                  UARTBuffer1[1] == CMD_RUN_GET_VOLTAGE ||      // get voltage
                  UARTBuffer1[1] == CMD_ZB2RS_RESET ||          // module reset
                  UARTBuffer1[1] == INTCOM_LORA_SET_SETTINGS || // set LoRa settings
                  UARTBuffer1[1] == INTCOM_LORA_GET_SETTINGS || // get LoRa settings
                  UARTBuffer1[1] == INTCOM_LORA_SET_ID_BY_SN || // set LoRa ID
                  (UARTBuffer1[1] == INTCOM_CONV_GET_SN_BY_ID && UARTBuffer1[0] != 100) || // get LoRa SN
                  UARTBuffer1[1] == INTCOM_LORA_GET_RSSI)       // get LoRa RSSI
               )) {

         crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, number_TX_bytes1, CRC_NORMAL);
         UARTBuffer1[number_TX_bytes1++] = crc_calc1 & 0xFF;
         UARTBuffer1[number_TX_bytes1++] = crc_calc1 / 0x100;

         if(transceiver == LORA){

          int time1 = slaveCommandTimeout;
          while(true) {
            if(slaveCommandTimeout - time1 > 15)
              break;
          }

          
          if(LoRa_channel_received)
            LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET,  RxMode); //Set settings to slave
          else  
            set_tx_flag((char *)UARTBuffer1, number_TX_bytes1);
    }
    else if(transceiver == XBEE){

      xbLength = xbSendPacketPrepare((char *)UARTBuffer1, number_TX_bytes1);    
      UART1Send( (uint8_t *)(&xbData[0]), xbLength);
    }

    UARTCount = 0;
    number_TX_bytes1 = 0;

    if (flags & (1 << reset_it)) {    
      while (rxTimeout0 < 1000);    
#if (STORAGE_TYPE == STORAGE_FLASH)
      flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
      eeprom_write(SYS_VARS_EE); 
#endif    
      LPC_WWDT->FEED = 0xAA;            
      LPC_WWDT->FEED = 0x50;    
      while(1);   
    }
  }
  UARTCount1 = 0;
}

short xBeeConf;

////////////////////////////////////////////////////
// from ZigBee / LoRa to sigma via converter(master)
void modbus_cmd1_master() {

  if(UARTCount1 > 0) { //ZigBEE

    memcpy((char *)UARTBuffer2, (char *)UARTBuffer1, BUFSIZE);
    UART2Send((uint8_t *)(UARTBuffer2), UARTCount1);

  }
  else{
    memcpy((char *)UARTBuffer2, (char *)module.rxBuffer, module.packetLength);
    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, module.packetLength, CRC_NORMAL);

    if(crc_calc2){
      crc_calc1 = modbus_crc((uint8_t *)UARTBuffer2, module.packetLength, CRC_BOOT);
      if(crc_calc1){
        crc_calc1 = modbus_crc((uint8_t *)UARTBuffer2, module.packetLength, CRC_UPGRADE_NANOD);
        if(crc_calc1 != 0)
          crc_calc1 = modbus_crc((uint8_t *)UARTBuffer2, module.packetLength, CRC_UPGRADE_KVARK);
        if(crc_calc1){
            UARTCount1 = 0;
            number_TX_bytes2 = 0;
            return;
        }
      }
    }

    if(UARTBuffer2[0] == slave_addr && UARTBuffer2[1] == CMD_ZB2RS_MASTER_SLAVE && crc_calc2 == 0 ){ // If conv_mode on slave side is configured as master
          conv_mode = UARTBuffer2[2];                                                                 
#if (STORAGE_TYPE == STORAGE_FLASH)
          flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
          eeprom_write(SYS_VARS_EE); 
#endif  
          UARTBuffer2[2] = MACK_OK;
          number_TX_bytes2 = 3;
          crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
          UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
          UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;
          set_tx_flag((char *)UARTBuffer2, number_TX_bytes2);
          return; 
        }

    if(UARTBuffer2[1] == MCMD_R_All_PARAM && crc_calc2 == 0){ //Read All Parameters - extract slave's LoRa ID

        UARTBuffer2[67] = 100; //default parent id = master   
        for(int n = 0 ; n < MAX_ROUTE_HOPS ; n++){    
          if(LoRa_route[UARTBuffer2[0]][n+1] == 0 || (n+1) == MAX_ROUTE_HOPS){ 
            if(n>0)
              UARTBuffer2[67] = LoRa_route[UARTBuffer2[0]][n-1];    //Append parent id
            break;    
          }   
        }  
      for(int i=0 ; i<4 ; i++)
        if(UARTBuffer2[26+i] >= LORA_ID_MASTER && UARTBuffer2[26+i] <= LORA_ID_MASTER + MAX_SLAVE_ADDR){    
          online_slaves |= (1ULL << (UARTBuffer2[26+i] - 100));   
          online_timeouts[UARTBuffer2[26+i]] = 50000;   
        }

      number_TX_bytes2 = module.packetLength - 2;
TX:
      crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
      UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
      UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;
    } 
    UART2Send((uint8_t *)(UARTBuffer2), module.packetLength);
  }
  UARTCount1 = 0;
  number_TX_bytes2 = 0;
}

volatile uint8_t UARTTest1[BUFSIZE];

////////////////////////////////////////////////////
// from positioner via ZigBee / LoRa converter to sigma
void modbus_cmd2() {

  if(UARTCount2 > 0) {
    memcpy((char *)UARTBuffer1, (char *)UARTBuffer2, BUFSIZE);

    modbus_cnt2 = 0;

    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, UARTCount2, CRC_NORMAL);
    crc_calc1 = modbus_crc((uint8_t *)UARTBuffer2, UARTCount2, CRC_BOOT);

    if(crc_calc2 == 0 || crc_calc1 == 0)
      baudrate_timeout = 0;  


    if((crc_calc2 == 0 || crc_calc1 == 0) && UARTBuffer1[0] > 0 && UARTBuffer1[0] < 255){
      int i = (UARTBuffer1[0] - 1) / 8;
      available_positioners[i] |= 1 << (UARTBuffer1[0] - i*8 - 1);
      online_timeouts[UARTBuffer1[0]] = 50000;
    }

    if(UARTCount2 != 0x93 && crc_calc2 == 0) { // not for upgrade reading mode
      switch (UARTBuffer2[1]) {
        case CMD_ZB2RS_MASTER_SLAVE: {  // changing SLAVE to MASTER

          conv_mode = UARTBuffer2[2];
          if(conv_mode == CONV_MODE_MASTER)
            slave_addr = LORA_ID_MASTER;
#if (STORAGE_TYPE == STORAGE_FLASH)
          flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
          eeprom_write(SYS_VARS_EE); 
#endif
          UARTBuffer2[2] = MACK_OK;
          number_TX_bytes2 = 3;

          goto TX;
          break;
        }

        case INTCOM_CONV_GET_SN_BY_ID: {  // get SN for defined slave
          if(UARTBuffer2[0] == 100) {
            UARTBuffer2[2] = (SN[0] >> 24) & 0xff;
            UARTBuffer2[3] = (SN[0] >> 16) & 0xff;
            UARTBuffer2[4] = (SN[0] >> 8)  & 0xff;  
            UARTBuffer2[5] =  SN[0]        & 0xff;  

            UARTBuffer2[6] = (SN[1] >> 24) & 0xff;
            UARTBuffer2[7] = (SN[1] >> 16) & 0xff;
            UARTBuffer2[8] = (SN[1] >> 8)  & 0xff;  
            UARTBuffer2[9] = SN[1]         & 0xff; 

            UARTBuffer2[10] = (SN[2] >> 24) & 0xff;
            UARTBuffer2[11] = (SN[2] >> 16) & 0xff;
            UARTBuffer2[12] = (SN[2] >> 8)  & 0xff;  
            UARTBuffer2[13] =  SN[2]        & 0xff; 

            UARTBuffer2[14] = (SN[3] >> 24) & 0xff;
            UARTBuffer2[15] = (SN[3] >> 16) & 0xff;
            UARTBuffer2[16] = (SN[3] >> 8)  & 0xff;  
            UARTBuffer2[17] =  SN[3]        & 0xff;

            number_TX_bytes2 = 18;
            goto TX;
          }
          break;    
        }
      }
    }

    if(transceiver == LORA){
      if(UARTBuffer1[1] == MCMD_R_All_PARAM && crc_calc2 == 0){ //Read All Parameters 
        UARTBuffer1[26] = slave_addr;
        UARTBuffer1[66] = LoRa_get_rssi();

        number_TX_bytes2 = UARTCount2 - 2;
        crc_calc2 = modbus_crc((uint8_t *)UARTBuffer1, number_TX_bytes2, CRC_NORMAL);
        UARTBuffer1[number_TX_bytes2++] = crc_calc2 & 0xFF;
        UARTBuffer1[number_TX_bytes2++] = crc_calc2 / 0x100;
      }

      set_tx_flag((char *)UARTBuffer1, UARTCount2);
    }
    else if(transceiver == XBEE){

      xbLength = xbSendPacketPrepare((char *)UARTBuffer1, UARTCount2);    
      UART1Send( (uint8_t *)(&xbData[0]), xbLength);
    }


    UARTCount2 = 0;
    return;

    TX:
    if(UARTBuffer2[1] == CMD_ZB2RS_MASTER_SLAVE ||
     (UARTBuffer2[1] == INTCOM_CONV_GET_SN_BY_ID && UARTBuffer2[0] == 100)
     ) {
      crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
      UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
      UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;

      UART2Send((uint8_t *)(UARTBuffer2), number_TX_bytes2);

      UARTCount2 = 0;
      number_TX_bytes2 = 0;
    }
  }
}

////////////////////////////////////////////////////
// packets from Sigma to ZigBee / Lora via converter       
  void modbus_cmd2_master() {

    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, UARTCount2, CRC_NORMAL);
    crc_calc1 = modbus_crc((uint8_t *)UARTBuffer2, UARTCount2, CRC_BOOT);

    if(crc_calc2 == 0 || crc_calc1 == 0)
      baudrate_timeout = 0;

    if(!(                                                // NOT condition
         ((UARTBuffer2[1] == CMD_RUN_GET_LOADER_VER ||
           UARTBuffer2[1] == CMD_RUN_GET_VERSION || 
           UARTBuffer2[1] == CMD_RUN_GET_VOLTAGE ||
           UARTBuffer2[1] == CMD_ZB2RS_MASTER_SLAVE || 
           UARTBuffer2[1] == CMD_ZB2RS_RESET || 
           UARTBuffer2[1] == INTCOM_LORA_SET_SETTINGS ||
           UARTBuffer2[1] == INTCOM_LORA_GET_SETTINGS || 
           UARTBuffer2[1] == INTCOM_LORA_GET_SLAVES   ||
           UARTBuffer2[1] == INTCOM_LORA_RESET_ROUTES) && 
         crc_calc2 == 0 && UARTBuffer2[0] == 0xFF) ||     // normal mode, broadcast (ID=0xFF)
         (UARTBuffer2[1] == INTCOM_CONV_GET_SN_BY_ID && UARTBuffer2[0] == 100) || // get SN from master
         ((UARTBuffer2[1] == INTCOM_LORA_SET_SETTINGS ||
           UARTBuffer2[1] == INTCOM_LORA_GET_ROUTE || 
           UARTBuffer2[1] == INTCOM_LORA_SET_ROUTE) &&
         UARTBuffer2[0] < 0xff && crc_calc2 == 0) // normal mode, not for master configuration (specified ID)
         ) ||
       crc_calc2 != 0) {                                // NOT upgrade

      if(UARTCount2 > 0) {
        memcpy((char *)UARTBuffer1, (char *)UARTBuffer2, BUFSIZE);

        // settings for AT commands
        if(UARTBuffer1[0] == 0x2B && UARTBuffer1[1] == 0x2B && UARTBuffer1[2] == 0x2B) {
          xBeeConf = 15000;
          UART1Init(9600);

        }

        if(transceiver == LORA && LoRa_bindMode_master == 0) {
          set_tx_flag((uint8_t *)(UARTBuffer2), UARTCount2);
        }
        else if (transceiver == XBEE) {
          UART1Send((uint8_t *)(UARTBuffer1), UARTCount2);
        }
      }
    }
    else {
      // check normal packet
      if(crc_calc2 == 0) {

        switch (UARTBuffer2[1]) {
          case CMD_RUN_GET_LOADER_VER: {
            unsigned short ver = getVersionB();
            UARTBuffer2[2] = MACK_OK;
            UARTBuffer2[3] = ver / 0x100;
            UARTBuffer2[4] = ver % 0x100;
            number_TX_bytes2 = 5;

            goto TX;
            break;
          }
          case CMD_RUN_GET_VERSION: {
            unsigned short ver = getVersion();
            UARTBuffer2[2] = MACK_OK;
            UARTBuffer2[3] = ver / 0x100;
            UARTBuffer2[4] = ver % 0x100;

            number_TX_bytes2 = 5;
            goto TX;
            break;
          }
          case CMD_RUN_GET_VOLTAGE: {
            float voltage = gpio_U();
            number_TX_bytes2 = mcmd_read_float(voltage, (char *)UARTBuffer2);
            goto TX;
            break;
          }
          case CMD_ZB2RS_MASTER_SLAVE: {
            conv_mode = UARTBuffer2[2];
#if (STORAGE_TYPE == STORAGE_FLASH)
          flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
          eeprom_write(SYS_VARS_EE); 
#endif  
          UARTBuffer2[2] = MACK_OK;
          number_TX_bytes2 = 3;
          goto TX;
          break;
        }
        case CMD_ZB2RS_RESET: {   
          flags |= (1 << reset_it);   
          reset_status = RESET_MANUAL;
          UARTBuffer2[2] = MACK_OK;
          number_TX_bytes2 = 3;
          goto TX;    
          break;    
        }

        case INTCOM_CONV_GET_SN_BY_ID: {

          UARTBuffer2[2] = (SN[0] >> 24) & 0xff;
          UARTBuffer2[3] = (SN[0] >> 16) & 0xff;
          UARTBuffer2[4] = (SN[0] >> 8)  & 0xff;  
          UARTBuffer2[5] =  SN[0]        & 0xff;  

          UARTBuffer2[6] = (SN[1] >> 24) & 0xff;
          UARTBuffer2[7] = (SN[1] >> 16) & 0xff;
          UARTBuffer2[8] = (SN[1] >> 8)  & 0xff;  
          UARTBuffer2[9] = SN[1]         & 0xff; 

          UARTBuffer2[10] = (SN[2] >> 24) & 0xff;
          UARTBuffer2[11] = (SN[2] >> 16) & 0xff;
          UARTBuffer2[12] = (SN[2] >> 8)  & 0xff;  
          UARTBuffer2[13] =  SN[2]        & 0xff; 

          UARTBuffer2[14] = (SN[3] >> 24) & 0xff;
          UARTBuffer2[15] = (SN[3] >> 16) & 0xff;
          UARTBuffer2[16] = (SN[3] >> 8)  & 0xff;  
          UARTBuffer2[17] =  SN[3]        & 0xff;

          number_TX_bytes2 = 18;
          goto TX; 
          break;    
        }

        case INTCOM_LORA_RESET_ROUTES: {
          for(int i=0 ; i<164 ; i++)
            for(int n=0 ; n<MAX_ROUTE_HOPS ; n++)
              LoRa_route[i][n] = 0; 

#if (STORAGE_TYPE == STORAGE_FLASH)
          flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
          eeprom_write(SYS_VARS_EE); 
#endif             

          UARTBuffer2[2] = MACK_OK;
          number_TX_bytes2 = 3;
          goto TX;    
          break;             
        }        

        case INTCOM_LORA_SET_SETTINGS: {

          if(LoRa_bindMode_master == 0){
            original.channel  = module.channel;
            original.power    = module.power;
            original.spFactor = module.spFactor;
            original.LoRa_BW  = module.LoRa_BW;
          }

          if(UARTBuffer2[2] != 0xff)
            module.channel = UARTBuffer2[2];   //if not bind_by_channel
          module.power = UARTBuffer2[3] & 0x03;  
          module.spFactor = (UARTBuffer2[3] & 0xf0) >> 4;
          module.LoRa_BW =  UARTBuffer2[4];

          


          if(UARTBuffer2[0] == 0xff){
            number_TX_bytes2 = 5;
            crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
            UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
            UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;

            get_route_order();
            tx_setting_route = MAX_SLAVE_ADDR;
            memcpy((char *)tx_settings_buffer, (char *) UARTBuffer2, number_TX_bytes2);
            number_TX_settings_bytes = number_TX_bytes2;
            tx_settings_flag = 1;                   //send settings through routes
          }


          if(UARTBuffer2[3] & 0x04){ //Enter bindmode command
            if(LoRa_bindMode_master == 0){
              if(UARTBuffer2[2] == 0xff)
                LoRa_Bind_Mode(MASTER_BY_CHANNEL);  //binding with the same channel
              else
                LoRa_Bind_Mode(MASTER); //Binding settings 488bps
              //memcpy((uint8_t*)BindPacket, (uint8_t*)UARTBuffer2, 8);
            //  LoRa_TxPacket((uint8_t *)(UARTBuffer2), number_TX_bytes2, 8000);   //Transmitt settings to slaves
            }

          } else if(LoRa_bindMode_master == 1)
            LoRa_Bind_Mode(DISABLE); 

            


          if(UARTBuffer2[0] < 0xff){              //transmitt original settings
            UARTBuffer2[2] = original.channel;
            UARTBuffer2[3] = original.power | (original.spFactor << 4);
            UARTBuffer2[4] = original.LoRa_BW;

            number_TX_bytes2 = 5;
            crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
            UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
            UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;

            set_tx_flag((uint8_t *)(UARTBuffer2), number_TX_bytes2);   //Transmitt settings to slaves
            set_settings_flag = 1;
          }

#if (STORAGE_TYPE == STORAGE_FLASH)
          flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
          eeprom_write(SYS_VARS_EE); 
#endif   

          if(UARTBuffer2[0] == 0xff){
            UARTBuffer2[2] = MACK_OK;
            number_TX_bytes2 = 3;
            goto TX;
          }
        }

        case INTCOM_LORA_GET_SETTINGS: {

          UARTBuffer2[2] = MACK_OK;
          UARTBuffer2[3] = module.channel;
          UARTBuffer2[4] = module.power | (module.spFactor << 4);
          if(LoRa_bindMode_master)
          UARTBuffer2[4] |= 0x04;  // Bind mode return bit
          UARTBuffer2[5] = module.LoRa_BW;
          
          number_TX_bytes2 = 6;
          goto TX;
          break;
        }  

        case INTCOM_LORA_GET_SLAVES: {
          for(int i=0 ; i<8 ; i++){
            UARTBuffer2[i+2] = (online_slaves >> i*8)  & 0xff;
          }
          number_TX_bytes2 = 10;
          goto TX;
          break;
        }

        case INTCOM_LORA_GET_ROUTE: {
          for(int i = 0; i < MAX_ROUTE_HOPS; i++) {
              UARTBuffer2[i + 2] = LoRa_route[UARTBuffer2[0]][i];
          }
          number_TX_bytes2 = 8;
          goto TX;
          break;
        }

        case INTCOM_LORA_SET_ROUTE: {

          int destination_id = 0;                         //find destination id
          for(int i = 0; i < MAX_ROUTE_HOPS ; i++)
            if(UARTBuffer2[2+i] == 0)
              break;
            else
              destination_id = UARTBuffer2[2+i];           //

          for(int i = 0; i < MAX_ROUTE_HOPS ; i++){
            LoRa_route[(unsigned char)(UARTBuffer2[0])][i] = 0; //Set pozicioner routes to 0
            LoRa_route[destination_id] [i] = 0;                 //set converter routes to 0
            }
          for(int i = 0; i < MAX_ROUTE_HOPS ; i++) {
            LoRa_route[(unsigned char)(UARTBuffer2[0])][i] = UARTBuffer2[i + 2]; // set positioner route
            LoRa_route[(unsigned char)(UARTBuffer2[0])][i] = UARTBuffer2[i + 2]; // set positioner route
            LoRa_route[destination_id ] [i] = UARTBuffer2[i + 2]; // set converter   route  
            if(destination_id == UARTBuffer2[i + 2]){
              if(i < 1 ){
                LoRa_route[(unsigned char)(UARTBuffer2[0])][i] = 0; //Set pozicioner route 0 to 0 if not routed (may not be needed)
                LoRa_route[destination_id] [i] = 0;                 //set converter route 0 to 0 if not routed (may not be needed)
              }  
              break;  //when destination id is reached, stop writing in LoRa_route array
            }
          }

          

#if (STORAGE_TYPE == STORAGE_FLASH)
          flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
          eeprom_write(SYS_VARS_EE); 
#endif   

          UARTBuffer2[2] = MACK_OK;
          number_TX_bytes2 = 3;

          goto TX;
          break;
        }

      }
    }
  }

  UARTCount2 = 0;
  number_TX_bytes1 = 0;
  return;

  TX: // answer for master settings
  if(UARTBuffer2[1] == CMD_RUN_GET_LOADER_VER || 
     UARTBuffer2[1] == CMD_RUN_GET_VERSION || 
     UARTBuffer2[1] == CMD_RUN_GET_VOLTAGE || 
     UARTBuffer2[1] == CMD_ZB2RS_RESET || 
     UARTBuffer2[1] == CMD_ZB2RS_MASTER_SLAVE || 
     UARTBuffer2[1] == INTCOM_CONV_GET_SN_BY_ID || 
     UARTBuffer2[1] == INTCOM_LORA_SET_SETTINGS || 
     UARTBuffer2[1] == INTCOM_LORA_GET_SETTINGS || 
     UARTBuffer2[1] == INTCOM_LORA_GET_SLAVES || 
     UARTBuffer2[1] == INTCOM_LORA_GET_ROUTE || 
     UARTBuffer2[1] == INTCOM_LORA_SET_ROUTE ||
     UARTBuffer2[1] == INTCOM_LORA_RESET_ROUTES) {

    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
    UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
    UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;

    UART2Send((uint8_t *)(UARTBuffer2), number_TX_bytes2);

    UARTCount2 = 0;
    number_TX_bytes2 = 0;

    if (flags & (1 << reset_it)) {    
      while (rxTimeout0 < 1000);    
#if (STORAGE_TYPE == STORAGE_FLASH)
      flash_write(SYS_VARS_ADDR);                   //save parameters
#elif (STORAGE_TYPE == STORAGE_EEPROM)
      eeprom_write(SYS_VARS_EE); 
#endif    
      LPC_WWDT->FEED = 0xAA;            
      LPC_WWDT->FEED = 0x50;    
      while(1);   
    }
  }
}

const char cmd2[] = {CMD_GET_STATUS, MCMD_R_All_PARAM, MCMD_R_boot_ver, MCMD_R_status, MCMD_R_events, MCMD_R_serial_numbers, MCMD_R_version};
char cmdIdx = 0;

/*void modbus_cmd_next2() {
  cmdIdx++;
  if(cmdIdx >= sizeof(cmd2))
    cmdIdx = 0;
}

void modbus_cmd_data2() {
  UARTBuffer2[0] = slave_addr;
  UARTBuffer2[1] = cmd2[cmdIdx];
  crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, 2, CRC_NORMAL);
  UARTBuffer2[2] = crc_calc2 & 0xFF;
  UARTBuffer2[3] = crc_calc2 / 0x100;
  number_TX_bytes2 = 4;
  UART2Send((uint8_t *)UARTBuffer2, number_TX_bytes2);
  number_TX_bytes2 = 0;
  modbus_cmd_next2();
}*/


unsigned int FloatToUIntBytes(float val) {
  unsigned int tmp = *((unsigned int *)((unsigned int) & val));
  return tmp;
}

/***********************************************************
  CRC MODBUS
************************************************************/
unsigned int modbus_crc(uint8_t *UARTBuff, int length, unsigned int crc_calc) {
  short lsb;
  unsigned int j,i;

  for (j = 0; j < length; j++)
  {               
    crc_calc ^= *(UARTBuff + j);  //XOR

    for (i = 0; i < 8; i++) //ponavljamo 8x - bayt
    {                                                                                     
      lsb = crc_calc & 0x0001;

      if (lsb)
      {
        crc_calc >>= 1;  //shiftamo za en bit v levo
        crc_calc &= 0x7FFF;
        crc_calc ^= 0xA001;     // crc polinom = 0xA001                                 
      }
      else
      {
        crc_calc >>= 1;  //shiftamo za en bit v levo
        crc_calc &= 0x7FFF;     
      }
    }
  }
  return crc_calc;
}

unsigned int mcmd_read_float(float param, char *pchData) {      //float
  float abc [1];                //kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  abc[0] = param;
  temp = *p;

  pchData[2] = temp / 0x1000000;
  pchData[3] = (temp / 0x10000) & 0xFF;
  pchData[4] = (temp / 0x100) & 0xFF;
  pchData[5] = (temp) & 0xFF;
  return 6;
}

// prepare packet for sending via xBee    
unsigned int xbSendPacketPrepare(char *pchData, unsigned int uiLength)    
{   
  unsigned char ucDeviceId = pchData[0];    
  unsigned int xbLength = 0;    
  unsigned int xbSum = 0;   
  memset(xbData, 0xff, BUFSIZE);    

  // packet to xBee   
  xbLength = uiLength + 18;   
  xbData[0] = 0x7E;           // frame delimiter    
  xbData[1] = (xbLength - 4) / 0x100;   // upper of length
  xbData[2] = (xbLength - 4) % 0x100;   // lower of length
  xbData[3] = 0x10;           // type Tx    
  xbData[4] = 0x0;            // frame ID   
  xbData[5] = 0xFF;        // 64 bit address upper    
  xbData[6] = 0xFF;        // 64 bit address upper    
  xbData[7] = 0xFF;        // 64 bit address upper    
  xbData[8] = 0xFF;        // 64 bit address upper    
  xbData[9] = 0xFF;        // 64 bit address lower    
  xbData[10] = 0xFF;       // 64 bit address lower    
  xbData[11] = 0xFF;       // 64 bit address lower    
  xbData[12] = 0xFF;       // 64 bit address lower    
  xbData[13] = 0x00;      // 16 bit address   
  xbData[14] = 0x00;      // 16 bit address   
  xbData[15] = 0x0;       // broadcast radius   
  xbData[16] = 0x0;       // Tx options   
  // data frame   
  for(int i = 0; i < uiLength; i++)   
    xbData[i + 17] = pchData[i];    
  // checksum   
  for(int i = 3; i < xbLength - 1; i++)   
    xbSum += xbData[i];   
  xbSum = 0xFF - (xbSum % 256);   
  xbData[xbLength - 1] = xbSum;   
  // escape characters (0x7E, 0x7D, 0x11, 0x13)   
  for(int i = 1; i < xbLength; i++) {   
    if(xbData[i] == 0x7E || xbData[i] == 0x7D || xbData[i] == 0x11 || xbData[i] == 0x13) {    
      xbData[i] ^= 0x20; // XOR escaped byte    
      for(int j = xbLength - 1; j >= i; j--)    
        xbData[j + 1] = xbData[j]; // move bytes one place higher   
      xbData[i] = 0x7D; // escape data byte   
      xbLength++; // increase all frame length    
    }   
  }   
  return xbLength;    
}

// restore packet for sending via xBee
unsigned int xbReceivePacketRestore(char *pchBuffer)    
{   
  unsigned int xbSum = 0;   
  // packet from xBee   
  memset(xbData, 0xff, BUFSIZE);    
  memcpy(xbData, pchBuffer, BUFSIZE);   
  // escape characters (0x7E, 0x7D, 0x11, 0x13)   
  for(int i = 1; i < BUFSIZE; i++) {    
    if(xbData[i] == 0x7D) {   
      xbData[i + 1] ^= 0x20; // XOR escaped byte    
      for(int j = i; j <= BUFSIZE - 1; j++)     
        xbData[j] = xbData[j + 1]; // move bytes one place lower    
      xbData[BUFSIZE - 1] = 0x00;   
    }   
  }   
  // not recived package    
  if(xbData[3] != 0x90)   
    return -1;

  // checksum   
  unsigned int xbLength = xbData[1] * 0x100 + xbData[2] + 4;
  for(int i = 3; i < xbLength - 1; i++)   
    xbSum += xbData[i];   
  xbSum = 0xFF - (xbSum % 256);
  if(xbData[xbLength - 1] != xbSum)   
    return -2;  

  // data frame   
  memset(pchBuffer, 0xff, BUFSIZE);   
  for(int i = 15; i < xbLength - 1; i++)    
    pchBuffer[i - 15] = xbData[i];    
  return (xbLength - 16);                       //payload length  
}

unsigned short getVersionB() {

  unsigned short ver;
  unsigned char *addr = (unsigned char *)(BOOT_ADDR);       

  ver = *(addr + 1) * 0x100 + *addr;

  return ver;
}

unsigned short getVersion() {

  return swVersion.sw_version;
}

/***********************************************************
  REST POSITION
************************************************************/
void modbus_timeout_handling(unsigned int *modbus_cnt) {
  if ((slave_addr >= MIN_SLAVE_ADDR) || (slave_addr <= MAX_SLAVE_ADDR)) {
    unsigned long long mtimeout = (modbus_timeout * 1000) + (modbus_timeout_delay * 1000 * (slave_addr - 1));
    if(mtimeout > 0xffffffff)
      mtimeout = 0xffffffff;                    //limit value
  
    if (modbus_timeout) {                       //timeout enabled   
      if (modbus_cnt >= mtimeout) {   //sekunde
        flags |= Modbus_timeout;
      }else{
      *modbus_cnt++;
      flags &= ~Modbus_timeout;
    } else {
      *modbus_cnt = 0;
      flags &= ~Modbus_timeout;
    }
  } else
    flags &= ~Modbus_timeout;
}
/////////////////////////////////////////////////////////////


uint8_t check_slaves(long long slaves, int timeout){
  uint8_t no_answer = 0;

  for(int i=1 ; i<64 ; i++){
    if(slaves & (1<<(i-1))){
      UARTBuffer2[0] = i + 99;
      UARTBuffer2[1] = CMD_RUN_GET_VERSION;

      uint8_t number_TX_bytes2 = 2;
      crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
      UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
      UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;  

      set_tx_flag((uint8_t *)(UARTBuffer2), number_TX_bytes2); 


      for(volatile int j=0 ; j < timeout ; j++){
        if(transceiver == LORA && module.packetReady){
          memcpy((char *)UARTBuffer2, (char *)module.rxBuffer, BUFSIZE);
          crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, module.packetLength, CRC_NORMAL);

          if(crc_calc2 == 0){
            no_answer = 0;
            module.packetReady = 0;
            break;
          }else no_answer = 1;       
        } 
      }
      if(no_answer)
        return 0;   
    }
  }
  return 1;
}    

void get_route_order(){   
    
  uint8_t n_routes = 0;   
  memset(routeOrders, 0x00, MAX_SLAVE_ADDR*MAX_ROUTE_HOPS);   
    
  for(int n=1 ; n<MAX_ROUTE_HOPS ; n++){    
    for(int i=0 ; i<=MAX_SLAVE_ADDR ; i++){   
      if((LoRa_route[i][n]) && check_coexistance(i,n)){   
        for(int m=0 ; m<=n ; m++){    
          routeOrders[n_routes][m] = LoRa_route[i][m];    
        }   
        n_routes++;   
      }   
    }   
  }   
}

uint8_t check_coexistance(int i, int n){    
    
  uint8_t LoRa_id = LoRa_route[i][n];   
  i--; //dont detect itself as the same   
    
  for( ; n>=0 ; n--){   
    for( ; i>=0 ; i--){   
      if(LoRa_id == LoRa_route[i][n])   
        return 0;   
    }   
    i=MAX_SLAVE_ADDR;   
  }   
  return 1;   
}
