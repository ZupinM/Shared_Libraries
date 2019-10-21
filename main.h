/*****************************************************************************
 *		Kvark header file
 *
 *		filename: main.h  
 *		pcb: tiv30F
 *
 *		Copyright(C) 2017, Sat Control d.o.o.
 *		All rights reserved.
 *
******************************************************************************/

#ifndef __MAIN__
#define __MAIN__

#include "config.h"
#include "uart.h"
#include <stdint.h>

#include <stdint.h>
#define LORA 1
#define XBEE 2
#define NONE 0


typedef struct {
  unsigned short int const sw_version;
  unsigned short int const blank1;
  unsigned short int const blank2;
  unsigned short int const blank3;
  unsigned short int const blank4;
  unsigned short int const blank5;
  unsigned short int const blank6;
  unsigned short int const blank7;
  unsigned short int const blank8;
}Version; 


extern uint8_t transceiver;
extern uint8_t transceiver_saved;
extern uint8_t LoRa_channel_received;
extern volatile uint8_t transmission;
extern volatile uint8_t lora_int_stat;
extern volatile uint8_t LoRa_bindMode;
extern  uint8_t LoRa_channel_received;
extern uint8_t LoRa_bindMode_slave;
extern int baudrate_timeout;
extern uint8_t checkRouting;

extern volatile uint8_t UARTBuffer0[BUFSIZE];

extern uint16_t online_timeouts[164];
extern uint8_t available_positioners[20];


void ActivateEvent(unsigned int ev);
void SetEventParameters(unsigned char motor);
void ClearStatus();
int realMain(char* reserve);
void HallVoltage(void);
void update_flash_backup();
void set_tx_flag(uint8_t* tx_buffer, uint8_t length);
void bindByChannel_check();
void onlineDevices_check();
int isOnlineDevice(unsigned int dev);
void auto_BaudRate();
void bindMode_check();

#endif
