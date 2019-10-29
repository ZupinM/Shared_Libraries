#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include "main.h"
#include "bldc.h"
#include "rtc.h"
#include "config.h" 
#include "suntracer.h"
#include "LPC15xx.h"
//#include "core_cmFunc.h"

#define flash_backup_ui ((unsigned int *)(&flash_backup[0]))
extern uint32_t SystemCoreClock;
extern unsigned int tracker_exstatus;
extern unsigned int tracker_status;
extern uint32_t cflags;
extern unsigned int bflags;
extern unsigned int buyflags;
extern unsigned int FocusMiddleA,FocusMiddleB;
extern time_t lastSyncTime;
extern uint8_t slave_addr;
extern unsigned int FlashWriteCounter;
extern unsigned int events;
extern unsigned int reset_status;
extern unsigned int SN[4];	

extern float err_currentA;
extern float err_positionA;
extern float err_voltageA;
extern float err_currentB;
extern float err_positionB;
extern float err_voltageB;

extern uint8_t voltage_select_0;
extern uint8_t voltage_select_1;

extern uint8_t LoRa_id;          //LoRa slave ID

extern float max_line_resistance;

extern unsigned int modbus_timeout;			// timeout, ko ni MODBUS komunikacije [sekunde]
extern unsigned int modbus_timeout_delay;

extern unsigned char pcb_version1;                //TIV           27   27C1=0x1B 0x43 0x01 .... 0x1B=27 0x43='C' 0x01=1 .... vpisi 0x1B4301 oziroma 1786625
extern char pcb_version2;                         //verzija TIVa  C
extern unsigned char pcb_version3;                //TIV polaganje 1

extern unsigned char ES_0_normallyOpenLo;
extern unsigned char ES_0_normallyOpenHi;
extern unsigned char ES_1_normallyOpenLo;
extern unsigned char ES_1_normallyOpenHi;

extern volatile unsigned int number_of_poles;

void flash_read (unsigned int read_address);
void eeprom_read (unsigned int read_address);
void update_flash_backup();
void flash_erase();
void eeprom_write(unsigned int write_address);
void read_SN();
  


#endif