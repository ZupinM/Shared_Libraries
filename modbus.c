/**************************************************************************
 *		"Pico" solar positioner software
 *
 *		filename: modbus.c  
 *		pcb: tiv29B
 *
 *		Copyright(C) 2011, Sat Control d.o.o.
 *		All rights reserved.
**************************************************************************/

#include "LPC15xx.h"
#include "modbus.h"
#include "config.h"
#include "gpio.h"
#include <string.h>
#include "aes.h"
#include "bldc.h"
#include "main.h"
#include "suntracer.h"
#include "SX1278.h"
#include "eeprom.h"

uint8_t writePacket1[0x80];
uint8_t writePacket2[0x80];
#define MODE_NORMAL 0
#define MODE_BOOT 1
#define MODE_UPGRADE 2
#define MODE_ERROR 3


unsigned int crc_calc;
unsigned int crc_calc1;
unsigned int crc_calc2;
unsigned int number_TX_bytes;
unsigned char m_ack_state;                              // vsebuje opis napake, za katero MODBUS ukaz NI bil izvrsen
unsigned int read_int_buf[30];
unsigned char broadcastID;                              // 1 = broadcast call, 0 = normal call by ID

unsigned char enabled = 0;
unsigned char missed_enable = 0;
unsigned char enable_tracking_retry = 0;
extern unsigned char enabled_in_micro;

extern uint8_t usb_drive;
extern float mzp_current;
extern uint8_t voltage_select_0;
extern uint8_t voltage_select_1;
extern float UVccHALL_0, UVccHALL_1;

extern MODE_TYPE mode;

unsigned int number_TX_bytes1;
unsigned int number_TX_bytes2;
extern volatile uint32_t UARTCount0;
extern volatile uint32_t UARTCount1;
extern volatile uint32_t UARTCount2;
extern volatile uint8_t UARTBuffer[BUFSIZE];
extern volatile uint8_t UARTBuffer0[BUFSIZE];
extern volatile uint8_t UARTBuffer1[BUFSIZE];
extern volatile uint8_t UARTBuffer2[BUFSIZE];
extern unsigned int backup_timeout;			// zakasnjen vpis v flash - backup
extern unsigned int modbus_indicator;			// stevec dolzine utripa ob rs485 sprejetju stringa

extern volatile unsigned int           bldc_Speed;     //RPM		
extern volatile unsigned int           number_of_poles;

extern unsigned int store_in_flash;

extern unsigned int bflags;

/* parameters */
extern unsigned int events;

extern unsigned int tracker_status;	
extern unsigned int tracker_exstatus;
extern uint8_t slave_addr;				// slave address on RS485
extern uint8_t LoRa_id;          //LoRa slave ID

extern unsigned int SN[4];				// vsebujejo serijske stevilke

extern unsigned int modbus_cnt;				// steje cas zadnjega modbus ukaza
extern unsigned int modbus_cnt1;        // steje cas zadnjega modbus ukaza
extern unsigned int modbus_cnt2;  
extern unsigned int modbus_timeout;			// timeout, ko ni MODBUS komunikacije [sekunde]
extern unsigned int modbus_timeout_delay;
extern unsigned int crc_errors;

extern unsigned int green_led;				// utripanje LED

/* flags registers */
extern unsigned int flags;

extern const Version swVersion;
extern volatile uint8_t UARTTxEmpty0;
extern volatile uint8_t UARTTxEmpty1;

extern float err_currentA;
extern float err_positionA;
extern float err_voltageA;
extern float err_currentB;
extern float err_positionB;
extern float err_voltageB;

extern float LineResistance;
extern float max_line_resistance;

extern volatile int rxTimeout0;

extern unsigned int reset_status;

extern uint16_t sigma_just_connected;
extern unsigned int tracker_status;

unsigned int xbLength;
char xbData[BUFSIZE];
extern volatile int delay_reset;
extern unsigned char uartMode;

#define ftoint(val) (*((unsigned int *)(unsigned int) & (val))) 
#define fsendval(val) (((ftoint(val) << 24) & 0xff000000) | ((ftoint(val) << 8) & 0xff0000) | ((ftoint(val) >> 24) & 0xff) | ((ftoint(val) >> 8) & 0xff00)) 

/***********************************************************
  MODBUS COMMANDS
************************************************************/
/***********************************************************
  RX from LORA (Sigma) for KVARK (this positioner)
************************************************************/
void modbus_cmd () {

  unsigned int rxcnt;
  unsigned int Utemp;
  unsigned int Utemp_old;
  float Ftemp;
  int eepromUpdate = 0;
  uint32_t UARTCount;

  if (uartMode == UART_MODE_XBEE) {
    unsigned int dataLength = 0;
    if (UARTBuffer1[0] == 0x7E) {

      // check XBEE connection AT command response
      if (UARTBuffer1[3] == 0x88) {
        transceiver = XBEE;
        UARTCount1 = 0;
        return;
      }

      dataLength = xbReceivePacketRestore((char *)UARTBuffer1, UARTCount1);
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
      memcpy((char *)UARTBuffer, (char *)UARTBuffer1, BUFSIZE);
      crc_calc1 = modbus_crc((uint8_t *)UARTBuffer, UARTCount, CRC_NORMAL);
    }
    else {
      UARTCount1 = 0;
      return;
    }
  } 
  else if (transceiver == LORA){
    memcpy((char *)UARTBuffer, (char *)module.rxBuffer, module.packetLength);
    UARTCount = module.packetLength;
  } 
  else if (uartMode == UART_MODE_RS485) {
    UARTCount = UARTCount0;
    memcpy((char *)UARTBuffer, (char *)UARTBuffer0, BUFSIZE);
  }

	
  //----- broadcast naslov? ------
  broadcastID = NO_BROADCAST;
  if ((UARTCount > 0) && (UARTBuffer[0] == 0))
    broadcastID = BROADCAST_CALL;
  //-------------------------------

  if ((UARTCount > 0) && (
   (UARTBuffer[0] == slave_addr) ||
   (UARTBuffer[0] == LoRa_id && transceiver == LORA) ||
   (broadcastID == BROADCAST_CALL) ||
   (UARTBuffer[0] != slave_addr && transceiver == XBEE && ((UARTBuffer[1] == CMD_RUN_GET_VOLTAGE || UARTBuffer[1] == CMD_RUN_GET_LOADER_VER ||
     UARTBuffer[1] == CMD_RUN_GET_VERSION) && crc_calc1 == 0))
     )) {

    crc_calc = modbus_crc((uint8_t *)UARTBuffer, UARTCount, CRC_NORMAL);

    if (crc_calc == 0) {
     //fsta baudrate_timeout = 0;
      if(transceiver == NONE)
        baudrate_timeout = 1;
      rxcnt = UARTCount - 2;
      modbus_indicator = 10;
      //green_led=25;		//utrip zelene LED, komunikacija RS485

      m_ack_state=0;
      modbus_cnt=0;             //no_modbus timeout

//fsta
//debug_printf("cmd: %.2x\n", UARTBuffer[1]);

      if(((UARTBuffer[0] == LoRa_id ||                       // slaveID or broadcast(0x0) for SN setting, set settings
            UARTBuffer[0] == 0x00 ||
            UARTBuffer[1] == INTCOM_LORA_SET_ID_BY_SN ||
            UARTBuffer[1] == INTCOM_LORA_SET_SETTINGS ||
            UARTBuffer[1] == INTCOM_LORA_GET_RSSI ||
              (UARTBuffer[0] == 0xFF &&                          // broadcast(0xFF) for SN getting
              UARTBuffer[1] == INTCOM_LORA_GET_SN_BY_ID)) &&
          transceiver == LORA) ||                               // << LoRa condition
        ((UARTBuffer[1] == CMD_RUN_GET_LOADER_VER ||
        UARTBuffer[1] == CMD_RUN_GET_VERSION ||
        UARTBuffer[1] == CMD_RUN_GET_VOLTAGE
        )
        && (transceiver == XBEE))) {                                  // ZigBee condition
    
    mode = MODE_NORMAL;

 //   UARTBuffer[0] = UARTBuffer[0];
   // UARTBuffer[1] = UARTBuffer[1];

    switch (UARTBuffer[1]) {

     case CMD_RUN_GET_LOADER_VER: {
        unsigned short ver = getVersionB();
        UARTBuffer[2] = MACK_OK;
        UARTBuffer[3] = ver / 0x100;
        UARTBuffer[4] = ver % 0x100;
        number_TX_bytes = 5;
        append_crc();
        goto TX;
        break;
      }
      case CMD_RUN_GET_VERSION: {

        unsigned short ver = getVersion();
        UARTBuffer[2] = MACK_OK;
        UARTBuffer[3] = ver / 0x100;
        UARTBuffer[4] = ver % 0x100;
        number_TX_bytes = 5;
        append_crc();
        goto TX;
        break;
      }
      case CMD_RUN_GET_VOLTAGE: {
        float voltage = bldc_U(SUPPLY);
        number_TX_bytes = mcmd_read_float_conv(voltage, (char*)UARTBuffer);
        append_crc();
        goto TX;
        break;
      }
      case CMD_ZB2RS_RESET: { 
        if (UARTBuffer[0] == LoRa_id || transceiver == XBEE) {
          flags |= (1 << reset_it);   
          reset_status = RESET_MANUAL;
          UARTBuffer[2] = MACK_OK;
          number_TX_bytes = 3;
          append_crc();
          goto TX;
        }
        break;    
      }

      case INTCOM_LORA_SET_SETTINGS: {

        if(!(UARTBuffer[0] == 0x0 || UARTBuffer[0] == 0xFF || UARTBuffer[0] == slave_addr)) {
          UARTCount = 0;
          number_TX_bytes = 0;
          return;
        }

        if(UARTBuffer[2] != 0xff) 
          module.channel = UARTBuffer[2];
        module.power = UARTBuffer[3] & 0x03;
        module.spFactor = (UARTBuffer[3] & 0xf0) >> 4;
        module.LoRa_BW =  UARTBuffer[4]; 
        eeprom_write(SYS_VARS_EE); 

        if(UARTBuffer[3] & 0x04){
          LoRa_channel_received = 1;
          UARTBuffer[2] = LoRa_get_rssi();
          number_TX_bytes = 3;
          append_crc();
          goto TX;                                  
        }

        LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET,  RxMode); //Set settings to slave
        //timeout_master_check = 400000;
        if(UARTBuffer[0] < 0xff){
          UARTBuffer[2] = MACK_OK;
          number_TX_bytes = 3;
          append_crc();
          goto TX;                     
        }

        break;
      }

        case INTCOM_LORA_SET_ID_BY_SN: { 
          int SNrx[4];  
          int updateSuccess = 0;

          if(UARTBuffer[0] == slave_addr){
            LoRa_id = UARTBuffer[18];
            updateSuccess = 1;
            //UARTBuffer[2] = MACK_OK;
          }else/* if(UARTBuffer[0] == 0xff || UARTBuffer[0] == 0x00)*/{

            SNrx[0]  = UARTBuffer[2] << 24;
            SNrx[0] |= UARTBuffer[3] << 16;
            SNrx[0] |= UARTBuffer[4] << 8;
            SNrx[0] |= UARTBuffer[5];

            SNrx[1]  = UARTBuffer[6] << 24;
            SNrx[1] |= UARTBuffer[7] << 16;
            SNrx[1] |= UARTBuffer[8] << 8;
            SNrx[1] |= UARTBuffer[9];

            SNrx[2]  = UARTBuffer[10] << 24;
            SNrx[2] |= UARTBuffer[11] << 16;
            SNrx[2] |= UARTBuffer[12] << 8;
            SNrx[2] |= UARTBuffer[13];

            SNrx[3]  = UARTBuffer[14] << 24;
            SNrx[3] |= UARTBuffer[15] << 16;
            SNrx[3] |= UARTBuffer[16] << 8;
            SNrx[3] |= UARTBuffer[17];

            if(SNrx[0] == SN[0] && SNrx[1] == SN[1] && SNrx[2] == SN[2] && SNrx[3] == SN[3]){
              LoRa_id = UARTBuffer[18];
              updateSuccess = 1;
            }
          }


          if(updateSuccess){
            // save data
            eeprom_write(SYS_VARS_EE); 

            read_int_buf[0] = (available_positioners[0] << 24) | (available_positioners[1] << 16) | (available_positioners[2] << 8) | available_positioners[3] ;
            read_int_buf[2] = (available_positioners[4] << 24) | (available_positioners[5] << 16) | (available_positioners[6] << 8) | available_positioners[7] ;

            mcmd_read_int(2, slave_addr);
            goto TX;
          }else{
            UARTCount  = 0;
            number_TX_bytes = 0;
            return;
          }
          break;
        }

        case INTCOM_LORA_GET_SN_BY_ID: {
            read_int_buf[0] = SN[0];
            read_int_buf[1] = SN[1];
            read_int_buf[2] = SN[2];
            read_int_buf[3] = SN[3];


            mcmd_read_int(4, LoRa_id); 
            goto TX;
            break;
        }
        
        case INTCOM_LORA_GET_RSSI: {    
            if(UARTBuffer[0] == LoRa_id){   
              UARTBuffer[2] = LoRa_get_rssi();
              number_TX_bytes = 3;
              append_crc();
              goto TX;
            }
            break;
          }
        }
                        
      } else {


      switch (UARTBuffer[1]) {
        // reset 
        case MCD_W_reset: {					 	
          ack_reply();
          //fsta flags |= (1 << reset_it);
          delay_reset = 10;

          reset_status = RESET_MANUAL;

//fsta           resetEnable = 1;

          break;
        }
        // R STATUS
        case MCMD_R_status: {

//fstadebug_printf("tracker_status: %x\n",tracker_status);                                       

          read_int_buf[0] = tracker_status;
          read_int_buf[1] = tracker_exstatus;
          mcmd_read_int(2, slave_addr);
          break;
        }
        // Clear STATUS
        case MCMD_W_status: {
          // A axis					 	
          ClearStatus();
          RMeasure_Stop();
          ack_reply();
          backup_timeout = 200;           // 4 sekundi zatem backup v flash
          break;
        }
        // W SLAVE ADDRESS
        case MCMD_W_slave_addr: {  				 	
          if((UARTBuffer[2] > 0) && (UARTBuffer[2] <= 64))
          {
            Utemp=UARTBuffer[2];
            ack_reply();
            slave_addr = Utemp;
            eepromUpdate = 1;
            backup_timeout = 200;         //4 sekundi zatem backup v flash
          } else {
            m_ack_state=MACK_UNRECOGNIZED_CMD;
            err_reply();
          }
          break;
        }
				
        case MCMD_W_SERIAL_slave_addr: {
          Utemp = UARTBuffer[2];
          Utemp |= ((unsigned int)UARTBuffer[3]) << 8;
          Utemp |= ((unsigned int)UARTBuffer[4]) << 16;
          Utemp |= ((unsigned int)UARTBuffer[5]) << 24;
          
          if (SN[0]==Utemp) {
            if((UARTBuffer[6] > 0) && (UARTBuffer[6] <= 128)) {
              Utemp=UARTBuffer[6];
              ack_reply();
              slave_addr = Utemp;
              eepromUpdate = 1;
              backup_timeout = 200;       //4 sekundi zatem backup v flash
              break;
            }
          }
          m_ack_state=MACK_UNRECOGNIZED_CMD;
          err_reply();
          break;
        }

        // USUPPLY
        case MCMD_R_Usupply: {				   
          mcmd_read_float(bldc_U(SUPPLY));
          break;
        }
        // IMOTOR
        case MCMD_R_Imotor: {					   
          mcmd_read_float(mzp_current);
          break;
        }

        case MCMD_R_MSpeed: {		
          read_int_buf[0] = bldc_Speed;		
          mcmd_read_int(1, slave_addr);		
          break;		
        }		
        case MCMD_R_NPoles: {		
          read_int_buf[0] = number_of_poles;		
          mcmd_read_int(1, slave_addr);		
          break;		
        }		
        case MCMD_W_NPoles: {							
          number_of_poles = mcmd_write_int(1,24);		
          backup_timeout = 200;			
          break;		
        }

           
        // SERIAL NUMBERS
        case MCMD_R_serial_numbers: {			   
          read_int_buf[0] = SN[0];
          read_int_buf[1] = SN[1];
          read_int_buf[2] = SN[2];
          read_int_buf[3] = SN[3];
          mcmd_read_int(4, slave_addr);
          break;
        }
        // VERSION
        case MCMD_R_version: {
          float temp;
          temp = (float)swVersion.sw_version;
          temp /= 1000.0;				// verzija je napisana v int 			   			
          mcmd_read_float(temp);
          break;
        } 
        case MCMD_R_boot_ver: {
          read_int_buf[0] = BOOT_VERSION;
          read_int_buf[1] = BOOT_DEVTYPE;
          read_int_buf[2] = BOOT_HW_REV;
          read_int_buf[3] = BOOT_APP_MINVERSION;
          mcmd_read_int(4, slave_addr);
          break;
        }
        // REMAIN IMPULSES A
        case MCMD_R_remain_A: {                         // ostanek impulzov do 0000 od zadnjega REF			 							
          mcmd_read_float(bldc_remaining(0));
          break;
        }
        // REMAIN IMPULSES B
        case MCMD_R_remain_B: {                         // ostanek impulzov do 0000 od zadnjega REF			 							
          mcmd_read_float(bldc_remaining(1));
          break;
        }
        case MCMD_R_events: {
          read_int_buf[0] = events;
          events = 0;
          mcmd_read_int(1, slave_addr);
          tracker_exstatus &= ~EFS_EVENTS_ACTIVE;
          events = 0;
          break;
        }
        
        case  MCMD_R_errorA_stats: {			
          read_int_buf[0] = fsendval( err_currentA );
          read_int_buf[1] = fsendval( err_voltageA );
          read_int_buf[2] = fsendval( err_positionA);
          read_int_buf[3] = 0;
          read_int_buf[4] = 0;
          read_int_buf[5] = 0;
          read_int_buf[6] = 0;
          read_int_buf[7] = 0;
          mcmd_read_int(8, slave_addr);
          break;
        }
        case  MCMD_R_errorB_stats: {			
          read_int_buf[0] = fsendval( err_currentB );
          read_int_buf[1] = fsendval( err_voltageB );
          read_int_buf[2] = fsendval( err_positionB);
          read_int_buf[3] = 0;
          read_int_buf[4] = 0;
          read_int_buf[5] = 0;
          read_int_buf[6] = 0;
          read_int_buf[7] = 0;
          mcmd_read_int(8, slave_addr);
          break;
        }
        //***** commands *****

        // STOP
        case MCMD_W_stop_motor: {					
          //stop_motor ();	
          RMeasure_Stop();                                                
          bldc_manual(1);  // mzp
          bldc_Stop(1);
          bldc_runout(RUNOUT_ACTIVATE);
          store_in_flash = 100;
          ack_reply();
          break;
        }
        // R POSITION A
        case MCMD_R_position_A: {					
          mcmd_read_float(bldc_position(0));
          break;
        }
        // R DESTINATION A
        case MCMD_R_destination_A: {														
          mcmd_read_float(bldc_target(0));
          break;
        }
        // W DESTINATION A
        case MCMD_W_destination_A: {		                        
          int windmode = 0;

          if ((rxcnt == 7) && (UARTBuffer[6]))
            windmode = 1;

            bldc_manual(0);  // mzp
            usb_drive = 0;
        
          int res = bldc_setPosition(0, mcmd_write_float1(), windmode);

          if (res ==  0) {
            ack_reply();
            //eepromUpdate = 1;
            break;
          }
          if(res == -2)
            m_ack_state = MACK_SEE_STATUS_BYTE;
          if(res == -3)
            m_ack_state = MACK_NOT_USED_DURING_REF;
          else
            m_ack_state = MACK_SEE_STATUS_BYTE;

          err_reply();
          break;
        }
        //W DESTINATION A [ipmulses]
        case MCMD_W_destinationImp_A: {		                        
          int windmode = 0;

          if((rxcnt == 7) && (UARTBuffer[6]))
            windmode = 1;

          int res = bldc_setPositionImp(0, mcmd_write_int1(), windmode);

          if(res ==  0){
            ack_reply();
            //eepromUpdate = 1;
            break;
          }
          if(res == -2) 
            m_ack_state = MACK_SEE_STATUS_BYTE;
          if(res == -3) 
            m_ack_state = MACK_NOT_USED_DURING_REF;
          else          
            m_ack_state = MACK_SEE_STATUS_BYTE;

          err_reply();
          break;
        }
        // GO REFERENCE A - CLEAR POSITION
        case MCMD_W_ref_A: {									
             bldc_manual(0);  // mzp   
             usb_drive = 0;
          int res = bldc_Home(0);
          if (res == 0) {
            ack_reply();
            eepromUpdate = 1;
          }
          else {
            m_ack_state = MACK_SEE_STATUS_BYTE;
            err_reply();
          }
          break;
        }

        // R POSITION B
        case MCMD_R_position_B: {					
          mcmd_read_float(bldc_position(1));
          break;
        }
        // R DESTINATION B
        case MCMD_R_destination_B: {														
          mcmd_read_float(bldc_target(1));
          break;
        }
        // W DESTINATION B
        case MCMD_W_destination_B: {		                        
          int windmode = 0;

          if((rxcnt == 7) && (UARTBuffer[6])) windmode = 1;
          bldc_manual(0);  // mzp
          usb_drive = 0;
          int res = bldc_setPosition(1, mcmd_write_float1(), windmode);

          if(res ==  0) {
            ack_reply();
            //eepromUpdate = 1;
            break;
          }
          if(res == -2)
            m_ack_state = MACK_SEE_STATUS_BYTE;
          if(res == -3)
            m_ack_state = MACK_NOT_USED_DURING_REF;
          else
            m_ack_state = MACK_SEE_STATUS_BYTE;

          err_reply();
          break;
        }
        //W DESTINATION B [ipmulses]
        case MCMD_W_destinationImp_B: {		                        
          int windmode = 0;

          if((rxcnt == 7) && (UARTBuffer[6]))
            windmode = 1;

          int res = bldc_setPositionImp(1, mcmd_write_int1(), windmode);

          if(res ==  0) {
            ack_reply();
            //eepromUpdate = 1;
            break;
          }
          if(res == -2) 
            m_ack_state = MACK_SEE_STATUS_BYTE;
          if(res == -3) 
            m_ack_state = MACK_NOT_USED_DURING_REF;
          else          
            m_ack_state = MACK_SEE_STATUS_BYTE;

          err_reply();
          break;
        }
        //GO REFERENCE B - CLEAR POSITION
        case MCMD_W_ref_B: {									
                bldc_manual(0);  // mzp
                usb_drive  = 0;
          int res = bldc_Home(1);
          if(res == 0) {
            eepromUpdate = 1;
            ack_reply();
          }
          else {
             m_ack_state = MACK_SEE_STATUS_BYTE;
             err_reply();
          }
          break;
        }
        
        case MCMD_R_AxisState: {
          read_int_buf[0] = bldc_GetEnabledMotors();
          mcmd_read_int(1, slave_addr);
          break;
        }

        case MCMD_W_SetAxisState: {
          Utemp = mcmd_write_int(0, 0xffffffff);
          bldc_EnableMotors(Utemp);

          bflags&=~(1<<time_enable); //Disable micro tracking 

          if(sigma_just_connected < 2000)             //
            if(mode == MODE_MICRO){
              tracker_status |= SF_TRACKING_ENABLED;
              mode = MODE_SLAVE_TRACKING;
            }else if (mode==MODE_OK){
              tracker_status &= ~SF_TRACKING_ENABLED;
              mode = MODE_SLAVE;
            } 


          if(Utemp & (1<<18)){
            if (enabled < 8)
              enabled++;
          }else 
            if (enabled > 2)
              enabled--;

          if(Utemp & (1<<19)){
            missed_enable = 0;
            if(enabled < 5){
              tracker_status |= SF_TRACKING_ENABLED;
              mode = MODE_SLAVE_TRACKING;
            }else if(enabled >= 5){
              tracker_status &= SF_TRACKING_ENABLED;
              mode = MODE_SLAVE;
            }
          } 

          if ((Utemp&(1<<18))&&(mode==MODE_SLAVE)){ //zamujen preklop
              missed_enable++;
              if(missed_enable > 10){
                mode = MODE_SLAVE_TRACKING;
                tracker_status |= SF_TRACKING_ENABLED;
                missed_enable = 0;
              }
            }

          if ((!(Utemp&(1<<18)))&&(mode==MODE_SLAVE_TRACKING)){ //zamujen preklop
              missed_enable++;
              if(enabled_in_micro)
                enable_tracking_retry++;
              if(missed_enable > 3){
                mode = MODE_SLAVE;
                tracker_status &= ~SF_TRACKING_ENABLED;
                missed_enable = 0;       
              }
            }
          if(enable_tracking_retry) enable_tracking_retry++, enabled_in_micro = 0; 

          if (enable_tracking_retry > 6){
            enable_tracking_retry = 0;
            mode = MODE_SLAVE_TRACKING;
            tracker_status |= SF_TRACKING_ENABLED;
          }    
          //if (enable_tracking_retry > 3){
           // enable_tracking_retry = 0;
           // mode = MODE_SLAVE_TRACKING;
           // tracker_status |= SF_TRACKING_ENABLED;
          //}        



          /*                                                                     

          if(Utemp & (1<<18)){
            if(sigma_just_connected > 2000)
              //if (!(Utemp_old&(1<<18)))
                tracker_status |= SF_TRACKING_ENABLED;
            mode = MODE_SLAVE_TRACKING;
          }
          else{
            if(sigma_just_connected > 2000)
              //if (Utemp_old&(1<<18))
                tracker_status &= ~SF_TRACKING_ENABLED;
            mode = MODE_SLAVE;
          }
          Utemp_old = Utemp;  */

          //eepromUpdate = 1;    mzp
          break;
        }	

        case MCMD_W_Voltage_hall: {
          Utemp = mcmd_write_int(3, 24);		 //omejitev vpisa
            if((Utemp & 0xff) != 0)
              voltage_select_0 = Utemp & 0xff;
            if(((Utemp >> 8) & 0xff) != 0)
              voltage_select_1 = (Utemp >> 8) & 0xff;
                    //HallVoltage(voltage_hall);
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          break;		
        }
  
        case MCMD_R_Voltage_hall: {
          read_int_buf[0] = voltage_select_0 | (voltage_select_1 << 8);
          mcmd_read_int(1, slave_addr);
          break;				
        }

        case MCMD_R_Batt_voltage: {
          mcmd_read_float(get_batt_U());
          break;				
        }

        // U hall
        case MCMD_R_Uhall_0:	{				   
          mcmd_read_float(bldc_U(HALL0));
          break;
        }
        case MCMD_R_Uhall_1:	{				   
          mcmd_read_float(bldc_U(HALL1));
          break;
        }


        case MCMD_R_Hall_cntDown: {
          read_int_buf[0] = bldc_GetInvertHall(0) ;
          read_int_buf[0] |= bldc_GetInvertHall(1) * 2;
          mcmd_read_int(1, slave_addr);
          break;
        }
        
        case MCMD_W_Hall_cntDown: {
          Utemp = mcmd_write_int(0, 0xffffffff);
          bldc_SetInvertHall(0, Utemp & (1 << 0));
          bldc_SetInvertHall(1, Utemp & (1 << 1));
          store_in_flash = 100;
          break;
        }
        
        case MCMD_R_Invert_motor: {
          read_int_buf[0] = bldc_GetInvert(0);
          read_int_buf[0] |= bldc_GetInvert(1) * 2;
          mcmd_read_int(1, slave_addr);
          break;
        }
        
        case MCMD_W_Invert_motor: {
          Utemp = mcmd_write_int(0, 0xffffffff);
          bldc_SetInvert(0, Utemp & (1 << 0));
          bldc_SetInvert(1, Utemp & (1 << 1));
          store_in_flash = 100;
          break;
        }

        case MCMD_R_NC_EndSwitch: {
          read_int_buf[0] = ES_0_normallyOpenLo;
          read_int_buf[0] |= (ES_1_normallyOpenLo << 1);
          read_int_buf[0] |= (ES_0_normallyOpenHi << 2);
          read_int_buf[0] |= (ES_1_normallyOpenHi << 3);
          mcmd_read_int(1, slave_addr);
          break;
        }

        case MCMD_W_NC_EndSwitch:{
          Utemp = mcmd_write_int(0, 0xffffffff);
          ES_0_normallyOpenLo = Utemp & (1<<0);
          ES_1_normallyOpenLo = (Utemp & (1<<1)) >> 1;
          ES_0_normallyOpenHi = (Utemp & (1<<2)) >> 2;
          ES_1_normallyOpenHi = (Utemp & (1<<3)) >> 3;
          eepromUpdate = 1;
          backup_timeout = 200;
          break;
        }
  
        case MCMD_R_EndSwithDetectA: {
          Ftemp = bldc_Motor(0)->end_switchDetect;								
          mcmd_read_float(Ftemp);
          break;
        }

        case MCMD_W_EndSwithDetectA: {
          Ftemp = mcmd_write_limit_float( bldc_Motor(0)->min_position, bldc_Motor(0)->max_position,0);    //omejitve vpisa 
          if (m_ack_state == 0) {
            bldc_Motor(0)->end_switchDetect = (Ftemp);
            eepromUpdate = 1;
          }
          break;
        }

        // MIN RANGE A
        case MCMD_R_min_range_A: {
          mcmd_read_float(bldc_Motor(0)->min_position);
          break;
        }
        case MCMD_W_min_range_A: {
          Ftemp = mcmd_write_float(DEFAULT_MIN_RANGE, bldc_Motor(0)->max_position);	//omejitve vpisa, ` ne sme biti nad max_range
          //Ftemp=mcmd_write_float(0.0,10000);
          if (m_ack_state==0) {
            bldc_Motor(0)->min_position = Ftemp;
            eepromUpdate = 1;
            backup_timeout=200;                          //4 sekundi zatem backup v flash
          }
          break;
        }

        // MAX RANGE A
        case MCMD_R_max_range_A: {
          mcmd_read_float(bldc_Motor(0)->max_position);
          break;
        }
        case MCMD_W_max_range_A: { 
          Ftemp = mcmd_write_float(bldc_Motor(0)->min_position, 5000);	//omejitve vpisa, max_range ne sme biti pod min_range
          if (m_ack_state == 0) {
            bldc_Motor(0)->max_position = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;                       //4 sekundi zatem backup v flash
          }
          break;
        }
        
        case  MCMD_R_ZeroOffsetA: {
          mcmd_read_float(bldc_Motor(0)->home_offset);
          break;
        }
        
        case  MCMD_W_ZeroOffsetA:{
          Ftemp = mcmd_write_float(-bldc_Motor(0)->max_position, bldc_Motor(0)->max_position);		//omejitve vpisa, max_range ne sme biti pod min_range
          if (m_ack_state == 0) {
            bldc_Motor(0)->home_offset = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;                       //4 sekundi zatem backup v flash
          }
          break;
        }
				
        // MAX I MOTOR A
        case MCMD_R_max_Imotor_A: {					
          mcmd_read_float(bldc_Motor(0)->I_limit);
          break;
        }
        case MCMD_W_max_Imotor_A: {
          Ftemp = mcmd_write_float(0.1, 10.0);		//omejitve vpisa	   
          if (m_ack_state == 0) {
            bldc_Motor(0)->I_limit = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;			//4 sekundi zatem backup v flash
          }
          break;
        }
        
        case MCMD_R_EndSwithDetectB: {
          Ftemp = bldc_Motor(1)->end_switchDetect;								
          mcmd_read_float(Ftemp);
          break;
        }

        case MCMD_W_EndSwithDetectB: {
          Ftemp = mcmd_write_limit_float( bldc_Motor(1)->min_position, bldc_Motor(1)->max_position, 0);		//omejitve vpisa 
          if (m_ack_state == 0) {
            bldc_Motor(1)->end_switchDetect = (Ftemp);
            eepromUpdate = 1;
          }
          break;
        }

        //MIN RANGE	B
        case MCMD_R_min_range_B: {
          mcmd_read_float(bldc_Motor(1)->min_position);
          break;
        }
        case MCMD_W_min_range_B: {
          Ftemp = mcmd_write_float(DEFAULT_MIN_RANGE, bldc_Motor(1)->max_position);		//omejitve vpisa, ` ne sme biti nad max_range
          //Ftemp=mcmd_write_float(0.0,10000);
          if (m_ack_state == 0) {
            bldc_Motor(1)->min_position = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;		//4 sekundi zatem backup v flash
          }
          break;
        }

        //MAX RANGE	B
        case MCMD_R_max_range_B: {
          mcmd_read_float(bldc_Motor(1)->max_position);
          break;
        }
        case MCMD_W_max_range_B: { 
          Ftemp = mcmd_write_float(bldc_Motor(1)->min_position, 5000);		//omejitve vpisa, max_range ne sme biti pod min_range
          if (m_ack_state == 0) {
            bldc_Motor(1)->max_position = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;		//4 sekundi zatem backup v flash
          }
          break;
        }
        
        case  MCMD_R_ZeroOffsetB: {
          mcmd_read_float(bldc_Motor(1)->home_offset);
          break;
        }
        
        case  MCMD_W_ZeroOffsetB: {
          Ftemp = mcmd_write_float(-bldc_Motor(1)->max_position, bldc_Motor(1)->max_position);		//omejitve vpisa, max_range ne sme biti pod min_range
          if (m_ack_state == 0) {
            bldc_Motor(1)->home_offset = Ftemp;
            eepromUpdate = 1;
            backup_timeout=200;		//4 sekundi zatem backup v flash
          }
          break;
        }
        
        //MAX I MOTOR B
        case MCMD_R_max_Imotor_B: {					
          mcmd_read_float(bldc_Motor(1)->I_limit);
          break;
        }
        case MCMD_W_max_Imotor_B: {
          Ftemp = mcmd_write_float(0.1, 10.0);		//omejitve vpisa	   
          if (m_ack_state == 0) {
            bldc_Motor(1)->I_limit = Ftemp;
            eepromUpdate = 1;
            backup_timeout=200;					//4 sekundi zatem backup v flash
          }
          break;
        }

        //U SUPPLY FACTOR
        case MCMD_R_Usupply_factor: {
          mcmd_read_float(bldc_config()->UConvertRatio);
          break;
        }
        case MCMD_W_Usupply_factor: {
          Ftemp = mcmd_write_float(0.0, 500.0);		//omejitve vpisa
          if (m_ack_state == 0) {
            bldc_config()->UConvertRatio = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;			//4 sekundi zatem backup v flash
          }
          break;									
        }
        // I MOTOR FACTOR
        case MCMD_R_Imotor_factor: {
          mcmd_read_float(bldc_config()->IConvertRatio);
          break;
        }
        case MCMD_W_Imotor_factor: {
          Ftemp = mcmd_write_float(0.0, 500.0);
          if (m_ack_state == 0) {
            bldc_config()->IConvertRatio = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }		
          break;
        }
        //REST POSITION	A
        case MCMD_R_modbus_timeout_position_A: {			
          mcmd_read_float(bldc_Motor(0)->modbus_timeout_position);
          break;
        }
        case MCMD_W_modbus_timeout_position_A: {
          Ftemp = mcmd_write_float(bldc_Motor(0)->min_position, bldc_Motor(0)->max_position);		//omejitve vpisa, rest_position mora biti med min_range in max_range
          if (m_ack_state == 0) {
            bldc_Motor(0)->modbus_timeout_position = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }
          break;
        }	
        // REST POSITION B
        case MCMD_R_modbus_timeout_position_B: {			
          mcmd_read_float(bldc_Motor(1)->modbus_timeout_position);
          break;
        }
        case MCMD_W_modbus_timeout_position_B: {
          Ftemp = mcmd_write_float(bldc_Motor(1)->min_position, bldc_Motor(1)->max_position);		//omejitve vpisa, rest_position mora biti med min_range in max_range
          if (m_ack_state == 0) {
            bldc_Motor(1)->modbus_timeout_position = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }
          break;
        }

        // MODBUS TIMEOUT
        case MCMD_R_modbus_timeout: {			
          read_int_buf[0] = modbus_timeout;
          mcmd_read_int(1, slave_addr);
          break;
        }
        
        case MCMD_W_modbus_timeout: {
          Utemp = mcmd_write_int(0, 4320000);		//omejitve vpisa, 4320000sekund = 50 dni
          if (m_ack_state==0) {
            modbus_timeout = Utemp;
            eepromUpdate = 1;
            backup_timeout = 200;		//4 sekundi zatem backup v flash
          }
          break;
        }
        
        // MODBUS TIMEOUT DELAY
        case MCMD_R_modbus_timeout_delay: {
          read_int_buf[0] = modbus_timeout_delay;
          mcmd_read_int(1, slave_addr);
          break;
        }
        
        case MCMD_W_modbus_timeout_delay: {
          Utemp = mcmd_write_int(0, 4320000);         //omejitev vpisa
          if (m_ack_state == 0) {
            modbus_timeout_delay = Utemp;
            eepromUpdate = 1;
            backup_timeout = 200;                     //4 sekundi zatem backup v flash
          }			 
          break;	
        }

        //TOO LONG REFERENCE
        case MCMD_R_ref_toolong: {			
          read_int_buf[0] = bldc_config()->homing_timeout;
          mcmd_read_int(1, slave_addr);
          break;
        }
        case MCMD_W_ref_toolong: {
          Utemp = mcmd_write_int(60, 4320000);		//omejitve vpisa, 60 s ... 4320000sekund = 50 dni
          if (m_ack_state == 0) {
            bldc_config()->homing_timeout = Utemp;
            eepromUpdate = 1;
            backup_timeout = 200;                       //4 sekundi zatem backup v flash
          }
          break;
        }
        //GEAR RATIO A
        case MCMD_R_gear_ratio_A: {
          mcmd_read_float(bldc_Motor(0)->gear_ratio);
          break;
        }
        case MCMD_W_gear_ratio_A: {
          Ftemp = mcmd_write_float(0.0, 100000.0);		//omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(0)->gear_ratio = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }		
          break;
        }

        case MCMD_W_StratI_ratioA: {
          Ftemp = mcmd_write_float(1, 10);		 //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(0)->I_Inrush_ratio  =Ftemp;
            eepromUpdate = 1;
            backup_timeout=200;				//4 sekundi zatem backup v flash
          }
          break;
        }

        case MCMD_W_StratI_timeA:{
          Ftemp = mcmd_write_float(100, 1000);		 //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(0)->I_Inrush_time = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;			//4 sekundi zatem backup v flash
          }
          break;
        }
		
        case MCMD_R_StartI_ratioA: {
          mcmd_read_float(bldc_Motor(0)->I_Inrush_ratio);
          break;
        }

        case MCMD_R_StartI_timeA: {
          mcmd_read_float(bldc_Motor(0)->I_Inrush_time);
          break;
        }

        case MCMD_W_Detection_I_A: {
          Ftemp = mcmd_write_float(0.05, 4);		 //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(0)->Idetection = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;			//4 sekundi zatem backup v flash
          }
          break;				
        }
        
        case MCMD_R_Detection_I_A: {
          mcmd_read_float(bldc_Motor(0)->Idetection);
          break;				
        }

        // GEAR RATIO B
        case MCMD_R_gear_ratio_B: {
          mcmd_read_float(bldc_Motor(1)->gear_ratio);
          break;
        }
        case MCMD_W_gear_ratio_B: {
          Ftemp = mcmd_write_float(0.0, 100000.0);		 //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(1)->gear_ratio = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }		
          break;
        }
                        
        case MCMD_W_StratI_ratioB: {
          Ftemp = mcmd_write_float(1, 10);		 //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(1)->I_Inrush_ratio = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }
          break;
        }

        case MCMD_W_StratI_timeB: {
          Ftemp = mcmd_write_float(100, 1000);		 //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(1)->I_Inrush_time = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }
          break;
        }

        case MCMD_R_StartI_ratioB: {
          mcmd_read_float(bldc_Motor(1)->I_Inrush_ratio);
          break;
        }

        case MCMD_R_StartI_timeB: {
          mcmd_read_float(bldc_Motor(1)->I_Inrush_time);
          break;
        }

        case MCMD_W_Detection_I_B: {
          Ftemp = mcmd_write_float(0.05, 4);                    //omejitev vpisa
          if (m_ack_state == 0) {
            bldc_Motor(1)->Idetection = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }
          break;				
        }
        
        case MCMD_R_Detection_I_B: {
          mcmd_read_float(bldc_Motor(1)->Idetection);
          break;				
        }

        case MCMD_R_All_PARAM: {

//fstadebug_printf("tracker_statusA1: %x\n",tracker_status);

          float temp;					
          bldc_motor *mot= bldc_Motor(0);
          bldc_motor *motB= bldc_Motor(1);

          temp=(float)swVersion.sw_version;
          temp/=1000.0;				//verzija je napisana v int 	
          read_int_buf[0] = FloatToUIntBytes(temp);//FloatToUIntBytes(temp);
          read_int_buf[1] = tracker_status;
          read_int_buf[2] = tracker_exstatus;
          read_int_buf[3] = BOOT_DEVTYPE;
          read_int_buf[4] = BOOT_VERSION;
          read_int_buf[5] = crc_errors;
          //read_int_buf[6]=0;

          read_int_buf[7]=FloatToUIntBytes(bldc_U(SUPPLY));
          read_int_buf[8]=FloatToUIntBytes(mzp_current);

          read_int_buf[9]=FloatToUIntBytes (bldc_remaining(0));
          read_int_buf[10]=FloatToUIntBytes(bldc_position(0));
          read_int_buf[11]=FloatToUIntBytes(bldc_target(0));
          read_int_buf[12]=FloatToUIntBytes(mot->I_limit);
          read_int_buf[13] = bldc_remainingImp(0);
          read_int_buf[14] = bldc_positionImp(0);
          read_int_buf[15] = bldc_targetImp(0);
          //read_int_buf[16]=0;
          read_int_buf[17] = FloatToUIntBytes (bldc_remaining(1));
          read_int_buf[18] = FloatToUIntBytes(bldc_position(1));
          read_int_buf[19] = FloatToUIntBytes(bldc_target(1));
          read_int_buf[20] = FloatToUIntBytes(motB->I_limit);
          read_int_buf[21] = tracker_exstatus;
          read_int_buf[22] = bldc_remainingImp(1);
          read_int_buf[23] = bldc_positionImp(1);
          read_int_buf[24] = bldc_targetImp(1);
          // read_int_buf[25]=0;
          mcmd_read_int(26, slave_addr);
          break;
        }	
				
        case MCMD_W_Lock_Tracker: {
          Utemp = mcmd_write_int(0, 0xffffffff);
          if(Utemp == SN[0])
            bldc_Lock(1);
          bldc_Stop(1);
          eepromUpdate = 1;
          backup_timeout = 200;
          break;
        }
        
        case MCMD_W_UnLockTracker: {
          Utemp = mcmd_write_int(0, 0xffffffff);
          if(Utemp == (SN[0] ^ SN[1] ^ SN[2] ^ SN[3] ^ 0x8f3b501c)) {
            bldc_Lock(0);
            eepromUpdate = 1;
            break;
          }
          m_ack_state = MACK_VALUE_OUT_OF_LIMIT;
          err_reply();
          break;
        }
        case MCMD_R_FlashWritCnt: {
          read_int_buf[0] = FlashWriteCounter;
          mcmd_read_int(1, slave_addr);
          break;
        }	
        
        case MCMD_R_Line_Resistance: {
          mcmd_read_float(LineResistance);
          break;	
        }
 
        case MCMD_C_Mesure_Line_Res: {
          Measure_Line_Resistance_Start();
          ack_reply();
          break;
        }
        
        case MCMD_R_MaxLine_Resistance: {
          mcmd_read_float(max_line_resistance);
          break;	
        }

        case MCMD_W_MaxLine_Resistance: {
          Ftemp = mcmd_write_float(0.01, 10);		 //omejitev vpisa
          if (m_ack_state == 0) {
            max_line_resistance = Ftemp;
            eepromUpdate = 1;
            backup_timeout = 200;				//4 sekundi zatem backup v flash
          }
          ack_reply();
          break;
        }

        case MCMD_R_RampA:
          m_ack_state = MACK_UNRECOGNIZED_CMD;
          err_reply();
          //mcmd_read_float(MotorA_ramp);
          break;
        case MCMD_W_RampA:
          m_ack_state=MACK_UNRECOGNIZED_CMD;
          err_reply();
        /*  Ftemp = mcmd_write_float(-5000,5000);		 //omejitev vpisa
          if (m_ack_state==0) {
                  MotorA_ramp=Ftemp;
                  backup_timeout=200;				//4 sekundi zatem backup v flash
          }*/
          break;

        case MCMD_R_RampB:
          m_ack_state = MACK_UNRECOGNIZED_CMD;
          err_reply();
          //mcmd_read_float(MotorB_ramp);
          break;
        case MCMD_W_RampB:
          m_ack_state = MACK_UNRECOGNIZED_CMD;
          err_reply();
        /*  Ftemp = mcmd_write_float(-5000,5000);		 //omejitev vpisa
          if (m_ack_state==0) {
                  MotorB_ramp=Ftemp;
                  backup_timeout=200;				//4 sekundi zatem backup v flash
          }*/
          break;

        //*** 	NOT RECOGNIZED COMMAND	***
        default: {
          m_ack_state = MACK_UNRECOGNIZED_CMD;
          err_reply();		//ukaz ni prepoznan
          break;
        }
      }
    }

  TX:
      if(eepromUpdate) {
        eeprom_write(SYS_VARS_EE);
      }

      if (broadcastID == NO_BROADCAST){ 	//if broadcast, do not send anything.

          if (uartMode == UART_MODE_XBEE) {
            memcpy((char *)UARTBuffer1, (char *)UARTBuffer, BUFSIZE);
            xbLength = xbSendPacketPrepare((char *)UARTBuffer1, number_TX_bytes);
            UART1Send( (uint8_t *)(&xbData[0]), xbLength);
            UARTCount1 = 0;
          }
          else if(transceiver == LORA){
            crc_calc2 = modbus_crc((uint8_t *)UARTBuffer, number_TX_bytes, CRC_NORMAL);
            if(UARTBuffer[1] == MCMD_R_All_PARAM && crc_calc2 == 0){ //Read All Parameters 
              UARTBuffer[26] = LoRa_id;
              UARTBuffer[66] = LoRa_get_rssi();

              number_TX_bytes = number_TX_bytes - 2;
              crc_calc2 = modbus_crc((uint8_t *)UARTBuffer, number_TX_bytes, CRC_NORMAL);
              UARTBuffer[number_TX_bytes++] = crc_calc2 & 0xFF;
              UARTBuffer[number_TX_bytes++] = crc_calc2 / 0x100;
            }

            set_tx_flag((char *)UARTBuffer, number_TX_bytes);
          }              
          else if (uartMode == UART_MODE_RS485) {
            memcpy((char *)UARTBuffer0, (char *)UARTBuffer, BUFSIZE);
            UART0Send( (uint8_t *)UARTBuffer0, number_TX_bytes );
//fsta
//debug_printf("id:%#02x crc:%d cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x length:%u  \n" , UARTBuffer0[0], crc_calc2, UARTBuffer0[1], UARTBuffer0[2], UARTBuffer0[3], UARTBuffer0[4], UARTBuffer0[5], UARTBuffer0[6], UARTBuffer0[7], number_TX_bytes);

            UARTCount0 = 0;
          }
      }
      broadcastID = NO_BROADCAST;
      number_TX_bytes = 0;
      if (flags & (1 << reset_it)) {				  //reset ukaz
        eeprom_write(SYS_VARS_EE);
        LPC_WWDT->FEED = 0xAA;				
        LPC_WWDT->FEED = 0x50;		//napacna sekvenca = takojsen reset
       // while(1);    			//cakaj na wdt reset 
      }

    } else {
      if(crc_errors % 0x10000 == 0xFFFF)
        crc_errors = (crc_errors / 0x10000) * 0x10000; // reset counter, only keep upper 4 bytes (checksum erors)

      crc_errors++;
    }
  }
  
  else if(transceiver == XBEE && UARTBuffer[0] != slave_addr) {
    memcpy((char *)UARTBuffer0, (char *)UARTBuffer, BUFSIZE);
    UART0Send( (uint8_t *)UARTBuffer0, UARTCount);
    UARTCount1 = 0;
  }

}

unsigned int addrPrev = 0;
/***********************************************************
  RX from LORA (Sigma) forwarding to RS485 (positioner)
************************************************************/
void modbus_cmd1() {

  uint32_t UARTCount;
  unsigned int dataLength = 0;
  char mode = MODE_NORMAL;
  unsigned int crcUpgrade = 0;

  if(transceiver == LORA)
    memcpy((uint8_t *)UARTBuffer1, (uint8_t *)module.rxBuffer, BUFSIZE);

  if (UARTBuffer1[0] == 0x7E || transceiver == LORA) {

    if(transceiver == XBEE)
      dataLength = xbReceivePacketRestoreConv((char *)UARTBuffer1);
    else if(transceiver == LORA)
      dataLength = module.packetLength;

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

  LPC_GPIO_PORT->B[1][17] = 0;

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
        crcUpgrade = CRC_UPGRADE_NANOD;
      else {
        crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount, CRC_UPGRADE_KVARK);
        if(crc_calc1 == 0)
          crcUpgrade = CRC_UPGRADE_KVARK;
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
            if(addr % 0x100 == 0x0 && size == 0x80) {//first part
              addrPrev = addr;
              memcpy((uint8_t *)writePacket1, (uint8_t *)(UARTBuffer1 + 8), 0x80);
              UARTBuffer1[2] = MACK_OK; // OK
              number_TX_bytes1 = 3;
              goto TX;
            }
            else if(addr % 0x100 == 0x80 && size == 0x80) {// second part
              if(addrPrev != addr - 0x80) {
                UARTBuffer1[2] = MACK_UNRECOGNIZED_CMD; // ERR
                number_TX_bytes1 = 3;
                addrPrev = 0;
                goto TX;
              }
              addrPrev = 0;
              UARTCount += 0x80;
              memcpy((uint8_t *)writePacket2, (uint8_t *)(UARTBuffer1 + 8), 0x80);
              memcpy((uint8_t *)(UARTBuffer1 + 0x08), (uint8_t *)writePacket1, 0x80);
              memcpy((uint8_t *)(UARTBuffer1 + 0x88), (uint8_t *)writePacket2, 0x80);
              UARTBuffer1[5] = 0x0;   // round address to 0x100
              UARTBuffer1[6] = 0x01;  // length 256
              UARTBuffer1[7] = 0x0;

              number_TX_bytes1 = 266;

              number_TX_bytes1 = encryptData((char *)&UARTBuffer1[1], number_TX_bytes1 - 1);
              crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, ++number_TX_bytes1, crcUpgrade);
              UARTBuffer1[number_TX_bytes1++] = crc_calc1 & 0xFF;
              UARTBuffer1[number_TX_bytes1++] = crc_calc1 / 0x100;

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
              UARTBuffer1[1] == INTCOM_LORA_GET_SN_BY_ID)) &&
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
        float voltage = bldc_U(SUPPLY);
        number_TX_bytes1 = mcmd_read_float_conv(voltage, (char *)UARTBuffer1);
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
        eeprom_write(SYS_VARS_EE); 

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
            LoRa_id = UARTBuffer1[18];
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
              LoRa_id = UARTBuffer1[18];
              updateSuccess = 1;
            }
          }
          if(updateSuccess){
              // save data
              eeprom_write(SYS_VARS_EE);     

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
            break;
          }       

          case INTCOM_LORA_GET_SN_BY_ID: {
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
            break;
          } 
                  
          case INTCOM_LORA_GET_RSSI: {    
            if(UARTBuffer1[0] == slave_addr){   
              UARTBuffer1[2] = LoRa_get_rssi();
              number_TX_bytes1 = 3;
              goto TX;
            }
            break;
          }

//fsta
          default: {  
            UARTCount1  = 0;
            number_TX_bytes1 = 0;
            return;
          }
//fsta

        }
      }


  // send to positioner
   
        memcpy((char *)UARTBuffer2, (char *)UARTBuffer1, BUFSIZE);

        UART0Send((uint8_t *)UARTBuffer2, UARTCount);
        LPC_GPIO_PORT->B[1][17] = 1;
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
                  UARTBuffer1[1] == INTCOM_LORA_GET_SN_BY_ID || // get LoRa SN
                  UARTBuffer1[1] == INTCOM_LORA_GET_RSSI)       // get LoRa RSSI
               //fsta )) {
               ) ){//|| mode == MODE_ERROR) { // error answer  // fsta 

         crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, number_TX_bytes1, CRC_NORMAL);
         UARTBuffer1[number_TX_bytes1++] = crc_calc1 & 0xFF;
         UARTBuffer1[number_TX_bytes1++] = crc_calc1 / 0x100;

         if(transceiver == LORA){
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
      eeprom_write(SYS_VARS_EE); 
      LPC_WWDT->FEED = 0xAA;            
      LPC_WWDT->FEED = 0x50;    
      while(1);   
    }
  }
  UARTCount1 = 0;
}

/***********************************************************
  RX from RS485 (positioner) forwarding to LORA (Sigma)
************************************************************/
void modbus_cmd2() {
  if(UARTCount0 > 0 ) {
    memcpy((char *)UARTBuffer1, (char *)UARTBuffer0, BUFSIZE);

    modbus_cnt = 0;

    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer0, UARTCount0, CRC_NORMAL);
    crc_calc1 = modbus_crc((uint8_t *)UARTBuffer0, UARTCount0, CRC_BOOT);

//fsta
//if(UARTBuffer2[1]==0x78)  
//debug_printf("cmd2 id:%#02x crc:%d cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x length:%u  \n" , UARTBuffer2[0], crc_calc2, UARTBuffer2[1], UARTBuffer2[2], UARTBuffer2[3], UARTBuffer2[4], UARTBuffer2[5], UARTBuffer2[6], UARTBuffer2[7], UARTCount2);

    if(crc_calc2 == 0)
    // fsta baudrate_timeout = 0;
      if(transceiver == NONE)
        baudrate_timeout = 1;  


    if((crc_calc2 == 0 || crc_calc1 == 0) && UARTBuffer1[0] > 0 && UARTBuffer1[0] < 255){
      int i = (UARTBuffer1[0] - 1) / 8;
      available_positioners[i] |= 1 << (UARTBuffer1[0] - i*8 - 1);
      online_timeouts[UARTBuffer1[0]] = 50000;
    }

    if(transceiver == LORA){
      if(UARTBuffer1[1] == MCMD_R_All_PARAM && crc_calc2 == 0){ //Read All Parameters 
        UARTBuffer1[26] = slave_addr;
        UARTBuffer1[66] = LoRa_get_rssi();

        number_TX_bytes = UARTCount0 - 2;
        crc_calc2 = modbus_crc((uint8_t *)UARTBuffer1, number_TX_bytes, CRC_NORMAL);
        UARTBuffer1[number_TX_bytes++] = crc_calc2 & 0xFF;
        UARTBuffer1[number_TX_bytes++] = crc_calc2 / 0x100;
      }

      set_tx_flag((char *)UARTBuffer1, UARTCount0);
    }
    else if(transceiver == XBEE){

      xbLength = xbSendPacketPrepare((char *)UARTBuffer1, UARTCount0);    
      UART1Send( (uint8_t *)(&xbData[0]), xbLength);
    }

    UARTCount0 = 0;
    return;

    TX:
    if(UARTBuffer0[1] == CMD_ZB2RS_MASTER_SLAVE) {
      crc_calc2 = modbus_crc((uint8_t *)UARTBuffer0, number_TX_bytes, CRC_NORMAL);
      UARTBuffer0[number_TX_bytes++] = crc_calc2 & 0xFF;
      UARTBuffer0[number_TX_bytes++] = crc_calc2 / 0x100;

      UART0Send((uint8_t *)(UARTBuffer0), number_TX_bytes);

      UARTCount0 = 0;
      number_TX_bytes = 0;
    }
  }
}

/***********************************************************
  RX from RS485 (positioner) forwarding to XBEE (Sigma)
************************************************************/
void modbus_cmd3() {
//fsta
//debug_printf("modbus_cmd3: %.2x %.2x %.2x %.2x %d\n",UARTBuffer0[0], UARTBuffer0[1], UARTBuffer0[2], UARTBuffer0[3], UARTCount0);

  memcpy((char *)UARTBuffer1, (char *)UARTBuffer0, BUFSIZE);
  xbLength = xbSendPacketPrepare((char *)UARTBuffer1, UARTCount0);

  UART1Send( (uint8_t *)(&xbData[0]), xbLength);
  UARTCount0 = 0;

}

unsigned int FloatToUIntBytes(float val) {
  unsigned int tmp = *((unsigned int *)((unsigned int) & val));
  return tmp;
}

/***********************************************************
  COMMON REPLIES
************************************************************/
/*void ack_code_replay()
{
	UARTBuffer[0]  = slave_addr; 
	UARTBuffer[1] &=~(1<<7);
	modbus_crc(18);
	UARTBuffer[18] = crc_calc&0xFF;
	UARTBuffer[19] = crc_calc/0x100;
	number_TX_bytes=20;
}*/

void ack_reply() {

  UARTBuffer[0] = slave_addr; 
  UARTBuffer[1] &=~ (1 << 7);
  UARTBuffer[2] = MACK_OK;
  crc_calc = modbus_crc((uint8_t *)UARTBuffer, 3, CRC_NORMAL);
  UARTBuffer[3] = crc_calc & 0xFF;
  UARTBuffer[4] = crc_calc / 0x100;
  number_TX_bytes = 5;
}

void ack_val_reply(float fVal) {
  float abc [1];				   			//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;
  abc[0]=fVal;
  temp=*p;

  UARTBuffer[0] = slave_addr; 
  UARTBuffer[1] &=~ (1<<7);
  UARTBuffer[2] = MACK_OK;

  UARTBuffer[3] = temp / 0x1000000;
  UARTBuffer[4] = (temp / 0x10000) & 0xFF;
  UARTBuffer[5] = (temp / 0x100) & 0xFF;
  UARTBuffer[6] = (temp) & 0xFF;

  crc_calc = modbus_crc((uint8_t*) UARTBuffer, 7, CRC_NORMAL);
  UARTBuffer[7] = crc_calc & 0xFF;
  UARTBuffer[8] = crc_calc / 0x100;
  number_TX_bytes = 9;
}

void ack_valUI_reply(unsigned int num_int) {
  unsigned int i = 0, j = 3;

  UARTBuffer[0] = slave_addr; 
  UARTBuffer[1] &=~ (1<<7);
  UARTBuffer[2] = MACK_OK;

  UARTBuffer[3] = num_int / 0x1000000;
  UARTBuffer[4] = (num_int / 0x10000) & 0xFF;
  UARTBuffer[5] = (num_int / 0x100) & 0xFF;
  UARTBuffer[6] = (num_int) & 0xFF;

  crc_calc = modbus_crc((uint8_t*) UARTBuffer, 7, CRC_NORMAL);
  UARTBuffer[7] = crc_calc & 0xFF;
  UARTBuffer[8] = crc_calc / 0x100;
  number_TX_bytes = 9;
}

void err_reply() {

  UARTBuffer[0] = slave_addr; 
  UARTBuffer[1] |=(1<<7);
  UARTBuffer[2] = m_ack_state;
  crc_calc = modbus_crc((uint8_t*)UARTBuffer, 3, CRC_NORMAL);
  UARTBuffer[3] = crc_calc&0xFF;
  UARTBuffer[4] = crc_calc/0x100;
  number_TX_bytes=5;
}
////////////////////////////////////
/*
void mcmd_read_byte (int data) {   	//en sam byte 0x00

	UARTBuffer[0] = slave_addr; 
	UARTBuffer[1] &=~(1<<7);		
	UARTBuffer[2] = data;
	modbus_crc (3);
	UARTBuffer[3] = crc_calc&0xFF;
	UARTBuffer[4] = crc_calc/0x100;
	number_TX_bytes=5;
}
*/
/*
unsigned int mcmd_write_byte (unsigned int dn_limit,unsigned int up_limit) {			//en sam byte 0x00
	unsigned int temp; 

	temp=UARTBuffer[2];
	if ((temp<=up_limit)&&(temp>=dn_limit)) ack_reply();
	else {
		m_ack_state=MACK_VALUE_OUT_OF_LIMIT;
		err_reply();
	}
	return temp;
}
*/
void mcmd_read_int(unsigned int num_int, uint8_t addr) { 		//vec int stevil "num_int" * 0x00000000
  unsigned int i = 0, j = 2;

  UARTBuffer[0] = addr; 
  UARTBuffer[1] &= ~(1<<7);

  do {
    UARTBuffer[j++] = read_int_buf[i] / 0x1000000;
    UARTBuffer[j++] = (read_int_buf[i] / 0x10000) & 0xFF;
    UARTBuffer[j++] = (read_int_buf[i] / 0x100) & 0xFF;
    UARTBuffer[j++] = (read_int_buf[i]) & 0xFF;
    num_int--;
    i++;
  } while (num_int != 0);

//fsta
//if(UARTBuffer[1]==0x78)
//debug_printf("%x %x %x %x %x %x %x %x %x %x %x %x %x\n",UARTBuffer[0],UARTBuffer[1],UARTBuffer[2],UARTBuffer[3],UARTBuffer[4],UARTBuffer[5],UARTBuffer[6],UARTBuffer[7],UARTBuffer[8],UARTBuffer[9],UARTBuffer[10],UARTBuffer[11],UARTBuffer[12]);

  crc_calc = modbus_crc((uint8_t *)UARTBuffer, j, CRC_NORMAL);
  UARTBuffer[j++] = crc_calc & 0xFF;
  UARTBuffer[j++] = crc_calc / 0x100;
  number_TX_bytes = j;
}

unsigned int mcmd_write_int(unsigned int dn_limit,unsigned int up_limit) {	  //eno int stevilo 0x00000000
  unsigned int temp; 

  temp = UARTBuffer[5];
  temp += UARTBuffer[4] * 0x100;
  temp += UARTBuffer[3] * 0x10000;
  temp += UARTBuffer[2] * 0x1000000;

  if ((temp <= up_limit) && (temp >= dn_limit)) 
    ack_valUI_reply(temp);
  else {
    m_ack_state = MACK_VALUE_OUT_OF_LIMIT;
    err_reply();
  }
  return temp;
}

unsigned int mcmd_write_int1() {
  unsigned int temp; 

  temp = UARTBuffer[5];
  temp += UARTBuffer[4] * 0x100;
  temp += UARTBuffer[3] * 0x10000;
  temp += UARTBuffer[2] * 0x1000000;

  return temp;
}

void mcmd_read_float(float param) {			//float
  float abc[1];				   			//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  abc[0] = param;
  temp = *p;

  UARTBuffer[0] = slave_addr; 
  UARTBuffer[1] &= ~(1<<7);
  UARTBuffer[2] = temp / 0x1000000;
  UARTBuffer[3] = (temp / 0x10000) & 0xFF;
  UARTBuffer[4] = (temp / 0x100) & 0xFF;
  UARTBuffer[5] = (temp) & 0xFF;
  crc_calc = modbus_crc((uint8_t*) UARTBuffer, 6, CRC_NORMAL);
  UARTBuffer[6] = crc_calc & 0xFF;
  UARTBuffer[7] = crc_calc / 0x100;
  number_TX_bytes = 8;
}


float mcmd_write_float1(){ 		//float
  float abc [1];				   							//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  temp = UARTBuffer[5];
  temp += UARTBuffer[4] * 0x100;
  temp += UARTBuffer[3] * 0x10000;
  temp += UARTBuffer[2] * 0x1000000;
  *p=temp;	

  return abc[0];
}


float mcmd_write_float(float dn_limit,float up_limit){ 		//float
  float abc[1];				   							//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  temp = UARTBuffer[5];
  temp += UARTBuffer[4] * 0x100;
  temp += UARTBuffer[3] * 0x10000;
  temp += UARTBuffer[2] * 0x1000000;
  *p = temp;	

  if ((abc[0] <= up_limit) && (abc[0] >= dn_limit)) 
    ack_val_reply(abc[0]);	  	//omejitev vpisa med 0 in 1000 v izogib trapastim vrednostim
  else {
    m_ack_state = MACK_VALUE_OUT_OF_LIMIT;
    err_reply();
  }
  return abc[0];
}

float mcmd_write_limit_float(float dn_limit,float up_limit,float offset){ 		//float
  float val ;				   							//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)&val;                   
  unsigned int temp;

  temp = UARTBuffer[5];
  temp += UARTBuffer[4] * 0x100;
  temp += UARTBuffer[3] * 0x10000;
  temp += UARTBuffer[2] * 0x1000000;
  *p = temp;	

  val += offset;

  if(val > up_limit)
    val = up_limit;
  if(val < dn_limit)
    val = dn_limit;

  ack_val_reply(val);	  	//omejitev vpisa med 0 in 1000 v izogib trapastim vrednostim

  return val;
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
unsigned int xbReceivePacketRestore(char *pchBuffer, unsigned int frameLength)		
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
  return (xbLength - 16);		
}

unsigned int xbReceivePacketRestoreConv(char *pchBuffer)    
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

unsigned int mcmd_read_float_conv(float param, char *pchData) {      //float
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

void append_crc(void){
    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer, number_TX_bytes, CRC_NORMAL);
    UARTBuffer[number_TX_bytes++] = crc_calc2 & 0xFF;
    UARTBuffer[number_TX_bytes++] = crc_calc2 / 0x100;
}

void xbee_conCheck() {

  NVIC_DisableIRQ(UART1_IRQn);
  UART1Init(115200);

  xbData[0] = 0X7E;
  xbData[1] = 0X0;
  xbData[2] = 0X04;
  xbData[3] = 0X08; // AT
  xbData[4] = 0X01;
  xbData[5] = 0X49; // ID
  xbData[6] = 0X44;
  xbData[7] = 0X69;
  UART1Send((uint8_t *)(&xbData[0]), 8);
}