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
#include "config.h"
#include "../gpio.h"
#include <string.h>
#include "SX1278.h"
#include "aes.h"
#include "../main.h"
#include "../eeprom.h"
#include "uart.h"

/////


unsigned int crc_calc1;
unsigned int crc_calc2;
unsigned int crc_calc;
unsigned char m_ack_state;                              // vsebuje opis napake, za katero MODBUS ukaz NI bil izvrsen
unsigned int read_int_buf[30];
unsigned char broadcastID;                              // 1 = broadcast call, 0 = normal call by ID
extern unsigned int modbus_indicator;			//stevec dolzine utripa ob rs485 sprejetju stringa 

unsigned char enabled = 0;
unsigned char missed_enable = 0;
unsigned char enable_tracking_retry = 0;
extern unsigned char enabled_in_micro;
extern uint16_t sigma_just_connected;
extern float           LineResistance;

extern uint8_t usb_drive;
extern float bldc_Current;
extern uint8_t voltage_select_0;
extern uint8_t voltage_select_1;
extern float UVccHALL_0, UVccHALL_1;
#ifdef KVARK
extern MODE_TYPE mode;
#endif
unsigned int number_TX_bytes0;
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
uint8_t LoRa_Responded = 0;

/* parameters */

extern unsigned char slave_addr;        // slave address on RS485
long long int online_slaves;


extern unsigned int SN[4];        // vsebujejo serijske stevilke

extern unsigned int modbus_cnt1;        // steje cas zadnjega modbus ukaza
extern unsigned int modbus_cnt2;        // steje cas zadnjega modbus ukaza
extern unsigned int modbus_cnt;
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
unsigned int xbLength;
char xbData[BUFSIZE];
unsigned int xbSendPacketPrepare(char *pchData, unsigned int uiLength);
unsigned int xbReceivePacketRestore(char *pchBuffer);

extern uint8_t routeOrders[MAX_SLAVE_ADDR+1][MAX_ROUTE_HOPS];

#define ftoint(val) (*((unsigned int *)(unsigned int) & (val))) 
#define fsendval(val) (((ftoint(val) << 24) & 0xff000000) | ((ftoint(val) << 8) & 0xff0000) | ((ftoint(val) >> 24) & 0xff) | ((ftoint(val) >> 8) & 0xff00)) 
char commShort = 1;
char commBuff1;
uint8_t writePacket1[0x80];
uint8_t writePacket2[0x80];
#define MODE_NORMAL 0
#define MODE_BOOT 1
#define MODE_UPGRADE 2
#define MODE_ERROR 3


/***********************************************************
  MODBUS COMMANDS
************************************************************/
#ifdef KVARK
/***********************************************************
  RX from LORA (Sigma) for KVARK (this positioner)
************************************************************/
void modbus_cmd() {

  unsigned int rxcnt;
  unsigned int Utemp;
  unsigned int Utemp_old;
  float Ftemp;
  int eepromUpdate = 0;

  if (uartMode == UART_MODE_XBEE) {
    unsigned int dataLength = 0;
    if (UARTBuffer1[0] == 0x7E) {

      // check XBEE connection AT command response
      if (UARTBuffer1[3] == 0x88) {
        transceiver = XBEE;
        UARTCount1 = 0;
        return;
      }

      dataLength = xbReceivePacketRestore((char *)UARTBuffer1);
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
      UARTCount0 = dataLength;
      memcpy((char *)UARTBuffer0, (char *)UARTBuffer1, BUFSIZE);
      crc_calc1 = modbus_crc((uint8_t *)UARTBuffer0, UARTCount0, CRC_NORMAL);
    }
    else {
      UARTCount1 = 0;
      return;
    }
  } 
  else if (transceiver == LORA){
    memcpy((char *)UARTBuffer0, (char *)module.rxBuffer, module.packetLength);
    UARTCount0 = module.packetLength;
  } 

	
  //----- broadcast naslov? ------
  broadcastID = NO_BROADCAST;
  if ((UARTCount0 > 0) && (UARTBuffer0[0] == 0))
    broadcastID = BROADCAST_CALL;
  //-------------------------------

  if ((UARTCount0 > 0) && (
   (UARTBuffer0[0] == slave_addr) ||
   (UARTBuffer0[0] == LoRa_id && transceiver == LORA) ||
   (broadcastID == BROADCAST_CALL) ||
   (UARTBuffer0[0] != slave_addr && transceiver == XBEE && ((UARTBuffer0[1] == CMD_RUN_GET_VOLTAGE || UARTBuffer0[1] == CMD_RUN_GET_LOADER_VER ||
     UARTBuffer0[1] == CMD_RUN_GET_VERSION) && crc_calc1 == 0))
     )) {

    crc_calc = modbus_crc((uint8_t *)UARTBuffer0, UARTCount0, CRC_NORMAL);

    if (crc_calc == 0) {
      if(transceiver == NONE)
        baudrate_timeout = 1;
      rxcnt = UARTCount0 - 2;
      modbus_indicator = 10;
      //green_led=25;		//utrip zelene LED, komunikacija RS485

      m_ack_state=0;
      modbus_cnt=0;             //no_modbus timeout

      if(LoRa_info_response((uint8_t*) UARTBuffer0, &number_TX_bytes0)){
        goto TX;
      }
      else {

//////////////////////////////////////////****NANO RESPONSE*****////////////////////////////////////////////////
      switch (UARTBuffer0[1]) {
        // reset 
        case MCD_W_reset: {					 	
          ack_reply();
          delay_reset = 10;

          reset_status = RESET_MANUAL;

          break;
        }
        // R STATUS
        case MCMD_R_status: {

          read_int_buf[0] = tracker_status;
          read_int_buf[1] = tracker_exstatus;
          number_TX_bytes0 = mcmd_read_int(2, slave_addr);
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
          if((UARTBuffer0[2] > 0) && (UARTBuffer0[2] <= MAX_SLAVE_ADDR))
          {
            Utemp=UARTBuffer0[2];
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
          Utemp = UARTBuffer0[2];
          Utemp |= ((unsigned int)UARTBuffer0[3]) << 8;
          Utemp |= ((unsigned int)UARTBuffer0[4]) << 16;
          Utemp |= ((unsigned int)UARTBuffer0[5]) << 24;
          
          if (SN[0]==Utemp) {
            if((UARTBuffer0[6] > 0) && (UARTBuffer0[6] <= 128)) {
              Utemp=UARTBuffer0[6];
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
          number_TX_bytes0 = mcmd_read_float(bldc_U(SUPPLY), (char *)UARTBuffer0);
          break;
        }
        // IMOTOR
        case MCMD_R_Imotor: {					   
          number_TX_bytes0 = mcmd_read_float(bldc_Current, (char *)UARTBuffer0);
          break;
        }

        case MCMD_R_MSpeed: {		
          read_int_buf[0] = bldc_Speed;		
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);		
          break;		
        }		
        case MCMD_R_NPoles: {		
          read_int_buf[0] = number_of_poles;		
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
          break;		
        }		
        case MCMD_W_NPoles: {
          Utemp = mcmd_write_int(0, 50);         //omejitev vpisa
          if (m_ack_state == 0) {
            number_of_poles = Utemp;
            eepromUpdate = 1;
            backup_timeout = 200;                     //4 sekundi zatem backup v flash
          }			 
          break;	
        }
           
        // SERIAL NUMBERS
        case MCMD_R_serial_numbers: {			   
          read_int_buf[0] = SN[0];
          read_int_buf[1] = SN[1];
          read_int_buf[2] = SN[2];
          read_int_buf[3] = SN[3];
          number_TX_bytes0 = mcmd_read_int(4, slave_addr);
          break;
        }
        // VERSION
        case MCMD_R_version: {
          float temp;
          temp = (float)swVersion.sw_version;
          temp /= 1000.0;				// verzija je napisana v int 			   			
          number_TX_bytes0 = mcmd_read_float(temp, (char *)UARTBuffer0);
          break;
        } 
        case MCMD_R_boot_ver: {
          read_int_buf[0] = BOOT_VERSION;
          read_int_buf[1] = BOOT_DEVTYPE;
          read_int_buf[2] = BOOT_HW_REV;
          read_int_buf[3] = BOOT_APP_MINVERSION;
          number_TX_bytes0 = mcmd_read_int(4, slave_addr);
          break;
        }
        // REMAIN IMPULSES A
        case MCMD_R_remain_A: {                         // ostanek impulzov do 0000 od zadnjega REF			 							
          number_TX_bytes0 = mcmd_read_float(bldc_remaining(0), (char *)UARTBuffer0);
          break;
        }
        // REMAIN IMPULSES B
        case MCMD_R_remain_B: {                         // ostanek impulzov do 0000 od zadnjega REF			 							
          number_TX_bytes0 = mcmd_read_float(bldc_remaining(1), (char *)UARTBuffer0);
          break;
        }
        case MCMD_R_events: {
          read_int_buf[0] = events;
          events = 0;
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(8, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(8, slave_addr);
          break;
        }
        //***** commands *****

        // STOP
        case MCMD_W_stop_motor: {					
          //stop_motor ();	
          RMeasure_Stop();                                                
          //bldc_manual(1);  // mzp
          bldc_Stop(1);
          bldc_runout(RUNOUT_ACTIVATE);
          store_in_flash = 100;
          ack_reply();
          break;
        }
        // R POSITION A
        case MCMD_R_position_A: {					
          number_TX_bytes0 = mcmd_read_float(bldc_position(0), (char *)UARTBuffer0);
          break;
        }
        // R DESTINATION A
        case MCMD_R_destination_A: {														
          number_TX_bytes0 = mcmd_read_float(bldc_target(0), (char *)UARTBuffer0);
          break;
        }
        // W DESTINATION A
        case MCMD_W_destination_A: {		                        
          int windmode = 0;

          if ((rxcnt == 7) && (UARTBuffer0[6]))
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

          if((rxcnt == 7) && (UARTBuffer0[6]))
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
          number_TX_bytes0 = mcmd_read_float(bldc_position(1), (char *)UARTBuffer0);
          break;
        }
        // R DESTINATION B
        case MCMD_R_destination_B: {														
          number_TX_bytes0 = mcmd_read_float(bldc_target(1), (char *)UARTBuffer0);
          break;
        }
        // W DESTINATION B
        case MCMD_W_destination_B: {		                        
          int windmode = 0;

          if((rxcnt == 7) && (UARTBuffer0[6])) windmode = 1;
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

          if((rxcnt == 7) && (UARTBuffer0[6]))
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
          break;				
        }

        case MCMD_R_Batt_voltage: {
          number_TX_bytes0 = mcmd_read_float(get_batt_U(), (char *)UARTBuffer0);
          break;				
        }

        // U hall
        case MCMD_R_Uhall_0:	{				   
          number_TX_bytes0 = mcmd_read_float(bldc_U(HALL0), (char *)UARTBuffer0);
          break;
        }
        case MCMD_R_Uhall_1:	{				   
          number_TX_bytes0 = mcmd_read_float(bldc_U(HALL1), (char *)UARTBuffer0);
          break;
        }


        case MCMD_R_Hall_cntDown: {
          read_int_buf[0] = bldc_GetInvertHall(0) ;
          read_int_buf[0] |= bldc_GetInvertHall(1) * 2;
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_float(Ftemp, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->min_position, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->max_position, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->home_offset, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->I_limit, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(Ftemp, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->min_position, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->max_position, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->home_offset, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->I_limit, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_config()->UConvertRatio, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_config()->IConvertRatio, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->modbus_timeout_position, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->modbus_timeout_position, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->gear_ratio, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->I_Inrush_ratio, (char *)UARTBuffer0);
          break;
        }

        case MCMD_R_StartI_timeA: {
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->I_Inrush_time, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(0)->Idetection, (char *)UARTBuffer0);
          break;				
        }

        // GEAR RATIO B
        case MCMD_R_gear_ratio_B: {
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->gear_ratio, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->I_Inrush_ratio, (char *)UARTBuffer0);
          break;
        }

        case MCMD_R_StartI_timeB: {
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->I_Inrush_time, (char *)UARTBuffer0);
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
          number_TX_bytes0 = mcmd_read_float(bldc_Motor(1)->Idetection, (char *)UARTBuffer0);
          break;				
        }

        case MCMD_R_All_PARAM: {

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
          read_int_buf[8]=FloatToUIntBytes(bldc_Current);

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
          number_TX_bytes0 = mcmd_read_int(26, slave_addr);
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
          number_TX_bytes0 = mcmd_read_int(1, slave_addr);
          break;
        }	
        
        case MCMD_R_Line_Resistance: {
          number_TX_bytes0 = mcmd_read_float(LineResistance, (char *)UARTBuffer0);
          break;	
        }
 
        case MCMD_C_Mesure_Line_Res: {
          Measure_Line_Resistance_Start();
          ack_reply();
          break;
        }
        
        case MCMD_R_MaxLine_Resistance: {
          number_TX_bytes0 = mcmd_read_float(max_line_resistance, (char *)UARTBuffer0);
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
          //number_TX_bytes0 = mcmd_read_float(MotorA_ramp);
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
          //number_TX_bytes0 = mcmd_read_float(MotorB_ramp);
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
    }                                         //--^--//
////////////////////////////////////////****NANO RESPONSE*****////////////////////////////////////////////////



  TX:
      if(eepromUpdate) {
        eeprom_write(SYS_VARS_EE);
      }
      crc_calc2 = modbus_crc((uint8_t *)UARTBuffer0, number_TX_bytes0, CRC_NORMAL);
      UARTBuffer0[number_TX_bytes0++] = crc_calc2 & 0xFF;
      UARTBuffer0[number_TX_bytes0++] = crc_calc2 / 0x100;

      if (broadcastID == NO_BROADCAST){ 	//if broadcast, do not send anything.

          if (uartMode == UART_MODE_XBEE) {
            memcpy((char *)UARTBuffer1, (char *)UARTBuffer0, BUFSIZE);
            xbLength = xbSendPacketPrepare((char *)UARTBuffer1, number_TX_bytes0);
            UART1Send( (uint8_t *)(&xbData[0]), xbLength);
            UARTCount1 = 0;
          }
          else if(transceiver == LORA){
            crc_calc2 = modbus_crc((uint8_t *)UARTBuffer0, number_TX_bytes0, CRC_NORMAL);
            if(UARTBuffer0[1] == MCMD_R_All_PARAM && crc_calc2 == 0){ //Read All Parameters 
              UARTBuffer0[26] = LoRa_id;
              UARTBuffer0[66] = LoRa_get_rssi();
              number_TX_bytes0 -= 2; //delete and recalculate crc for LoRa packet
              crc_calc2 = modbus_crc((uint8_t *)UARTBuffer0, number_TX_bytes0, CRC_NORMAL);
              UARTBuffer0[number_TX_bytes0++] = crc_calc2 & 0xFF;
              UARTBuffer0[number_TX_bytes0++] = crc_calc2 / 0x100;
            }

            set_tx_flag((char *)UARTBuffer0, number_TX_bytes0);
            //debug_printf("id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x \n" , UARTBuffer0[0], UARTBuffer0[1], UARTBuffer0[2], UARTBuffer0[3], UARTBuffer0[4], UARTBuffer0[5], UARTBuffer0[6], UARTBuffer0[7], UARTBuffer0[8], UARTBuffer0[9], UARTBuffer0[10]);
  
          }              
          else if (uartMode == UART_MODE_RS485) {
            UARTSend( (uint8_t *)UARTBuffer0, number_TX_bytes0);
            UARTCount0 = 0;
          }
      }
      broadcastID = NO_BROADCAST;
      number_TX_bytes0 = 0;
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
  
  else if(transceiver == XBEE && UARTBuffer0[0] != slave_addr) {
    UARTSend( (uint8_t *)UARTBuffer0, UARTCount0);
    UARTCount1 = 0;
  }
  LoRa_Responded = 0;
}
#endif

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

    //memcpy((uint8_t *)UARTBuffer0, (uint8_t *)UARTBuffer1, BUFSIZE);
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
    memcpy((uint8_t *)UARTBuffer1, (uint8_t *)UARTBuffer0, BUFSIZE);   
    // check boot packet
    crc_calc1 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount, CRC_BOOT);

    if(crc_calc1 != 0) {
      memcpy((uint8_t *)UARTBuffer1, (uint8_t *)UARTBuffer0, BUFSIZE);   
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
  else if(LoRa_info_response((uint8_t *)UARTBuffer1, &number_TX_bytes1)){
    mode = MODE_NORMAL;
    goto TX;
  }

  // send to positioner
  if(upgradeCount == 1) {
    // upgrade write command
    UARTSend((uint8_t *)UARTBuffer2_long, number_TX_bytes1);
    upgradeCount = 0;

  } else {
    memcpy((char *)UARTBuffer2, (char *)UARTBuffer1, BUFSIZE);
    UARTSend((uint8_t *)UARTBuffer2, UARTCount);
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
         )) 
  {
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
       //debug_printf("id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x \n" , UARTBuffer1[0], UARTBuffer1[1], UARTBuffer1[2], UARTBuffer1[3], UARTBuffer1[4], UARTBuffer1[5], UARTBuffer1[6], UARTBuffer1[7], UARTBuffer1[8], UARTBuffer1[9], UARTBuffer1[10]);
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
    UARTSend((uint8_t *)(UARTBuffer2), UARTCount1);

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
    UARTSend((uint8_t *)(UARTBuffer2), module.packetLength);
  }
  UARTCount1 = 0;
  number_TX_bytes2 = 0;
}

volatile uint8_t UARTTest1[BUFSIZE];

////////////////////////////////////////////////////
// from positioner via ZigBee / LoRa converter to sigma
void modbus_cmd2() {

#ifdef KVARK
memcpy((char *)UARTBuffer2, (char *)UARTBuffer0, BUFSIZE);
UARTCount2 = UARTCount0;
#endif

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

        UARTCount2 -= 2;
        crc_calc2 = modbus_crc((uint8_t *)UARTBuffer1, UARTCount2, CRC_NORMAL);
        UARTBuffer1[UARTCount2++] = crc_calc2 & 0xFF;
        UARTBuffer1[UARTCount2++] = crc_calc2 / 0x100;
      }

      set_tx_flag((char *)UARTBuffer1, UARTCount2);
//      debug_printf("id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x" , UARTBuffer1[0], UARTBuffer1[1], UARTBuffer1[2], UARTBuffer1[3], UARTBuffer1[4], UARTBuffer1[5], UARTBuffer1[6], UARTBuffer1[7], UARTBuffer1[8], UARTBuffer1[9], UARTBuffer1[10]);
//      debug_printf("%u \n" , UARTCount2);
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

      UARTSend((uint8_t *)(UARTBuffer2), number_TX_bytes2);

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
             UARTBuffer2[1] == INTCOM_LORA_SET_ROUTE
           
           || UARTBuffer2[1] == INTCOM_LORA_GET_PARAMS  // Helios commands
           || (UARTBuffer2[1] == MCMD_W_fromH && UARTBuffer2[2] == MCMD_W_sepH &&
            (UARTBuffer2[3] == MCMD_W_LoRaChannelH || UARTBuffer2[3] == MCMD_W_LoRaTxPower
             || UARTBuffer2[3] == MCMD_W_LoRaSF || UARTBuffer2[3] == MCMD_W_LoRaBW
            )
           )
          
          ) &&
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
            LoRa_Responded = 1;
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

        case INTCOM_LORA_SET_SETTINGS:
        case MCMD_W_fromH:
        {

          if(LoRa_bindMode_master == 0){
            original.channel  = module.channel;
            original.power    = module.power;
            original.spFactor = module.spFactor;
            original.LoRa_BW  = module.LoRa_BW;
          }

          if (UARTBuffer2[1] == MCMD_W_fromH && UARTBuffer2[2] == MCMD_W_sepH) {
            char cmd = UARTBuffer2[3];

            uint8_t ch = module.channel;
            uint8_t pwr = module.power;
            uint8_t sf = module.spFactor;
            uint8_t bw = module.LoRa_BW;

            switch (cmd) {
              case MCMD_W_LoRaChannelH:
              {
                int j = 0;
                ch = 0;
                for(int i = UARTCount2 - 4; i >= 4; i--) {
                  ch += (uint8_t)(UARTBuffer2[i] - 0x30) * (uint8_t)pow(10, j);
                  j++;
                }
                module.channel = ch;
                break;
              }

              case MCMD_W_LoRaTxPower:
              {
                pwr = UARTBuffer2[4] - 0x30;
                module.power = pwr;
                break;
              }
              
              case MCMD_W_LoRaSF:
              {
                int j = 0;
                sf = 0;
                for(int i = UARTCount2 - 4; i >= 4; i--) {
                  sf += (uint8_t)(UARTBuffer2[i] - 0x30) * (uint8_t)pow(10, j);
                  j++;
                }
                module.spFactor = sf;
                break;
              }
              case  MCMD_W_LoRaBW:
              {
                bw = UARTBuffer2[4] - 0x30;
                module.LoRa_BW = bw;
                break;
              }
            }

            // for slaves
            UARTBuffer2[0] = 0xFF;
            UARTBuffer2[1] = INTCOM_LORA_SET_SETTINGS;
            UARTBuffer2[2] = ch;
            UARTBuffer2[3] = pwr + sf * 0x10;
            UARTBuffer2[4] = module.LoRa_BW;
          }
          else if (UARTBuffer2[1] == INTCOM_LORA_SET_SETTINGS) {
            if(UARTBuffer2[2] != 0xff)
              module.channel = UARTBuffer2[2];   //if not bind_by_channel
            module.power = UARTBuffer2[3] & 0x03;  
            module.spFactor = (UARTBuffer2[3] & 0xf0) >> 4;
            module.LoRa_BW =  UARTBuffer2[4];
          }
          else
            break;

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


          if(UARTBuffer2[3] & 0x04 && UARTBuffer2[1] == INTCOM_LORA_SET_SETTINGS){ //Enter bindmode command
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
          break;
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

          case INTCOM_LORA_GET_PARAMS: {
          UARTBuffer2[2] = MCMD_W_sepH;  // sign separator: $
          UARTBuffer2[3] = MCMD_R_LoRaData;   // Master SN
          char buff[3];
          sprintf(buff, "%.2x\0", (SN[0] >> 24) & 0xff);
          UARTBuffer2[4] = buff[0];
          UARTBuffer2[5] = buff[1];
          sprintf(buff, "%.2x\0", (SN[0] >> 16) & 0xff);
          UARTBuffer2[6] = buff[0];
          UARTBuffer2[7] = buff[1];
          sprintf(buff, "%.2x\0", (SN[0] >> 8) & 0xff);
          UARTBuffer2[8] = buff[0];
          UARTBuffer2[9] = buff[1];
          sprintf(buff, "%.2x\0", SN[0] & 0xff);
          UARTBuffer2[10] = buff[0];
          UARTBuffer2[11] = buff[1];
          sprintf(buff, "%.2x\0", (SN[1] >> 24) & 0xff);
          UARTBuffer2[12] = buff[0];
          UARTBuffer2[13] = buff[1];
          sprintf(buff, "%.2x\0", (SN[1] >> 16) & 0xff);
          UARTBuffer2[14] = buff[0];
          UARTBuffer2[15] = buff[1];
          sprintf(buff, "%.2x\0", (SN[1] >> 8) & 0xff);
          UARTBuffer2[16] = buff[0];
          UARTBuffer2[17] = buff[1];
          sprintf(buff, "%.2x\0", SN[1] & 0xff);
          UARTBuffer2[18] = buff[0];
          UARTBuffer2[19] = buff[1];
          sprintf(buff, "%.2x\0", (SN[2] >> 24) & 0xff);
          UARTBuffer2[20] = buff[0];
          UARTBuffer2[21] = buff[1];
          sprintf(buff, "%.2x\0", (SN[2] >> 16) & 0xff);
          UARTBuffer2[22] = buff[0];
          UARTBuffer2[23] = buff[1];
          sprintf(buff, "%.2x\0", (SN[2] >> 8) & 0xff);
          UARTBuffer2[24] = buff[0];
          UARTBuffer2[25] = buff[1];
          sprintf(buff, "%.2x\0", SN[2] & 0xff);
          UARTBuffer2[26] = buff[0];
          UARTBuffer2[27] = buff[1];
          sprintf(buff, "%.2x\0", (SN[3] >> 24) & 0xff);
          UARTBuffer2[28] = buff[0];
          UARTBuffer2[29] = buff[1];
          sprintf(buff, "%.2x\0", (SN[3] >> 16) & 0xff);
          UARTBuffer2[30] = buff[0];
          UARTBuffer2[31] = buff[1];
          sprintf(buff, "%.2x\0", (SN[3] >> 8) & 0xff);
          UARTBuffer2[32] = buff[0];
          UARTBuffer2[33] = buff[1];
          sprintf(buff, "%.2x\0", SN[3] & 0xff);
          UARTBuffer2[34] = buff[0];
          UARTBuffer2[35] = buff[1];
          number_TX_bytes2 = 36;
          number_TX_bytes2 += sprintf((char *)&UARTBuffer2[number_TX_bytes2], "$%c%d", MCMD_W_LoRaChannelH, (int)module.channel);
          number_TX_bytes2 += sprintf((char *)&UARTBuffer2[number_TX_bytes2], "$%c%d", MCMD_W_LoRaTxPower, (int)module.power);
          number_TX_bytes2 += sprintf((char *)&UARTBuffer2[number_TX_bytes2], "$%c%d", MCMD_W_LoRaSF, (int)module.spFactor);
          number_TX_bytes2 += sprintf((char *)&UARTBuffer2[number_TX_bytes2], "$%c%d", MCMD_W_LoRaBW, (int)module.LoRa_BW);
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
     UARTBuffer2[1] == INTCOM_LORA_RESET_ROUTES
     
     || UARTBuffer2[1] == INTCOM_LORA_GET_PARAMS
     
    ) {

    crc_calc2 = modbus_crc((uint8_t *)UARTBuffer2, number_TX_bytes2, CRC_NORMAL);
    UARTBuffer2[number_TX_bytes2++] = crc_calc2 & 0xFF;
    UARTBuffer2[number_TX_bytes2++] = crc_calc2 / 0x100;

    UARTSend((uint8_t *)(UARTBuffer2), number_TX_bytes2);

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

/***********************************************************
  RX from RS485 (positioner) forwarding to XBEE (Sigma)
************************************************************/
void modbus_cmd3() {
  memcpy((char *)UARTBuffer1, (char *)UARTBuffer0, BUFSIZE);
  xbLength = xbSendPacketPrepare((char *)UARTBuffer1, UARTCount0);

  UART1Send( (uint8_t *)(&xbData[0]), xbLength);
  UARTCount0 = 0;

}

const char cmd2[] = {CMD_GET_STATUS, MCMD_R_All_PARAM, MCMD_R_boot_ver, MCMD_R_status, MCMD_R_events, MCMD_R_serial_numbers, MCMD_R_version};
char cmdIdx = 0;

uint8_t LoRa_info_response(uint8_t * UARTBuffer, unsigned int* number_TX_bytes){ //function for returning information of LoRa Slave unit
  LoRa_Responded = 0;
  if(!
      (
        (
          ( UARTBuffer[0] == LoRa_id ||                    // slaveID or broadcast(0x0) for SN setting, set settings
            UARTBuffer[0] == 0x00 ||
            UARTBuffer[1] == INTCOM_LORA_SET_ID_BY_SN ||
            UARTBuffer[1] == INTCOM_LORA_SET_SETTINGS ||
            UARTBuffer[1] == INTCOM_LORA_GET_RSSI ||
              (UARTBuffer[0] == 0xFF &&                    // broadcast(0xFF) for SN getting
               UARTBuffer[1] == INTCOM_CONV_GET_SN_BY_ID
              )
          ) &&
          transceiver == LORA                              // << LoRa condition
        ) 
        ||                              
        (
          //(UARTBuffer[1] == CMD_RUN_GET_LOADER_VER ||
          //UARTBuffer[1] == CMD_RUN_GET_VERSION ||
          //UARTBuffer[1] == CMD_RUN_GET_VOLTAGE
          //)
          //&& 
          (transceiver == XBEE)                        // ZigBee condition
        )
      )
    )
  {                                  
    return 0;
  }
    

  switch (UARTBuffer[1]) {

    case CMD_RUN_GET_LOADER_VER: {
      unsigned short ver = getVersionB();
      UARTBuffer[2] = MACK_OK;
      UARTBuffer[3] = ver / 0x100;
      UARTBuffer[4] = ver % 0x100;
      *number_TX_bytes = 5;
      LoRa_Responded = 1;
      break;
    }

    case CMD_RUN_GET_VERSION: {
      unsigned short ver = getVersion();
      UARTBuffer[2] = MACK_OK;
      UARTBuffer[3] = ver / 0x100;
      UARTBuffer[4] = ver % 0x100;
      *number_TX_bytes = 5;
      LoRa_Responded = 1;
      break;
    }

    case CMD_RUN_GET_VOLTAGE: {
      float voltage = gpio_U();
      LoRa_Responded = 1;
      *number_TX_bytes = mcmd_read_float(voltage, (char*)UARTBuffer);
      break;
    }

    case CMD_ZB2RS_RESET: { 
      if (UARTBuffer[0] == LoRa_id || transceiver == XBEE) {
        flags |= (1 << reset_it);   
        reset_status = RESET_MANUAL;
        UARTBuffer[2] = MACK_OK;
        *number_TX_bytes = 3;
        LoRa_Responded = 1;
      }
      break;    
    }

    case INTCOM_LORA_SET_SETTINGS: {
      if(!(UARTBuffer[0] == 0x0 || UARTBuffer[0] == 0xFF || UARTBuffer[0] == slave_addr)) {
        UARTCount0 = 0;
        number_TX_bytes = 0;
        return 0;
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
        *number_TX_bytes = 3;
        LoRa_Responded = 1;                                  
      }

      LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET,  RxMode); //Set settings to slave
      //timeout_master_check = 400000;
      if(UARTBuffer[0] < 0xff){
        UARTBuffer[2] = MACK_OK;
        *number_TX_bytes = 3;
        LoRa_Responded = 1;                   
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

        UARTBuffer[2] = available_positioners[0];
        UARTBuffer[3] = available_positioners[1]; 
        UARTBuffer[4] = available_positioners[2];
        UARTBuffer[5] = available_positioners[3];
        UARTBuffer[6] = available_positioners[4];
        UARTBuffer[7] = available_positioners[5];
        UARTBuffer[8] = available_positioners[6];
        UARTBuffer[9] = available_positioners[7];   

        *number_TX_bytes = 10;
       LoRa_Responded = 1;
      }else{
        UARTCount0  = 0;
        number_TX_bytes = 0;
        return 0;
      }
      break;
    }

    case INTCOM_CONV_GET_SN_BY_ID: {
        UARTBuffer[2] = (SN[0] >> 24) & 0xff;
        UARTBuffer[3] = (SN[0] >> 16) & 0xff;
        UARTBuffer[4] = (SN[0] >> 8)  & 0xff;  
        UARTBuffer[5] =  SN[0]        & 0xff;  

        UARTBuffer[6] = (SN[1] >> 24) & 0xff;
        UARTBuffer[7] = (SN[1] >> 16) & 0xff;
        UARTBuffer[8] = (SN[1] >> 8)  & 0xff;  
        UARTBuffer[9] = SN[1]         & 0xff; 

        UARTBuffer[10] = (SN[2] >> 24) & 0xff;
        UARTBuffer[11] = (SN[2] >> 16) & 0xff;
        UARTBuffer[12] = (SN[2] >> 8)  & 0xff;  
        UARTBuffer[13] =  SN[2]        & 0xff; 

        UARTBuffer[14] = (SN[3] >> 24) & 0xff;
        UARTBuffer[15] = (SN[3] >> 16) & 0xff;
        UARTBuffer[16] = (SN[3] >> 8)  & 0xff;  
        UARTBuffer[17] =  SN[3]        & 0xff;

        *number_TX_bytes = 18;
        LoRa_Responded = 1;
        break;
    }

    case INTCOM_LORA_GET_RSSI: {    
        if(UARTBuffer[0] == LoRa_id){   
          UARTBuffer[2] = LoRa_get_rssi();
          *number_TX_bytes = 3;
          LoRa_Responded = 1;
        }
        break;
    }
  }
  return LoRa_Responded;
}

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
  UARTSend((uint8_t *)UARTBuffer2, number_TX_bytes2);
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

unsigned int mcmd_write_int1() {
  unsigned int temp; 

  temp = UARTBuffer0[5];
  temp += UARTBuffer0[4] * 0x100;
  temp += UARTBuffer0[3] * 0x10000;
  temp += UARTBuffer0[2] * 0x1000000;

  return temp;
}

unsigned int mcmd_read_float(float param, char *pchData) {      //float
  float abc [1];                //kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  abc[0] = param;
  temp = *p;

  if(!LoRa_Responded)
    UARTBuffer0[1] &= ~(1<<7);

  pchData[2] = temp / 0x1000000;
  pchData[3] = (temp / 0x10000) & 0xFF;
  pchData[4] = (temp / 0x100) & 0xFF;
  pchData[5] = (temp) & 0xFF;
  return 6;
}


unsigned int mcmd_read_int(unsigned int num_int, uint8_t addr) { 		//vec int stevil "num_int" * 0x00000000
  unsigned int i = 0, j = 2;

  UARTBuffer0[0] = addr;
  if(!LoRa_Responded)
    UARTBuffer0[1] &= ~(1<<7);

  do {
    UARTBuffer0[j++] = read_int_buf[i] / 0x1000000;
    UARTBuffer0[j++] = (read_int_buf[i] / 0x10000) & 0xFF;
    UARTBuffer0[j++] = (read_int_buf[i] / 0x100) & 0xFF;
    UARTBuffer0[j++] = (read_int_buf[i]) & 0xFF;
    num_int--;
    i++;
  } while (num_int != 0);
  return j;
}



unsigned int mcmd_write_int(unsigned int dn_limit,unsigned int up_limit) {	  //eno int stevilo 0x00000000
  unsigned int temp; 

  temp = UARTBuffer0[5];
  temp += UARTBuffer0[4] * 0x100;
  temp += UARTBuffer0[3] * 0x10000;
  temp += UARTBuffer0[2] * 0x1000000;

  if ((temp <= up_limit) && (temp >= dn_limit)) 
    ack_valUI_reply(temp);
  else {
    m_ack_state = MACK_VALUE_OUT_OF_LIMIT;
    err_reply();
  }
  return temp;
}

void ack_reply() {

  UARTBuffer0[0] = slave_addr; 
  UARTBuffer0[1] &=~ (1 << 7);
  UARTBuffer0[2] = MACK_OK;
  number_TX_bytes0 = 3;
}

float mcmd_write_float1(){ 		//float
  float abc [1];				   							//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  temp = UARTBuffer0[5];
  temp += UARTBuffer0[4] * 0x100;
  temp += UARTBuffer0[3] * 0x10000;
  temp += UARTBuffer0[2] * 0x1000000;
  *p=temp;	

  return abc[0];
}


float mcmd_write_float(float dn_limit,float up_limit){ 		//float
  float abc[1];				   							//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;

  temp = UARTBuffer0[5];
  temp += UARTBuffer0[4] * 0x100;
  temp += UARTBuffer0[3] * 0x10000;
  temp += UARTBuffer0[2] * 0x1000000;
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

  temp = UARTBuffer0[5];
  temp += UARTBuffer0[4] * 0x100;
  temp += UARTBuffer0[3] * 0x10000;
  temp += UARTBuffer0[2] * 0x1000000;
  *p = temp;	

  val += offset;

  if(val > up_limit)
    val = up_limit;
  if(val < dn_limit)
    val = dn_limit;

  ack_val_reply(val);	  	//omejitev vpisa med 0 in 1000 v izogib trapastim vrednostim

  return val;
}

void ack_val_reply(float fVal) {
  float abc [1];				   			//kazalec deluje samo na array - ne vem zakaj???
  unsigned int *p = (unsigned int *)abc;                   
  unsigned int temp;
  abc[0]=fVal;
  temp=*p;

  UARTBuffer0[0] = slave_addr; 
  UARTBuffer0[1] &=~ (1<<7);
  UARTBuffer0[2] = MACK_OK;

  UARTBuffer0[3] = temp / 0x1000000;
  UARTBuffer0[4] = (temp / 0x10000) & 0xFF;
  UARTBuffer0[5] = (temp / 0x100) & 0xFF;
  UARTBuffer0[6] = (temp) & 0xFF;

  number_TX_bytes0 = 7;
}

void ack_valUI_reply(unsigned int num_int) {
  unsigned int i = 0, j = 3;

  UARTBuffer0[0] = slave_addr; 
  UARTBuffer0[1] &=~ (1<<7);
  UARTBuffer0[2] = MACK_OK;

  UARTBuffer0[3] = num_int / 0x1000000;
  UARTBuffer0[4] = (num_int / 0x10000) & 0xFF;
  UARTBuffer0[5] = (num_int / 0x100) & 0xFF;
  UARTBuffer0[6] = (num_int) & 0xFF;

  number_TX_bytes0 = 7;
}

void err_reply() {

  UARTBuffer0[0] = slave_addr; 
  UARTBuffer0[1] |=(1<<7);
  UARTBuffer0[2] = m_ack_state;
  number_TX_bytes0=3;
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
      if (*modbus_cnt >= mtimeout) {   //sekunde
        flags |= Modbus_timeout;
      }else{
        *modbus_cnt++;
        flags &= ~Modbus_timeout;
      }
    }else{
      *modbus_cnt = 0;
      flags &= ~Modbus_timeout;
    }
  }else
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

int isOnlineDevice(unsigned int dev) {
  int n = (dev - 1) / 8;

  if((available_positioners[n] & (1 << (dev - n * 8 - 1))) == (1 << (dev - n * 8 - 1)))
    return 1;

  return 0;
}