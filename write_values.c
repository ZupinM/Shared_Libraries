/**************************************************************************
 *		"Pico" solar positioner software
 *
 *		filename: modbus.c
 *		pcb: tiv29B
 *
 *		Copyright(C) 2011, Sat Control d.o.o.
 *		All rights reserved.
**************************************************************************/

#include <string.h>
#include <stdint.h>
#include "suntracer.h"	
#include "asciibox.h"
#include "display_values.h"
#include "write_values.h"
#include "LPC15xx.h" 
#include "bldc.h"

//#include "config.h"
#include "../eeprom.h"
//#include "usb_lib.h"
//#include "hw_config.h"
//#include "usb_pwr.h"
//#include "stm32f10x_iwdg.h"
//#include "motor.h"
//#include "focus.h"
//#include "rtc.h"
#include "sun.h"

uint32_t crypt_output;               //used as output in crypted parameter change
uint32_t too_much_tries;             //prevec napacnih poskusov vpisa ciphertexta

extern unsigned char status_homing;

extern MODE_TYPE mode;

unsigned char enabled_in_micro  = 0;


//USART3
//extern unsigned char USART_RxBuffer[USART_RXBUFFSIZE];

//general
extern float parameters [N_parameters];
extern float parameters_fixed [N_parameters_fixed];
extern uint32_t cflags;
extern unsigned int buyflags;
extern unsigned int tracker_status;	
extern unsigned int H_direrror;                 //impulz kot napacna smer - motnje motorcka
extern unsigned int V_direrror;

extern unsigned int rtc_counter;
extern unsigned int day_of_year;
extern uint16_t months_length[13];
extern unsigned int store_in_flash;
extern unsigned int flags;
extern unsigned int bflags;
extern unsigned int no_run_time;
extern unsigned int ref_toolong_cnt_A;
extern unsigned int ref_toolong_cnt_B;
extern unsigned char move_direction;
extern unsigned int error_motor_A;		
extern unsigned int error_hall_A;	  	
extern unsigned int error_cable_A;		
extern unsigned int error_motor_B;		
extern unsigned int error_hall_B;	  	
extern unsigned int error_cable_B;		
extern unsigned int backup_timeout;
extern uint32_t ID_number_1;
extern uint32_t ID_number_2;
extern uint32_t ID_number_3;
extern volatile int delay_reset;
extern uint32_t green_led;
extern unsigned int FocusMiddleA,FocusMiddleB;
extern uint32_t adc3_VAL;
extern uint32_t adc4_VAL;
extern uint32_t goref_Nday_cnt_B;
extern uint32_t goref_Nday_cnt_A;

extern uint8_t motor_manual_timer;
extern uint8_t usb_drive;

extern time_t time;

extern time_t lastSyncTime;

extern unsigned int zerospeedA, zerospeedB;
extern unsigned int diff_lowA, diff_lowB;

extern uint8_t slave_addr;
extern int baudrate;

///////////////////////////////////////////////

void write_values(unsigned char box, unsigned int IntTemp, float FloatTemp, unsigned char *StringTemp){

  green_led=1;   //blink with green LED to indicate valid command

  strLower((char *)StringTemp);

  if(CFG_locked && ( ((box>=70)&&(box<=171)) || (box==ctarget_H)|| (box==ctarget_V)) ){
    //exceptions
    if(!((box>=cGMT_sec)&&(box<=cGMT_year))) return;
  }
         
  switch(box) {
    case chours: {
        setSolarHours(IntTemp);       //URE
        sun_schedule_recalc();
        break;
    }
    case cminutes: {
        setSolarMinutes(IntTemp);        //MINUTE
        sun_schedule_recalc();
        break;
    }
    case cseconds: {
        setSolarSeconds(IntTemp);       //SEKUNDE
        sun_schedule_recalc();
        break;
    }
    case cdate: {
        setSolarMday(IntTemp);          //DATE
        sun_schedule_recalc();
        break;
    }
    case cmonth: {
        setSolarMonth(IntTemp);   //MONTH
        sun_schedule_recalc();
        break;
    }
    case cyear:{
        setSolarYear(IntTemp);   //year
        sun_schedule_recalc();
        break;
    }
     case ctime_zone:{
        time_zone = FloatTemp;   //time zone
        sun_schedule_recalc();
        break;
    }
    
    
    
    case cGMT_sec:
        setGMTSeconds(IntTemp);   //year
        sun_schedule_recalc();
        break;
    case cGMT_min:
        setGMTMinutes(IntTemp);        //MINUTE
        sun_schedule_recalc();
        break;
    case cGMT_hour:
        setGMTHours(IntTemp);       //URE
        sun_schedule_recalc();
        break;
    case cGMT_day:
        setGMTMday(IntTemp);          //DATE
        sun_schedule_recalc();
        break;
    case cGMT_mon:
        setGMTMonth(IntTemp);   //MONTH
        sun_schedule_recalc();
        break;
    case cGMT_year:
        setGMTYear(IntTemp);   //year
        sun_schedule_recalc();
        break;
    

        
    case clatitude: {           //LATITUDE
        latitude=FloatTemp;
        sun_schedule_recalc();
        store_in_flash=101;
        break;
    }
    case clongitude: {          //LONGITUDE
        longitude=FloatTemp;
        sun_schedule_recalc();
        store_in_flash=101;
        //eeprom_write(EEPROM_ADDR_MAIN, 0);
        break;
    }
    
    case ctimeDst:            // DST time
        time_dst=FloatTemp;
        sun_schedule_recalc();
        break;
        
    // last sync time
    case csdate: {
        lastSyncTime.date=IntTemp;
        break;
    }
    case csmonth: {
        lastSyncTime.month=IntTemp;
        break;
    }
    case csyear:{
        lastSyncTime.year=IntTemp;
        break;
    }
    case cshours:
        lastSyncTime.hours=IntTemp;
        break;
    case csminutes: {
        lastSyncTime.minutes=IntTemp;
        break;
    }
    case csseconds: {
        lastSyncTime.seconds=IntTemp;
        break;
    }

    case cmotor_speedA:{
        motor_speedA = IntTemp;
        break;
    }    
    case cmotor_speedB:{
        motor_speedB = IntTemp;
        break;
    }
    
    case cnumber_of_poles: {
      number_of_poles = IntTemp;
      break;
    }

    case cLoRa_id:
    LoRa_id = FloatTemp, eeprom_write(EEPROM_ADDR_BACKUP);
    break;

    case cchannel:              module.channel = IntTemp; break;
    case cpower:                module.power = IntTemp; break;
    case cspread_factor:        module.spFactor = IntTemp; break;
    case cbandwidth:            module.LoRa_BW = IntTemp; break;

    case cbaudrate:
      baudrate = IntTemp;
      UART0Init(baudrate);
      break;

    case cmotor_operation: {
      motor_operation = IntTemp;
      break;
    }

    case cpid_pA: {
      bldc_Motor(0)->pid.pgain = FloatTemp;
      break;
    }
    case cpid_iA: {
      bldc_Motor(0)->pid.igain = FloatTemp;
      break;
    }
    case cpid_dA: {
      bldc_Motor(0)->pid.dgain = FloatTemp;
      break;
    }
    case cpid_pB: {
      bldc_Motor(1)->pid.pgain = FloatTemp;
      break;
    }
    case cpid_iB: {
      bldc_Motor(1)->pid.igain = FloatTemp;
      break;
    }
    case cpid_dB: {
      bldc_Motor(1)->pid.dgain = FloatTemp;
      break;
    }

    case cdeadbandA: {
      bldc_Motor(0)->pid.deadband = IntTemp;
      break;
    }
    case cdeadbandB: {
      bldc_Motor(1)->pid.deadband = IntTemp;
      break;
    }

    case cshow_angle_A: {       //ANGLE OF A AXIS
        if (!(bflags&(1<<time_enable))){
            bldc_manual(0);
            usb_drive = 0;
            bldc_setPosition(0, geometry_selection (1,FloatTemp,(int)geometry_mode_A), 0);
            //m_goA(geometry_selection (1,FloatTemp,(int)geometry_mode_A));     //izbira geometrije
            no_run_time=0;                                                          //takoj zacni
        }
        break;
    }
    case cshow_angle_B: {       //ANGLE OF B AXIS
        if (!(bflags&(1<<time_enable))){
            bldc_manual(0);
            usb_drive = 0;
            bldc_setPosition(1, geometry_selection (2,FloatTemp,(int)geometry_mode_B), 0);            
            //m_goB(geometry_selection (2,FloatTemp,(int)geometry_mode_B));      //izbira geometrije
            no_run_time=0;                                                          //takoj zacni
        }
        break;
    }
    case cdestination_A: {       //IMPULSES OF A AXIS
        if (!(bflags&(1<<time_enable))){
            bldc_manual(0);
            usb_drive = 0;
            bldc_setPosition(0, IntTemp, 0);
            //usb_drive = 1;
            //m_goA(IntTemp);
            no_run_time=0;                                                          //takoj zacni
        }
        break;
    }
    case cdestination_B: {       //IMPULSES OF B AXIS
        if (!(bflags&(1<<time_enable))){
            bldc_manual(0);
            usb_drive = 0;
            bldc_setPosition(1, IntTemp, 0);
            //usb_drive = 1;
            //m_goB(IntTemp);
            no_run_time=0;                                                          //takoj zacni
        }
        break;
    }
    case cMode: {               //TIME CONTROLLED
       // if(GetMode()) SetMode(MICRO_MODE_STANDALONE);//cancel slave mode
        
        if ((*StringTemp=='N')||(*StringTemp=='n')) {
            if(mode != MODE_SLAVE && mode != MODE_SLAVE_TRACKING){
                bflags&=~(1<<time_enable);	
                tracker_status &= ~SF_TRACKING_ENABLED;
                mode = MODE_OK;
            }
            if(mode == MODE_SLAVE_TRACKING){
                tracker_status &= ~SF_TRACKING_ENABLED;
                mode = MODE_SLAVE;
            }
            //bldc_Stop(1);
            //BKP_WriteBackupRegister(BKP_DR5,bflags);	//write in RTC RAM
        }
        if ((*StringTemp=='Y')||(*StringTemp=='y')) {	
            if(mode != MODE_SLAVE && mode != MODE_SLAVE_TRACKING){ 	
                bflags|=(1<<time_enable);
                tracker_status |= SF_TRACKING_ENABLED;		
                mode = MODE_MICRO;
            }    
            if(mode == MODE_SLAVE){
                enabled_in_micro = 1;
                tracker_status |= SF_TRACKING_ENABLED;
                mode = MODE_SLAVE_TRACKING;
            }
            no_run_time=0;                              //takoj pozeni
            Recalc_sun();
        }
        break;
    }
    case cNavigation: {                                 //MANUAL DRIVE WITH NAVIGATION BUTTONS
                                                        //commands are only available when time is disabled (except REF)  
        motor_manual_timer = 0;
        status_homing = 0;
        usb_drive = 1;
          switch (*StringTemp){
                  case '2': {		            //left
                        if (!(tracker_status&SF_TRACKING_ENABLED)) {
			         bldc_manual(0);
                     bldc_setPosition(0, bldc_Motor(0)->max_position, 0);
			}
                        break;
                  }
                  case '4': {		            //right
		        if (!(tracker_status&SF_TRACKING_ENABLED)) {
                     bldc_manual(0);
                     bldc_setPosition(0, bldc_Motor(0)->min_position, 0);
			}
                        break;
                  }
                  case '1': {		            //up
		        if (!(tracker_status&SF_TRACKING_ENABLED)) {
                     bldc_manual(0);
                     bldc_setPosition(1, bldc_Motor(0)->max_position, 0);
			}
                        break;
                  }
                  case '3': {		            //down
		        if (!(tracker_status&SF_TRACKING_ENABLED)) {
                     bldc_manual(0);
                     bldc_setPosition(1, bldc_Motor(0)->min_position, 0);
			}
                        break;
                  }
                  /*********************/
                  case '5': {	                                                    //ROCNA REFERENCA A  
                         ClearStatus();
                         bldc_manual(0);
                         status_homing = 1;
                         bldc_Home(0);
                       break;
                  }
                  case '6': {                                   //ROCNA REFERENCA B
                          ClearStatus();
                          bldc_manual(0);
                          status_homing = 1;
                          bldc_Home(1);
                        break;
                  }
                  case 's': {
                        bldc_Stop(1);
                        break;
                  }
            }
    break;
    }

    case cpcb_version: {

       // if (strlen((char *)StringTemp) < 4)
        //  break;
        char version[10] = "0";

        strncpy(version, (char *)StringTemp, 4);

        if (strcmp(version, "30F1") == 0 || strcmp(version, "30f1") == 0) {
          if (f_pcb_version == 0x1E4601) // not changed
            break;
          f_pcb_version = 0x1E4601; // TIV30f1
        } 
        else if (strcmp(version, "36A1") == 0 || strcmp(version, "36a1") == 0) {
          if (f_pcb_version == 0x244101) // not changed
            break;
          f_pcb_version = 0x244101; // TIV36A1          
        } 
        else if (strcmp(version, "29C3") == 0 || strcmp(version, "29c3") == 0) {
          if (f_pcb_version == 0x1D4303)
            break;
          f_pcb_version = 0x1D4303; // TIV29C3
        }
        store_in_flash = 101;
        /* if (decrypt(box,IntTemp)) {
            f_pcb_version=crypt_output;                  //CRYPTED: MM xx yy zz, kjer xx=tiv27, yy="B" (ascii), zz=01
            store_in_flash=100;
        } */

        
        break;
    }
    case cinterval: {                                   //INTERVAL
        interval=FloatTemp;
        store_in_flash=101;
        break;
    }
    case cStatus: {             //DEBUG okno
            ClearStatus();
            return;
        break;
    }
    

    case cFocusMiddleA: {  //middle offset za externi sensor
          FocusMiddleA=IntTemp;
          store_in_flash=101;
          break;
    }
    case cFocusMiddleB: {  //middle offset za externi sensor
          FocusMiddleB=IntTemp;
          store_in_flash=101;
          break;
    }


    case cboot: {

        if (strncmp((char *)StringTemp, "resetstart", 10) == 0) {               //DEBUG okno
          delay_reset = 1000;

        } else if (strncmp((char *)StringTemp, "reset", 5) == 0) {               //DEBUG okno

            delay_reset = 1000;           //1000 * 1ms = 1s, da Helios zapre COM port

            bflags |= (1 << BootUpdate);                                //postavi zastavico za avtomatic update v BKP register.
            
            Chip_SYSCTL_PeriphReset(RESET_USB);
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_USBPHY_PD);
            Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_USB);
            


            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_USBPLL_PD);

        }
        break;
    }
    //change parameters

    case cA1_A:                 A1_A=FloatTemp; break;
    case cA2_A:                 A2_A=FloatTemp; break;
    case cA3_A:                 A3_A=FloatTemp; break;
    case cA4_A:                 A4_A=FloatTemp; break;
    case cA5_A:                 A5_A=FloatTemp; break;
    case cA6_A:                 A6_A=FloatTemp; break;
    case cB1_A:                 B1_A=FloatTemp; break;
    case cB2_A:                 B2_A=FloatTemp; break;
    case cgear_ratio_A:
      bldc_Motor(0)->gear_ratio = FloatTemp;
      break; 
    case cmin_range_A:
      bldc_Motor(0)->min_position = FloatTemp;
      break;
    case cmax_range_A:
      bldc_Motor(0)->max_position = FloatTemp;
      break;
    case czero_offsetA:
      bldc_Motor(0)->home_offset = FloatTemp;
      break;
    case ccoordinate_mode_A:    coordinate_mode_A=FloatTemp; break;
    case cgeometry_mode_A:      {
        geometry_mode_A=FloatTemp;
        //goref_Nday_cnt_A=0;                             //brisanje raznih registrov A osi
        break;      }
    case cmax_Imotor_A:
      bldc_Motor(0)->I_limit = FloatTemp;
      break;
    case cimotor_factor_A:
      bldc_config()->IConvertRatio = FloatTemp;    
      break; 
  
    case chome_position_A:      if (strstr((char *)StringTemp,"off")) {home_position_A=1000; break;}     // 1000.0 se uporablja kot zastavica za onemogocenje osi, da se postavi v wind-safe pozicijo
                              else home_position_A=FloatTemp; break; 
    
  
    case cgoref_Nday_A:         goref_Nday_A=FloatTemp; break;

    case cA1_B:                 A1_B=FloatTemp; break;
    case cA2_B:                 A2_B=FloatTemp; break;
    case cA3_B:                 A3_B=FloatTemp; break;
    case cA4_B:                 A4_B=FloatTemp; break;
    case cA5_B:                 A5_B=FloatTemp; break;
    case cA6_B:                 A6_B=FloatTemp; break;
    case cB1_B:                 B1_B=FloatTemp; break;
    case cB2_B:                 B2_B=FloatTemp; break;
    case cgear_ratio_B:
      bldc_Motor(1)->gear_ratio = FloatTemp;
      break;
    case cmin_range_B:
      bldc_Motor(1)->min_position = FloatTemp;
      break;
    case cmax_range_B:
      bldc_Motor(1)->max_position = FloatTemp;
      break;
    case czero_offsetB:
      bldc_Motor(1)->home_offset = FloatTemp;
      break;
    case ccoordinate_mode_B:    coordinate_mode_B=FloatTemp; break;
    case cgeometry_mode_B:      geometry_mode_B=FloatTemp; break; 
        //goref_Nday_cnt_B=0;                             //brisanje raznih registrov B osi
            
    case cmax_Imotor_B:
      bldc_Motor(1)->I_limit = FloatTemp;
      break;
    case cimotor_factor_B:
      bldc_config()->IConvertRatio = FloatTemp;
      break;
    case chome_position_B:      if (strstr((char *)StringTemp,"off")) {home_position_B=1000; break;}     // 1000.0 se uporablja kot zastavica za onemogocenje osi, da se postavi v wind-safe pozicijo
                                else home_position_B=FloatTemp; break;      

    case cgoref_Nday_B:         goref_Nday_B=(int)FloatTemp; break;

    case cNightMode_time:       NightMode_time=FloatTemp;      sun_schedule_recalc(); break;
    case cDayMode_time:         DayMode_time=FloatTemp;       sun_schedule_recalc();break;
    case cusolar_factor:
      bldc_config()->UConvertRatio = FloatTemp;
      break;
    case ccflags:               bldc_Stop(1);
                                //if(GetMode())cflags=(int)FloatTemp&CFLAGS_SLAVE_MODE_MASK;
                                cflags=(int)FloatTemp;
                                if(cflags&(1<<swap_halls_A))
                                  bldc_SetInvertHall(0,1);
                                else
                                  bldc_SetInvertHall(0,0);
                                if(cflags&(1<<swap_halls_B))
                                  bldc_SetInvertHall(1,1);
                                else
                                  bldc_SetInvertHall(1,0);

                                if(cflags&(1<<SwapRotation_A))
                                  bldc_SetInvert(0,1);
                                else
                                  bldc_SetInvert(0,0);
                                if(cflags&(1<<SwapRotation_B))
                                  bldc_SetInvert(1,1);
                                else
                                  bldc_SetInvert(1,0);
                                 
                                eeprom_write(EEPROM_ADDR_BACKUP);
                                break;
                              
    case cnormally_close_es:
      ES_0_normallyOpenLo = IntTemp & (1<<0);
      ES_1_normallyOpenLo = (IntTemp & (1<<1)) >> 1;
      ES_0_normallyOpenHi = (IntTemp & (1<<2)) >> 2;
      ES_1_normallyOpenHi = (IntTemp & (1<<3)) >> 3;
      eeprom_write(EEPROM_ADDR_BACKUP);
      break;
    case cslave_id:             slave_addr=FloatTemp, eeprom_write(EEPROM_ADDR_MAIN); break;
    //case cbuyflags:             change_buyFlags(IntTemp); cfg_wind_input(); break; //if (decrypt(box,IntTemp)) buyflags^=1<<(crypt_output&0x000000FF); break;        //CRYPTED:  MM AA AA xx, kjer xx pomeni kateri bit bo exor-an
    case crun_delay_home:       run_delay_home=FloatTemp; break;
    case crun_delay:            run_delay=FloatTemp; break;
    case cgroup:                group=FloatTemp; break;
    case cWidePanelA:           WidePanelA=FloatTemp; break;
    case cSpacePanelA:          SpacePanelA=FloatTemp; break;
    case cWidePanelB:           WidePanelB=FloatTemp; break;
    case cSpacePanelB:          SpacePanelB=FloatTemp; break;
    case crtc_correction:       {
                                  rtc_correction=(int)FloatTemp;
                                  break;
                                }
    case cSoftRTCCorrection:    SoftRTCCorrection=(int)FloatTemp;
//                                RTC_softwareCorrection((int)SoftRTCCorrection);
                                break;
    
    case cWindSpeedThreshold:   WindSpeedThreshold=FloatTemp; break;
    case cWindWaitOn:           WindWaitOn=FloatTemp; break;
    case cWindWaitOff:          WindWaitOff=FloatTemp; break;
    case cWindDestinationA:     if (strstr((char *)StringTemp,"off")) {WindDestinationA=1000; break;}     // 1000.0 se uporablja kot zastavica za onemogocenje osi, da se postavi v wind-safe pozicijo
                                else WindDestinationA=FloatTemp; break;
    case cWindDestinationB:     if (strstr((char *)StringTemp,"off")) {WindDestinationB=1000; break;}
                                else WindDestinationB=FloatTemp; break;
    case cWindFactor:           WindFactor=FloatTemp; break;
    case cWindSensorType:       WindSensorType=FloatTemp; break;
    case cSnowDestinationA:     if (strstr((char *)StringTemp,"off")){SnowDestinationA=1000; break;}
                                else SnowDestinationA=FloatTemp; break;
    case cSnowDestinationB:     if (strstr((char *)StringTemp,"off")){SnowDestinationB=1000; break;}
                                else SnowDestinationB=FloatTemp; break;
    
    case cVoltage_select_0:     voltage_select_0 = IntTemp; break;
    case cVoltage_select_1:     voltage_select_1 = IntTemp; break;
        
    //S
    case cOverTempShift:       OverTempShift=FloatTemp; break;
    case cTime_out_of_focus:   Time_out_of_focus=FloatTemp; break;
    ///S
    case cInrush_ratioA:      
      bldc_Motor(0)->I_Inrush_ratio = FloatTemp; 
      break; 
    case cInrush_ratioB:
      bldc_Motor(1)->I_Inrush_ratio = FloatTemp;
      break;
    case cInrush_timeA:
      bldc_Motor(0)->I_Inrush_time = FloatTemp;
      break;
    case cInrush_timeB:
      bldc_Motor(1)->I_Inrush_time = FloatTemp;
      break;
    case cfocus_max_offset:     focus_max_offset  = FloatTemp; break;
   
    case cdeviation_A  : deviation_A   = FloatTemp;  break;   
    case cinclination_A: inclination_A = FloatTemp;  break;    
    case cpanel_space_A: panel_space_A = FloatTemp;  break;   
    case cpanel_width_A: panel_width_A = FloatTemp;  break;   
    case cpanel_thick_A: panel_thick_A = FloatTemp;  break;    
   
    case cdeviation_B  : deviation_B  =FloatTemp;  break;   
    case cinclination_B: inclination_B =FloatTemp;  break;    
    case cpanel_space_B: panel_space_B=FloatTemp;  break;   
    case cpanel_width_B: panel_width_B=FloatTemp;  break;   
    case cpanel_thick_B: panel_thick_B=FloatTemp;  break;   
   
   
   //heliostat
   case ctarget_H:target_H = FloatTemp; store_in_flash=101;break;  
   case ctarget_V: target_V = FloatTemp; store_in_flash=101;break;
   
   case cHTarget2: if (strstr((char *)StringTemp,"off")){
                    HTarget2 = 1000;
                    VTarget2 = 1000;
                  }else{
                    HTarget2 = FloatTemp;
                    if(VTarget2==1000) VTarget2=0;
                  }  
                  break;
   
   case cVTarget2: if (strstr((char *)StringTemp,"off")){
                      VTarget2 = 1000;
                      HTarget2 = 1000;
                  }else{         
                      VTarget2 = FloatTemp;
                      if(HTarget2==1000) HTarget2=0;
                  }
                  break;
   
   case cHTarget3: if (strstr((char *)StringTemp,"off")){
                      HTarget3 = 1000;
                      VTarget3 = 1000;
                  }else{                                  
                      HTarget3 = FloatTemp;
                      if(VTarget3==1000) VTarget3=0; 
                  }
                  break;
  
   case cVTarget3: if (strstr((char *)StringTemp,"off")){
                      VTarget3 = 1000;
                      HTarget3 = 1000;
                  }else{
                      VTarget3 = FloatTemp;
                      if(HTarget3==1000) HTarget3=0; 
                  }
                  break;
                  
//   case cHeliostatP1_start: HeliostatP1_start = TimeStrToTics((char *)StringTemp); break; 
//   case cHeliostatP1_end:   HeliostatP1_end   = TimeStrToTics((char *)StringTemp); break;  
//   case cHeliostatP2_start: HeliostatP2_start = TimeStrToTics((char *)StringTemp); break;    
//   case cHeliostatP2_end:   HeliostatP2_end   = TimeStrToTics((char *)StringTemp); break;  
//   case cHeliostatP3_start: HeliostatP3_start = TimeStrToTics((char *)StringTemp); break;   
//   case cHeliostatP3_end:   HeliostatP3_end   = TimeStrToTics((char *)StringTemp); break; 
   
   
   //locking parameters
   case clflags:   LFLAGS = IntTemp; break;
   case clock_pin: if(!CFG_locked)lock_pin_code = IntTemp;break;
   case cUnlock:   if(IntTemp == lock_pin_code) CFG_unlock(); break;
   case cLock :    CFG_lock();   break;
     
  }
  if (box>=cA1_A)store_in_flash=101;                   //index >= 70? -> means parameters, store them all (in flash, last 128. page)

  write_value_limit();      //omejitev vrednosti vpisa

}

/***********************************************************
    Value limitation
**********************************************************/
void write_value_limit(void){

    if ((interval<60)&&(!(buyflags&(1<<resolution_01_enable)))) interval=60;               //interval = 1 minuto ... 15 minutami
    if ((interval<5)&&(buyflags&(1<<resolution_01_enable))) interval=5;                    //interval = 1 sekundo ... 15 minutami (za 0,1 stopinjsko natancnost)
    if (interval>900) interval=900;
  


    if (max_Imotor_A>10.0)     max_Imotor_A=10.0;   //najvec 10A
    if (max_Imotor_B>10.0)     max_Imotor_B=10.0;

    if(min_range_A>max_range_A) min_range_A=max_range_A-1;

    if(min_range_B>max_range_B) min_range_B=max_range_B-1;

    if(focus_max_offset > 10) focus_max_offset = 10.0;
    if(focus_max_offset < 0.1) focus_max_offset  = 0.1;
    
    
   if(target_H > 180) target_H=180;
   if(target_H <-180) target_H=-180;
   if(target_V >  90) target_V= 90;
   if(target_V < -90) target_V=-90;
  
   
   if((HTarget2!=1000)&&(HTarget2> 180)) HTarget2 = 180;
   if((HTarget2!=1000)&&(HTarget2<-180)) HTarget2 =-180;
   if((VTarget2!=1000)&&(VTarget2> 180)) VTarget2 = 180;
   if((VTarget2!=1000)&&(VTarget2<-180)) VTarget2 =-180;
   if(HTarget2==1000) VTarget2=1000;
   if(VTarget2==1000) HTarget2=1000;
   
   
   if((HTarget3!=1000)&&(HTarget3> 180)) HTarget3 = 180;
   if((HTarget3!=1000)&&(HTarget3<-180)) HTarget3 =-180;
   if((VTarget3!=1000)&&(VTarget3> 180)) VTarget3 = 180;
   if((VTarget3!=1000)&&(VTarget3<-180)) VTarget3 =-180;
   if(HTarget3==1000) VTarget3=1000;
   if(VTarget3==1000) HTarget3=1000;
   
   
//   if((HTarget3==1000)&&(HTarget2==1000)){
//      HeliostatP1_start = TIME_TO_TICS(0,0); 
//      HeliostatP1_end   = TIME_TO_TICS(23,0); 
//   }
   
   if(HeliostatP2_start>HeliostatP2_end)HeliostatP2_end = HeliostatP2_start;  
   if(HeliostatP3_start>HeliostatP3_end)HeliostatP3_end = HeliostatP3_start;
   
   
   buyflags&=~(1<<CAN_enable);
              
#ifdef SELECT_TIV27C
   cflags&=((1<<number_of_halls_A)|(1<<swap_halls_A)|(1<<number_of_halls_B)|(1<<swap_halls_B)|(1<<RunSoftwareRTC)|(1<<DisableReference));
   //(1<<FocusSensInUse)|(1<<SnowSensInUse)|(1<<WindSensInUse)|(1<<RunBothMotor)|
   buyflags&=((1<<resolution_01_enable)|(1<<heliostat));
   //(1<<heliostat)(1<<wind_secondary)(1<<snow_secondary)(1<<stopclear_input)(1<<goRef_input)
#endif
   
      
#ifdef SELECT_TIV27E
   cflags&=((1<<number_of_halls_A)|(1<<swap_halls_A)|(1<<RunSoftwareRTC)|(1<<DisableReference));
   //(1<<FocusSensInUse)|(1<<SnowSensInUse)|(1<<WindSensInUse)|(1<<RunBothMotor)|
   buyflags&=((1<<resolution_01_enable));
   //(1<<heliostat)(1<<wind_secondary)(1<<snow_secondary)(1<<stopclear_input)(1<<goRef_input)
#endif
   
}

/***********************************************************
    Decryption of protected parameters
**********************************************************/
unsigned int decrypt (unsigned int rotate, unsigned int chiper) {

#define key1 0x8B4F8F35;                //kljuci za kodiranje zascitenih parametrov

  uint32_t temp1,temp3;
  uint8_t var1,var2,var3,var4;

  crypt_output=0;
  if(too_much_tries > 5) return 0;     //prevec poskusov ("brutal force" attack)

  temp1=~ID_number_3;

  //if (flags&(1<<CAN_USB_select)) temp2=RxMessage.Data[1];     //- CAN
  //else
  //temp2=buffer_out[1];                                   //- USB

  //if (flags&(1<<CAN_USB_select)) temp=(int)atof(&RxMessage.Data[2]);     //Key and ID exor-ing - CAN
  //else
  //temp=(int)atof((char const *)&buffer_out[2]);                                   //Key and ID exor-ing - USB

  chiper=chiper^key1;
  chiper=chiper^ID_number_1;
  chiper=chiper^ID_number_2;
  chiper=chiper^temp1;

  while (rotate!=0){     //rotiraj levo
    temp3=chiper<<1;
    if(temp3<chiper)temp3++;  //overflow - add carry
    chiper=temp3;
    rotate--;
  }

  temp1=(chiper&0xFF000000)/0x1000000;

  var1=chiper;                    //AA BB BB BB check if BB*BB*BB=xxxxAA
  var2=chiper/0x100;
  var3=chiper/0x10000;
  var4=var1*var2*var3;



  if(temp1==var4)  {
    crypt_output=(var3*0x10000)+(var2*0x100)+var1;
    too_much_tries=0;       //resetiraj števec napacnih vpisov
    return 1;
  }
  else {
    too_much_tries++;       //števec prevec napacnih vpisov
    return 0;               //if failed, return 0
  }
}

/***********************************************************
    Clear error status flags
**********************************************************/
//void ClearStatus (void) {
//
//    tracker_status  = 0;			//brisi zastavice	
//    H_direrror      = 0;
//    V_direrror      = 0;
//    motor_clear_status();
//    focus_restart();
//    backup_timeout=BACKUP_TO;		//4 sekunde zatem backup v flash
//    flags&=~(1<<Modbus_timeout);
//}



