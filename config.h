/*****************************************************************************
 *		"Pico" solar positioner software
 *
 *		filename: config.h  
 *		pcb: tiv29B
 *
 *		Copyright(C) 2011, Sat Control d.o.o.
 *		All rights reserved.
 *
******************************************************************************/

/*
Overview:
Header file za SunTracer
*/
#ifndef __CONFIG_H__
#define __CONFIG_H__

//#define sw_version 3.0
#define DEFAULT_MIN_RANGE 0

#define PORT_0 0
#define PORT_1 1
#define PORT_2 2
#define PORT_3 3

#define RED    4
#define GREEN  5
#define GREEN_uC  3
#define BLUE  0

#define STORAGE_NONE 0
#define STORAGE_FLASH 1
#define STORAGE_EEPROM 2
#define STORAGE_TYPE STORAGE_EEPROM

#define SYS_VARS_ADDR	0x0003F000
#define SYS_VARS_ADDR1	0x0003F400

#define SYS_VARS_EE     0x0     // in memory map: 0x03200000
#define SYS_VARS_EE1    0x500   // in memory map: 0x0320000
#define ROUTE_ADDR      0x100

#define MIN_SLAVE_ADDR  1
#define MAX_SLAVE_ADDR  64
#define MAX_ROUTE_HOPS  6
 
//		flags;		
#define tick_1ms 			3           //"1" = 20ms tick appear 
#define flash_erase_done                10 			//"1" = flash erase for settings is done
#define reset_it                        11			//"1" = do reset
#define flash_write_done                15
#define lineR_measure			17
#define lineR_init                      18
#define Modbus_timeout			19
#define buttonstuck                     20

//auto state
//dont use bit position 0
#define force_homeA               (1<<1)
#define force_homeB               (1<<2)

//tracker_status flags
#define ERR_OVERCURRENT_MOTOR_A		(1<<0)		//if motor has exceed max_I_motor value
#define ERR_HALL_A			(1<<1)		//if there is no position feedback signal, but current is present 
#define ERR_TOOLONG_REF_A		(1<<2)		//if moving to ref is more than XX minutes
#define ERR_CABLE_A			(1<<3)		//if there is no hall nor current flow
#define ERR_OVERCURRENT_MOTOR_B		(1<<4)		
#define ERR_HALL_B			(1<<5)		 
#define ERR_TOOLONG_REF_B		(1<<6)
#define ERR_CABLE_B			(1<<7)
		
#define SF_POWER_FAILURE		(1<<8)		//uC reset occured
#define SF_BUTTON_PRESSED		(1<<9)		//if button was pressed anytime after last status clear
#define SF_NO_MODBUS			(1<<10)		//MODBUS timeout occure in the past
#define SF_MOVING_OUT_A		   	(1<<11)		//motor A is moving in
#define	SF_MOVING_IN_A			(1<<12)		//motor A is moving out
#define SF_MOVING_REF_CLR_A		(1<<13)		//motor A is executing command REF_CLR
//#define SF_MOVING_REF_NOCLR_A		(1<<14)		//motor A is executing command REF_NOCLR
#define SF_MOVING_OUT_B		   	(1<<15)		//motor B is moving in
#define SF_MOVING_IN_B			(1<<16)		//motor B is moving out		
#define SF_MOVING_REF_CLR_B		(1<<17)		//motor B is executing command REF_CLR		
//#define SF_MOVING_REF_NOCLR_B		(1<<18)		//motor B is executing command REF_NOCLR
#define SF_ENDSW_A_LO_PRESSED		(1<<19)		//end switch pressed A - LO
#define SF_ENDSW_A_HI_PRESSED		(1<<20)		//end switch pressed A - HI
#define SF_ENDSW_B_LO_PRESSED		(1<<21)		//end switch pressed B - LO
#define SF_ENDSW_B_HI_PRESSED		(1<<18)		//end switch pressed B - HI
#define SF_HALL_WIRING_A		(1<<23)
#define SF_HALL_WIRING_B		(1<<24)

//mzp
#define SF_TRACKING_ENABLED		(1<<25)
#define SF_SNOW_MODE			(1<<26)
#define SF_WIND_MODE			(1<<27)
#define SYS_PARAM_EEPROM_ERR		(1<<29)
#define SYS_PARAM_FLASH_ERR		(1<<30)		//parameters were not stored in flash. Check parameters if they are ok (reset occured after flash erase?)


#define EFS_BUTTON_STUCK				(1U<<31)
#define EFS_OVERVOLTAGE					(1U<<30)
#define EFS_UNDERVOLTAGE				(1U<<29)
#define ESF_MOVE_OUT_ERR_A				(1U<<28)//where was motor moving when error //else motor was moving in
#define ESF_MOVE_OUT_ERR_B				(1U<<27)
#define ESF_MOVE_OUT_ERR_C				(1U<<26)
#define EFS_LOCKED					(1U<<25)
#define EFS_EVENTS_ACTIVE				(1U<<24)
#define EFS_MOTOR_CUTOFF				(1U<<23)
#define EFS_LINE_RES_MEASURING                          (1U<<22)
#define EFS_LINE_RESISTANCE_HIGH                        (1U<<21)
#define EFS_END_SWA_FAIL                                (1U<<20)
#define EFS_END_SWB_FAIL                                (1U<<19)
#define EFS_VOLTAGE_TO_LOW				(1U<<18)
#define EFS_BATTERY_LOW                                 (1U<<17)

// reset status
#define RESET_UNKNOWN 0
#define RESET_MANUAL  1     // reset by request button
#define CLEAR_TIME 900000   // 15 min

#define EFS_ERROR_STATES				(EFS_LOCKED|EFS_LINE_RESISTANCE_HIGH)
#define EFS_ERROR_STATESA				 EFS_END_SWA_FAIL
#define EFS_ERROR_STATESB                                EFS_END_SWB_FAIL
#define EFS_ERROR_CLEAR					(~(EFS_LOCKED|EFS_EVENTS_ACTIVE|EFS_LINE_RES_MEASURING|ESF_MOVE_OUT_ERR_A|ESF_MOVE_OUT_ERR_B))

#define OPT_LOCKED                                      (1U<<0 )
#define OPT_INIT                                        (1U<<31)

#define EVENT_HOME_A_FINISHED				(1<<0)
#define EVENT_HOME_B_FINISHED				(1<<1)
#define EVENT_LINE_R_FINISHED				(1<<2)
#define EVENT_LINE_R_CANCELED				(1<<3)



#define BOOT_VERSION_ADDR         0x7F00
#define BOOT_HW_REV_ADDR          0x7F04
#define BOOT_DEVTYPE_ADDR         0x7F08
#define BOOT_APP_MINVERSION_ADDR  0x7F0C

#define BOOT_VERSION		*((unsigned int *)BOOT_VERSION_ADDR)
#define BOOT_HW_REV		*((unsigned int *)BOOT_HW_REV_ADDR)
#define BOOT_DEVTYPE		*((unsigned int *)BOOT_DEVTYPE_ADDR)
#define BOOT_APP_MINVERSION	*((unsigned int *)BOOT_APP_MINVERSION_ADDR)

//************** deklaracije ****************

void AutoClearFlag(void);
void manual_button_drive (void);
int move_in_A (void);
int move_out_A (void);
int move_in_B (void);
int move_out_B (void);
void stop_motor (void);
void motor_check_position (void);
void go_ref(void);
//void flash_read (unsigned int read_address);
//void flash_erase (void);
void analog_value_management(void);

void led_handling(void);
void modbus_timeout_handling(unsigned int *modbus_cnt);
void moving_check(void);
void end_switches (void);
void Watchdog_Init(void);

void AxisSetState(unsigned int index, unsigned int state);
void AxisSetStates(unsigned int state);
int AxisEnabled(unsigned int index);

 int BMotor_idle(void);
 int AMotor_idle(void);
 int Motors_idle(void);

void ActivateEvent(unsigned int ev);
void SetEventParametersA(void);
void SetEventParametersB(void);

void Measure_Line_Resistance_Start(void);
void RMeasure_Stop(void);
void Measure_Line_Resistance(void);
void end_switch_check(void);
void check_hall_connection(void);

#endif
/*********************************************************************************
**                            End Of File
*********************************************************************************/
