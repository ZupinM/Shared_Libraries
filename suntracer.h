/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUNTRACER_H
#define __SUNTRACER_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "display_values.h"
#include "bldc.h"
//#include "stm32f10x_lib.h"
//#include "rtc.h"
/* Definitions ---------------------------------------------------------------*/


#define MICRO_MODE_STANDALONE 0
#define MICRO_MODE_SLAVE      1



#define PI   3.1415926535897932384626433832795028841971
#define pwm_off_value       65475           //64050             //values for timers where PWM=0%
#define unique_ID_address   0x1FFFF7E8      //unique number for every single uC
#define BldVersionAddress   0x08000150      //v bootloaderju naslov, kjer je napisana verzija bootloaderja
#define ADC1_DR_Address     0x4001244C

#define ZEROSPEED           2000            //ko se motor ne vrti, je speed==ZEROSPEED
#define ZEROSPEED_S         4000
#define ZEROSPEED_VS        8000

#define ACCEL_LOW           30              //pospesek: 30-tiv27c, 60-tiv26a        //near destination_A - slow move (startup=1900ms)
#define ACCEL_HIGH          3
#define DIFF_HIGH           150             //razlika v impulzih, da starta z ACCEL_HIGH (funkcija: motor_check_position)
#define DIFF_LOW            50              //razlika v impulzih, da starta z ACCEL_LOW, ali pa sploh ne 
#define DIFF_LOW_S          5
#define DIFF_LOW_VS         0

#define DIFF_STOP           10              //razlika v impulzih, da se koncno ustavi (interrupt impuse counting)
//#define START_UP_CURR_TIME  30              //30*20ms = 600ms - trajanje zagonskega toka
#define NO_RUN_TIME_1S      50              //0.02s*50 = 1 sekundo po koncu voznje enega drugi ne sme
#define NO_RUN_TIME_SHORT   1  * 1000       // 2 sekundi
#define NO_RUN_TIME_LONG    10 * 1000       //10 sekund po neuspesnem vrtenju ali napaki pocaka na ponovni poskus
#define REF_TOOLONG_PARAM   30 * 60000      //30 minut
#define BACKUP_TO	    5		    //brisi flash za backup parametrov po 5*20ms=100ms
#define ES_DEBOUNCE         2               //debouncing na koncnih stikalih 40ms
#define NO_WIND             10000           //stevec za merjenje hitrosti vetra - vrednost ko nic ne piha
#define WIND_SAMPLES        20              //minimalno 3
#define BUTTON_STUCK_TIMEOUT ((15 * 60000)/20)  //30min timeout 20ms tick
//#define AUTOCLEAR_ERROR_CNT 90000           //90000*0.02s = 30minut


/* ADC SWSTART mask */
#define CR2_SWSTART_Set             ((uint32_t)0x00400000)

//move_direction;
#define MOVE_DIR_IN 		0x01
#define MOVE_DIR_OUT 		0x02	 	//motor_check_position ()

enum {
  HOUR_ANGLE=0,
  ELEVATION
};

//mode
typedef enum {
  MODE_OK=0,
  MODE_2FAR,
  MODE_DISABLE,
  MODE_WIND,
  MODE_SNOW,
  MODE_SLAVE,
  MODE_SLAVE_TRACKING,
  MODE_MICRO
}MODE_TYPE;

extern float flash_backup[0x100];

//tracker_status;			//status kondicije, v kateri je tracker (napake, halli...)
//#define ERR_OVERCURRENT_MOTOR_A		0		//if motor has exceed max_I_motor value
#define ERR_HALL_A			1		//if there is no position feedback signal, but current is present
//#define ERR_TOOLONG_REF_A		2		//if moving to ref is more than XX minutes
#define ERR_CABLE_A			3		//if there is no hall nor current flow
//#define ERR_OVERCURRENT_MOTOR_B		4		
#define ERR_HALL_B			5		
//#define ERR_TOOLONG_REF_B		6
#define ERR_CABLE_B			7
		
//#define SF_POWER_FAILURE		8		//uC reset occured
#define SF_BUTTON_PRESSED		9		//if button was pressed anytime after last status clear
#define ERR_SYNCH_RUN_DIFFERENCE	10		//too much differences between A and B in synchronous run mode
//#define SF_MOVING_OUT_A		   	11		//motor A is moving in
//#define	SF_MOVING_IN_A			12		//motor A is moving out
//#define SF_MOVING_REF_CLR_A		13		//motor A is executing command REF_CLR
#define SF_BUTTON_STUCK		        14		//motor A is executing command REF_NOCLR
//#define SF_MOVING_OUT_B		   	15		//motor B is moving in
//#define SF_MOVING_IN_B			16		//motor B is moving out		
//#define SF_MOVING_REF_CLR_B		17		//motor B is executing command REF_CLR		
//#define SF_ENDSW_B_HI_PRESSED		18		//motor B is executing command REF_NOCLR
//#define SF_ENDSW_A_LO_PRESSED		19		//end switch pressed A - LO
//#define SF_ENDSW_A_HI_PRESSED		20		//end switch pressed A - HI
//#define SF_ENDSW_B_LO_PRESSED		21		//end switch pressed B - LO
#define SF_MOT_A_LOSING_HAL		22		//end switch pressed B - HI
#define SF_MOT_B_LOSING_HAL             23
////#define SF_ENDSW_B_HI_PRESSED           24
//
//
//#define SYS_PARAM_FLASH_ERR		30		//parameters were not stored in flash. Check parameters if they are ok (reset occured after flash erase?)

#define EFS_BUTTON_STUCK		(1U<<31)
#define EFS_OVERVOLTAGE		        (1U<<30)
#define EFS_UNDERVOLTAGE		(1U<<29)
#define ESF_MOVE_OUT_ERR_A		(1U<<28)//where was motor moving when error //else motor was moving in
#define ESF_MOVE_OUT_ERR_B		(1U<<27)
#define ESF_MOVE_OUT_ERR_C		(1U<<26)
#define EFS_LOCKED			(1U<<25)
#define EFS_EVENTS_ACTIVE		(1U<<24)
#define EFS_MOTOR_CUTOFF		(1U<<23)
#define EFS_LINE_RES_MEASURING		(1U<<22)
#define EFS_LINE_RESISTANCE_HIGH	(1U<<21)
#define EFS_END_SWA_FAIL		(1U<<20)
#define EFS_END_SWB_FAIL		(1U<<19)
#define EFS_VOLTAGE_TO_LOW		(1U<<18)


#define EFS_ERROR_STATES					(EFS_LOCKED|EFS_LINE_RESISTANCE_HIGH)
#define EFS_ERROR_STATESA				   EFS_END_SWA_FAIL
#define EFS_ERROR_STATESB					 EFS_END_SWB_FAIL
#define EFS_ERROR_CLEAR					  (~(EFS_LOCKED|EFS_EVENTS_ACTIVE|EFS_LINE_RES_MEASURING|ESF_MOVE_OUT_ERR_A|ESF_MOVE_OUT_ERR_B))

#define EVENT_HOME_A_FINISHED				(1<<0)
#define EVENT_HOME_B_FINISHED				(1<<1)
#define EVENT_LINE_R_FINISHED				(1<<2)
#define EVENT_LINE_R_CANCELED				(1<<3)


//flags;			//after reset content is unknown
#define multi_key           0	//"1" = one button is already pressed
#define manual_drive        1	//"1" = motor is moving manualy
//#define btn_stuck           2	//"1" = B axis motor in function
#define tick_20ms           3   //"1" = 20ms tick appear
#define tick_1ms            4	//"1" = naredi/se dela referenca A
#define do_ref_B            5	//"1" = naredi/se dela referenca B

#define focus_detect        6   //"1" = 
#define CAN_USB_select      7   //"1" = 
//#define focus_ha_detected            8   //"1" = 
//#define focus_ea            9   //"1" = 

#define equationsAE_ok      10  //"1" = sledenje smiselno, enacbe OK    (koordinatne enacbe)
#define equationsPM_ok      11  //"1" = sledenje smiselno, enacbe OK    (koordinatne enacbe)
#define moveback_trigger    12  //"1" = ob 23:00 prožena zahteva za povratek
#define moveback_notrack    13  //"1" = ura je vec kot NightMode_time, zato sledenje ne preracunava
#define moveback_delcalc    14  //zascita, da se calculacije ne izvedejo ve
#define rtc_corr_updown     15  //"1" = odvzemi sekundo na XX sekund. "0" = dodaj sekundo
#define WindModeOn          16  //"1" = wind senzor aktiviran
#define SnowModeOn          17  //"1" = snow senzor aktiviran
#define OffsetTrigger           18  //za zmanjsanje hitrosti offset popravka fokusa
#define SelectFineTuneSensorA   19  //fine tune senzor...da ne regulirata oba hkrati
#define SelectFineTuneSensorB   20
#define tiv2627_InUse           21  //"1" = pomeni, da je v uprabi TIV26 ali 27E nima senzinga na koncnem stikalu - referenca se dela ko je timeout impulzov (tiv26, tiv27)
#define Modbus_timeout		22

//bflags;
#define time_enable     0	//"1" = time used for positioning
#define BootUpdate     10       //'1' = Avtomatic update iz Heliosa


//uint32_t cflags;                                 //configuration flags (stored in flash)
#define number_of_halls_A 0                 //'1' = only one hall in use
#define swap_halls_A      1		    //smer home, zaporedje hallov: '1' = H1->H2 ... '0'= H2->H1
#define number_of_halls_B 2                 //'1' = only one hall in use
#define swap_halls_B      3		    //smer home, zaporedje hallov: '1' = H1->H2 ... '0'= H2->H1
#define FocusSensInUse    4                 //external focus sensor in use
#define WindSensInUse     5                 //wind sensor in use
#define SnowSensInUse     6                 //snow sensor in use
#define RunBothMotors     7                 //'1' = oba motorja istocasno
#define RunSoftwareRTC    8                 //when powered up run rtc from hardware RTC
#define DisableReference  9                 //disable goto reference
#define SwapRotation_A    10
#define SwapRotation_B    11

#define CFLAGS_SLAVE_MODE_MASK (~((1<<DisableReference)|(1<<RunBothMotors)))

//uint32_t buyflags;
#define CAN_enable 0                        //'1' = omogoceno oboje - CAN in USB, '0' = samo USB
#define resolution_01_enable 1              //'1' = 0,1 deg resolucija
#define heliostat       2                         //'1' = omogoceno gibanje kot heliostat
#define wind_secondary  3
#define snow_secondary  4
#define stopclear_input 5
#define goRef_input     6


//lflags
#define cfgs_locked (1<<0)

#define LFLAGS       (*((int *)&lflags))
#define CFG_locked   (LFLAGS &   cfgs_locked)
#define CFG_lock()   (LFLAGS |=  cfgs_locked)
#define CFG_unlock() (LFLAGS &=~ cfgs_locked)



#define ADDR_parameters         0x0801FC00  //naslov odprtih parametrov v flashu (128. zadnji page)
//float parameters[];
#define N_parameters 105
extern float parameters [N_parameters];

#define A1_A                  flash_backup[158]   //factors for different geometric
#define A2_A                  flash_backup[159]
#define A3_A                  flash_backup[160]
#define A4_A                  flash_backup[161]
#define A5_A                  flash_backup[162]
#define A6_A                  flash_backup[163]   //angles for different geometric
#define B1_A                  flash_backup[164]
#define B2_A                  flash_backup[165]
#define gear_ratio_A        flash_backup[27]   //gear ratio factor for V axis
#define max_range_A         flash_backup[21]  //max impulses @ moving range=96 (96*59,5=5700)
#define min_range_A         flash_backup[20]
#define coordinate_mode_A     flash_backup[166]
#define latitude              flash_backup[167]  //users latitude
#define usolar_factor       flash_backup[14]  //U solar voltage
#define max_Imotor_A        flash_backup[23]
#define imotor_factor_A     flash_backup[15]  //I motor current (725.5 means 0.0007255V/step)
#define run_delay_home        flash_backup[69]  //zamik, da vsi na enkrat ne pozenejo za home (v sekundah)
#define run_delay             flash_backup[70]  //zamik, da vsi na enkrat ne pozenejo pri sledenju (v sekundah)
#define interval              flash_backup[71]  //interval pomika v sekundah
#define group                 flash_backup[72]  //koliko sledilnikov je v skupini, za zamaknjen pomik
#define WidePanelA            flash_backup[73]  //sirina panela (za sencenje)
#define SpacePanelA           flash_backup[74]  //razmak med paneli (za sencenje)
#define A1_B                  flash_backup[75]  //factors for different geometric
#define A2_B                  flash_backup[76]
#define A3_B                  flash_backup[77]
#define A4_B                  flash_backup[78]
#define A5_B                  flash_backup[79]
#define A6_B                  flash_backup[80]  //angles for different geometric
#define B1_B                  flash_backup[81]
#define B2_B                  flash_backup[82]
#define gear_ratio_B        flash_backup[48]  //gear ratio factor for H axis
#define max_range_B         flash_backup[42]  //max number of impulses of V axis @520mm
#define min_range_B         flash_backup[41]
#define coordinate_mode_B     flash_backup[83]
#define home_position_A       flash_backup[84]  //kam se bo zvecer vrnila os A (v stopinjah)
#define home_position_B       flash_backup[85]  //kam se bo zvecer vrnila os B (v stopinjah)
#define longitude             flash_backup[57]
#define max_Imotor_B        flash_backup[44]
#define imotor_factor_B     flash_backup[15]
#define rtc_correction        flash_backup[86]
#define goref_Nday_A          flash_backup[87]
#define goref_Nday_B          flash_backup[88]
#define WindSpeedThreshold    flash_backup[89]  //meja (m/s), kdaj gre v pozicijo
#define WindWaitOn            flash_backup[90]  // <--------------  trenutno ni v uporabi - morda bo
#define WindWaitOff           flash_backup[91]  // v minutah
#define WindDestinationA      flash_backup[92]  //pozicija za veter
#define WindDestinationB      flash_backup[93]
#define WindFactor            flash_backup[94]  //faktor pretvorbe v realno vrednost za napetost ali impulze
#define WindSensorType        flash_backup[95]  //Tip senzorja za veter
#define SnowDestinationA      flash_backup[96]  //pozicija za sneg
#define SnowDestinationB      flash_backup[97]
#define reserved01             flash_backup[98]
#define target_H              flash_backup[99]  //Heliostat
#define target_V              flash_backup[100]
#define f_FocusMiddleA        flash_backup[101]  //sredina - externi light sensor
#define f_FocusMiddleB        flash_backup[102]
#define WidePanelB            flash_backup[103]  //sirina panela (za sencenje)
#define SpacePanelB           flash_backup[104]  //razmak med paneli (za sencenje)
//S
#define OverTempShift         flash_backup[105]
#define Time_out_of_focus     flash_backup[106]
///S
#define Mot_inrush_ratio_A  flash_backup[24]
#define Mot_inrush_ratio_B  flash_backup[45]
#define Mot_inrush_time_A   flash_backup[25]
#define Mot_inrush_time_B   flash_backup[46]
#define focus_max_offset      flash_backup[107]

#define deviation_A           flash_backup[108]
#define inclination_A         flash_backup[109]
#define panel_space_A         flash_backup[110]
#define panel_width_A         flash_backup[111]
#define panel_thick_A         flash_backup[112]

#define deviation_B           flash_backup[113]
#define inclination_B         flash_backup[114]
#define panel_space_B         flash_backup[115]
#define panel_width_B         flash_backup[116]
#define panel_thick_B         flash_backup[117] 

#define time_zone             flash_backup[118] 
#define NightMode_time        flash_backup[119] 
#define DayMode_time          flash_backup[120] 

#define lflags                flash_backup[121] 
#define lock_pin_code         flash_backup[122]


#define HTarget2              flash_backup[123]
#define VTarget2              flash_backup[124]
#define HTarget3              flash_backup[125]
#define VTarget3              flash_backup[126]

#define HeliostatP1_start     flash_backup[127]
#define HeliostatP1_end       flash_backup[128]
#define HeliostatP2_start     flash_backup[129]
#define HeliostatP2_end       flash_backup[130]
#define HeliostatP3_start     flash_backup[131]
#define HeliostatP3_end       flash_backup[132]
#define SoftRTCCorrection     flash_backup[133]

#define zero_offsetA      flash_backup[29]
#define zero_offsetB      flash_backup[50]

#define modbus_timeout_posA  flash_backup[26]
#define modbus_timeout_posB  flash_backup[47]
#define modbus_timeoutID        flash_backup[134]

#define modbus_m_enabled        flash_backup[135]

#define time_dst                flash_backup[136]
#define lastSync_date           flash_backup[137]
#define lastSync_time           flash_backup[138]
#define lastSync_year           flash_backup[139]

#define motor_speedA            flash_backup[140]   // 0 - normal, 1 - slow, 2 - wery slow
#define motor_speedB            flash_backup[141]

#define ADDR_parameters_fixed  0x0801F800      //naslov skritih parametrov v flashu (127. predzadnji page)
//float parameters_fixed [];
#define N_parameters_fixed 7                   //stevilo parametrov

#define f_pcb_version       flash_backup[142]  //verzija TIV
#define slave_id          flash_backup[0]  //RS485 slave ID
#define f_cflags            flash_backup[143]  //cflags v float obliki
#define f_bflags            flash_backup[147]
#define f_buyflags          flash_backup[144]  //buyflags v float obliki (** options **)
#define geometry_mode_A     flash_backup[145]  //izbrana geometrija prve osi
#define geometry_mode_B     flash_backup[146]  //izbrana geometrija druge osi

#define overvoltageOcc      flash_backup[148]

//#define BootUpgradeFlag   parameters_fixed[6]  //zastavica za avtomatsko posodobitev iz Heliosa



//******************************************
//fsta
//DEVICE SPECIFIC CONFIG
#define SELECT_MICRO
//#define SELECT_TIV27C
//#define SELECT_TIV27E


#ifdef SELECT_TIV27C
  #define SELECT_TIV27C_E
  #define DISABLE_FOCUS
  #define DISABLE_SENSORS
#endif

#ifdef SELECT_TIV27E
  #define SELECT_TIV27C_E
  //#define DISABLE_MOTOR_B  
  #define DISABLE_HELISTAT
  #define DISABLE_SENSORS
#endif

#ifdef DISABLE_SENSORS
  #ifndef DISABLE_FOCUS
    #define DISABLE_FOCUS
  #endif
#endif

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void RTC_Configuration(void);
void TIM1_config(void);
void ADC_config(void);
void DMA_config(void);
void EXTI_config(void);

void manual_button_drive (void);
void check_time(void);
void eot_and_declination(uint32_t doy);
float length2pulses(double length);
void analog_value_management(void);

void suns_position_PM (double *elevation, double *azimuth, double ha, double declination);
int anti_shadowing (double *NewAngle, double InputAngle, double space, double wide,char type);
void mirror_equations (double target_azi, double target_ele, double Aos);
void mirror_equationsAE (double target_azi, double target_ele) ;
double coordinate_selection (uint32_t coordinate_mode,int axis);
unsigned int geometry_selection (uint32_t axis, double angle, uint32_t geometry_mode);
double geometry_selection_show (uint32_t axis, uint32_t geometry_mode);

double geometry_1 (int axis,double angle, double A1, double gear_ratio);
double geometry_1_show (int axis,int32_t position, double A1, double gear_ratio);
double geometry_11(double angle, double A1,double gear_ration);
double geometry_11_show(int32_t position, double A1, double gear_ratio); 
double geometry_2 (double ha, double A1, double A2, double A3, double A4, double B1, double B2, int left, double gear_ratio);
double geometry_2_show (int32_t position, double A1, double A2, double A3, double A4, double B1, double B2,int left, double gear_ratio);
double geometry_3 (double elev, double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio);
double geometry_3_show(int32_t position, double A1, double A2, double A3, double A5, double alphaBeta, int bottom, double gear_ratio);
//geometrija 13 za coordinate mode 11 elevacija
double geometry_13(double elev, double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio);
double geometry_13_show(int32_t encPuls ,double A1, double A2, double A3, double A5, double alphaBeta, int bottom, double gear_ratio);

void Recalc_sun();

void read_parameters (void);
void write_parameters (void);

void USART_config(void);
void USB_config(void);

void weather_sensor (void);
void led_handling(void);
void tracker_status_check(void);

void user_interface(void);
void end_switches (void);

void CAN_config(void) ;
void can_usb_switch(void);

void saveBackup();
int Focusing();

void cfg_wind_input();
void change_buyFlags(unsigned int newVal);

char * strLower(char *str);


void modbus_reset_timer();
int  GetMode();
void SetMode(int mode);
int  AxisEnabled(unsigned int index);
void AxisSetStates(unsigned int state);

extern uint8_t voltage_select_0;
extern uint8_t voltage_select_1;


//typedef struct {
//        uint32_t seconds;
//  uint32_t minutes;
//  uint32_t hours;
//  uint32_t day;
//  uint32_t date;
//  uint32_t month;
//        uint32_t year;
//}time_t;


//----------------------------------
#endif /*__SUNTRACER_H*/

