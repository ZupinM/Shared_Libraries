#ifndef BLDC
#define BLDC
#include "pid.h"
#include <stdint.h>



typedef struct{
  //motor index
  unsigned char index;
  //runtime variables
  unsigned char state;
  int status;
  int ctrl;
  int position;
  int target;
  int home_remaining;
  int lastPosition;
  unsigned char hall_state;

  pid pid;
  //configuration
  float         gear_ratio;
  float         I_limit;
  float         I_Inrush_ratio;
  float         I_Inrush_time;
  float         home_offset;
  float         min_position;
  float         max_position;
  float         Idetection;
  float         end_switchDetect;
  float         modbus_timeout_position;



  unsigned char i_err_cnt;
  unsigned int  cable_err_time;
  unsigned char cable_err_cnt;
  unsigned int  homing_time;
}bldc_motor;


typedef struct{
  float UConvertRatio;
  float HConvertRatio;
  float BConvertRatio;
  float H1ConvertRatio;
  float IConvertRatio;
  float TConvertRatio;
  unsigned int  homing_timeout;
}bldc_misc;

//Voltage measuring points
#define HALL0   0
#define HALL1   1
#define SUPPLY  2

#define BLDC_MOTOR_STATE_ENABLED (1<<0)
#define BLDC_MOTOR_STATE_INVERT_DIRECTION  (1<<1)
#define BLDC_MOTOR_STATE_INVERT_HALL      (1<<2)

#define BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST (1<<3)
#define BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE (1<<4)
#define BLDC_MOTOR_STATE_INVERT_HALL_REQUEST (1<<5)
#define BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE (1<<6)


#define BLDC_STATUS_ACTIVE       (1<<0)
#define BLDC_STATUS_CCW          (1<<1)
#define BLDC_STATUS_MANUAL       (1<<2)
#define BLDC_STATUS_STALL        (1<<3)
#define BLDC_STATUS_WIND_MODE    (1<<4)

#define BLDC_STATUS_ERR          (1<<6)//global error flag
#define BLDC_STATUS_OVERCURRENT  (1<<7)
#define BLDC_STATUS_HALL_FAULT   (1<<8)
#define BLDC_STATUS_HOME_TIMEOUT (1<<9)
#define BLDC_STATUS_CABLEERROR   (1<<10)

#define BLDC_STATUS_MOVING_OUT       (1<<11)
#define BLDC_STATUS_MOVING_IN        (1<<12)
#define BLDC_STATUS_HOMING           (1<<13)
#define BLDC_STATUS_ENDSWITCH_ERROR  (1<<14)
#define BLDC_STATUS_ENDSWITCH_LOW_ACTIVE (1<<15)
#define BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE (1<<16)
#define BLDC_STATUS_ERR_MOVEOUT      (1<<17) //0=moving in 1=moving out

#define BLDC_STATUS_MOVING           (1<<18)

#define BLDC_STATUS_TO_DST_ERR       (1<<19)

#define BLDC_STATUS_CLEARMASK (BLDC_STATUS_ERR|BLDC_STATUS_STALL|BLDC_STATUS_OVERCURRENT|BLDC_STATUS_CABLEERROR|BLDC_STATUS_HALL_FAULT|BLDC_STATUS_HOME_TIMEOUT|BLDC_STATUS_ENDSWITCH_ERROR|BLDC_STATUS_MOVING_TO_DST_ERR)

//bldc_status
//#define BLDC_INIT               (1U<<7)
#define BLDC_OVERVOLTAGE        (1U<<6)
#define BLDC_UNDERVOLTAGE	(1U<<5)
#define BLDC_MOTOR_CUTOFF	(1U<<4)
#define BLDC_VOLTAGE_TO_LOW	(1U<<3)
#define BLDC_SHUTTING_DOWN	(1U<<2)
#define BLDC_SAVE               (1U<<1)
#define BLDC_LOCKED             (1U<<0)

#define BLDC_CTRL_STOP         (1<<3)

#define OTHER_MOTOR(M)   ((M)^1)
#define target_error(M) (abs(bldc_motors[M].target - bldc_motors[M].position))

#define RUNOUT_ACTIVATE 1
#define RUNOUT_FREEWHEEL 0

#define BLDC_DEAD_BAND   1


#define MOTOR_START_VOLTAGE24 		20.0
#define UNDERVOLTAGE_LEVEL24		20.0
#define MOTOR_CUTOF_LEVEL24              17.0


#define MOTOR_START_VOLTAGE12 		10.2
#define UNDERVOLTAGE_LEVEL12		10
#define MOTOR_CUTOF_LEVEL12             9.8

#define SHUTDOWN_VOLTAGE                8

void LEDGreenSet(unsigned char state);
void LEDOrangeSet(unsigned char state);
unsigned char ButtonStates();





float       bldc_U(unsigned char measuring_point);
float       bldc_I(unsigned char motor);
float       get_batt_U();
int         bldc_getStatus(unsigned char motor);
int         bldc_Status();
bldc_motor *bldc_Motor(unsigned char motor);
float       bldc_position(unsigned char motor);
int         bldc_positionImp(unsigned char motor);
float       bldc_target(unsigned char motor);
int         bldc_targetImp(unsigned char motor);
float       bldc_remaining(unsigned char motor);
int         bldc_remainingImp(unsigned char motor);
int         bldc_HomeSwitchActive(unsigned char motor, unsigned char switch_h_l);

void bldc_EnableMotors(unsigned int state);
void bldc_EnableMotor(unsigned char motor, unsigned char state);
void bldc_SetDrivers(unsigned char NewState, unsigned char motor);

void getFocus(void);
int m_A_idle();
int m_B_idle();
int   motorA_moving();
int   motorB_moving();
unsigned int motor_err(uint8_t motor);

bldc_misc *  bldc_config();
unsigned int bldc_GetEnabledMotors();
int          bldc_Enabled(unsigned char motor);
unsigned int bldc_GetInvert(unsigned char motor);
unsigned int bldc_GetInvertHall(unsigned char motor);


void bldc_init(int LoadDefaults);
void bldc_initStatus(unsigned int status);
void bldc_process();//run 1ms interval 
void bldc_ClearStatus();
void bldc_Stop(int CancelManual);
void bldc_ReleaseDrivers();
void bldc_manual(int enable);
int  bldc_setPosition(unsigned char motor, float newpos, int windmode);
int  bldc_setPositionImp(unsigned char motor, int newposImp, int windmode);
int  bldc_Home(unsigned char motor);
void bldc_EnableMotors(unsigned int state);
int  bldc_Active(unsigned char motor);
void bldc_Lock(int state);
void bldc_SetInvert(unsigned char motor, unsigned int state);
void bldc_SetInvertHall(unsigned char motor, unsigned int state);
void bldc_runout(int state);


#endif