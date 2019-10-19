#ifndef __MOTOR__
#define __MOTOR__

//#include "motor_io.h"
//#include "motor_low.h"

#define MOTOR_STATE_MOVE_OUT  (1<<0)
#define MOTOR_STATE_MOVE_IN   (1<<1)
#define MOTOR_STATE_MOVE_FREE (1<<2)
#define MOTOR_STATE_REF       (1<<3)
#define MOTOR_STATE_ERR_WAIT  (1<<4)



#define MOTOR_ERR_OVERCURRENT (1<<0) 
#define MOTOR_ERR_NO_HALL     (1<<1)
#define MOTOR_ERR_REF_TIMOUT  (1<<2)
#define MOTOR_ERR_CABLE       (1<<3)


#define MOTOR_VOLTAGE_TO_LOW (1<<0)
#define MOTOR_OUT_OF_SYNC    (1<<1)

///warnings
#define MOTOR_A_LOSING_PULSES (1<<0)
#define MOTOR_B_LOSING_PULSES (1<<1)


#define MOTOR_MANUAL_TIMEOUT     25 //cycles 0,5s
#define MOTOR_PROCESS_FREQ       50
#define NO_ENDSITCH_PRESSED_TIME 100 //2s

#define HALL_LOSING_IMPULSES_TRESHOLD 10

typedef struct{
  float current;
  float position;
  float voltage;
}err_parm_t;

void init_motor(unsigned int posA, unsigned int  posB, unsigned int flags,unsigned int warnings);
void motor_process();

int m_go_refA();
int m_go_refB();
int m_goA(int newpos);
int m_goB(int newpos);
int m_go_AB();
int m_go_manualA(int in);
int m_go_manualB(int in);
void m_manual_stop();
void m_stop();



int m_posA();
int m_posB();
int m_destA();
int m_destB();
int m_remainA();
int m_remainB();

unsigned int motor_stateA();
unsigned int motor_stateB();
unsigned int motor_stateComm();
unsigned int motor_stateWarn();
unsigned int motor_errA();
unsigned int motor_errB();
void motor_clear_status();

unsigned int motor_exStat();
void motor_exStatClr(unsigned int mask);
unsigned int motor_event();
err_parm_t *motor_err_stateA();
err_parm_t *motor_err_stateB();

int m_A_idle();//motor in complete stop  (State and Hardware)
int m_B_idle();//motor in complete stop  (State and Hardware)
int m_idle();//both motors idle
int m_referencing();//one of motors is executing reference


void m_windmode_moveA();
void m_windmode_moveB();

void ActivateEvent(unsigned int ev);//mogoce ne spada sem

#endif