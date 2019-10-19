#include "suntracer.h"
#include "focus.h"
#include <math.h>

#include "bldc.h"

extern float parameters [N_parameters];
extern float parameters_fixed [N_parameters_fixed];
extern float offset_A, offset_B;
extern uint32_t adc3_VAL;
extern uint32_t adc4_VAL;

extern bldc_motor bldc_motors[BLDC_MOTOR_COUNT];

float focus_offsetA;
float focus_offsetB;




#define FOCUS_STATE_START           (1<<0)
#define FOCUS_STATE_MOTOR_START_A   (1<<1)
#define FOCUS_STATE_MOTOR_START_B   (1<<2)
#define FOCUS_STATE_MOTOR_FIN_A     (1<<3)
#define FOCUS_STATE_MOTOR_FIN_B     (1<<4)
#define FOCUS_STATE_MOTOR_IA        (1<<5)
#define FOCUS_STATE_MOTOR_IB        (1<<6)
#define FOCUS_STATE_NO_LIGHT        (1<<7)
#define FOCUS_STATE_LOCKED_A        (1<<8)
#define FOCUS_STATE_LOCKED_B        (1<<9)

#define FOCUS_STATE_CLEAR_A (FOCUS_STATE_MOTOR_START_A|FOCUS_STATE_MOTOR_FIN_A|FOCUS_STATE_MOTOR_IA)
#define FOCUS_STATE_CLEAR_B (FOCUS_STATE_MOTOR_START_B|FOCUS_STATE_MOTOR_FIN_B|FOCUS_STATE_MOTOR_IB)
unsigned int focus_state;





void focus_init(){
   signed short tmps;
   
   //tmps  = BKP_ReadBackupRegister(BKP_DR6); 
   focus_offsetA=((float)tmps)/3000;
   
   
   //tmps = BKP_ReadBackupRegister(BKP_DR7);
   focus_offsetB=((float)tmps)/3000;
   
   //limit
   if(focus_offsetA >  focus_max_offset)  focus_offsetA =  focus_max_offset;
   if(focus_offsetA < -focus_max_offset)  focus_offsetA = -focus_max_offset;  
   if(focus_offsetB >  focus_max_offset)  focus_offsetB  =  focus_max_offset;
   if(focus_offsetB < -focus_max_offset)  focus_offsetB  = -focus_max_offset;
   
   offset_A      = 0;
   offset_B      = 0;
   focus_state   = 0;

}



void focus_calc_A(){
   double diff;
   signed short   tmp;
   
   diff           = f_FocusMiddleA - adc4_VAL; 
   focus_offsetA -= diff * 0.0001;
  
   if(focus_offsetA >  focus_max_offset) focus_offsetA =  focus_max_offset;
   if(focus_offsetA < -focus_max_offset) focus_offsetA = -focus_max_offset;
     
   
   tmp=(signed short)(focus_offsetA*3000.0);
   //BKP_WriteBackupRegister(BKP_DR6,tmp);
}


void focus_calc_B(){
   double diff;
   signed short   tmp;
   
   diff           = f_FocusMiddleB - adc3_VAL ;
   focus_offsetB -= diff * 0.0001;
  
 
   if(focus_offsetB >  focus_max_offset)  focus_offsetB  =  focus_max_offset;
   if(focus_offsetB < -focus_max_offset)  focus_offsetB  = -focus_max_offset;
   
   tmp=(signed short)(focus_offsetB*3000.0);
   //BKP_WriteBackupRegister(BKP_DR7,tmp);
}


void focus_reset(){
  offset_A      = 0;
  offset_B      = 0;
  focus_state   = 0;
  focus_offsetA = 0;
  focus_offsetB = 0;
  //BKP_WriteBackupRegister(BKP_DR7,0);
  //BKP_WriteBackupRegister(BKP_DR6,0);
}

void focus_restart(){ 
  focus_state   = FOCUS_STATE_START;
}

void focus_start(){
  if(focus_state&FOCUS_STATE_MOTOR_FIN_A)focus_state &=~ FOCUS_STATE_CLEAR_A;
  if(focus_state&FOCUS_STATE_MOTOR_FIN_B)focus_state &=~ FOCUS_STATE_CLEAR_B;
    
  focus_state  |= FOCUS_STATE_START;
  focus_update();
}

void focus_cancel()
{
    focus_state  &= ~FOCUS_STATE_START;
    focus_state  &=~ FOCUS_STATE_CLEAR_A;
    focus_state  &=~ FOCUS_STATE_CLEAR_B;
}



//sample focus sensor when motor reaches position
void focus_process(){ 

getFocus();

 uint32_t errA = abs((unsigned int)f_FocusMiddleA - adc4_VAL);
 uint32_t errB = abs((unsigned int)f_FocusMiddleB - adc3_VAL);
 
 
 if(bldc_motors[0].state & BLDC_STATUS_ENDSWITCH_LOW_ACTIVE){
   focus_state |=  FOCUS_STATE_NO_LIGHT;
   focus_cancel();
 } else   focus_state &=~ FOCUS_STATE_NO_LIGHT;
 
 
 
 if((focus_state&FOCUS_STATE_START)&&(!focus_locked())){
    
   if(((!m_A_idle())||(focus_state&FOCUS_STATE_MOTOR_IA))&&(!(focus_state&FOCUS_STATE_MOTOR_FIN_A))){//A shoud move first
      focus_state|=FOCUS_STATE_MOTOR_IA;
      if(motorA_moving()){
        focus_state|=FOCUS_STATE_MOTOR_START_A;
      }else{    
        if( (focus_state&FOCUS_STATE_MOTOR_START_A)&&(!(focus_state&FOCUS_STATE_MOTOR_FIN_A))){
          focus_calc_A();
          focus_state|=FOCUS_STATE_MOTOR_FIN_A;
        }   
      }
   }else{
     focus_state |= FOCUS_STATE_MOTOR_FIN_A;
   } 
    
   
   
   if(((!m_B_idle())||(focus_state&FOCUS_STATE_MOTOR_IB)) && (!(focus_state&FOCUS_STATE_MOTOR_FIN_B))){ 
     focus_state |= FOCUS_STATE_MOTOR_IB;
      if(motorB_moving()){
        focus_state|=FOCUS_STATE_MOTOR_START_B;
      }else{    
        if( (focus_state&FOCUS_STATE_MOTOR_START_B)&&(!(focus_state&FOCUS_STATE_MOTOR_FIN_B))){
          focus_calc_B();   
          focus_state|=FOCUS_STATE_MOTOR_FIN_B;
        }  
      }  
   }else {
     focus_state |= FOCUS_STATE_MOTOR_FIN_B;
   }

   //Save offset and set state locked A 
   if(focus_state & FOCUS_STATE_MOTOR_FIN_A)
     if(errA<30.0)focus_state       |= FOCUS_STATE_LOCKED_A;   
     
   //Save offset and set state locked B 
   if(focus_state & FOCUS_STATE_MOTOR_FIN_B)
     if(errB<30.0)focus_state       |= FOCUS_STATE_LOCKED_B;     
   
 }
}


int focus_A_OK(){
  return  (motor_err(0) || (geometry_mode_A==0))?0:1;
}
int focus_B_OK(){ 
  return  (motor_err(1) || (geometry_mode_B==0))?0:1;
}

int focus_locked()
{ 
  if(focus_state&FOCUS_STATE_NO_LIGHT) return 1;//no light return locked ..state changes to AVG
  
  if ((!focus_A_OK()) && ( focus_B_OK())) 
     if(focus_state&FOCUS_STATE_LOCKED_B)return 1;
  if (( focus_A_OK()) && ( !focus_B_OK())) 
     if(focus_state&FOCUS_STATE_LOCKED_A)return 1;
  //both ok   
  if((focus_state&FOCUS_STATE_LOCKED_A)&&(focus_state&FOCUS_STATE_LOCKED_B))return 1;
  
  return 0;
}

void focus_update()
{
  offset_A = focus_offsetA;
  offset_B = focus_offsetB;
}

float focus_angle_A() {return offset_A;}
float focus_angle_B() {return offset_B;}