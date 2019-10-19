#include "suntracer.h"
#include "motor.h"
#include "motor_io.h"
#include "motor_low.h"
#include <math.h>


#define MOTOR_STATE_STOP_ALL_MASK (0xffffffff)
#define MOTOR_STATE_MOVING_MASK   (MOTOR_STATE_MOVE_OUT|MOTOR_STATE_MOVE_IN|MOTOR_STATE_MOVE_FREE)
#define MOTOR_STATE_STOP_MASK     (MOTOR_STATE_MOVE_OUT|MOTOR_STATE_MOVE_IN|MOTOR_STATE_MOVE_FREE)
#define MOTOR_STATE_STOP_INOUT    (MOTOR_STATE_MOVE_OUT|MOTOR_STATE_MOVE_IN)
#define MOTOR_STATE_DOING_REF     (MOTOR_STATE_MOVE_IN |MOTOR_STATE_REF)

#define MOTOR_IDLE_A              (!m_stateA)
#define MOTOR_IDLE_B              (!m_stateB)

#define MOTOR_SYNCED              (cflags&(1<<RunBothMotors))
#define MOTOR_ONE_HALLA           (cflags&(1<<number_of_halls_A))
#define MOTOR_ONE_HALLB           (cflags&(1<<number_of_halls_B))

extern float parameters [N_parameters];
extern float parameters_fixed [N_parameters_fixed];
extern u32 bflags;

extern u32 HA_speed;
extern u32 HB_speed;
extern u32 VA_speed;
extern u32 VB_speed;

extern signed int movetimeA;
extern signed int movetimeB;
extern u32 cflags;

//fsta
extern unsigned int zerospeedA, zerospeedB;
extern unsigned int diff_lowA, diff_lowB;
//fsta

int motor_posA;         
int motor_destA;     
int remain_A;
#define motor_movetimeA        movetimeA
int motor_overcurrent_cntA;
int motor_hall_err_cntA;
int motor_cable_err_cntA;
unsigned int m_errA;
unsigned int m_stateA;
int time_out_A;


int motor_posB ;            
int motor_destB;   
int remain_B;
#define motor_movetimeB       movetimeB
int motor_overcurrent_cntB;
int motor_hall_err_cntB;
int motor_cable_err_cntB;
unsigned int  m_errB;
unsigned int m_stateB;
int time_out_B;

int motor_manual_timeout;
int m_common_err;
unsigned int m_common_warn;

unsigned int m_ext_stat;
unsigned int event;
err_parm_t err_parm_A;
err_parm_t err_parm_B;

int m_A_pressed_detect;
int m_B_pressed_detect;
int m_save;

int m_windmode;

void _clear_A_Err(){m_stateA&=~MOTOR_STATE_ERR_WAIT;}
void _clear_B_Err(){m_stateB&=~MOTOR_STATE_ERR_WAIT;}

void init_motor(unsigned int posA, unsigned int  posB, unsigned int flags,unsigned int warnings)
{
  motor_low_init();
  motor_posA = motor_destA = posA;
  motor_posB = motor_destB = posB;
  
  //setup error flags
  m_errA       =  flags    &0x0f;
  m_errB       = (flags>>4)&0x0f;

  m_ext_stat   =  0;
  m_common_err =  0;
  
  m_common_warn=  warnings;
  
  motor_cable_err_cntA=0;
  motor_hall_err_cntA=0;
  motor_overcurrent_cntA=0;
 
  motor_cable_err_cntB=0;
  motor_hall_err_cntB=0;
  motor_overcurrent_cntB=0;
  
  m_stateA=0;
  m_stateB=0;
  remain_B=0;
  remain_A=0;
  
  time_out_A=0;
  time_out_B=0;
  
  m_A_pressed_detect=0;
  m_B_pressed_detect=0;
  m_save=3;
  m_windmode=0;
}

void ActivateEvent(unsigned int ev){
	m_ext_stat       |= EFS_EVENTS_ACTIVE;
	event   	 = ev;
}

void SetEventParametersA(){
   err_parm_A.current  = motor_Current();
   err_parm_A.position = motor_posA/gear_ratio_A;
   err_parm_A.voltage  = system_Voltage();
}

void SetEventParametersB(){
   err_parm_B.current  = motor_Current();
   err_parm_B.position = motor_posB/gear_ratio_B;
   err_parm_B.voltage  = system_Voltage();
}


void _motor_check_move()
{
    int diffA,diffB;
    
        
    double vcor =  (system_Voltage() -24.0)/2.0;
    int    Astop;
    int    Bstop;
    
    
    if(motor0_full_pwm()){
      if(vcor>=0) Astop = DIFF_HIGH + (int)( vcor * (DIFF_HIGH/5.0));
      else        Astop = DIFF_HIGH;
    }else{
      //fsta Astop=DIFF_LOW;
      Astop = diff_lowA;
    }
    
    if(motor1_full_pwm()){
      if(vcor>=0) Bstop = DIFF_HIGH + (int)( vcor * (DIFF_HIGH/5.0));
      else        Bstop = DIFF_HIGH;
    }else{
       //fsta Bstop=DIFF_LOW;
      Bstop = diff_lowB;
    } 
    
    //fsta if(Astop<DIFF_LOW)Astop=DIFF_LOW;
    if(Astop < diff_lowA) Astop = diff_lowA;
    //fsta if(Bstop<DIFF_LOW)Bstop=DIFF_LOW;
    if(Bstop < diff_lowB )Bstop = diff_lowB;
    
    diffA = motor_destA - motor_posA;	                        //destination_A is different like current position?
    diffB = motor_destB - motor_posB;	                        //destination_B is different like current position?
    
    
    
    
    //safety timeout for manual
    if((m_stateB & MOTOR_STATE_MOVE_FREE) || (m_stateA & MOTOR_STATE_MOVE_FREE)){                                                       //Timeout manual mode move
      motor_manual_timeout++;
      if(motor_manual_timeout >= MOTOR_MANUAL_TIMEOUT) m_manual_stop();
    }
    
   
    if(1){// move one axis at a time
      
      if(m_stateA & MOTOR_STATE_MOVE_FREE) motor_destA=motor_posA;//correct position for manual(free) movement
      if(m_stateB & MOTOR_STATE_MOVE_FREE) motor_destB=motor_posB;//correct position for manual(free) movement

      
      if((!(m_stateA & MOTOR_STATE_MOVE_FREE))&&(!(m_stateB & MOTOR_STATE_MOVE_FREE))){ 
	
        if  ((!m_errA)&&((!(m_stateB & MOTOR_STATE_MOVING_MASK))||(MOTOR_SYNCED)))  {//MOTOR A
            if (m_stateA&MOTOR_STATE_REF){
                m_stateA &= ~MOTOR_STATE_MOVE_OUT;
                m_stateA |=  MOTOR_STATE_MOVE_IN; 
                m_ext_stat &= ~ESF_MOVE_OUT_ERR_A;
            }
            //fsta else if ((diffA>=DIFF_LOW)||(diffA<=-DIFF_LOW)){
            else if ((diffA >= diff_lowA) || (diffA <= -diff_lowA)) {
		  if(abs(diffA)>Astop){		
                        if (diffA>0) {
				 m_stateA &= ~MOTOR_STATE_MOVE_IN;
                                 m_stateA |=  MOTOR_STATE_MOVE_OUT;
                                 m_ext_stat |= ESF_MOVE_OUT_ERR_A;
			}
			else {
				m_stateA &= ~MOTOR_STATE_MOVE_OUT;
                                m_stateA |=  MOTOR_STATE_MOVE_IN; 
                                m_ext_stat &= ~ESF_MOVE_OUT_ERR_A;
			}
                   }else m_stateA&=~MOTOR_STATE_STOP_INOUT;
            }else m_stateA&=~MOTOR_STATE_STOP_INOUT;  
        }
        
#ifndef DISABLE_MOTOR_B
        if ((!m_errB)&&((!(m_stateA&MOTOR_STATE_MOVING_MASK))||(MOTOR_SYNCED))) { //MOTOR B
	    if (m_stateB&MOTOR_STATE_REF){
              m_stateB &= ~MOTOR_STATE_MOVE_OUT;
              m_stateB |=  MOTOR_STATE_MOVE_IN;  
              m_ext_stat &= ~ESF_MOVE_OUT_ERR_B;
            } 
            //fsta else if ((diffB>=DIFF_LOW)||(diffB<=-DIFF_LOW)){
            else if ((diffB >= diff_lowB) || (diffB <= -diff_lowB)){
                if(abs(diffB)>Bstop){	
                        if (diffB>0) {
				  m_stateB &= ~MOTOR_STATE_MOVE_IN;
                                  m_stateB |=  MOTOR_STATE_MOVE_OUT; 
                                  m_ext_stat |= ESF_MOVE_OUT_ERR_B;
			}
			else {
				  m_stateB &= ~MOTOR_STATE_MOVE_OUT;
                                  m_stateB |=  MOTOR_STATE_MOVE_IN; 
                                  m_ext_stat &= ~ESF_MOVE_OUT_ERR_B;
			}
                }else m_stateB&=~MOTOR_STATE_STOP_INOUT;
           }else m_stateB&=~MOTOR_STATE_STOP_INOUT;
	}
#endif
      }
     }

    
    //min max check
    if (!(m_stateA&MOTOR_STATE_REF)){
      if(m_stateA & MOTOR_STATE_MOVE_IN)
         if(motor_posA<=min_range_A)m_stateA&=~MOTOR_STATE_STOP_MASK;
      if(m_stateA &MOTOR_STATE_MOVE_OUT)
        if(motor_posA>=max_range_A)m_stateA&=~MOTOR_STATE_STOP_MASK;
     
      if(!(m_stateA & MOTOR_STATE_MOVE_FREE)){
        if((motor_posA >= motor_destA)&&(m_stateA&MOTOR_STATE_MOVE_OUT))  m_stateA&=~MOTOR_STATE_STOP_MASK;
        if((motor_posA <= motor_destA)&&(m_stateA&MOTOR_STATE_MOVE_IN))   m_stateA&=~MOTOR_STATE_STOP_MASK;
      }
    }
    
    
#ifndef DISABLE_MOTOR_B    
    if (!(m_stateB&MOTOR_STATE_REF)){
      if(m_stateB &MOTOR_STATE_MOVE_IN)
       if(motor_posB<=min_range_B)m_stateB&=~MOTOR_STATE_STOP_MASK;
      if(m_stateB &MOTOR_STATE_MOVE_OUT)
       if(motor_posB>=max_range_B)m_stateB&=~MOTOR_STATE_STOP_MASK;
     
      //motor destination reached
      if(!(m_stateB & MOTOR_STATE_MOVE_FREE)){
        if((motor_posB >= motor_destB)&&(m_stateB&MOTOR_STATE_MOVE_OUT))  m_stateB&=~MOTOR_STATE_STOP_MASK;
        if((motor_posB <= motor_destB)&&(m_stateB&MOTOR_STATE_MOVE_IN))   m_stateB&=~MOTOR_STATE_STOP_MASK;
      }
    }
#endif
    
    //sync position when stoped
    if(MOTOR_IDLE_A&&MOTOR_IDLE_B){
      if(MOTOR_SYNCED){
         motor_destA = motor_posA;
         motor_destB = motor_posA;
      }
    }
    
   /* //limit motor range
    if((motor_posA>max_range_A) && (!(m_stateA&MOTOR_STATE_MOVING_MASK))&& (!(m_stateB&MOTOR_STATE_MOVING_MASK)) &&(!(m_stateA&MOTOR_STATE_REF))) motor_destA = (unsigned int)max_range_A;
    if((motor_posA<min_range_A) && (!(m_stateA&MOTOR_STATE_MOVING_MASK))&& (!(m_stateB&MOTOR_STATE_MOVING_MASK)) &&(!(m_stateA&MOTOR_STATE_REF))) motor_destA = (unsigned int)min_range_A;
#ifndef DISABLE_MOTOR_B
    if((motor_posB>max_range_B) && (!(m_stateA&MOTOR_STATE_MOVING_MASK))&& (!(m_stateB&MOTOR_STATE_MOVING_MASK)) &&(!(m_stateA&MOTOR_STATE_REF))) motor_destB = (unsigned int)max_range_B;
    if((motor_posB<min_range_B) && (!(m_stateA&MOTOR_STATE_MOVING_MASK))&& (!(m_stateB&MOTOR_STATE_MOVING_MASK)) &&(!(m_stateA&MOTOR_STATE_REF))) motor_destB = (unsigned int)min_range_B;
#endif*/
}

void _endswitch_check()
{
  float motor_I = motor_Current();

  //sync move no current check
  if((m_stateA & MOTOR_STATE_DOING_REF)&&(m_stateB & MOTOR_STATE_DOING_REF))
    if((m_stateA&MOTOR_STATE_MOVE_IN)&&(m_stateB&MOTOR_STATE_MOVE_IN)){
      if(motorA_moving() && motorB_moving()){
         if(!MOTOR_ONE_HALLA){
           //fsta if((HA_speed==ZEROSPEED) && (HB_speed==ZEROSPEED)&&(m_A_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
           if((HA_speed == zerospeedA) && (HB_speed == zerospeedB) && (m_A_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)) {
             m_stateA   &=~MOTOR_STATE_STOP_ALL_MASK;
             remain_A    = motor_posA;	
             motor_posA  = 0;
             motor_destA = 0;
             SetEventParametersA();
             ActivateEvent(EVENT_HOME_A_FINISHED);
             saveBackup();
          }
         //fsta }else if((HA_speed==ZEROSPEED)&&(m_A_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
         } else if((HA_speed == zerospeedA) && (m_A_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)) {
           m_stateA   &=~MOTOR_STATE_STOP_ALL_MASK;
           remain_A    = motor_posA;	
           motor_posA  = 0;
           motor_destA = 0;
           ActivateEvent(EVENT_HOME_B_FINISHED);
           saveBackup();
         }
         
      
         if(!MOTOR_ONE_HALLB){
           //fsta if((VA_speed==ZEROSPEED) && (VB_speed==ZEROSPEED)&&(m_B_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
           if((VA_speed == zerospeedA) && (VB_speed == zerospeedB) && (m_B_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)) {
            m_stateB   &=~MOTOR_STATE_STOP_ALL_MASK;
            remain_B    = motor_posB;	
            motor_posB  = 0;
            motor_destB = 0;
            ActivateEvent(EVENT_HOME_B_FINISHED);
            saveBackup();
           }
         //fsta }else if((VA_speed==ZEROSPEED)&&(m_B_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
         } else if((VA_speed == zerospeedA) && (m_B_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)){
           m_stateB   &=~MOTOR_STATE_STOP_ALL_MASK;
           remain_B    = motor_posB;	
           motor_posB  = 0;
           motor_destB = 0;
           ActivateEvent(EVENT_HOME_B_FINISHED);
           saveBackup();
         }
      
      }else{
       m_A_pressed_detect=0;
       m_B_pressed_detect=0;
      }
      
      return;
    }

  //motor A endswitch check
  if((m_stateA & MOTOR_STATE_DOING_REF)==MOTOR_STATE_DOING_REF){
#ifndef SELECT_TIV27C_E
    if(motorA_endsw_pressed_lo()){
      m_stateA   &=~MOTOR_STATE_STOP_ALL_MASK;
      remain_A    = motor_posA;	
      motor_posA  = 0;
      motor_destA = 0;
      SetEventParametersA();
      ActivateEvent(EVENT_HOME_A_FINISHED);
      saveBackup();
    }else
#endif      
    if(motor_movetimeA==REF_TOOLONG_PARAM){
      m_stateA    &= ~MOTOR_STATE_STOP_ALL_MASK;
      m_errA      |=  MOTOR_ERR_REF_TIMOUT;
    }
    
    if((motorA_moving())&&(!motorB_moving())&&(motor_I <= 0.06)){
      if(!MOTOR_ONE_HALLA){
        //fsta if((HA_speed==ZEROSPEED) && (HB_speed==ZEROSPEED)&&(m_A_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
        if((HA_speed == zerospeedA) && (HB_speed == zerospeedB) && (m_A_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)){
           m_stateA   &=~MOTOR_STATE_STOP_ALL_MASK;
           remain_A    = motor_posA;	
           motor_posA  = 0;
           motor_destA = 0;
           SetEventParametersA();
           ActivateEvent(EVENT_HOME_A_FINISHED);
           saveBackup();
        }
      //fsta }else if((HA_speed==ZEROSPEED)&&(m_A_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
      } else if((HA_speed == zerospeedA) && (m_A_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)){
           m_stateA   &=~MOTOR_STATE_STOP_ALL_MASK;
           remain_A    = motor_posA;	
           motor_posA  = 0;
           motor_destA = 0;
           SetEventParametersA();
           ActivateEvent(EVENT_HOME_A_FINISHED);
           saveBackup();
      }
    }else  m_A_pressed_detect=0;
  }else m_A_pressed_detect=0;
  
  
#ifndef DISABLE_MOTOR_B                
  //motor B endswitch check                                                  
  if((m_stateB & MOTOR_STATE_DOING_REF)==MOTOR_STATE_DOING_REF){
#ifndef SELECT_TIV27C_E
    if( motorB_endsw_pressed_lo()){
      m_stateB   &=~MOTOR_STATE_STOP_ALL_MASK;
      remain_B    = motor_posB;	
      motor_posB  =0;
      motor_destB =0;
      ActivateEvent(EVENT_HOME_B_FINISHED);
      saveBackup();
    }else
#endif      
      if(motor_movetimeB == REF_TOOLONG_PARAM){
      m_stateB      &=~MOTOR_STATE_STOP_ALL_MASK;
      m_errB        |= MOTOR_ERR_REF_TIMOUT;
    }
                  
    if((motorB_moving())&&(!motorA_moving())&&(motor_I <= 0.06)){
      if(!MOTOR_ONE_HALLB){
        //fsta if((VA_speed==ZEROSPEED) && (VB_speed==ZEROSPEED) && (m_B_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
        if((VA_speed == zerospeedA) && (VB_speed == zerospeedB) && (m_B_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)) {
           m_stateB   &=~MOTOR_STATE_STOP_ALL_MASK;
           remain_B    = motor_posB;	
           motor_posB  = 0;
           motor_destB = 0; 
           ActivateEvent(EVENT_HOME_B_FINISHED);
           saveBackup();
        }
      //fsta }else if((VA_speed==ZEROSPEED)&&(m_B_pressed_detect++>=NO_ENDSITCH_PRESSED_TIME)){
      } else if((VA_speed == zerospeedA) && (m_B_pressed_detect++ >= NO_ENDSITCH_PRESSED_TIME)) {
           m_stateB   &=~MOTOR_STATE_STOP_ALL_MASK;
           remain_B    = motor_posB;	
           motor_posB  = 0;
           motor_destB = 0;
           ActivateEvent(EVENT_HOME_B_FINISHED);
           saveBackup();
      }
   }else  m_B_pressed_detect=0;
  }else m_B_pressed_detect=0;
#endif
}



void _error_detection(void) {
    int diffAB;    
    float motor_I = motor_Current();
    float ILimit;
    float IInrush;
    unsigned int errA=0;
    unsigned int errB=0;

    diffAB = abs(motor_posA - motor_posB);	
    
    if (system_Voltage()<6.0){
        m_common_err |=  MOTOR_VOLTAGE_TO_LOW;
        m_stop();
    }else m_common_err &= ~MOTOR_VOLTAGE_TO_LOW;


    
    
    if(motorA_moving()&& motorB_moving()){
       if(((motor_I > (2*max_Imotor_A))&&(motor_movetimeA >= Mot_inrush_time_A)) || ((motor_I > (2*max_Imotor_A*Mot_inrush_ratio_A))&&(motor_movetimeA < Mot_inrush_time_A))) {//Overcurrent A
        SetEventParametersA();
        SetEventParametersB();
        m_stateA |= MOTOR_STATE_ERR_WAIT;
        if (motor_overcurrent_cntA<5) motor_overcurrent_cntA++;
      }
    }else{
                        //single axis move
      if(motorA_moving()){  
        if(m_windmode&1){
           ILimit  = max_Imotor_A * 1.5;
           IInrush = max_Imotor_A * Mot_inrush_ratio_A;
        }else{
           ILimit  = max_Imotor_A ;
           IInrush = max_Imotor_A * Mot_inrush_ratio_A;
        }
        
        if(((motor_I > ILimit)&&(motor_movetimeA >= Mot_inrush_time_A)) || ((motor_I > IInrush)&&(motor_movetimeA < Mot_inrush_time_A))) {//Overcurrent A
         m_stateA |= MOTOR_STATE_ERR_WAIT;
         SetEventParametersA();
         if (motor_overcurrent_cntA<8) motor_overcurrent_cntA++;
       }
      } 
      
      
      
      if(motorB_moving()){
         if(m_windmode&2){
           ILimit  = max_Imotor_B * 1.5;
           IInrush = max_Imotor_B * Mot_inrush_ratio_B;
        }else{
           ILimit  = max_Imotor_B ;
           IInrush = max_Imotor_B * Mot_inrush_ratio_B;
        }
        
        if(((motor_I > ILimit)&&(motor_movetimeB >= Mot_inrush_time_B)) || ((motor_I > IInrush)&&(motor_movetimeB <Mot_inrush_time_B))) {//Overcurrent B
         SetEventParametersB();
         m_stateB |= MOTOR_STATE_ERR_WAIT;
         if (motor_overcurrent_cntB<8) motor_overcurrent_cntB++;
        }
      }
    }
       
    
    //Hall or Cable error detection MOTOR A
    //fsta if ((motorA_moving())&&((HA_speed==ZEROSPEED) || ((!MOTOR_ONE_HALLA)&&(HB_speed==ZEROSPEED)))) {
    if ((motorA_moving()) && ((HA_speed == zerospeedA) || ((!MOTOR_ONE_HALLA) && (HB_speed == zerospeedB)))) {
      if (time_out_A==150) {                    	        //3 sekunde time-out-a (3000*1ms)
        m_stateA |= MOTOR_STATE_ERR_WAIT;
        //nepricakovana zaustavitev izven REF
          
          if (motor_posA<1000) motor_destA = motor_posA = 0;         //koncno stikalo je prej ustavilo - naredi referenco
          
          if (motor_I>0.06) {
            if (motor_hall_err_cntA<5) motor_hall_err_cntA++;                             //tok je, hallov ni   
          }else {
#ifndef SELECT_MICRO            
            if (motor_posA >= (max_range_A*0.8)){
               m_stateA      &=~MOTOR_STATE_STOP_ALL_MASK;
               motor_destA    = motor_posA;
            }else
#endif 
            if (motor_cable_err_cntA<5) motor_cable_err_cntA++;  //ni toka, ni hallov
          }
       //}
      }else time_out_A++;
    }else time_out_A=0;


    //Hall or Cable error detection MOTOR A
    //fsta if ((motorB_moving())&&((VA_speed==ZEROSPEED) || ((!MOTOR_ONE_HALLB)&&(VB_speed==ZEROSPEED)))) {
    if ((motorB_moving()) && ((VA_speed == zerospeedA) || ((!MOTOR_ONE_HALLB) && (VB_speed == zerospeedB)))) {
      if (time_out_B==150) {                    	        //3 sekunde time-out-a (3000*1ms)
        m_stateB |= MOTOR_STATE_ERR_WAIT;
         
          if (motor_posB<1000) motor_destB = motor_posB = 0;         //koncno stikalo je prej ustavilo - naredi referenco
         
          if (motor_I>0.06) {
            if (motor_hall_err_cntB<5) motor_hall_err_cntB++;                             //tok je, hallov ni
          }else {
#ifndef SELECT_MICRO            
            if (motor_posB >= (max_range_B*0.8)){
               m_stateB      &=~MOTOR_STATE_STOP_ALL_MASK;
               motor_destB    = motor_posB;
            }else
#endif 
            if (motor_cable_err_cntB<5) motor_cable_err_cntB++;                           //ni toka, ni hallov
          }
      }else time_out_B++;
    }else time_out_B=0;

    
   
    if(MOTOR_SYNCED){//synced move
      if(motor_overcurrent_cntA>=1){m_errA |= MOTOR_ERR_OVERCURRENT;errA=1;}
      if(motor_hall_err_cntA   >=1){m_errA |= MOTOR_ERR_NO_HALL    ;errA=1;}
      if(motor_cable_err_cntA  >=1){m_errA |= MOTOR_ERR_CABLE      ;errA=1;}
      
      if(motor_overcurrent_cntB>=1) {m_errB |= MOTOR_ERR_OVERCURRENT;errA=1;}
      if (motor_hall_err_cntB  >=1) {m_errB |= MOTOR_ERR_NO_HALL    ;errA=1;}
      if (motor_cable_err_cntB >=1) {m_errB |= MOTOR_ERR_CABLE      ;errA=1;}
      
      if(errA){
         SetEventParametersA();
         SetEventParametersB();
         m_stop();
      }
    }else{//single axis move
      if(motor_overcurrent_cntA>=8){m_errA |= MOTOR_ERR_OVERCURRENT;errA=1;}
      if(motor_hall_err_cntA   >=5){m_errA |= MOTOR_ERR_NO_HALL    ;errA=1;}
      if(motor_cable_err_cntA  >=5){m_errA |= MOTOR_ERR_CABLE      ;errA=1;}
      
      if(motor_overcurrent_cntB>=8) {m_errB |= MOTOR_ERR_OVERCURRENT;errB=1;}
      if (motor_hall_err_cntB  >=5) {m_errB |= MOTOR_ERR_NO_HALL    ;errB=1;}
      if (motor_cable_err_cntB >=5) {m_errB |= MOTOR_ERR_CABLE      ;errB=1;}

      if(errA){
         m_stateA      &=~MOTOR_STATE_STOP_ALL_MASK;
         motor_destA    = motor_posA;
         SetEventParametersA();
      }
    
      if(errB){
         m_stateB      &=~MOTOR_STATE_STOP_ALL_MASK;
         motor_destB    = motor_posB;
         SetEventParametersB();
      }
    }
       
    if(MOTOR_SYNCED){
      if(diffAB>=group){
       SetEventParametersA();
       SetEventParametersB();
       m_stop();
       m_common_err|=MOTOR_OUT_OF_SYNC;
      }else  m_common_err&=~MOTOR_OUT_OF_SYNC;
    }else m_common_err&=~MOTOR_OUT_OF_SYNC;
    
    
}

extern int HallCnt1A,HallCnt1B,HallCnt2A,HallCnt2B;

void _auto_err_cnt_clear()
{
  if(MOTOR_IDLE_A){
     motor_overcurrent_cntA=0;
     motor_hall_err_cntA   =0;
     motor_cable_err_cntA  =0;
     HallCnt1A=0;
     HallCnt1B=0;  
     if(!(m_save&1)){
        m_save|=1;
        saveBackup();
     }
  }else{
    if(abs(HallCnt1A-HallCnt1B) >= HALL_LOSING_IMPULSES_TRESHOLD) m_common_warn |= MOTOR_A_LOSING_PULSES; 
  }
  
  if(MOTOR_IDLE_B){
     motor_overcurrent_cntB=0;
     motor_hall_err_cntB   =0;
     motor_cable_err_cntB  =0;
     HallCnt2A=0;
     HallCnt2B=0;
     if(!(m_save&2)){
        m_save|=2;
        saveBackup();
     }   
  }else{
      if(abs(HallCnt2A-HallCnt2B) >= HALL_LOSING_IMPULSES_TRESHOLD)m_common_warn |= MOTOR_B_LOSING_PULSES;  
  }
}

void auto_remove_windmode()
{
  if(m_A_idle())m_windmode&=~1;
  if(m_B_idle())m_windmode&=~2;
}


void motor_process(){
  _motor_check_move();
  _endswitch_check();
  _error_detection();
  _auto_err_cnt_clear();
  auto_remove_windmode();
}
   
   
void m_stop(){
  m_stateA &= ~MOTOR_STATE_STOP_ALL_MASK;
  m_stateB &= ~MOTOR_STATE_STOP_ALL_MASK;
  if(!MOTOR_SYNCED){
    motor_destA = motor_posA;
    motor_destB = motor_posB;
  }else{
    motor_destA = motor_posA;
    motor_destB = motor_posA;
  }
}

int _goA(){
  int run=0;
  if (GetMode()== MICRO_MODE_STANDALONE){
     run = (int)geometry_mode_A; 
  }else{
     run = AxisEnabled(0); 
  }
  return run; 
}

int _goB(){
  int run=0;
  if (GetMode()== MICRO_MODE_STANDALONE){
     run = (int)geometry_mode_B; 
  }else{
     run = AxisEnabled(1); 
  }
  return run; 
}


int m_go_refA()
{
  if(cflags&(1<<DisableReference)) return -1;
  if(MOTOR_SYNCED){
    if ((geometry_mode_A==0)||(geometry_mode_B==0)) return -1;  
    if((m_errA)||(m_errB)||(m_common_err))return -1; 
    m_stateA|=MOTOR_STATE_REF;
    m_stateB|=MOTOR_STATE_REF;
    m_save&=~3; //enable save when stopped
    return 0;
  }
  
  if((!m_errA)&&(_goA())){
     m_stateA|=MOTOR_STATE_REF;
     m_save&=~3; //enable save when stopped
     return 0;
  }
  return  -1;
}

int m_go_refB()
{
#ifndef DISABLE_MOTOR_B  
  if(cflags&(1<<DisableReference)) return -1;
  if(MOTOR_SYNCED){
    if((geometry_mode_A==0)||(geometry_mode_B==0)) return -1;  
    if((m_errA)||(m_errB)||(m_common_err))return -1;
    m_stateA|=MOTOR_STATE_REF;
    m_stateB|=MOTOR_STATE_REF;
    m_save&=~3; //enable save when stopped
    return 0;
  }
  if((!m_errB)&&(_goB())){
    m_stateB|=MOTOR_STATE_REF;
    m_save&=~3; //enable save when stopped
    return 0;
  }
#endif
  return  -1;
}

int m_goA(int newpos)
{
  if(MOTOR_SYNCED)
    if((m_errA)||(m_errB)||(m_common_err)||
       (m_stateA&MOTOR_STATE_REF)||
       (m_stateB&MOTOR_STATE_REF)||
         ((geometry_mode_A==0)||(geometry_mode_B==0))){
           motor_destA = motor_posA;
           motor_destB = motor_posA;
           return -1;
         }
  

  
  if((!m_errA)&&(!(m_stateA&MOTOR_STATE_REF))&&(_goA())){
    if(newpos > max_range_A)newpos=(int)max_range_A;
    if(newpos < min_range_A)newpos=(int)min_range_A;
    motor_destA = newpos;
    if(MOTOR_SYNCED)motor_destB=newpos;
    m_save&=~3; //enable save when stopped
    return 0;
  }
  
  motor_destA = motor_posA;
  return  -1;
}


int m_goB(int newpos)
{
#ifndef DISABLE_MOTOR_B 
  if(MOTOR_SYNCED) return -1;
  if((!m_errB)&&(!(m_stateB&MOTOR_STATE_REF))&&(_goB())){
    if(newpos>max_range_B)newpos=(int)max_range_B;
    if(newpos<min_range_B)newpos=(int)min_range_B;
    motor_destB = newpos;
    m_save&=~3; //enable save when stopped
    return 0;
  }
  motor_destB = motor_posB;
#endif
  return  -1;
}


int m_go_manualA(int in){
  //if (bflags&(1<<time_enable)) return -1;
  
  if(MOTOR_SYNCED){
    if((m_errA)||(m_errB)||(m_common_err)||(geometry_mode_A==0)||(geometry_mode_B==0)) return -1;
  }
  
  if((!m_errA)&&(_goA())){
    m_stateB       &=~MOTOR_STATE_STOP_ALL_MASK;
    m_stateA        = MOTOR_STATE_MOVE_FREE;
    if(in)m_stateA |= MOTOR_STATE_MOVE_IN; 
    else  m_stateA |= MOTOR_STATE_MOVE_OUT;
  
    if(MOTOR_SYNCED)m_stateB=m_stateA;
    motor_manual_timeout=0;
    m_save&=~3; //enable save when stopped
    return 0;
  }
  return -1;
}


int m_go_manualB(int in){
  //if (bflags&(1<<time_enable)) return -1;  
#ifndef DISABLE_MOTOR_B
  if(MOTOR_SYNCED)
    if((m_errA)||(m_errB)||(m_common_err)||(geometry_mode_A==0)||(geometry_mode_B==0)) return -1;
  
  if((!m_errB)&&_goB()){
      m_stateA       &=~MOTOR_STATE_STOP_ALL_MASK;
      m_stateB        = MOTOR_STATE_MOVE_FREE;
      if(in)m_stateB |= MOTOR_STATE_MOVE_IN; 
      else  m_stateB |= MOTOR_STATE_MOVE_OUT; 
      
      if(MOTOR_SYNCED)m_stateA=m_stateB;
      motor_manual_timeout=0;
      m_save&=~3; //enable save when stopped
    return 0;
  }
#endif  
  return -1;
}


void m_manual_stop()
{
  if(m_stateA&MOTOR_STATE_MOVE_FREE){
    m_stateA&=~MOTOR_STATE_STOP_ALL_MASK;
    m_stop();
  }
  if(m_stateB&MOTOR_STATE_MOVE_FREE){
    m_stateB&=~MOTOR_STATE_STOP_ALL_MASK;
    m_stop();
  }
}


void m_windmode_moveA()
{
  m_windmode|=1;
}

void m_windmode_moveB()
{
  m_windmode|=2;
}


int m_A_idle(){
  //fsta return ((abs(motor_destA - motor_posA)<DIFF_LOW)&&(!motorA_moving()))?1:0;
  return ((abs(motor_destA - motor_posA) < diff_lowA) && (!motorA_moving()))?1:0;	
}

int m_B_idle(){
  //fsta return ((abs(motor_destB - motor_posB)<DIFF_LOW)&&(!motorB_moving()))?1:0;
  return ((abs(motor_destB - motor_posB) < diff_lowB) && (!motorB_moving()))?1:0;
}

int m_idle(){
  if(m_A_idle() && m_B_idle()) return 1;
  return 0;
}

int m_referencing(){
 if((m_stateA&MOTOR_STATE_REF)||(m_stateB&MOTOR_STATE_REF)) return 1;
 return 0;
}

int m_posA()
{
    return motor_posA;
}

int m_posB()
{
    return motor_posB;
}
int m_destA()
{
    return motor_destA;
}
int m_destB()
{
    return motor_destB;
}

unsigned int motor_stateA()
{
    return m_stateA;
}

unsigned int motor_stateB()
{
    return m_stateB;
}
unsigned int motor_stateComm()
{
    return m_common_err;
}

unsigned int motor_stateWarn()
{
 return m_common_warn;
}

unsigned int motor_errA()
{
    return m_errA;
}

unsigned int motor_errB()
{
    return m_errB;
}

unsigned int motor_exStat(){
  return m_ext_stat;
}

void motor_exStatClr(unsigned int mask){
  m_ext_stat&=~mask;
}

err_parm_t *motor_err_stateA(){
  return &err_parm_A;
}

err_parm_t *motor_err_stateB(){
  return &err_parm_B;
}
  
unsigned int motor_event()
{
  return event;
}


void motor_clear_status()
{
  //m_stop();
  motor_destA = motor_posA;
  motor_destB = motor_posB;
  m_errA      = 0;
  m_errB      = 0; 
  m_common_err= 0;
  
  motor_overcurrent_cntA  = 0;
  motor_hall_err_cntA     = 0;
  motor_cable_err_cntA    = 0;
  
  motor_overcurrent_cntB  = 0;
  motor_hall_err_cntB     = 0;
  motor_cable_err_cntB    = 0;
  m_common_warn           = 0;
}

int m_remainA(){
 return remain_A;
}

int m_remainB(){
 return  remain_B;
}




