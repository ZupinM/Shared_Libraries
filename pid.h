#ifndef __PID__
#define __PID__


typedef struct  _pid {
    int *pv;  	/*pointer to an integer that contains the process value*/
    int *sp;  	/*pointer to an integer that contains the set point*/ 
   
    
    float integral;
    float pgain;
    float igain;
    float dgain;
    int deadband;
    int last_error;
    int last_position;
}pid;

typedef struct  {
  int   *pos;  	
  int   *dest;  	
  int   *speed;
  int   *speed_target;
  float   *pwm;
  
  int   steps;
  
  int   state; 
  float ramp;
}ramp_t;





void pid_init(pid *a, int* pv,int* sp);
void pid_tune(pid *a,float p_gain,float i_gain,float d_gain,int dead_band);
void get_gains(pid *a,float *p_gain,float *i_gain,float *d_gain,int *dead_band);
void pid_setinteg(pid *a,float new_integ);
void pid_bumpless(pid *a);
float pid_calc(pid *a);
float pid_calc_inc(pid *a, float max);

void  ramp_init(ramp_t *ramp, int *pos, int *dest, int *speed, int *speedTarget, float *pwm);
float pid_ramp(ramp_t *ramp, pid *a);


#endif