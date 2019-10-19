//The following is a pretty basic C PID controller, but it has worked very well with good stability when tuned correctly.
// The code doesn't have alot of features and uses a independant gains equation, but you can dress it up or down as much as you want.
// I just pasted it from another application, adapt as needed.  Good Luck.


#include "pid.h"
#include "bldc.h"
#include <math.h>
     

/*------------------------------------------------------------------------
pid_init

DESCRIPTION   This function initializes the pointers in the _pid structure
              to the process variable and the setpoint.  *pv and *sp are
              integer pointers.
------------------------------------------------------------------------*/
void pid_init(pid *a, int* pv,int* sp)
{
    a->pv = pv;
    a->sp = sp;
}


/*------------------------------------------------------------------------
pid_tune

DESCRIPTION   Sets the proportional gain (p_gain), integral gain (i_gain),
              derivitive gain (d_gain), and the dead band (dead_band) of
              a pid control structure _pid.
------------------------------------------------------------------------*/
void pid_tune(pid *a,float p_gain,float i_gain,float d_gain,int dead_band)
{
    a->pgain = p_gain;
    a->igain = i_gain;
    a->dgain = d_gain;
    a->deadband = dead_band;
    a->integral=0.0;
    a->last_error=0;
}

/*------------------------------------------------------------------------
get_gains

DESCRIPTION   Returns the gains and dead band in a _pid control structure
              in the locations pointed to by the p_gain, i_gain, d_gain,
              and dead_band pointers.
              
ALSO SEE      pid_tune              
------------------------------------------------------------------------*/
void get_gains(pid *a,float *p_gain,float *i_gain,float *d_gain,int *dead_band)
{
    *p_gain = a->pgain;
    *i_gain = a->igain;
    *d_gain = a->dgain;
    *dead_band = a->deadband;
}

/*------------------------------------------------------------------------
pid_setinteg

DESCRIPTION   Set a new value for the integral term of the pid equation.
              This is useful for setting the initial output of the
              pid controller at start up.
------------------------------------------------------------------------*/
void pid_setinteg(pid *a,float new_integ)
{
    a->integral=new_integ;
    a->last_error=0;
}

/*------------------------------------------------------------------------
pid_bumpless

DESCRIPTION   Bumpless transfer algorithim.  When suddenly changing
              setpoints, or when restarting the PID equation after an
              extended pause, the derivative of the equation can cause 
              a bump in the controller output.  This function will help 
              smooth out that bump. The process value in *pv should
              be the updated just before this function is used.
------------------------------------------------------------------------*/
void pid_bumpless(pid *a)
{
    a->last_error = *(a->sp) - *(a->pv);
}
    
/*------------------------------------------------------------------------
pid_calc

DESCRIPTION   Performs PID calculations for the _pid structure *a.  This
              function uses the positional form of the pid equation, and
              incorporates an integral windup prevention algorithim.
              Rectangular integration is used, so this function must be
              repeated on a consistent time basis for accurate control.

RETURN VALUE  The new output value for the pid loop.

USAGE         #include "control.h"
              main() {
                   struct _pid PID;
                   int process_variable, set_point;
                   pid_init(&PID, &process_variable, &set_point);
                   pid_tune(&PID, 4.3, 0.2, 0.1, 2);
                   set_point = 500;
                   pid_setinteg(&PID,30.0);
                   process_varialbe = read_temp();
                   pid_bumpless(&PID);
                   for(;;) {
                        process_variable = read_temp();
                        output( pid_calc(&PID) );
                        wait(1.0);
                   }
              }
------------------------------------------------------------------------*/
/*float pid_calc(pid *a)
{
    int err;
    float pterm, dterm, result, ferror;

    err = *(a->sp) - *(a->pv);
    if (fabsf(err) > a->deadband) {
         ferror = (float) err;    //do integer to float conversion only once
         pterm = a->pgain * ferror;
         if (pterm >= 100 || pterm <= -100) 
              a->integral = 0.0;
         else {
              a->integral += a->igain * ferror;
              if (a->integral > 100) a->integral=100;
              else if (a->integral < -100) a->integral=-100;
              //else if (a->integral < 0.0) a->integral=0.0;
         }
         dterm = (err - a->last_error) * a->dgain;
         result = pterm + a->integral + dterm;
    }else result = 0;
    a->last_error = err;


    return (result>100)?100:(result<-100)?-100:result;
}
*/
int result_old;
int bounceBack = 0;
int bounce_stop = 0;
extern bldc_motor *bldc_cm;
float pid_calc(pid *a)
{
    int err;
    float pterm, dterm, result, ferror;

    err = *(a->sp) - *(a->pv);

    if(err > 100 || err < -100){    //When motors spin more, reset bounce preventing counters
      bounceBack = 0;
      bounce_stop = 0;
    }

    if (fabsf(err) > a->deadband + bounce_stop) {
       ferror = (float) err;    /*do integer to float conversion only once*/
       pterm = a->pgain * ferror;
       if (pterm >= 100 || pterm <= -100) 
            a->integral = 0.0;
       else {
            a->integral += a->igain * ferror;
            if (a->integral > 100) a->integral=100;
            else if (a->integral < -100) a->integral=-100;
            //else if (a->integral < 0.0) a->integral=0.0;
       }
       dterm = (*(a->pv) - a->last_position) * a->dgain;
       result = pterm + a->integral + dterm;
    }else{ 
      result = 0;
      a->integral = 0;
    }
    a->last_error = err;

    a->last_position = *(a->pv);

    if(result > 0 && result < 2)
      result = 2;
    if(result < 0 && result > -2)
      result = -2;


    if(((result + result_old) == result) && (result != 0)){ //check if motor direction changed
      bounceBack++;
      a->integral += a->igain * ferror * 50;
    }
    if(bounceBack > 2){
      bounce_stop++;
      bounceBack = 0;
    }
    if(bounce_stop>2)
      bounce_stop = 0;
    result_old = result;

    return (result>100)?100:(result<-100)?-100:result;
}

void  ramp_init(ramp_t *ramp, int *pos, int *dest, int *speed, int *speedTarget, float *pwm){
  ramp->speed= speed;
  ramp->pos  = pos;
  ramp->dest = dest;
  ramp->pwm  = pwm;
  ramp->speed_target=speedTarget;

  ramp->state=0;
}


float pid_ramp(ramp_t *ramp, pid *a){
  int err = *(ramp->dest) - *(ramp->pos);
  
  int rampval = pid_calc(a) * 40;
  
  if(err>0){
    if(rampval >= *ramp->pwm){
        if(*ramp->pwm < 0xfff)   *ramp->pwm += 3;
      }else  {
         *ramp->pwm = rampval;
      }
  }else{
      
     if(rampval <=*ramp->pwm){ 
      if(*ramp->pwm > -0xfff)*ramp->pwm -= 3;
      }else  {
         *ramp->pwm = rampval;
      }
  }
}


float pid_calc_inc(pid *a, float max)
{
    int err;
    float pterm, dterm, result, ferror;
    
    if( *a->pv == 0)err = *(a->sp);
    else            err = *(a->sp) -  *a->pv;
    
    ferror = (float) err;    
    pterm = a->pgain * ferror;
      
    a->integral += a->igain * ferror;
    if (a->integral > 1000.0) a->integral=1000.0;
    else if (a->integral < -1000) a->integral=-1000;
    
    result = pterm + a->integral;
    
   if(result>=max) return  max;
   if(result<=(-max)) return (-max);
   return result;
   // return (result>=max)?max:(result<=(-max))?(-max):0;
}