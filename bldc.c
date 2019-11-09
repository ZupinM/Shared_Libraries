#include "LPC15xx.h"

#define BLDC_PA_POS (1<<5)
#define BLDC_PA_NEG (1<<1)
#define BLDC_PB_POS (1<<6)
#define BLDC_PB_NEG (1<<2)
#define BLDC_PC_POS (1<<7)
#define BLDC_PC_NEG (1<<3)

#define BLDC_CW_S1 1
#define BLDC_CW_S2 5
#define BLDC_CW_S3 4
#define BLDC_CW_S4 6
#define BLDC_CW_S5 2
#define BLDC_CW_S6 3

#define BLDC_CCW_S1 1
#define BLDC_CCW_S2 3
#define BLDC_CCW_S3 2
#define BLDC_CCW_S4 6
#define BLDC_CCW_S5 4
#define BLDC_CCW_S6 5


#define BLDC_CW_P1  (BLDC_PC_NEG | BLDC_PB_POS)
#define BLDC_CW_P2  (BLDC_PC_NEG | BLDC_PA_POS)
#define BLDC_CW_P3  (BLDC_PB_NEG | BLDC_PA_POS)
#define BLDC_CW_P4  (BLDC_PB_NEG | BLDC_PC_POS)
#define BLDC_CW_P5  (BLDC_PA_NEG | BLDC_PC_POS)
#define BLDC_CW_P6  (BLDC_PA_NEG | BLDC_PB_POS)


#define BLDC_CCW_P1 (BLDC_PA_NEG | BLDC_PB_POS)
#define BLDC_CCW_P2 (BLDC_PA_NEG | BLDC_PC_POS)
#define BLDC_CCW_P3 (BLDC_PB_NEG | BLDC_PC_POS)
#define BLDC_CCW_P4 (BLDC_PB_NEG | BLDC_PA_POS)
#define BLDC_CCW_P5 (BLDC_PC_NEG | BLDC_PA_POS)
#define BLDC_CCW_P6 (BLDC_PC_NEG | BLDC_PB_POS)

//Next CW number
const unsigned char bldc_cw_next[7][3]={
                          {0,0,0},
                          {BLDC_CW_S2, BLDC_CW_P2, BLDC_CW_S3},
                          {BLDC_CW_S6, BLDC_CW_P6, BLDC_CW_S1},
                          {BLDC_CW_S1, BLDC_CW_P1, BLDC_CW_S2},
                          {BLDC_CW_S4, BLDC_CW_P4, BLDC_CW_S5},
                          {BLDC_CW_S3, BLDC_CW_P3, BLDC_CW_S4},
                          {BLDC_CW_S5, BLDC_CW_P5, BLDC_CW_S6}};

 

const unsigned char bldc_ccw_next[7][3]={
                          {0,0,0},
                          {BLDC_CCW_S2, BLDC_CCW_P2, BLDC_CCW_S3},
                          {BLDC_CCW_S4, BLDC_CCW_P4, BLDC_CCW_S5},
                          {BLDC_CCW_S3, BLDC_CCW_P3, BLDC_CCW_S4},
                          {BLDC_CCW_S6, BLDC_CCW_P6, BLDC_CCW_S1},
                          {BLDC_CCW_S1, BLDC_CCW_P1, BLDC_CCW_S2},
                          {BLDC_CCW_S5, BLDC_CCW_P5, BLDC_CCW_S6}};

//const 

#include "bldc.h"
#include "lpc15xx.h"
#include "pid.h"
#include "main.h"
#include "eeprom.h"

#include "focus.h"

#include "adc_15xx.h"

#include "../gpio.h"

#include <math.h>
#include <stdlib.h>

#include "suntracer.h"

LPC_ADC0_Type *LPC_ADC[2] = {(LPC_ADC0_Type           *) LPC_ADC0_BASE , (LPC_ADC0_Type           *) LPC_ADC1_BASE} ;

#define BLDC_CTRL_IDLE         0
#define BLDC_CTRL_TRACKING     (1<<1)
#define BLDC_CTRL_HOMING       (1<<2)





 #define BLDC_ADC_CONTROL     ( 11 - 1 )
 #define BLDC_ADC_U_MEASURE   (0x01) 
 #define BLDC_ADC_I_MEASURE   (0x02) 
 #define BLDC_ADC_START       (1<<24)




float        bldc_pwm;
int          bldc_runtime;
int          bldc_idletime;
int          bldc_status;
int          bldc_pause;


unsigned int  bldc_Voltage;
float         bldc_Uavg;
unsigned int  bldc_Current;
float         bldc_Iavg;
float         zeroCurrent_voltage_0;
float         zeroCurrent_voltage_1;

float UVccHALL_0, UVccHALL_1;
float UVccHALL_0_avg, UVccHALL_1_avg;
float battery_voltage = 3;
float temperature;


uint64_t                        bldc_Speed;     //RPM
volatile unsigned int           number_of_poles = 2;
volatile unsigned int           speed_sample_counter;
volatile unsigned int           bldc_Speed_raw; //ticks from comutation to comutation

extern unsigned int tracker_status;

extern unsigned int store_in_flash;
extern unsigned char phase_active;
extern int mosfet_protection_cnt;

extern uint32_t adc3_VAL;
extern uint32_t adc4_VAL;
uint32_t adc3_SUM = 0;
uint32_t adc4_SUM = 0;
uint32_t adc_CNT = 0;

extern uint32_t cflags;

extern uint32_t SystemCoreClock;

unsigned int direction_delay = 0;
unsigned int moving_counter_a = 0;
unsigned int moving_counter_b = 0;
unsigned char any_motor_moving = 0;


unsigned char ES_0_normallyOpenLo = 0;
unsigned char ES_0_normallyOpenHi = 0;
unsigned char ES_1_normallyOpenLo = 0;
unsigned char ES_1_normallyOpenHi = 0;

unsigned char hall_fault = 0;
unsigned char hall_detect = 0;
unsigned char motor_a_disconnected = 0;
unsigned char motor_b_disconnected = 0;
unsigned char commutation_counter = 0;
extern int bounce_stop;


float MOTOR_START_VOLTAGE;
float UNDERVOLTAGE_LEVEL;
float MOTOR_CUTOF_LEVEL; 

extern float mzp_current;   



bldc_misc  bldc_cfg;
bldc_motor bldc_motors[BLDC_MOTOR_COUNT];            //motors
bldc_motor *bldc_cm = &bldc_motors[0];



#define ADC_CONVERT_TICS 120

void ActivateDrivers(int dir);
void Flag_check();
void bldc_Comutate(unsigned char motor);

unsigned char ButtonStates() {
  unsigned char val = 0;

  if(~LPC_GPIO_PORT->PIN[BUTTON1_PORT] & (1 << BUTTON1_PIN))
    val |= 1<<0;
  if(~LPC_GPIO_PORT->PIN[BUTTON2_PORT] & (1 << BUTTON2_PIN))
    val |= 1<<1;
  #ifdef BUTTON3_PORT
  if(~LPC_GPIO_PORT->PIN[BUTTON3_PORT] & (1 << BUTTON3_PIN))
    val |= 1<<2;
  #endif
  return (val & 0x07);
}

int bldc_ReadHall(unsigned char motor){

  if (motor == 1) {
  #ifdef  HALL_B_1_PORT
    int res = 0;     
      if(LPC_GPIO_PORT->PIN[HALL_B_1_PORT] & (1<<HALL_B_1_PIN))
        res |= 1<<0;
      if(LPC_GPIO_PORT->PIN[HALL_B_2_PORT] & (1<<HALL_B_2_PIN))
        res |= 1<<1;
      if(LPC_GPIO_PORT->PIN[HALL_B_3_PORT] & (1<<HALL_B_3_PIN))
        res |= 1<<2;
    return  res;
  #endif
  }
  else if ( motor == 0){
    int res = 0;     
      if(LPC_GPIO_PORT->PIN[HALL_A_1_PORT] & (1<<HALL_A_1_PIN))
        res |= 1<<0;
      if(LPC_GPIO_PORT->PIN[HALL_A_2_PORT] & (1<<HALL_A_2_PIN))
        res |= 1<<1;
      if(LPC_GPIO_PORT->PIN[HALL_A_3_PORT] & (1<<HALL_A_3_PIN))
        res |= 1<<2;
    return  res;
  }
}


void bldc_init_motors(int LoadDefaults)
{
  if(LoadDefaults) {
    for(int i = 0; i < BLDC_MOTOR_COUNT; i++) {
      //init parameters
      bldc_motors[i].state           = 0;
      bldc_motors[i].index           = i;
      bldc_motors[i].ctrl            = 0;
      bldc_motors[i].position        = 0;
      bldc_motors[i].target          = 0;
      bldc_motors[i].status          = 0;
      bldc_motors[i].target          = 0;
      bldc_motors[i].home_remaining  = 0;

      bldc_motors[i].gear_ratio      = 800;
      bldc_motors[i].I_limit         = 2;
      bldc_motors[i].I_Inrush_ratio  = 3; 
      bldc_motors[i].I_Inrush_time   = 500;
      bldc_motors[i].home_offset     = 0;
      bldc_motors[i].min_position    = 0;
      bldc_motors[i].max_position    = 800;
      bldc_motors[i].modbus_timeout_position = 400;
      bldc_motors[i].Idetection      = 0.07;
      bldc_motors[i].end_switchDetect = 10;

      //init hall state
      bldc_motors[i].hall_state = bldc_ReadHall((unsigned char)i);

      //init PID
      pid_init(&bldc_motors[i].pid, &bldc_motors[i].position, &bldc_motors[i].target);
      pid_tune(&bldc_motors[i].pid, 0.1, 0.00001 , 0, BLDC_DEAD_BAND);
      //pid_setinteg(&bldc_motors[i].pid, 0.0);
      pid_bumpless(&bldc_motors[i].pid);
    }
  } else {
    for(int i=0 ;i < BLDC_MOTOR_COUNT; i++) {  
      bldc_motors[i].index           = i;
      bldc_motors[i].ctrl            = BLDC_CTRL_IDLE;
      bldc_motors[i].status          &= 0xf0 | BLDC_STATUS_ENDSWITCH_ERROR | BLDC_STATUS_ERR_MOVEOUT; 

      //range sanity check
      if(!isnormal(bldc_motors[i].max_position))
        bldc_motors[i].max_position = 0;
      if(!isnormal(bldc_motors[i].min_position))
        bldc_motors[i].min_position = 0;
      if(bldc_motors[i].max_position > 5000)
        bldc_motors[i].max_position = 0;
      if(bldc_motors[i].min_position < DEFAULT_MIN_RANGE)
        bldc_motors[i].min_position = 0;
      if(bldc_motors[i].min_position > bldc_motors[i].max_position) {
        bldc_motors[i].min_position = bldc_motors[i].max_position = 0;
      }

      //current sanity check
      if(!isnormal(bldc_motors[i].I_limit))
        bldc_motors[i].I_limit = 0;
      if(bldc_motors[i].I_limit<0 || bldc_motors[i].I_limit>10)
        bldc_motors[i].I_limit = 0;

      //home offset sanity check
      if(!isnormal(bldc_motors[i].home_offset))
        bldc_motors[i].home_offset = 0;
      if(bldc_motors[i].home_offset > bldc_motors[i].max_position)
        bldc_motors[i].home_offset = 0;
      if(bldc_motors[i].home_offset < bldc_motors[i].min_position)
        bldc_motors[i].home_offset = 0;

      //end switch sanity check
      if(!isnormal(bldc_motors[i].end_switchDetect))
        bldc_motors[i].end_switchDetect = 10;
      if(bldc_motors[i].end_switchDetect > bldc_motors[i].max_position)
        bldc_motors[i].end_switchDetect = 10;
      if(bldc_motors[i].end_switchDetect < 0)
        bldc_motors[i].end_switchDetect = 10;

      if(!isnormal(bldc_motors[i].Idetection))
        bldc_motors[i].Idetection = 0.07;
      if(!isnormal(bldc_motors[i].I_Inrush_ratio))
        bldc_motors[i].I_Inrush_ratio = 3;
      if(!isnormal(bldc_motors[i].I_Inrush_time))
        bldc_motors[i].I_Inrush_ratio = 500;

      //pid sanity check
      if(!isnormal(bldc_motors[i].pid.pgain))
        bldc_motors[i].pid.pgain = 0.1;
      if(!isnormal(bldc_motors[i].pid.igain))
        bldc_motors[i].pid.igain = 1/100000;
      if(!isnormal(bldc_motors[i].pid.dgain))
        bldc_motors[i].pid.dgain = 0;
      if(bldc_motors[i].pid.deadband>100)
        bldc_motors[i].pid.deadband = BLDC_DEAD_BAND;
      
      bldc_motors[i].target = bldc_motors[i].position;

      pid_init(&bldc_motors[i].pid, &bldc_motors[i].position, &bldc_motors[i].target);
      pid_setinteg(&bldc_motors[i].pid, 0.0);
      pid_bumpless(&bldc_motors[i].pid);
    }

    bldc_status &= BLDC_LOCKED | BLDC_MOTOR_CUTOFF;
  }
}

void bldc_init_cfg(int LoadDefaults) {
  if (!LoadDefaults)
    return;
  bldc_cfg.IConvertRatio = 28;
  bldc_cfg.UConvertRatio = 28;
  bldc_cfg.BConvertRatio = 28;
  bldc_cfg.TConvertRatio = 28;
  bldc_cfg.HConvertRatio = 28;
  bldc_cfg.H1ConvertRatio = 28;
  bldc_cfg.homing_timeout = 600;
}


/* Start ADC calibration */
void Chip_ADC_StartCalibration(LPC_ADC0_Type *pADC)
{
	/* Set calibration mode */
	pADC->CTRL |= ADC_CR_CALMODEBIT;

	/* Clear ASYNC bit */
	pADC->CTRL &= ~ADC_CR_ASYNMODE;

	/* Setup ADC for about 500KHz (per UM) */
	Chip_ADC_SetClockRate(pADC, 500000);

	/* Clearn low power bit */
	pADC->CTRL &= ~ADC_CR_LPWRMODEBIT;

	/* Calibration is only complete when ADC_CR_CALMODEBIT bit has cleared */
}

void Chip_ADC_SetClockRate(LPC_ADC0_Type *pADC, uint32_t rate)
{
	uint32_t div;

	/* Get ADC clock source to determine base ADC rate. IN sychronous mode,
	   the ADC base clock comes from the system clock. In ASYNC mode, it
	   comes from the ASYNC ADC clock and this function doesn't work. */
	div = SystemCoreClock / rate;
	if (div == 0) {
		div = 1;
	}

	Chip_ADC_SetDivider(pADC, (uint8_t) div - 1);
}


void bldc_adcinit() {

  //Battery measurment
  LPC_GPIO_PORT->CLR[BAT_ADC_PORT] |= 1<<BAT_ADC_PIN;   //UBAT ADC set as input   source = input == pulled up to 3,3V
  
  LPC_GPIO_PORT->SET[BAT_GATE_PORT] |= 1<<BAT_GATE_PIN; //Disable measurment mosfet //out=3,3V Vgs = 0V -> mosfet off

  LPC_GPIO_PORT->DIR[BAT_GATE_PORT] |= 1<<BAT_GATE_PIN; //Disable measurment mosfet 



  LPC_IOCON->PIO[I0_ADC_PORT][I0_ADC_PIN] &= ~(3<<3);   //disable I adc pullups
  #ifdef I1_ADC_PORT
  LPC_IOCON->PIO[I1_ADC_PORT][I1_ADC_PIN] &= ~(3<<3);
  #endif

  LPC_SWM->PINENABLE[0] = ~(0);
  LPC_SWM->PINENABLE[0] &= ~(1<<BAT_ADC_CHANNEL+BAT_ADC_GROUP*12);
  LPC_SWM->PINENABLE[0] &= ~(1<<U_ADC_CHANNEL+U_ADC_GROUP*12);
  LPC_SWM->PINENABLE[0] &= ~(1<<I0_ADC_CHANNEL+I0_ADC_GROUP*12);
#ifdef HALL_U_0_PORT
  LPC_SWM->PINENABLE[0] &= ~(1<<HALL_U_0_CHANNEL+HALL_U_0_GROUP*12);
  LPC_SWM->PINENABLE[0] &= ~(1<<HALL_U_1_CHANNEL+HALL_U_1_GROUP*12);
  LPC_SWM->PINENABLE[0] &= ~(1<<I1_ADC_CHANNEL+I1_ADC_GROUP*12);
#endif

//  ADC0 - Motor A
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 27);    // enable ADC0 clk register
  LPC_SYSCON->PRESETCTRL0 &= ~(1 << 27);  // clear ADC0 reset


  LPC_SYSCON->PDRUNCFG &= ~(1 << 10) ;  // ADC0_PD powered

  Chip_ADC_StartCalibration(LPC_ADC0);
  while (!(Chip_ADC_IsCalibrationDone(LPC_ADC0))) {}

  LPC_ADC0->INTEN = 0;  // disable ADC0 interrupts
  LPC_ADC0->CTRL = 0;   // disable all flags
  LPC_ADC0->CTRL |= 2; // clock division 2

  LPC_ADC0->SEQA_CTRL &= (1 < 31);  // Disable sequence A


  //  ADC1 - Motor B


  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << 28);    // enable ADC1 clk register
  LPC_SYSCON->PRESETCTRL0 &= ~(1 << 28);  // clear ADC1 reset
  // LPC_INMUX->DMA_ITRIG_INMUX[0] = 0;  // ADC0_SEQA_IRQ
  LPC_SYSCON->PDRUNCFG &= ~(1 << 11);  // ADC1_PD powered
  // callibration

  Chip_ADC_StartCalibration(LPC_ADC1);
  while (!(Chip_ADC_IsCalibrationDone(LPC_ADC1))) {}

  //  LPC_ADC1->CTRL = (144<<0) | (1<30);
  //  while(LPC_ADC1->CTRL & (1<<30));

  //LPC_ADC0->INTEN = 0;  // disable ADC0 interrupts
  LPC_ADC1->CTRL = 0;   // disable all flags
  LPC_ADC1->CTRL |= 2; // clock division 2

  LPC_ADC1->SEQA_CTRL &= (1 < 31);  // Disable sequence Aad

  LPC_ADC[BAT_ADC_GROUP]->SEQA_CTRL |= (1<<BAT_ADC_CHANNEL); // Enable sampling of channels
  LPC_ADC[U_ADC_GROUP]->SEQA_CTRL |= (1<<U_ADC_CHANNEL);
  LPC_ADC[I0_ADC_GROUP]->SEQA_CTRL |= (1<<I0_ADC_CHANNEL);
#ifdef HALL_U_0_CHANNEL
  LPC_ADC[HALL_U_0_GROUP]->SEQA_CTRL |= (1<<HALL_U_0_CHANNEL);
  LPC_ADC[HALL_U_1_GROUP]->SEQA_CTRL |= (1<<HALL_U_1_CHANNEL);
  LPC_ADC[I1_ADC_GROUP]->SEQA_CTRL |= (1<<I1_ADC_CHANNEL);
#endif

  LPC_ADC0->SEQA_CTRL |= (1<<18) | (1<<31) | (1<<27); //Enable sequence A, TRIGPOL- rising edge

  LPC_ADC1->SEQA_CTRL |= (1<<18) | (1<<31) | (1<<27); //Enable sequence A, TRIGPOL- rising edge, burst mode

  for(int i=0 ; i<50 ; i++);

  zeroCurrent_voltage_0 =  ((LPC_ADC0->DAT[5]>>4) & 0xfff) >> 2;
  zeroCurrent_voltage_1 =  ((LPC_ADC1->DAT[2]>>4) & 0xfff) >> 2;

}


void bldc_init(int LoadDefaults){
  bldc_init_motors(LoadDefaults);
  bldc_init_cfg(LoadDefaults);

  bldc_pwm = 0;

//****initialize ports*******

  //-------------Clear all output drivers----------------

  LPC_GPIO_PORT->CLR[MOTOR_A_HI1_PORT] |= 1<<MOTOR_A_HI1_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_A_HI2_PORT] |= 1<<MOTOR_A_HI2_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_A_HI3_PORT] |= 1<<MOTOR_A_HI3_PIN;

  LPC_GPIO_PORT->CLR[MOTOR_A_LO1_PORT] |= 1<<MOTOR_A_LO1_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_A_LO2_PORT] |= 1<<MOTOR_A_LO2_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_A_LO3_PORT] |= 1<<MOTOR_A_LO3_PIN;
#ifdef MOTOR_B_HI1_PORT
  LPC_GPIO_PORT->CLR[MOTOR_B_HI1_PORT] |= 1<<MOTOR_B_HI1_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_B_HI2_PORT] |= 1<<MOTOR_B_HI2_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_B_HI3_PORT] |= 1<<MOTOR_B_HI3_PIN;

  LPC_GPIO_PORT->CLR[MOTOR_B_LO1_PORT] |= 1<<MOTOR_B_LO1_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_B_LO2_PORT] |= 1<<MOTOR_B_LO2_PIN;
  LPC_GPIO_PORT->CLR[MOTOR_B_LO3_PORT] |= 1<<MOTOR_B_LO3_PIN;
  //-----------------------------------------------------
#endif

  //-----------Set direction of output drivers-----------
  LPC_GPIO_PORT->DIR[MOTOR_A_HI1_PORT] |= 1<<MOTOR_A_HI1_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_A_HI2_PORT] |= 1<<MOTOR_A_HI2_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_A_HI3_PORT] |= 1<<MOTOR_A_HI3_PIN;
                 
  LPC_GPIO_PORT->DIR[MOTOR_A_LO1_PORT] |= 1<<MOTOR_A_LO1_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_A_LO2_PORT] |= 1<<MOTOR_A_LO2_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_A_LO3_PORT] |= 1<<MOTOR_A_LO3_PIN;
#ifdef MOTOR_B_HI1_PORT                 
  LPC_GPIO_PORT->DIR[MOTOR_B_HI1_PORT] |= 1<<MOTOR_B_HI1_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_B_HI2_PORT] |= 1<<MOTOR_B_HI2_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_B_HI3_PORT] |= 1<<MOTOR_B_HI3_PIN;
                 
  LPC_GPIO_PORT->DIR[MOTOR_B_LO1_PORT] |= 1<<MOTOR_B_LO1_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_B_LO2_PORT] |= 1<<MOTOR_B_LO2_PIN;
  LPC_GPIO_PORT->DIR[MOTOR_B_LO3_PORT] |= 1<<MOTOR_B_LO3_PIN;
#endif

  //-----------Set direction of Hall inputs------------
  LPC_GPIO_PORT->DIR[HALL_A_1_PORT] &= ~(1<<HALL_A_1_PIN);
  LPC_GPIO_PORT->DIR[HALL_A_2_PORT] &= ~(1<<HALL_A_2_PIN);
  LPC_GPIO_PORT->DIR[HALL_A_3_PORT] &= ~(1<<HALL_A_3_PIN);
#ifdef MOTOR_B_HI1_PORT
  LPC_GPIO_PORT->DIR[HALL_B_1_PORT] &= ~(1<<HALL_B_1_PIN);
  LPC_GPIO_PORT->DIR[HALL_B_2_PORT] &= ~(1<<HALL_B_2_PIN);
  LPC_GPIO_PORT->DIR[HALL_B_3_PORT] &= ~(1<<HALL_B_3_PIN);
  //---------------------------------------------------
#endif

  // enable Hysteresis on hall A, 666ns glitch filter == 3/64*clk
  LPC_IOCON->PIO[HALL_A_1_PORT][HALL_A_1_PIN] |= (1<<5) | (3<<11) | (4<<13);
  LPC_IOCON->PIO[HALL_A_2_PORT][HALL_A_2_PIN] |= (1<<5) | (3<<11) | (4<<13);
  LPC_IOCON->PIO[HALL_A_3_PORT][HALL_A_3_PIN] |= (1<<5) | (3<<11) | (4<<13);
  //Diasble pullups
  LPC_IOCON->PIO[HALL_A_1_PORT][HALL_A_1_PIN] &= ~(3<<3);
  LPC_IOCON->PIO[HALL_A_2_PORT][HALL_A_2_PIN] &= ~(3<<3);
  LPC_IOCON->PIO[HALL_A_3_PORT][HALL_A_3_PIN] &= ~(3<<3);

#ifdef MOTOR_B_HI1_PORT
  // enable Hysteresis on hall B, 666ns glitch filter == 3/64*clk
  LPC_IOCON->PIO[HALL_B_1_PORT][HALL_B_1_PIN] |= (1<<5) | (3<<11) | (4<<13);
  LPC_IOCON->PIO[HALL_B_2_PORT][HALL_B_2_PIN] |= (1<<5) | (3<<11) | (4<<13);
  LPC_IOCON->PIO[HALL_B_3_PORT][HALL_B_3_PIN] |= (1<<5) | (3<<11) | (4<<13);

  LPC_IOCON->PIO[HALL_B_1_PORT][HALL_B_1_PIN] &= ~(3<<3);
  LPC_IOCON->PIO[HALL_B_2_PORT][HALL_B_2_PIN] &= ~(3<<3);
  LPC_IOCON->PIO[HALL_B_3_PORT][HALL_B_3_PIN] &= ~(3<<3);
#endif

  LPC_GPIO_PORT->DIR[CHARGE_PUMP_PORT] |= 1<<CHARGE_PUMP_PIN; //charge pump

  LPC_SYSCON->IOCONCLKDIV |= 1;
 

  LPC_INMUX->PINTSEL[2] = HALL_A_1_PIN + 32*HALL_A_1_PORT;
  LPC_INMUX->PINTSEL[3] = HALL_A_2_PIN + 32*HALL_A_2_PORT;
  LPC_INMUX->PINTSEL[4] = HALL_A_3_PIN + 32*HALL_A_3_PORT;
#ifdef MOTOR_B_HI1_PORT
  LPC_INMUX->PINTSEL[5] = HALL_B_1_PIN + 32*HALL_B_1_PORT;
  LPC_INMUX->PINTSEL[6] = HALL_B_2_PIN + 32*HALL_B_2_PORT;
  LPC_INMUX->PINTSEL[7] = HALL_B_3_PIN + 32*HALL_B_3_PORT;
#endif

  LPC_PINT->IENR |= (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable rising edge interrupts
  LPC_PINT->IENF |= (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable falling edge interrupts

  NVIC_EnableIRQ(PIN_INT2_IRQn);
  NVIC_EnableIRQ(PIN_INT3_IRQn);
  NVIC_EnableIRQ(PIN_INT4_IRQn);
#ifdef MOTOR_B_HI1_PORT
  NVIC_EnableIRQ(PIN_INT5_IRQn);
  NVIC_EnableIRQ(PIN_INT6_IRQn);
  NVIC_EnableIRQ(PIN_INT7_IRQn);
#endif

  //initialize ADC
  bldc_adcinit();

//*****initialize timers********

  // motor pwm timer
  LPC_SYSCON->SYSAHBCLKCTRL1 |= (1<<2); // enable SCT0
  LPC_SYSCON->PRESETCTRL1 &= ~(1<<2);  // clear SCT0 reset
  LPC_SCT0->CONFIG = (1 << 0) | (1 << 17); // unified 32-bit timer, auto limit
  LPC_SCT0->CTRL |= (1 << 2); // halt by setting bit 2 of the CTRL
  LPC_SCT0->CTRL |= (5<<5); //Clock prescaler
  LPC_SCT0->MATCHREL0 = 0xfff; //SystemCoreClock/100; // match 0 @ 100 Hz = 10 msec
  LPC_SCT0->MATCHREL2 = 0x000;



  LPC_SCT0->OUT0_CLR = (1 << 2); // event 2 will clear SCT0_OUT0
  LPC_SCT0->OUT1_CLR = (1 << 2); // event 2 will clear SCT0_OUT1
  LPC_SCT0->OUT2_CLR = (1 << 2); // event 2 will clear SCT0_OUT2

  LPC_SCT0->OUT0_SET = (1 << 0);  // MB H1  Event0 sets outputs
  LPC_SCT0->OUT1_SET = (1 << 0);  // MB H2
  LPC_SCT0->OUT2_SET = (1 << 0);  // MB H3

  LPC_SCT0->EV0_STATE = 0xFFFFFFFF; // event 0 happens in all states
  LPC_SCT0->EV2_STATE = 0xFFFFFFFF; // event 2 happens in all states
  LPC_SCT0->EV0_CTRL = (1 << 12); // match 0 condition only
  LPC_SCT0->EV2_CTRL = (2<<0) | (1 << 12); //match register2,  match 1 condition only

  // Speed measuring timer
  LPC_SYSCON->SYSAHBCLKCTRL1 |= (1<<3); // enable SCT1
  LPC_SCT1->CONFIG = (1 << 0) | (1 << 17); // unified 32-bit timer, auto limit
  LPC_SCT1->MATCHREL0 = 7200000*1;

  LPC_SCT1->EV0_STATE = 0xFFFFFFFF; // event 0 happens in all states
  LPC_SCT1->EV0_CTRL = (1 << 12); // match 0 condition only
  LPC_SCT1->EVEN |= (1 << 0); // event 0 - disabled interrupt  

  // NVIC_EnableIRQ(SCT1_IRQn);
  LPC_SCT1->CTRL &= ~(1 << 2); // unhalt by clearing bit 2 of the CTRL

  //charge pump timer
  LPC_SYSCON->SYSAHBCLKCTRL1 |= (1<<4); // enable SCT2
  LPC_SYSCON->PRESETCTRL1 &= ~(1<<4);  // clear SCT2 reset
  LPC_SCT2->CONFIG = (1 << 0) | (1 << 17); // unified 32-bit timer, auto limit
  LPC_SCT2->CTRL |= (1 << 2); // halt by setting bit 2 of the CTRL
  LPC_SCT2->MATCHREL0 = 8000; //4800; //10khz 50% duty
  LPC_SCT2->RES = 0x03;          //simultaneous set/clear toggles output
  LPC_SCT2->OUT0_CLR = (1 << 0); // event 0 will clear SCT2_OUT0
  LPC_SCT2->OUT0_SET = (1 << 0); // event 0 will set SCT2_OUT0
  LPC_SCT2->EV0_STATE = 0xffffffff;    //state mask for all states
  LPC_SCT2->EV0_CTRL = (1 << 12); // match 0 condition only
  LPC_SWM->PINASSIGN[8] &= ~(0xFF << 24);
  LPC_SWM->PINASSIGN[8] |= ((CHARGE_PUMP_PIN+32*CHARGE_PUMP_PORT) << 24);        // PIO1_25 - CHARGE PUMP

  LPC_SCT2->CTRL &= ~(1 << 2); // unhalt SCT2 by clearing bit 2 of the CTRL
  LPC_SCT0->CTRL &= ~(1 << 2); // unhalt SCT0 by clearing bit 2 of the CTRL


  for(int i = 0; i < 4000000; i++);//wait for voltage samples

  //setup voltages
  if(bldc_U(SUPPLY) >= 16) {//24V system
    MOTOR_START_VOLTAGE = MOTOR_START_VOLTAGE24;
    UNDERVOLTAGE_LEVEL  = UNDERVOLTAGE_LEVEL24;
    MOTOR_CUTOF_LEVEL   = MOTOR_CUTOF_LEVEL24;
  } else {          //12V system
    MOTOR_START_VOLTAGE= MOTOR_START_VOLTAGE12;
    UNDERVOLTAGE_LEVEL = UNDERVOLTAGE_LEVEL12;
    MOTOR_CUTOF_LEVEL  = MOTOR_CUTOF_LEVEL12;
  }

}

//******* PUBLIC CONTROL FUNCTIONS ******
void bldc_Lock(int state) {
  if(state)
    bldc_status |= BLDC_LOCKED;
  else
    bldc_status &= ~BLDC_LOCKED;
}

void bldc_Stop(int CancelManual){ //stop all motors

	bldc_SetDrivers(0,0);
	bldc_SetDrivers(0,1);

  hall_fault = 0;


  for(int i = 0; i < BLDC_MOTOR_COUNT; i++){

  	uint8_t selected_motor = bldc_cm->index;

    if (CancelManual)
      bldc_cm = &bldc_motors[i];

    bldc_motors[i].status &= ~BLDC_STATUS_WIND_MODE;
    bldc_cm->target = bldc_cm->position;
     
     ActivateDrivers(0);

      bldc_cm->target = bldc_cm->position;
       bldc_motors[i].ctrl = BLDC_CTRL_STOP;

    bldc_cm = &bldc_motors[selected_motor];	 //loads previuosly selected motor to finish comutation

  }
}


void bldc_ReleaseDrivers() {
  ActivateDrivers(0);
}

void bldc_ClearStatus() {
  bldc_status &= ~(BLDC_MOTOR_CUTOFF | BLDC_VOLTAGE_TO_LOW);
  for(int i = 0; i < BLDC_MOTOR_COUNT; i++){
    //bldc_motors[i].status &= ~BLDC_STATUS_CLEARMASK;
    bldc_motors[i].status = 0;
    bldc_motors[i].i_err_cnt = 0;
    bldc_EnableMotor(0,1);
    bldc_EnableMotor(1,1);
    //bldc_Stop(1);
  }
}

int bldc_setPosition(unsigned char motor, float newpos, int windmode) { //go to position
  if(motor > BLDC_MOTOR_COUNT)
    return -1;
  //bldc_cm = &bldc_motors[motor];
  bldc_motor *mptr = &bldc_motors[motor];

  if(mptr->status & BLDC_STATUS_ERR)
    return -2;                                                //motor in error state
  if(mptr->ctrl & (BLDC_CTRL_HOMING) && (!windmode) )
    return -3;                                                //cancel 

  if(bldc_status & BLDC_UNDERVOLTAGE) {
    bldc_status |= BLDC_VOLTAGE_TO_LOW;
    return -4;
  }

  if(mptr->status & BLDC_STATUS_MANUAL)
    return -5;                                                //controlled manually

  newpos += mptr->home_offset;

  if(newpos < mptr->min_position)
    newpos = mptr->min_position;                              //limit min position
  if(newpos > mptr->max_position)
    newpos = mptr->max_position;                              //limit max position
    mptr->target = newpos * mptr->gear_ratio;
    mptr->ctrl = BLDC_CTRL_TRACKING;

  //wind mode current limit *1.3
  if(windmode)
    bldc_cm->status |=  BLDC_STATUS_WIND_MODE;
  else
    bldc_cm->status &= ~BLDC_STATUS_WIND_MODE;

   int err0 = bldc_motors[0].target - bldc_motors[0].position;
   int err1 = bldc_motors[1].target - bldc_motors[1].position;


   //switch motors immediately after send (current motor is not moving)

   if(bldc_cm->index == 1)
    if ((abs(err0) > 100 && abs(err1) < 100) && !any_motor_moving) {
      bldc_cm = &bldc_motors[0];
      eeprom_write(SYS_VARS_EE); 
    }
#if BLDC_MOTOR_COUNT > 1
   if(bldc_cm->index == 0) 
    if ((abs(err1) > 100 && abs(err0) < 100) && !any_motor_moving) {
      bldc_cm = &bldc_motors[1];
      eeprom_write(SYS_VARS_EE);
    }
#endif
  Flag_check();
  return  0;
 }

int bldc_setPositionImp(unsigned char motor, int newposImp, int windmode){ //go to position
  if(motor > BLDC_MOTOR_COUNT) 
    return -1;
  bldc_motor *mptr = &bldc_motors[motor];

  if(mptr->status & BLDC_STATUS_ERR) 
    return -2;                    //motor in error state
  if(mptr->ctrl & (BLDC_CTRL_HOMING) && (!windmode)) 
    return -3; //cancel 

  if(bldc_status & BLDC_UNDERVOLTAGE) {
    bldc_status |= BLDC_VOLTAGE_TO_LOW;
    return -4;
  }

  if(mptr->status & BLDC_STATUS_MANUAL) 
    return -5;                 //controlled manually

  newposImp += mptr->home_offset / mptr->gear_ratio;

  if(newposImp < (mptr->min_position * mptr->gear_ratio)) 
    newposImp = mptr->min_position * mptr->gear_ratio; //limit min position
  if(newposImp > (mptr->max_position * mptr->gear_ratio)) 
    newposImp = mptr->max_position * mptr->gear_ratio; //limit max position
  mptr->target = newposImp;
  mptr->ctrl = BLDC_CTRL_TRACKING;

  //wind mode current limit *1.3
  if(windmode) 
    bldc_cm->status |=  BLDC_STATUS_WIND_MODE;
  else         
    bldc_cm->status &= ~BLDC_STATUS_WIND_MODE;

  Flag_check();
  return  0;
}

int bldc_Home(unsigned char motor) {  //execute homing
  if(motor>BLDC_MOTOR_COUNT)
    return -1;

  bldc_cm = &bldc_motors[motor];
  bldc_motor *mptr = &bldc_motors[motor];

  if(mptr->status& BLDC_STATUS_ERR)
    return -2;                        //motor in error state
  if(bldc_status&BLDC_UNDERVOLTAGE) {
    bldc_status |= BLDC_VOLTAGE_TO_LOW;
    return-4;
  }
  mptr->homing_time = 0;
  mptr->ctrl = BLDC_CTRL_HOMING;
  bldc_cm->status &= ~BLDC_STATUS_WIND_MODE;
  Flag_check();
  return 0;
}

void bldc_EnableMotors(unsigned int state){
  int changed = 0;
  unsigned int newstate=state>>16;

  for(int i = 0; i < BLDC_MOTOR_COUNT; i++)
    if(state & (1 << i)) {         //axis 
      if(newstate&(1<<i))
        bldc_motors[i].state |= BLDC_MOTOR_STATE_ENABLED;
      else
        bldc_motors[i].state &= ~BLDC_MOTOR_STATE_ENABLED;
    }   					
}

void bldc_EnableMotor(unsigned char motor, unsigned char state){
	if(state == 1)
		bldc_motors[motor].state |= BLDC_MOTOR_STATE_ENABLED;
	if(state == 0)
		bldc_motors[motor].state &= ~BLDC_MOTOR_STATE_ENABLED;
}

unsigned int bldc_GetEnabledMotors(){
  unsigned int state;

  for(int i = 0; i < BLDC_MOTOR_COUNT; i++){
    state |= 1 << (16 + i);
    if(bldc_motors[i].state&BLDC_MOTOR_STATE_ENABLED)
      state |= 1 << i; 
  }
  return state;
}

int bldc_Enabled(unsigned char motor) {
  if(motor > BLDC_MOTOR_COUNT)
    return 0 ;
  bldc_motor *mptr = &bldc_motors[motor];  

  return mptr->state&BLDC_MOTOR_STATE_ENABLED ? 1: 0;
}

void bldc_SetInvert(unsigned char motor, unsigned int state){
  if(motor > BLDC_MOTOR_COUNT)
    return;
  bldc_motor *mptr = &bldc_motors[motor];  

  bldc_manual(1);  // mzp
  bldc_Stop(1);
  bldc_runout(RUNOUT_ACTIVATE);

  if(state){
    mptr->state |= BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    mptr->state |= BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE;
//    if(motor) cflags |= 1<<SwapRotation_A;
//    else cflags |= 1<<SwapRotation_B;
  }
  else{
    mptr->state |= BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    mptr->state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE;
//    if(motor) cflags &= ~(1<<SwapRotation_A);
//    else      cflags &= ~(1<<SwapRotation_B);
  }  
}

unsigned int bldc_GetInvert(unsigned char motor) {
  if(motor > BLDC_MOTOR_COUNT)
    return 0 ;
   bldc_motor *mptr = &bldc_motors[motor];  

  return mptr->state&BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE ? 1: 0;
}

void bldc_SetInvertHall(unsigned char motor, unsigned int state) {
  if(motor > BLDC_MOTOR_COUNT)
    return;
  bldc_motor *mptr = &bldc_motors[motor];  

  bldc_Stop(0);
  bldc_runout(RUNOUT_ACTIVATE);


  if(state){
    mptr->state |= BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    mptr->state |= BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE;
  }else{
    mptr->state |= BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    mptr->state &= ~BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE;

  }
}

unsigned int bldc_GetInvertHall(unsigned char motor) {
  if(motor > BLDC_MOTOR_COUNT)
    return 0 ;
  bldc_motor *mptr = &bldc_motors[motor];  

  return   mptr->state&BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE ? 1: 0;
}

int bldc_position_to_pulses(unsigned char motor, float pos) {
  if(motor > BLDC_MOTOR_COUNT)
    return 0;
  bldc_motor *mptr = &bldc_motors[motor];
  return  pos * mptr->gear_ratio;
}

float bldc_U(unsigned char measuring_point) {

  //bldc_Voltage = (((LPC_ADC0->DAT[10]>>4) & 0xfff) >> 2 ) / bldc_cfg.TConvertRatio; 
  //temperature = temperature + ( ((float)bldc_Voltage - temperature)*0.1);//integrator 

  #ifdef HALL_U_0_GROUP 
    bldc_Voltage = ((LPC_ADC[HALL_U_0_GROUP]->DAT[HALL_U_0_CHANNEL]>>4) & 0xfff) >> 2; 
    UVccHALL_0_avg = UVccHALL_0_avg + ( ((float)bldc_Voltage - UVccHALL_0_avg)*0.1);//integrator 


    bldc_Voltage = ((LPC_ADC[HALL_U_0_GROUP]->DAT[HALL_U_1_CHANNEL]>>4) & 0xfff) >>  2; 
    UVccHALL_1_avg = UVccHALL_1_avg + ( ((float)bldc_Voltage - UVccHALL_1_avg)*0.1);//integrator 

  #endif

  bldc_Voltage = ((LPC_ADC[U_ADC_GROUP]->DAT[U_ADC_CHANNEL]>>4) & 0xfff) >> 2;

  bldc_Uavg = bldc_Uavg + ( ((float)bldc_Voltage - bldc_Uavg)*0.1);//integrator

  //LPC_ADC1->SEQA_CTRL |= (1<<26); // START next conversion

  switch(measuring_point){
    case SUPPLY :
      return (float)bldc_Uavg  / bldc_cfg.UConvertRatio;
    case HALL0 :
      return (float)UVccHALL_0_avg  / bldc_cfg.HConvertRatio;
    case HALL1 :
      return (float)UVccHALL_1_avg  / bldc_cfg.H1ConvertRatio;
  }
}

float get_batt_U(){
  LPC_GPIO_PORT->DIR[BAT_GATE_PORT] &= ~(1<<BAT_GATE_PIN);  //Enable measurment mosfet
  LPC_IOCON->PIO[BAT_ADC_PORT][BAT_ADC_PIN] &= ~(3<<3);     //diasble pullup on battery adc
  LPC_SWM->PINENABLE[0] &= ~(1<<(BAT_ADC_CHANNEL+12*BAT_ADC_GROUP));  //enable ADC on selected pin

  for(volatile int i=0 ; i<100 ; i++);

  battery_voltage = 0; 

  for(int i=0 ; i<10 ; i++){


    while(!(LPC_ADC[BAT_ADC_GROUP]->DAT[BAT_ADC_CHANNEL] & (1<<31)));  //wait for data valid
    bldc_Voltage = (((LPC_ADC[BAT_ADC_GROUP]->DAT[BAT_ADC_CHANNEL]>>4) & 0xfff) >> 2 ) / 11.1; 

    battery_voltage +=  ( (float)bldc_Voltage * 0.1);//integrator 
  }

  battery_voltage /= bldc_cfg.BConvertRatio;

  LPC_SWM->PINENABLE[0] |= 1<<(BAT_ADC_CHANNEL+12*BAT_ADC_GROUP);  //disable adc1_9 on pin  0_16
  LPC_IOCON->PIO[BAT_ADC_PORT][BAT_ADC_PIN] |= 2<<3; //enable pullup on ADC -> Vds = -0,3v , Vgs = 0v
  LPC_GPIO_PORT->DIR[BAT_GATE_PORT] |= 1<<BAT_GATE_PIN; //Disable measurment mosfet

  return battery_voltage;
}

float bldc_I(unsigned char motor) {
  if(motor == 1){
    if(bldc_motors[1].status & BLDC_STATUS_ACTIVE){
    #ifdef I1_ADC_CHANNEL
      bldc_Current=((LPC_ADC[I1_ADC_GROUP]->DAT[I1_ADC_CHANNEL]>>4) & 0xfff) >> 2;
      bldc_Current -= zeroCurrent_voltage_1;
    #endif
    }
    else
      bldc_Current = 0;
  }
  else if(motor == 0){
    if(bldc_motors[0].status & BLDC_STATUS_ACTIVE){
      bldc_Current=((LPC_ADC[I0_ADC_GROUP]->DAT[I0_ADC_CHANNEL]>>4) & 0xfff) >> 2;

      bldc_Current -= zeroCurrent_voltage_0;
    }
    else
      bldc_Current = 0;
  }

  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING)){
    zeroCurrent_voltage_0 +=  ((((LPC_ADC[I0_ADC_GROUP]->DAT[I0_ADC_CHANNEL]>>4) & 0xfff) >> 2) - zeroCurrent_voltage_0)*0.001;

  }
  #ifdef I1_ADC_CHANNEL
  if(!(bldc_motors[1].status & BLDC_STATUS_MOVING)){
    zeroCurrent_voltage_0 +=  ((((LPC_ADC[I1_ADC_GROUP]->DAT[I1_ADC_CHANNEL]>>4) & 0xfff) >> 2) - zeroCurrent_voltage_0)*0.001;
  }
  #endif

  bldc_Iavg = bldc_Iavg + ( ((float)bldc_Current - bldc_Iavg)*0.01);//integrator

  mzp_current = (bldc_Iavg / bldc_cfg.IConvertRatio) * 0.5;

  if(mzp_current < 0)            //if negative current
   mzp_current = 0; 


  if(bldc_cm->index == motor)
  return (mzp_current);
}

void getFocus(void){
	adc3_SUM += ((LPC_ADC1->DAT[11]>>4) &0xfff) >> 2;
	adc4_SUM += ((LPC_ADC1->DAT[8] >>4) &0xfff) >> 2;

	adc_CNT++;
	if (adc_CNT > 500){
		adc_CNT = 0;
		adc3_VAL = adc3_SUM / 500;
		adc4_VAL = adc4_SUM / 500;
		adc3_SUM = 0;
		adc4_SUM = 0;
	}

}

int m_A_idle(){
  return ((abs(bldc_motors[0].target - bldc_motors[0].position) < 100 && (!(bldc_motors[0].state & BLDC_STATUS_ACTIVE))))?1:0;	
}

int m_B_idle(){
  return ((abs(bldc_motors[1].target - bldc_motors[1].position) < 100 && (!(bldc_motors[1].state & BLDC_STATUS_ACTIVE))))?1:0;	
}

int m_idle(){
  if(m_A_idle() && m_B_idle()) return 1;
  return 0;
}

int m_referencing(){
 if((bldc_motors[0].state & BLDC_STATUS_HOMING)||(bldc_motors[1].state & BLDC_STATUS_HOMING)) return 1;
 return 0;
}

int   motorA_moving()
{
  	return (bldc_motors[0].state & BLDC_STATUS_ACTIVE && bldc_cm->index == 0)?1:0;
}

int   motorB_moving()
{
 	return (bldc_motors[1].state & BLDC_STATUS_ACTIVE && bldc_cm->index == 1)?1:0;
} 

int bldc_getStatus(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return 0;
  return bldc_motors[motor].status;
}

int bldc_Status() {
  return  bldc_status;
}

void bldc_initStatus(unsigned int status) {
  bldc_status=status;
}

int bldc_Active(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return 0;
  return bldc_motors[motor].status & BLDC_STATUS_ACTIVE;
}

void bldc_manual(int enable) {
  for(int i = 0; i < BLDC_MOTOR_COUNT; i++)   
    if(enable)
      bldc_motors[i].status |=  BLDC_STATUS_MANUAL;
    else
      bldc_motors[i].status &= ~BLDC_STATUS_MANUAL;
}


uint8_t ignore_setpoint = 0;

void bldc_runout(int state){

  if(state == RUNOUT_ACTIVATE){
    ignore_setpoint = 1;   
  }

  if(ignore_setpoint){
    bldc_cm->target = bldc_cm->position;
    if(!(bldc_cm->status & BLDC_STATUS_MOVING)){
      ignore_setpoint = 0;
    }
  }   
   
}

float bldc_position(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];
 
  return (float)mptr->position / (float)mptr->gear_ratio;
}

int bldc_positionImp(unsigned char motor){
  if(motor>BLDC_MOTOR_COUNT) 
    return -0xfff;;
  bldc_motor *mptr = &bldc_motors[motor];
 
  return mptr->position;
}

float bldc_target(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return (float)mptr->target / (float)mptr->gear_ratio;
}

int bldc_targetImp(unsigned char motor){
  if(motor>BLDC_MOTOR_COUNT) 
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return mptr->target;
}

float bldc_remaining(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return (float)mptr->home_remaining / (float)mptr->gear_ratio;
}

int bldc_remainingImp(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT) 
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return mptr->home_remaining;
}

bldc_motor *bldc_Motor(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return (void*)(0);
  return &bldc_motors[motor];
}

bldc_misc * bldc_config() {
  return &bldc_cfg;
}

void Flag_check() {
  int val;
  for(int i = 0; i < BLDC_MOTOR_COUNT; i++) {
    bldc_motors[i].status &= ~(BLDC_STATUS_MOVING_OUT | BLDC_STATUS_MOVING_IN | BLDC_STATUS_HOMING | BLDC_STATUS_ENDSWITCH_LOW_ACTIVE | BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE);
  
    bldc_motors[i].status |=  bldc_HomeSwitchActive(i,0) ? BLDC_STATUS_ENDSWITCH_LOW_ACTIVE: 0;
    bldc_motors[i].status |=  bldc_HomeSwitchActive(i,1) ? BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE: 0;

    val = (bldc_motors[i].target - bldc_motors[i].position);

    if(val >= (-bldc_motors[i].pid.deadband -bounce_stop) && val <= (bldc_motors[i].pid.deadband + bounce_stop))
      bldc_motors[i].status &= ~(BLDC_STATUS_MOVING_OUT | BLDC_STATUS_MOVING_IN);
    else {
      if(val>0) {

        bldc_motors[i].status |= BLDC_STATUS_MOVING_OUT;
        bldc_motors[i].status &= ~BLDC_STATUS_ERR_MOVEOUT;
      } else if(val < 0) {

        bldc_motors[i].status |= BLDC_STATUS_MOVING_IN;
        bldc_motors[i].status &= ~BLDC_STATUS_ERR_MOVEOUT;
      }
      if(bldc_motors[i].ctrl == BLDC_CTRL_HOMING)
        bldc_motors[i].status|= BLDC_STATUS_HOMING;
    }
  }

  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING_IN) && !(bldc_motors[0].status & BLDC_STATUS_MOVING_OUT) &&
    !(bldc_motors[1].status & BLDC_STATUS_MOVING_IN) && !(bldc_motors[1].status & BLDC_STATUS_MOVING_OUT))
    bldc_Speed = 0;

}

uint32_t end_time;
uint32_t s_time = 0;

void bldc_update_pwm(unsigned short value) {
 // if (value < 100)
  //  value = 100;

 // if (value >= 0x7ff)
  //  value = 0xfff;

  LPC_SCT0->MATCHREL2 = value;//////////////////////////////////////////////////////////////
}

void bldc_ClearHallIntRequest() {
//  LPC_GPIO2->IC = (1<<4)|(1<<5)|(1<<6);
}

int bldc_HomeSwitchActive(unsigned char motor , unsigned char switch_h_l) {
  if (motor == 0){

    if(!ES_0_normallyOpenLo && !switch_h_l)
      return !(LPC_GPIO_PORT->PIN[END_SW_0_LO_PORT] & (1<<END_SW_0_LO_PIN));
    else if(ES_0_normallyOpenLo && !switch_h_l)
      return (LPC_GPIO_PORT->PIN[END_SW_0_LO_PORT] & (1<<END_SW_0_LO_PIN));
    else if(!ES_0_normallyOpenHi && switch_h_l)
      return !(LPC_GPIO_PORT->PIN[END_SW_0_HI_PORT] & (1<<END_SW_0_HI_PIN));
    else if(ES_0_normallyOpenHi && switch_h_l)
      return (LPC_GPIO_PORT->PIN[END_SW_0_HI_PORT] & (1<<END_SW_0_HI_PIN));
  }

  else if (motor == 1){
    #ifdef END_SW_1_HI_PORT

    if(!ES_1_normallyOpenLo && !switch_h_l)
      return !(LPC_GPIO_PORT->PIN[END_SW_1_LO_PORT] & (1<<END_SW_1_LO_PIN));
    else if(ES_1_normallyOpenLo && !switch_h_l)
      return (LPC_GPIO_PORT->PIN[END_SW_1_LO_PORT] & (1<<END_SW_1_LO_PIN));
    else if(!ES_1_normallyOpenHi && switch_h_l)
      return !(LPC_GPIO_PORT->PIN[END_SW_1_HI_PORT] & (1<<END_SW_1_HI_PIN));
    else if(ES_1_normallyOpenHi && switch_h_l)
      return (LPC_GPIO_PORT->PIN[END_SW_1_HI_PORT] & (1<<END_SW_1_HI_PIN));

   #endif
  }
}


unsigned char OldState;

void bldc_SetDrivers(unsigned char NewState, unsigned char motor){
  //disable pwm drivers
  phase_active = 0;
  mosfet_protection_cnt = 0;

  // WDT reset 
  LPC_WWDT->FEED = 0xAA;		
  LPC_WWDT->FEED = 0x55;


  if(motor==1){
  #ifdef MOTOR_B_LO1_PORT

    //disable low side
    if(!(NewState & BLDC_PA_NEG))
      LPC_GPIO_PORT->CLR[MOTOR_B_LO1_PORT] |= 1<<MOTOR_B_LO1_PIN; // B L L1
    if(!(NewState & BLDC_PB_NEG))
      LPC_GPIO_PORT->CLR[MOTOR_B_LO2_PORT] |= 1<<MOTOR_B_LO2_PIN; // B L L2
    if(!(NewState & BLDC_PC_NEG))
      LPC_GPIO_PORT->CLR[MOTOR_B_LO3_PORT] |= 1<<MOTOR_B_LO3_PIN; // B L L3

    //activate pwm high side driver
    switch(NewState & 0xf0){
      case BLDC_PA_POS:
        LPC_SWM->PINASSIGN[7] = (PIN_B_HI1 << 8) | 0xff; break;         // MB H1
      case BLDC_PB_POS:
        LPC_SWM->PINASSIGN[7] = (PIN_B_HI2 << 16) | 0xff; break;        // MB H2
      case BLDC_PC_POS:
        LPC_SWM->PINASSIGN[7] = (PIN_B_HI3 << 24) | 0xff; break;        // MB H3
     }
    //activate low side driver
    switch(NewState & 0x0f){
      case BLDC_PA_NEG:
        LPC_GPIO_PORT->SET[MOTOR_B_LO1_PORT] |= 1<<MOTOR_B_LO1_PIN; phase_active++; break; // B L L1
      case BLDC_PB_NEG:
        LPC_GPIO_PORT->SET[MOTOR_B_LO2_PORT] |= 1<<MOTOR_B_LO2_PIN; phase_active++;break; // B L L2
      case BLDC_PC_NEG:
        LPC_GPIO_PORT->SET[MOTOR_B_LO3_PORT] |= 1<<MOTOR_B_LO3_PIN; phase_active++;break; // B L L3
    }
   #endif
   }

  else if (motor == 0){

    //disable low side
    if(!(NewState & BLDC_PA_NEG))
      LPC_GPIO_PORT->CLR[MOTOR_A_LO1_PORT] |= 1<<MOTOR_A_LO1_PIN; // A L L1
    if(!(NewState & BLDC_PB_NEG))
      LPC_GPIO_PORT->CLR[MOTOR_A_LO2_PORT] |= 1<<MOTOR_A_LO2_PIN; // A L L2
    if(!(NewState & BLDC_PC_NEG))
      LPC_GPIO_PORT->CLR[MOTOR_A_LO3_PORT] |= 1<<MOTOR_A_LO3_PIN; // A L L3 

    //activate pwm high side driver
    switch(NewState & 0xf0){
      case BLDC_PA_POS:
        LPC_SWM->PINASSIGN[7] = (PIN_A_HI1 <<  8) | 0xff; break;       //  MA H1
      case BLDC_PB_POS:
        LPC_SWM->PINASSIGN[7] = (PIN_A_HI2 << 16) | 0xff; break;       //  MA H2
      case BLDC_PC_POS:
        LPC_SWM->PINASSIGN[7] = (PIN_A_HI3 << 24) | 0xff; break;       //  MA H3
    }
    //activate low side driver
    switch(NewState & 0x0f){
      case BLDC_PA_NEG:
        LPC_GPIO_PORT->SET[MOTOR_A_LO1_PORT] |= 1<<MOTOR_A_LO1_PIN; phase_active++; break;// A L L1
      case BLDC_PB_NEG:
        LPC_GPIO_PORT->SET[MOTOR_A_LO2_PORT] |= 1<<MOTOR_A_LO2_PIN; phase_active++; break;// A L L2
      case BLDC_PC_NEG:
        LPC_GPIO_PORT->SET[MOTOR_A_LO3_PORT] |= 1<<MOTOR_A_LO3_PIN; phase_active++; break;// A L L3
    }
  }

}


void ActivateDrivers(int dir) {
  if (bldc_cm->status & BLDC_STATUS_ACTIVE) {
    bldc_idletime = 0;
    bldc_runtime++;
    if(dir != 0)
      return;
    bldc_SetDrivers(0,bldc_cm->index);  //stop
    bldc_cm->status &= ~(BLDC_STATUS_ACTIVE | BLDC_STATUS_MANUAL | BLDC_STATUS_WIND_MODE);

  } else {        // only on 1.st entry --- next one is hall comutated
    if(dir == 0)   
      return;

    bldc_runtime = 0;
    bldc_pwm = 0;
    bldc_cm->status |= BLDC_STATUS_ACTIVE;
    bldc_Comutate(bldc_cm->index);
  }
}

unsigned int motor_err(uint8_t motor)
{
    return 1; //bldc_motors[motor].target - bldc_motors[motor].position;
}


#define BLDC_RAMP_UP 2
#define BLDC_RAMP_DOWN 10

void motor_ramp() {
  int err = bldc_cm->target - bldc_cm->position;
  int dir = (err <= 0) ? 1: -1;

  int rampval = pid_calc(&bldc_cm->pid) * 40.95; //resulting max rampval = 0xfff

  if(direction_delay)
    direction_delay--;
  if(!direction_delay){
    
  //changing direction -- smooth transition
    if (err>bldc_cm->pid.deadband && bldc_pwm<0 ) {
      if(bldc_pwm < -BLDC_RAMP_DOWN)
        bldc_pwm += BLDC_RAMP_DOWN;
      else{
        bldc_pwm  = 0;
        //direction_delay = 100;
      }
      return;
    }
  
    if(err<-bldc_cm->pid.deadband && bldc_pwm>0 ) {
      if(bldc_pwm > BLDC_RAMP_DOWN)
        bldc_pwm -= BLDC_RAMP_DOWN;
      else{
        bldc_pwm  = 0;
        //direction_delay = 100;
      }
      return;
    }

  //stop request
  if(rampval == 0 && bldc_pwm != 0) {
    if(bldc_pwm < 0) {
      if(bldc_pwm < -BLDC_RAMP_DOWN){
        bldc_pwm += BLDC_RAMP_DOWN;
        bldc_cm->status |= BLDC_STATUS_ACTIVE;
        //bldc_cm->target = bldc_cm->position;
        }
      else
        bldc_pwm = rampval;
      //bldc_cm->status &= ~BLDC_STATUS_ACTIVE;
        //bldc_SetDrivers(0,0);
        //bldc_cm->target = bldc_cm->position;
    }
    if(bldc_pwm > 0) {
      if(bldc_pwm>BLDC_RAMP_DOWN){
        bldc_pwm -= BLDC_RAMP_DOWN;
        bldc_cm->status |= BLDC_STATUS_ACTIVE;
        }
      else
        bldc_pwm  = rampval;
        //bldc_SetDrivers(0,0);
        //bldc_cm->target = bldc_cm->position;
    }
    //direction_delay = 100;
    return;
  }

  if(err > 0) {  //CW  
    if(rampval > bldc_pwm){
      if(bldc_pwm < 0xfff)
        bldc_pwm += BLDC_RAMP_UP;
    } else {
      bldc_pwm = rampval;
    }
  
  } else {  //CCW
     if(rampval < bldc_pwm) { 
       if(bldc_pwm > -0xfff)
        bldc_pwm -= BLDC_RAMP_UP;
    }else {
      bldc_pwm = rampval;
    }

  }
  }
}

unsigned int  bldc_recovery_overvoltage=0;
unsigned int  bldc_recovery_undervoltage=0;
unsigned int  bldc_recovery_timeout=0;
unsigned int  bldc_undervoltage_timer=0;
unsigned int  bldc_overvoltageDetectDelay=0;

void voltage_detection() {
  float voltage = bldc_U(SUPPLY);
  UVccHALL_0 = bldc_U(HALL0);
  UVccHALL_1 = bldc_U(HALL1);

  bldc_recovery_timeout++;
  bldc_recovery_undervoltage++;
  bldc_recovery_overvoltage++;

  //  	//420=15V, 450=16V, 480=17V, 510=18V  770=27.5 812=29V
  //Overvoltage detection
  if(voltage >= 32.0) {
    if(bldc_overvoltageDetectDelay++ >= 1000) {
      bldc_recovery_overvoltage = 0;
      bldc_overvoltageDetectDelay = 1000;
      bldc_status |= BLDC_OVERVOLTAGE;
    }
  } else if((voltage < 30.0) && (bldc_recovery_overvoltage >= 10000)) {
    bldc_status &= ~BLDC_OVERVOLTAGE;
    bldc_overvoltageDetectDelay = 0;
  }

  //Undervoltage detection
  if (voltage < UNDERVOLTAGE_LEVEL) { //undervoltage	
    bldc_undervoltage_timer++;
    bldc_recovery_undervoltage=0;
    bldc_status |= BLDC_UNDERVOLTAGE;
  } else if ((voltage >= (UNDERVOLTAGE_LEVEL)) && (bldc_recovery_undervoltage >= 5000)) {	
    bldc_status &= ~BLDC_UNDERVOLTAGE;
  }	

  if(voltage < MOTOR_CUTOF_LEVEL){
    if(bldc_cm->status & BLDC_STATUS_ACTIVE) bldc_status |= BLDC_MOTOR_CUTOFF;
    bldc_Stop(1);
  }
	
  //turn motors off
  if(voltage <= SHUTDOWN_VOLTAGE) {
    bldc_status |= BLDC_SHUTTING_DOWN;
    bldc_recovery_timeout = 0;

    /*if ((bldc_status&(1<<flash_erase_done))&&(!(bldc_status&(1<<flash_write_done)))) {
            if(undervoltage_timer<=200)tracker_exstatus &=~EFS_UNDERVOLTAGE;
            flash_write(SYS_VARS_ADDR);
            bldc_status|=(1<<flash_write_done);
    }*/

  } else if( voltage>MOTOR_CUTOF_LEVEL && bldc_status&BLDC_SHUTTING_DOWN) { //15.0v 10s
    bldc_recovery_timeout++;
    if(bldc_recovery_timeout > 10000) {
        bldc_status &= ~BLDC_SHUTTING_DOWN;	
    }
  }
}

uint32_t max_I_A, max_I_B;

//MOTOR core control function
void bldc_process() {

  bldc_runout(RUNOUT_FREEWHEEL);
  if (bldc_GetInvertHall(0))
             cflags |= 1<<swap_halls_A;
  else cflags &= ~(1<<swap_halls_A);

  if (bldc_GetInvertHall(1))
             cflags |= 1<<swap_halls_B;
  else cflags &= ~(1<<swap_halls_B);	


  if (bldc_GetInvert(0))
       cflags |= 1<<SwapRotation_A;
  else cflags &= ~(1<<SwapRotation_A);

  if (bldc_GetInvert(1)) 
       cflags |= 1<<SwapRotation_B;
  else cflags &= ~(1<<SwapRotation_B);   

   int err0 = bldc_motors[0].target - bldc_motors[0].position;
   int err1 = bldc_motors[1].target - bldc_motors[1].position;


   any_motor_moving = 0;

   if(moving_counter_a){
    bldc_motors[0].status |= BLDC_STATUS_MOVING;
    moving_counter_a --;
    any_motor_moving = 1;
   // LPC_GINT1->PORT_ENA[1] = 0; //Disable interrupts
  }
   else{
    bldc_motors[0].status &= ~BLDC_STATUS_MOVING; 

    //LPC_GINT1->CTRL |= 1<<0;
   // LPC_GINT1->PORT_ENA[1] |= (1<<1) | (1<<2) | (1<<3);  //Enable interrupts
   } 
 

  if(moving_counter_b){
    bldc_motors[1].status |= BLDC_STATUS_MOVING;
    moving_counter_b --;
    any_motor_moving = 1;
  //  LPC_GINT0->PORT_ENA[0] = 0; //Disable interrupts
  }
  else{
    bldc_motors[1].status &= ~BLDC_STATUS_MOVING;   

   // LPC_GINT0->CTRL |= 1<<0;
   // LPC_GINT0->PORT_ENA[0] |= (1<<13) | (1<<30) | (1<<31);  //Enable interrupts
  }


  //check if motor is disconnected
  if(bldc_ReadHall(0) == 7 && bldc_motors[0].status & BLDC_STATUS_ACTIVE){
    motor_a_disconnected++;
    if (motor_a_disconnected > 50){
      bldc_motors[0].status |= BLDC_STATUS_HALL_FAULT;
      //bldc_motors[0].status |= BLDC_STATUS_CABLEERROR;
      bldc_EnableMotor(0,1);
    } 
  }else if(motor_a_disconnected){
    motor_a_disconnected--;
    if(motor_a_disconnected == 0)
      bldc_motors[0].status &= BLDC_STATUS_HALL_FAULT;
  }

  if(bldc_ReadHall(1) == 7 && bldc_motors[1].status & BLDC_STATUS_ACTIVE){
    motor_b_disconnected++;
    if(motor_b_disconnected > 50){
      bldc_motors[1].status |= BLDC_STATUS_HALL_FAULT;
      //bldc_motors[1].status |= BLDC_STATUS_CABLEERROR;
      bldc_EnableMotor(1,1);
    }  
  }else if(motor_b_disconnected){
    motor_b_disconnected--;
    if(motor_b_disconnected == 0)
      bldc_motors[0].status &= BLDC_STATUS_HALL_FAULT;
  }

  


//switch motors when current motor is finished

#if BLDC_MOTOR_COUNT > 1

   if(bldc_cm->index == 1)
      if ((abs(err0) > 100 && abs(err1) < 100) && !any_motor_moving){
        bldc_Stop(0);
        eeprom_write(SYS_VARS_EE);
        for(int i=0 ; i<2000000 ; i++)
        bldc_cm = &bldc_motors[0];
        bldc_cm->ctrl = BLDC_CTRL_TRACKING;
        //return;
        //bldc_Stop(0);
        //bldc_cm->status &= ~BLDC_STATUS_ACTIVE;  
    }
    
   if(bldc_cm->index == 0) 
    if ((abs(err1) > 100 && abs(err0) < 100) && !any_motor_moving){
      bldc_Stop(0);
      eeprom_write(SYS_VARS_EE);
      for(int i=0 ; i<2000000 ; i++)
      bldc_cm = &bldc_motors[1];
      bldc_cm->ctrl = BLDC_CTRL_TRACKING;
      //bldc_Stop(0);
      //bldc_cm->status &= ~BLDC_STATUS_ACTIVE;  
    }

#endif

  voltage_detection();
  Flag_check();

 // bldc_motor *bldc_cm = &bldc_motors[1];

  //***control disabled error state*** 
 if(bldc_status & BLDC_LOCKED || !(bldc_cm->state & BLDC_MOTOR_STATE_ENABLED) || bldc_cm->status & BLDC_STATUS_ERR) {
    ActivateDrivers(0);
    bldc_pause = 0;
    bldc_cm->ctrl = BLDC_CTRL_IDLE;
    bldc_cm->target = bldc_cm->position;
    return;
  }

  //****over current detection****
  float ILimit;
  if (bldc_runtime < bldc_cm->I_Inrush_time)
    ILimit = bldc_cm->I_limit *  bldc_cm->I_Inrush_ratio;
  else if(bldc_cm->status & BLDC_STATUS_WIND_MODE)
    ILimit = bldc_cm->I_limit *  1.3;
  else
    ILimit = bldc_cm->I_limit;
  
  if( bldc_I(bldc_cm->index) >= ILimit) {
    SetEventParameters(bldc_cm->index);
  
    ActivateDrivers(0); //stop motor now
  
    bldc_cm->i_err_cnt++;
    if(bldc_cm->i_err_cnt>=1) bldc_cm->status |= BLDC_STATUS_ERR | BLDC_STATUS_OVERCURRENT;
    else bldc_pause = 3 * 1000; //5s timeout

    return;
  }

//****cable error detection****
    if(bldc_cm->status & BLDC_STATUS_ACTIVE && bldc_cm->status & BLDC_STATUS_STALL && abs(bldc_pwm)>=0xa00){
      SetEventParameters(bldc_cm->index);
      ActivateDrivers(0);
      bldc_cm->status |= BLDC_STATUS_ERR | BLDC_STATUS_CABLEERROR;
      return;
    }

  //****end switch error detection****
  if(bldc_HomeSwitchActive(bldc_cm->index,0) && bldc_cm->ctrl == BLDC_CTRL_TRACKING && bldc_cm->position > bldc_position_to_pulses(bldc_cm->index, bldc_cm->end_switchDetect - 0.1)){
    SetEventParameters(bldc_cm->index);
    ActivateDrivers(0);
    bldc_cm->status|= BLDC_STATUS_ENDSWITCH_ERROR;
    return;
  }

    if(bldc_HomeSwitchActive(bldc_cm->index,1) && bldc_cm->ctrl == BLDC_CTRL_TRACKING && bldc_cm->position > bldc_position_to_pulses(bldc_cm->index, bldc_cm->end_switchDetect - 0.1)){
    SetEventParameters(bldc_cm->index);
    ActivateDrivers(0);
    bldc_cm->status|= BLDC_STATUS_ENDSWITCH_ERROR;
    return;
  }
    
  //****end switch detection****

  if(bldc_HomeSwitchActive(bldc_cm->index,0) && bldc_cm->ctrl & BLDC_CTRL_HOMING) {
    ActivateEvent(EVENT_HOME_A_FINISHED);
    bldc_cm->ctrl = BLDC_CTRL_IDLE;
    bldc_cm->home_remaining = bldc_cm->position;
    bldc_cm->position = bldc_cm->target = 0;
    ActivateDrivers(0);
    return;
  }


  //****Invert direction of rotation or hall****
  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING) && (bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST)){ //After motor A stoped moving
    bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    if(bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE)
      bldc_motors[0].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;
    else
      bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION;
  }
  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING) && (bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST)){ //After motor A stoped moving
    bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    if(bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE)
      bldc_motors[0].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;   
    else
      bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION; 
  }

  if(!(bldc_motors[1].status & BLDC_STATUS_MOVING) && (bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST)){ //After motor B stoped moving
    bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    if(bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE)
      bldc_motors[1].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;
    else
      bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION;
  }

  if(!(bldc_motors[1].status & BLDC_STATUS_MOVING) && (bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST)){ //After motor B stoped moving
    bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    if(bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE)
      bldc_motors[1].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;   
    else
      bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION; 
  }
  //*************************************************************


//  if(bldc_HomeSwitchActive(bldc_cm->index,0) && bldc_cm->target < bldc_cm->position){
//    bldc_cm->target = bldc_cm->position;
//    bldc_cm->ctrl = BLDC_CTRL_STOP;
//    ActivateDrivers(0);
//    return;
//  }
//
//    if(bldc_HomeSwitchActive(bldc_cm->index,1) && bldc_cm->target > bldc_cm->position){
//    bldc_cm->target = bldc_cm->position;
//    bldc_cm->ctrl = BLDC_CTRL_STOP;
//    ActivateDrivers(0);
//    return;
//  }

  // Homing timeout
  if(bldc_cm->ctrl == BLDC_CTRL_HOMING && bldc_cm->homing_time >= (bldc_cfg.homing_timeout * 1000)) {
    bldc_cm->status |= BLDC_STATUS_ERR | BLDC_STATUS_HOME_TIMEOUT;
  }

  

  //***Process control function*****
  if(bldc_cm->ctrl & BLDC_CTRL_STOP) { //STOP REQUEST
    bldc_cm->target = bldc_cm->position;                    //force ramp down
  }else if(bldc_cm->ctrl == BLDC_CTRL_HOMING) {
    bldc_cm->target = bldc_cm->position - bldc_position_to_pulses(0, 10);      //set destination negative
    bldc_cm->homing_time++;
  }

  // calculate next step
  motor_ramp();

  // limit pwm duty to working range
  if(bldc_pwm >  0xfff)
    bldc_pwm =   0xfff;
  if(bldc_pwm < -0xfff)
    bldc_pwm =  -0xfff;


  //if(!direction_delay)
  if((bldc_pwm > 0) && !(any_motor_moving && bldc_cm->status&BLDC_STATUS_CCW)) {
    bldc_cm->status &= ~BLDC_STATUS_CCW;
    ActivateDrivers(1);
    bldc_update_pwm(bldc_pwm);
  }else if((bldc_pwm < 0) && !(any_motor_moving && !(bldc_cm->status&BLDC_STATUS_CCW))) {
    bldc_cm->status |= BLDC_STATUS_CCW;
    ActivateDrivers(-1);
    bldc_update_pwm(-bldc_pwm); 
  } else {  //deactivate drivers
    bldc_idletime++;
    if(bldc_idletime>=100) {
      bldc_idletime = 100;
      bldc_cm->ctrl = BLDC_CTRL_IDLE;  
      ActivateDrivers(0);    
    }
  } 
}


//commutation 
void bldc_Comutate(unsigned char motor){

    bldc_cm = &bldc_motors[motor];
    bldc_cm->status  &=  ~BLDC_STATUS_STALL;

    //take sample of comutation speed
    if(!LPC_SCT1->EVFLAG & 1<<0){ //read speed only if timer not overflowed
      bldc_Speed_raw += LPC_SCT1->COUNT;
      if(++speed_sample_counter >= 6 * number_of_poles){
        speed_sample_counter = 0;
        bldc_Speed =  (60ULL * SystemCoreClock)  / bldc_Speed_raw;	    
        bldc_Speed_raw = 0;
      }
    }

    LPC_SCT1->EVFLAG |= 1 << 0;   // reset overflow flag
    LPC_SCT1->CTRL |= (1 << 2); // halt speed measurment timer
    LPC_SCT1->CTRL |= (1 << 3); // clear count

    unsigned char state = bldc_ReadHall(motor); 

    if(state > 0 && state < 7)
      hall_detect++;
    else 
      hall_detect = 0;


    if(commutation_counter < 6)
         commutation_counter++;
    else commutation_counter = 0, hall_detect = 0; 

      
    if((hall_detect < 6) && (commutation_counter == 6)){
      hall_fault++;
      if (hall_fault > 5){
        bldc_motors[bldc_cm->index].status |= BLDC_STATUS_HALL_FAULT;
        bldc_EnableMotor(bldc_cm->index,0);      
        hall_fault = 0;
        }
    }



    
    if(bldc_cm->state & BLDC_MOTOR_STATE_INVERT_HALL){ //Swap direct. of encoder count){//inverter 
        if      (bldc_ccw_next[bldc_cm->hall_state][0] == state)  
          bldc_cm->position++;
        else  if(bldc_cw_next [bldc_cm->hall_state][0] == state)  
          bldc_cm->position--;
    }else{                                       //Normal
        if      (bldc_ccw_next[bldc_cm->hall_state][0] == state)  
          bldc_cm->position--;
        else  if(bldc_cw_next [bldc_cm->hall_state][0] == state)  
          bldc_cm->position++;
    }
    
    
    
    if(bldc_cm->status & BLDC_STATUS_ACTIVE){   
        if(bldc_cm->state & BLDC_MOTOR_STATE_INVERT_DIRECTION){//Inverted operation

            if(bldc_cm->status & BLDC_STATUS_CCW)
              bldc_SetDrivers(bldc_cw_next [state][1], motor);
            else                                 
              bldc_SetDrivers(bldc_ccw_next[state][1], motor); 
        }else{                                       //NORMAL operation         
            
            if(bldc_cm->status & BLDC_STATUS_CCW)
              bldc_SetDrivers(bldc_ccw_next[state][1], motor);
            else                                 
              bldc_SetDrivers(bldc_cw_next [state][1], motor); 
        }
        LPC_SCT1->CTRL &= ~(1 << 2); // unhalt speed measurment timer
    }else bldc_Speed = 0;
    
    bldc_cm->hall_state = state;//save state
}


// HALL Interrupts
void PIN_INT2_IRQHandler() {
  moving_counter_a = 100;
  bldc_Comutate(0);
  LPC_PINT->IST = 1<<2; //clear edge interrupt
}  

void PIN_INT3_IRQHandler() {
  moving_counter_a = 100;
  bldc_Comutate(0);
  LPC_PINT->IST = 1<<3; //clear edge interrupt
}  

void PIN_INT4_IRQHandler() {
  moving_counter_a = 100;
  bldc_Comutate(0);
  LPC_PINT->IST = 1<<4; //clear edge interrupt
}  

void PIN_INT5_IRQHandler() {
  moving_counter_b = 100;
  bldc_Comutate(1);
  LPC_PINT->IST = 1<<5; //clear edge interrupt
}  

void PIN_INT6_IRQHandler() {
  moving_counter_b = 100;
  bldc_Comutate(1);
  LPC_PINT->IST = 1<<6; //clear edge interrupt
}  

void PIN_INT7_IRQHandler() {
  moving_counter_b = 100;
  bldc_Comutate(1);
  LPC_PINT->IST = 1<<7; //clear edge interrupt
}  

