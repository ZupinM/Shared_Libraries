
#include "micro.h"
#include "bldc.h"
#include "config.h"
#include "sun.h"

#include "LPC15xx.h"
#include "../gpio.h"

/***********************************************************
  CHECK TIME
************************************************************/
extern bldc_motor bldc_motors[BLDC_MOTOR_COUNT];
//extern bldc_motor *bldc_cm;
extern uint8_t usb_drive;
extern uint32_t systick_count;
extern float WindSpeed;

extern int WindWaitCnt;
extern int SnowWaitCnt;

extern MODE_TYPE mode;

extern unsigned int tracker_status;

int WindFreq = 0;
int WindCnt = 0;

int force_recalc=0;
void Recalc_sun(){
  force_recalc = 1;
}

void init_encoder(){
//  H_enc_new=0;
//  V_enc_new=0;
//  
//  H_enc_new |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6))?(1<<0):0;
//  H_enc_new |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))?(1<<1):0;
//  V_enc_new |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8))?(1<<0):0;
//  V_enc_new |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))?(1<<1):0;
//
//  H_enc_old = H_enc_new;
//  V_enc_old = V_enc_new;
}
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

//PA4  wind
//PB15 snow

void change_buyFlags(unsigned int newVal){
  unsigned int old = buyflags;
  buyflags         = newVal;
  
  if((buyflags & (1<<snow_secondary))&&(buyflags & (1<<goRef_input))){
    if((old^newVal)&(1<<snow_secondary)){
      buyflags&=~(1<<goRef_input);
    }else{
      buyflags&=~(1<<snow_secondary);
    }
  }
  
  if((buyflags & (1<<wind_secondary))&&(buyflags & (1<<stopclear_input))){
    if((old^newVal)&(1<<wind_secondary)){
      buyflags&=~(1<<stopclear_input);
    }else{
      buyflags&=~(1<<wind_secondary);
    }
  }  
}

void cfg_wind_input(){
//  if(buyflags& (1<<wind_secondary)){
//    NVIC_DisableIRQ(EXTI1_IRQChannel);
//    NVIC_EnableIRQ(EXTI4_IRQChannel);
//  }else{
//    NVIC_DisableIRQ(EXTI4_IRQChannel);
//    NVIC_EnableIRQ(EXTI1_IRQChannel);
//  }
}


int Snow_Input(){
  //if(buyflags & (1<<snow_secondary))return; 
  return   !(LPC_GPIO_PORT->PIN[SNOW_SENSOR_PORT]&(1<<SNOW_SENSOR_PIN));
//  return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
}



void check_time() {

//    double run_delay_A,temp1;    //seconds24 = {0 ... 86400 s}
    unsigned int temp; //, destA, destB;
    float HTraget_sel;
    float VTarget_sel;
    time_t time_from,time_to;
    //eot_and_declination(day_of_year);

    seconds24=(time.hours*3600)+(time.minutes*60)+(time.seconds);	//transforming to seconds (in 24hours)
    temp=(unsigned int) (seconds24/interval);                                            //interval
    seconds24=(uint32_t)(temp*interval);

    
   //****************FOCUS SENSOR PROCESSING****************** 
   //Seek State
    temp=0;
    if(last_seconds24 != (unsigned int)seconds24){//focus sensor correction
     last_seconds24 = (unsigned int)seconds24;//save current state
     temp=1;

    if(cflags&(1<<FocusSensInUse)){
       if(!Focus_avg_timeout){ 
          focus_update();//update offset values..before focus_start!!!
          focus_start();
          if(focus_locked())Focus_avg_timeout=1; //start averaging  
       }
      }else{
        focus_reset(); 
        Focus_avg_timeout=0;
      }   
    }
   
    
    //AVG state
    if(cflags&(1<<FocusSensInUse)){
      if(Focus_avg_timeout){
          if(++Focus_avg_timeout >= (1*50*60)){ //50==1s
            Focus_avg_timeout=0; //1 min averaging time
            focus_restart();
          }
      }
      
      if( (flags&(1<<SnowModeOn))|| (flags&(1<<WindModeOn)))  focus_cancel();      
      else if(Focus_avg_timeout)                              seconds24 += interval/2;
    }

    if((temp==0)&&(!force_recalc)) return;
    force_recalc = 0;
    //*******************************************************
    
    ////////////////
    sun_AziEle(longitude,  latitude,  &AE_azimuth, &AE_elevation, &hour_angle, &declination);
    flags|=(1<<equationsAE_ok);
    
  
    
    latitude_2 = latitude;     
    if (latitude_2 < 0) {      
        latitude_2=-latitude;
    }
 
    suns_position_PM (&PM_elevation, &PM_azimuth, hour_angle, declination);

    //mzp
//    flags|=(1<<equationsAE_ok);            //default pravilni izracuni enacb
//    flags|=(1<<equationsPM_ok);               //default pravilni izracuni enacb
//    if(Time_lower(&time, sun_sunrise())||Time_higherEq(&time,sun_sunset())){
//      flags&=~(1<<equationsAE_ok);         //sledenje nima smisla
//      flags&=~(1<<equationsPM_ok);         //sledenje nima smisla
//      AE_elevation=0;
//    }
    ////////////////////
    
    
  /*  hour_angle=((seconds24+(60.0*eot_offset))*(0.25/60.0))-180.0;	//hour angle v stopinjah iz seconds24, upostevajoc EOT
    
    //--- južna polobla ---
    //obnem deklinacijo za 180 stopinj in zrcalim hour_angle - enacbe za AE in PM ostanejo nespremenjene
    latitude_2=latitude;             //"latitude" je prava, "latitude_2" pa samo pozitivne vrednosti
    if (latitude<0) {
      hour_angle=-hour_angle;         //na Južno poloblo obni urni kot
      latitude_2=-latitude;
    }//--------------------

    //suns_position_AE (hour_angle);                                      //azimuta in elevaciji za koordinatna sistema: AZI/ELE in POLARMOUNT
   // suns_position_PM (hour_angle);*/
    
    
    //Select active target
    temp = 0;
    HTraget_sel = target_H;
    VTarget_sel = target_V;
    
    /*TicksToDateTime((uint32_t)HeliostatP1_start, &time_from);
    TicksToDateTime((uint32_t)HeliostatP1_end  , &time_to);
    if(Time_higherEq(&time, &time_from) && Time_lower(&time, &time_to)){
      HTraget_sel = target_H;
      VTarget_sel = target_V;
      temp = 1;
    }*/
      
    TicksToDateTime((uint32_t)HeliostatP2_start, &time_from);
    TicksToDateTime((uint32_t)HeliostatP2_end  , &time_to);
    if(Time_higherEq(&time,&time_from) && Time_lower(&time, &time_to)&&((HTarget2!=1000)&&(VTarget2!=1000))){
      HTraget_sel = HTarget2;
      VTarget_sel = VTarget2;
      temp = 1;
    }
    
    TicksToDateTime((uint32_t)HeliostatP3_start, &time_from);
    TicksToDateTime((uint32_t)HeliostatP3_end  , &time_to);
    if(Time_higherEq(&time,&time_from) && Time_lower(&time, &time_to)&&((HTarget3!=1000)&&(VTarget3!=1000))&&(!temp)){
      HTraget_sel = HTarget3;
      VTarget_sel = VTarget3;
      temp = 1;
    }
    
    
    mirror_equations(HTraget_sel,VTarget_sel,180.0);
    mirror_equationsAE (HTraget_sel, VTarget_sel);
/*    if (last_seconds24!=seconds24){                                     //proženje odštevanja ob spremembi "seconds24"
        temp=(int) (can_id/group);                                      //CAN_ID sledilca v posamezni grupi pomeni koliko zamaknjeno zacne voziti (da niso vsi naenkrat - velik tok).
        track_delay=((int)can_id-(temp*(int)group))*(int)run_delay;     //Npr: CAN_ID=125, group=50 run_delay=3s .... 125/50=2,5 ... 2*50=100 ... 125-100=25 ... 25*3s=75s zamika
        back_delay=((int)can_id-(temp*(int)group))*(int)run_delay_home;
        last_seconds24=(int)seconds24;
    } */

    //return back to reference
/*    if ((time.hours==NightMode_time)&&(time.minutes==0)&&(time.seconds==0)&&(bflags&(1<<time_enable))&&(!(flags&(1<<WindModeOn)))&&(!(flags&(1<<SnowModeOn)))) {    //ne gre home, ce je tracking izkljucen
      if (!(flags&(1<<moveback_delcalc))) {
        temp=(int)(can_id/group);                                                      //CAN_ID sledilca v posamezni grupi pomeni koliko zamaknjeno zacne voziti (da niso vsi naenkrat - velik tok).
        back_delay=((int)can_id-(temp*(int)group))*(int)run_delay_home;                        //Npr: CAN_ID=125, group=50 run_delay=3s .... 125/50=2,5 ... 2*50=100 ... 125-100=25 ... 25*3s=75s zamika
        flags|=(1<<moveback_trigger);                               //da se samo enkrat izvede
        flags|=(1<<moveback_notrack);                               //po polnoci se brise (rutina RTC interrupt)
        flags|=(1<<moveback_delcalc);
      }
    }
    else flags&=~(1<<moveback_delcalc); */

    //avtomatsko delanje reference na vsake N dni... razen ce je "synch run mode"
    if (!(cflags&(1<<RunBothMotors))) {
      if (goref_Nday_A!=0) {                            //A os dela referenco ob ko je najblizje vzhodu (min_range)
        if ((goref_Nday_cnt_A>=goref_Nday_A)&&(bldc_position(0)<(min_range_A+1000))) {
          bldc_manual(0);
          status_homing = 1;
          bldc_Home(0);
          goref_Nday_cnt_A=0; 
        }
      }
      else goref_Nday_cnt_A=0;

      if ((goref_Nday_B!=0)&&(geometry_mode_B!=0)) {          //B os dela referenco takoj ob polnoci, ko je najverjetneje v nocni poziciji najblizje 90st
        if (goref_Nday_cnt_B>=goref_Nday_B) {
          bldc_manual(0);
          status_homing = 1;
          bldc_Home(1);
          goref_Nday_cnt_B=0; 
        }
      }
      else goref_Nday_cnt_B=0;
    }


    // tracking podnevi...
   if ((bflags&(1<<time_enable))/*&&(!(flags&(1<<moveback_notrack)))*/&&(!(flags&(1<<WindModeOn)))&&(!(flags&(1<<SnowModeOn)))) {
         if(Time_higherEq(&time,sun_sunrise()) && Time_lower(&time, sun_sunset())) {
          
            if ((SpacePanelA!=0)&&(WidePanelA!=0)) anti_shadowing (&PM_azimuth,PM_azimuth,SpacePanelA,WidePanelA,HOUR_ANGLE);
            if ((SpacePanelB!=0)&&(WidePanelB!=0)) anti_shadowing (&PM_elevation,PM_elevation,SpacePanelB,WidePanelB,ELEVATION);

            angle_A=coordinate_selection((int)coordinate_mode_A,1);        //dolocitev kotov, kateri se bo nastavljal za posamezno os
            angle_B=coordinate_selection((int)coordinate_mode_B,2);
            
//S
// Odmik sledilnika od sonca v primeru previsoke temperature-senzor je na pb3
  if( Snow_Input() && (!(cflags&&(1<<SnowSensInUse))) ){  // 
    count_S = (signed int)Time_out_of_focus*3000;   // mnozi s 3000, da dobis zakasnitev v imp iz minut
  }
  
  if( (count_S > 0) && (!(cflags&&(1<<SnowSensInUse))) ){        
    count_S--;
    angle_A += focus_angle_A()+OverTempShift; // odmik od sownca v primeru previsoke temperature za obe osi za kot OverTempShift
    angle_B += focus_angle_B()+OverTempShift;
    focus_cancel();
  }else{                          
    angle_A+=focus_angle_A();        //dodajanje offseta[deg]
    angle_B+=focus_angle_B();
  }                   
///S 
            //---geometry selection---
            bldc_manual(0);
            usb_drive = 0;
           // bldc_motors[0].ctrl &= ~BLDC_CTRL_STOP;
            bldc_setPosition(0, geometry_selection(1,angle_A,(int)geometry_mode_A), 0);
            bldc_process();
            //if(!(bldc_motors[0].status & BLDC_STATUS_ACTIVE))
           //wait_for_motor0;
            bldc_setPosition(1, geometry_selection(2,angle_B,(int)geometry_mode_B), 0);

        }
    //tracking ponoci...
        else {
          if ((time.month!=0)/*&&(back_delay==0)*/){                                         //time.month => na zacetku preden prebere uro, je ta 0:00:00, 0.0.0000, zato gre po resetu za eno sekundo na night position
            if((int)home_position_A!=1000)bldc_manual(0),  bldc_setPosition(0, geometry_selection (1,home_position_A,(int)geometry_mode_A), 0);       //postavi se na pozicijo za cez noc
            if((int)home_position_B!=1000)bldc_manual(0),  bldc_setPosition(1, geometry_selection (2,home_position_B,(int)geometry_mode_B), 0);
          } 
        }
    }
} 

/***********************************************************
  EQUATION OF TIME and DECLINATION
************************************************************/
/*void eot_and_declination(uint32_t doy) {

	double temp;

	//temp=doy;
	//temp=((360.0/364.0)*(temp-81.0))*3.1416/180.0;
	eot_offset=(9.87*sin(2*temp)-7.53*cos(temp)-1.5*sin(temp)); //14.875;
        //eot_offset v minutah

      //  if (doy==81)doy=82;     //ob deklinaciji==0 --> rezultat 1. osi vedno 0 (enacbe)

       // temp=(doy+284);								//day_of_year -> declination (1. part)...mora biti v dveh delih zaradi vrstnega reda izracuna
	//declination=(sin((((temp*360.0)/365.0)*PI)/180.0))*((23.45*PI)/180.0);	//day_of_year -> declination (2. part); DEG->RAD
        //za Južno poloblo:
       // if (latitude<0) declination=(sin((((temp*360.0)/365.0)*PI)/180.0))*(-((23.45*PI)/180.0));   //na južni polobli obnemo deklinacij za 180 stopinj

}       //izhod: deklinacija v radianih
*/
/***********************************************************
  SUN'S LOCATION IN AZI-ELE ANGLES

// input ha (degrees)
// output PM_azimuth (degrees)
// output PM_elevation (degrees)
************************************************************/
//extern unsigned int m_errA;
//static double old=0,oldAzi=0,oldpos=0;
/*void suns_position_AE (double ha) {	

    double Atemp;


    
    ha=(ha*PI)/180.0;	//deg->rad

    Atemp=asin(sin((latitude_2*PI)/180.0)*sin(declination)+cos((latitude_2*PI)/180.0)*cos(declination)*cos(ha));	 //elevation
    AE_elevation=(Atemp*180.0)/PI;

    if (AE_elevation<0) {                   //sonce za horizontom?
      AE_elevation=0;
      Atemp=0;
      flags&=~(1<<equationsAE_ok);         //sledenje nima smisla
    }

    Atemp=(sin(Atemp)*sin((latitude_2*PI)/180.0)-sin(declination))/(cos(Atemp)*cos((latitude_2*PI)/180.0));             //azimuth angle
    Atemp=acos(Atemp);                                                                                                  //acos mora biti posebej, drugace acos (1/1) vrne NaN vrednost (compiler bug)
    //Atemp=acos((sin(Atemp)*sin((latitude_2*PI)/180.0)-sin(declination))/(cos(Atemp)*cos((latitude_2*PI)/180.0)));     //
    if(ha<0)Atemp=-Atemp;           //enacba da vedno pozitiven kot, zato popravek pri dopoldanskih kotih
    AE_azimuth=(Atemp*180.0)/PI;
    
  //  if(oldAzi>AE_azimuth)m_errA|=MOTOR_ERR_NO_HALL;
   // oldAzi=AE_azimuth;
    
}
*/


/***********************************************************
  SUN'S LOCATION IN POLAR-MOUNT ANGLES

// input ha (degrees)
// output PM_azimuth (degrees)
// output PM_elevation (degrees)
************************************************************/
/*
void suns_position_PM (double ha) {          //  ha v stopinjah

    double psi,Atemp; 

    flags|=(1<<equationsPM_ok);               //default pravilni izracuni enacb

    if (ha<(-90.0)) {                       //omejitev PM enacb - pravilne med 6 in 18 uro (ha: -90...+90)
      ha=-90.0;
      flags&=~(1<<equationsPM_ok);            //za omejitvijo, enacbe padejo, sledenje nima smisla
    }
    if (ha>90.0) {
      ha=90.0;
      flags&=~(1<<equationsPM_ok);
    }

    ha=(ha*PI)/180.0;	                    //deg->rad

    if((ha > -0.001)&&(ha<0.0))
    ha=-0.001;
 
    if((ha < 0.001)&&(ha>0.0))
    ha=0.001;
    
    Atemp=atan(cos(declination)*cos(ha)/sin(declination));	        //zenith elevation
    if (declination<0.0) psi=-(Atemp+(PI/2.0));					
    else psi=(PI/2.0)-Atemp;					        //psi

    Atemp=(PI/2.0)-((latitude_2*PI)/180.0)+psi;		                //elevation
    PM_elevation=(Atemp*180.0)/PI;                                      //elevation in degrees

    Atemp=PI + 2.0*atan((cos(declination)*sin(ha)*sin(psi)) / (sin(declination)-sin(psi)));   //azimuth
    PM_azimuth=(Atemp*180.0)/PI;                                                            //azimuth in degrees
    if (ha<0.0) PM_azimuth-=360.0;
    
    //testing 
  //  if(old>PM_azimuth)m_errA|=MOTOR_ERR_OVERCURRENT;
   // old=PM_azimuth;
    
    //if ((ha<0.0017)&&(ha>-0.0017)) PM_azimuth=0.0;	// atan (x/0) @ha==0 -> error. Prihaja do nepravih prevojev, ko sta "declination" in "psi" zelo enaka (blizu).
                                                        //0.0017rad = 0.1deg
    //primer napake je bil: datum: 15/11 ura: 11:45
}
*/
void suns_position_PM (double *elevation, double *azimuth, double ha, double declination) {          //  ha v stopinjah

	double psi,Atemp;
	double ele,azi,haa;
	ele=*elevation;
	azi=*azimuth;
	haa=ha;
	

        
        
	if (haa<(-90.0)) {                       //omejitev PM enacb - pravilne med 6 in 18 uro (ha: -90...+90)
		haa=-90.0;
		//    flags&=~(1<<equationsPM_ok);            //za omejitvijo, enacbe padejo, sledenje nima smisla
	}
	if (haa>90.0) {
		haa=90.0;
		//    flags&=~(1<<equationsPM_ok);
	}

	haa=(haa*PI)/180.0;	                    //deg->rad atan (0/0) @ha==0 -> error
	if((haa > -0.001)&&(haa<0.0))
	haa=-0.001;

	if((haa < 0.001)&&(haa>0.0))
	haa=0.001;

	Atemp=atan(cos(declination)*cos(haa)/sin(declination));	        //zenith elevation
	if (declination<0.0) psi=-(Atemp+(PI/2));					
	else psi=(PI/2)-Atemp;					        //psi

	Atemp=(PI/2.0)-((latitude_2*PI)/180.0)+psi;		                //elevation
	ele=(Atemp*180.0)/PI;                                      //elevation in degrees


	Atemp=PI + 2.0*atan((cos(declination)*sin(haa)*sin(psi)) / (sin(declination)-sin(psi)));   //azimuth
	azi=(Atemp*180.0)/PI;                                                            //azimuth in degrees
	if (haa<0.0) (azi)-=360.0;

	*elevation=ele;
	*azimuth=azi;
}

/***********************************************************
        ANTI-SHADOWING

input: namesto -50...+50deg, pricakuje 0...180
output: Ce sencenja ni, vrne nazaj enako vrednost azimuta.
return: 1 - antishadowing je
************************************************************/
int anti_shadowing (double *NewAngle, double InputAngle, double space, double wide, char type){	

        double temp,A_crtica,solutionA,solutionB;

        temp=InputAngle;
        if (type==HOUR_ANGLE) InputAngle=InputAngle+90.0;       //urni kot se meri -50....50, enacbe racunajo med 0...180

        InputAngle=(InputAngle*PI)/180.0;
        A_crtica=(space*sin(InputAngle)) - (wide/2.0);  //gama = 90-alpha+beta .... smiselno, ko je alpha=beta, zato je gama=90 -> sin(gama)=1

        if ((2*A_crtica)<wide) {                          //ja, senca je cez panel, zato izracunaj nov kot

          solutionA=asin((space/wide)*sin(InputAngle))-(PI/2)+(InputAngle);         //dve resitvi za arcsin ...vzemi tisto, ki ima panel obrnjen v soncu
          solutionB=(PI/2)-asin((space/wide)*sin(InputAngle))+(InputAngle);
          solutionA=(solutionA*180.0)/PI;
          solutionB=(solutionB*180.0)/PI;

          if (type==HOUR_ANGLE){
            if(temp<0)  *NewAngle = solutionB-90.0;                  //dopoldanski koti ha
            else        *NewAngle = solutionA-90.0;                  //popoldanski koti ha
          }
          else *NewAngle = solutionB;   //elevacijsko sencenje ima vedno resitev 2 (resitev 1 je za kot vecje od 90st)
          return 1;
        }
        else return 0;
}

/***********************************************************
        MIRROR-equations

target_azi  :   kot med jugom in tarco v vodoravni smeri
target_ele  :   kot med tlemi in tarco po visini
Aos         :   pod katerim kotom glede na jug lezi azimutna os (default = 180.0)
************************************************************/

void mirror_equations (double target_azi, double target_ele, double Aos) {

    double lambda,gama,hzrc,azrc,hos,psi,AE_azimuth2,AE_elevation2;

    target_azi=(target_azi*PI)/180.0;       //degrees to radians
    target_ele=(target_ele*PI)/180.0;
    Aos=(Aos*PI)/180.0;
    AE_azimuth2=(AE_azimuth*PI)/180.0;
    AE_elevation2=(AE_elevation*PI)/180.0;

    lambda=acos(sin(AE_elevation2)*sin(target_ele)+cos(AE_elevation2)*cos(target_ele)*cos(AE_azimuth2 - target_azi));   //kot med soncem in tarco
    gama=PI-2*((PI/2)-atan((cos(target_ele)*sin(AE_azimuth2-target_azi))/(sin(lambda)+cos(AE_elevation2)*sin(target_ele)-sin(AE_elevation2)*cos(target_ele)*cos(AE_azimuth2-target_azi))));    //kot med stranicama Z-S in S-T

    hzrc=asin(sin(AE_elevation2)*cos(lambda/2)+cos(AE_elevation2)*sin(lambda/2)*cos(gama));   //višina (elevacija) zrcala hzrc
    azrc=AE_azimuth2-asin((sin(gama)*sin(lambda/2))/cos(hzrc));                              //azimut zrcala Azrc

    hos=-atan((1/tan(hzrc))*cos(azrc-Aos));                                     //višina (elevacija) osi hos - izhod
    psi=2*atan((-cos(hzrc)*sin(azrc-Aos)*cos(hos))/(cos(hos)+sin(hzrc)));       //kot zasuka zrcala - izhod

    HE_elevation=90.0-((hos*180.0)/PI);
    HE_azimuth=(psi*180.0)/PI;

}



void mirror_equationsAE (double target_azi, double target_ele) {
   HE_AE_elevation = AE_elevation +((target_ele - AE_elevation)/2.0);
   HE_AE_azimuth   = AE_azimuth   +((target_azi - AE_azimuth)/2.0);
}
//*************************ALES TRAVNIK*************************************
double DtoR(double degrees)
{
	return (degrees * PI / 180);
}

double  RtoD(double radians)
{
	return (radians * 180 / PI);
}


// Funkcija za izraÄun rotacije horizontalnega (z moÅ¾nostjo nagiba) sledilnika
// azimuthD [-k*180, +k*180] stopinje - azimut sonca (smer J proti Z)
// elevationD (0, 180) stopinje - elevacija sonca
// deviationD [-360, 360] stopinje - odklon smeri sledilnika od smeri J proti Z (J-S = 0, Z-V = 90)  //rotacija na ravnini
// inclinationD [-90, 90] stopinje - !koti nat 45Â° so vpraÅ¡ljivi in mogoÄe zahtevajo drugo metodo sledenja! rotacija pravokotno na ravnino
// return (-90, 90) stopinje - rotacija sledilnika (0 = polje celic vodoravno)
double IdealRotationAngle(double azimuthD, double elevationD, double deviationD, double inclinationD)
{
        double azimuthR;
        double elevationR;   
        double deviationR;
        double inclinationR;
          
        if (elevationD <= 0.0 || elevationD >= 180.0){
          return 0.0;
        }

	azimuthR = DtoR(azimuthD);
	elevationR = DtoR(elevationD);
	deviationR = DtoR(deviationD);

	if (inclinationD == 0.0) return RtoD(atan(sin(azimuthR - deviationR) / tan(elevationR)));
        inclinationR = DtoR(inclinationD);
	return RtoD(atan(sin(azimuthR - deviationR) / (cos(inclinationR) * tan(elevationR) + sin(inclinationR) * cos(azimuthR - deviationR))));
}

// Funkcija za izraÄun popravljene rotacije horizontalnega sledilnika (BACKTRACKING)
// idealRotationAngleD (-90, 90) - izhod iz funkcije IdealRorationAngle
// panelWidth (x dolÅ¾inska enota) - Å¡irina panela
// panelSafetyWidth (x dolÅ¾inska enota) - varnostni dodatek k Å¡irini panela
// trackerDistance (x dolÅ¾inska enota) - razdalja med sledilniki
// // x dolÅ¾inska enota je lahko v mm, cm, dm, m oz. karkoli drugega. Pomembno je le, da imajo vsi parametri enako enoto!
// return (-90, 90) stopinje - popravljena rotacija sledilnika (0 = polje celic vodoravno)
double CorrectedRotationAngle(double idealRotationAngleD, double panelWidth, double panelSafetyWidth, double trackerDistance)
{
	double idealRotationAngleR = DtoR(idealRotationAngleD);
	double width = panelWidth + panelSafetyWidth;
	
	double shadowDistance = width / cos(idealRotationAngleR);
	double shadowReach = trackerDistance / shadowDistance;
	
	if (shadowReach >= 1.0) return idealRotationAngleD;
	
	return RtoD(idealRotationAngleR - ((idealRotationAngleR > 0.0) - (idealRotationAngleR < 0.0)) * acos(shadowReach));
}








/***********************************************************
        COORDINATE SELECTION

coordinate_mode   .... izbira kateri kot se nastavlja

************************************************************/
double coordinate_selection (uint32_t coordinate_mode, int axis) {

  double angle;

        switch (coordinate_mode) {
        case 1: angle=AE_azimuth;
                break;
        case 2: angle=AE_elevation;
                break;
        case 3: angle=PM_azimuth;
                break;
        case 4: angle=PM_elevation;
                break;
        case 5: if (buyflags&(1<<heliostat)) angle=HE_azimuth;      //zasciteni parametri
                else angle=0;
                break;
        case 6: if (buyflags&(1<<heliostat)) angle=HE_elevation;
                else angle=0;
                break;
        case 7:if (buyflags&(1<<heliostat)) angle=HE_AE_azimuth;      //zasciteni parametri
               else angle=0;
               break;
        case 8:if (buyflags&(1<<heliostat)) angle=HE_AE_elevation;
                else angle=0;
                break;
        case 11:
                if(axis==1) angle =  IdealRotationAngle(AE_azimuth, AE_elevation, -deviation_A, inclination_A);
                else        angle =  IdealRotationAngle(AE_azimuth, AE_elevation, -deviation_B, inclination_B);
                
                if(axis==1){
                    if((panel_width_A != 0)&&(panel_space_A !=0 ))
                      angle =  CorrectedRotationAngle(angle, panel_width_A, panel_thick_A,panel_space_A);
                }else{
                    if((panel_width_B != 0)&&(panel_space_B !=0 ))
                      angle =  CorrectedRotationAngle(angle, panel_width_B, panel_thick_B,panel_space_B);
                }
                
                break;
        }
        return angle;
}
/***********************************************************
        GEOMETRY SELECTION

axis            .... za katero os velja (1 ali 2)
angle           .... na kateri kot naj se postavi
geometry_mode   .... izbira geometrije

output          .... destinacija za zeleno os

************************************************************/
unsigned int geometry_selection (uint32_t axis, double angle, uint32_t geometry_mode) {      //sledenje

  double dest;

    if (axis==1){                   //A os, A parametri
        switch (geometry_mode) {
        case 1: dest=geometry_1(axis,angle,A1_A,gear_ratio_A);                               //skeliranje - pomik v krogu (SM3, azimutni pomiki preko polža)
                break;
        case 11:dest = geometry_11(angle, A1_A,gear_ratio_A);
		break;
        case 2: dest=geometry_2(angle,A1_A,A2_A,A3_A,A4_A,B1_A,B2_A,!(int)A6_A,gear_ratio_A);      //Etop geometrija - enoosni, nagib osi
                break;
        case 3: dest=geometry_3(angle,A1_A,A2_A,A3_A,A5_A,B1_A+B2_A,(int)A6_A,gear_ratio_A);                //SM34 - vertikalna os
                break;
        case 13:geometry_13    (angle,A1_A,A2_A,A3_A,A5_A,B1_A+B2_A,(int)A6_A,gear_ratio_A);
                break;
        }

        dest /= gear_ratio_A;

        if (dest>max_range_A) dest=(int)max_range_A;	//limit
        if (dest<min_range_A) dest=(int)min_range_A;  //lower limit
        return (uint32_t) dest;
    }

    else {                          //B os, B parametri
        switch (geometry_mode) {
        case 1: dest=geometry_1(axis,angle,A1_B,gear_ratio_B);                               //skeliranje - pomik v krogu (SM3, azimutni pomiki preko polža)
                break;
        case 11:
		dest = geometry_11(angle, A1_B,gear_ratio_B);
		break;
        case 2: dest=geometry_2(angle,A1_B,A2_B,A3_B,A4_B,B1_B,B2_B,!(int)A6_B,gear_ratio_B);      //Etop geometrija - enoosni, nagib osi
                break;
        case 3: dest=geometry_3(angle,A1_B,A2_B,A3_B,A5_B,B1_B+B2_B,(int)A6_B,gear_ratio_B);                //SM34 - vertikalna os
                break;
        case 13:
		dest = geometry_13(angle,A1_B,A2_B,A3_B,A5_B,B1_B+B2_B,(int)A6_B,gear_ratio_B);
		break;
        }

        dest /= gear_ratio_B;

        if (dest>max_range_B) dest=(int)max_range_B;       //limit
        if (dest<min_range_B) dest=(int)min_range_B;       //lower limit
        return (uint32_t) dest;
    }

}
/***********************************************************
        GEOMETRY SELECTION - SHOW on PC

axis            .... za katero os velja (1 ali 2)
geometry_mode   .... izbira geometrije
vrne vrednost kota v stopinjah
************************************************************/
double geometry_selection_show (uint32_t axis, uint32_t geometry_mode) {           //prikaz na PCju

    double temp;

    if (axis==1){                   //A os, A parametri
        switch (geometry_mode) {
        case 1: temp=geometry_1_show(axis, bldc_position(0),A1_A,gear_ratio_A);                   //SM3 - enoosni
                break;
        case 2: temp=geometry_2_show(bldc_position(0),A1_A,A2_A,A3_A,A4_A,B1_A,B2_A,!(int)A6_A,gear_ratio_A);                   //Etop geometrija - enoosni
                break;
        case 3: temp=geometry_3_show(bldc_position(0),A1_A,A2_A,A3_A,A5_A,B1_A+B2_A,(int)A6_A,gear_ratio_A);                   //SM34 - vertikalna os
                break;
        case 11:temp=geometry_11_show(bldc_position(0),A1_A,gear_ratio_A);
                break;
        case 13:temp=geometry_13_show(bldc_position(0),A1_A,A2_A,A3_A,A5_A,B1_A+B2_A,(int)A6_A,gear_ratio_A);
                break;
        }
    }

    else {                          //B os, B parametri
        switch (geometry_mode) {
        case 1: temp=geometry_1_show(axis, bldc_position(1),A1_B,gear_ratio_B);                         //SM3 - enoosni
                break;
        case 2: temp=geometry_2_show(bldc_position(1),A1_B,A2_B,A3_B,A4_B,B1_B,B2_B,!(int)A6_B,gear_ratio_B);                   //Etop geometrija - enoosni
                break;
        case 3: temp=geometry_3_show(bldc_position(1),A1_B,A2_B,A3_B,A5_B,B1_B+B2_B,(int)A6_B,gear_ratio_B);                   //SM34 - vertikalna os
                break;
        case 11:temp=geometry_11_show(bldc_position(1),A1_B,gear_ratio_B);
                break;
        case 13:temp=geometry_13_show(bldc_position(1),A1_B,A2_B,A3_B,A5_B,B1_B+B2_B,(int)A6_B,gear_ratio_B);
                break;
        }
    }
    return temp;
}

/***********************************************************
    GEOMETRY 1 - gred SM3 motorja

A2 = sredina relat. ranga (7735imp)
************************************************************/
//izracun azimuth (v stopinjah) -> destination (imp)
double geometry_1 (int axis ,double angle, double A1, double gear_ratio) {  
  double new_angle=0;
  int coordiante_mode;
  
  if(axis==1)coordiante_mode = (int)coordinate_mode_A;
  else       coordiante_mode = (int)coordinate_mode_B;

  
  if((coordiante_mode==1)||(coordiante_mode==3)||(coordiante_mode==5)||(coordiante_mode==11))  new_angle = 180.0+angle-A1;
  if((coordiante_mode==2)||(coordiante_mode==4)||(coordiante_mode==6))                         new_angle = 90.0 +angle-A1;
  
  if(new_angle>360) new_angle = 360;
  if(new_angle<0  ) new_angle=0;

  return new_angle*gear_ratio;
}
//--------------------------------------------------
//sprotni izracun pozicije v stopinjah za prikaz na PCju
double geometry_1_show (int axis,int32_t position, double A1, double gear_ratio){
  int coordiante_mode;
  
  if(axis==1)coordiante_mode = (int)coordinate_mode_A;
  else       coordiante_mode = (int)coordinate_mode_B;

  
  if((coordiante_mode==1)||(coordiante_mode==3)||(coordiante_mode==5)||(coordiante_mode==11))    return (((position/gear_ratio)+A1)-180.0);
  if((coordiante_mode==2)||(coordiante_mode==4)||(coordiante_mode==6))                           return (((position/gear_ratio)+A1)-90.0); 
  return 0;
}



//elevation angle slew drive coordinate mode 11
double geometry_11(double angle, double A1,double gear_ratio) {
	double new_angle=0;
	  new_angle = 90.0  + (90-angle) - A1;

	if(new_angle>360) new_angle=360;
	if(new_angle<0  ) new_angle=0;

	return new_angle*gear_ratio;
}

double geometry_11_show(int32_t position, double A1, double gear_ratio) {
	double new_angle=0;
	
        new_angle = A1 +(position/gear_ratio)-90;

	return new_angle*gear_ratio;
}

/***********************************************************
    GEOMETRY 2 - Etop konstrukcija enoosnega sledilca

    A1
    A2
    A3
    A4 (staticni del)
    B1
    B2
************************************************************/
//"ha" v stopinjah
double geometry_2 (double ha, double A1, double A2, double A3, double A4, double B1, double B2, int left, double gear_ratio){

     /* double c;

      c=((90.0-B1-B2+ha)*PI)/180.0;

      if (c<0) c=0;                                    //negativni kot ne sme biti, ker povzroci pozitivne impulze, ko teh ne sme biti

      c=pow(A1, 2)+pow(A2, 2)-(2*A1*A2*cos(c));

      c=sqrt(c-pow(A3,2));*/
      
      //if(oldpos>((gear_ratio*(c-A4))))m_errA|=MOTOR_ERR_CABLE;
      //oldpos=((gear_ratio*(c-A4)));
      double Atemp;

      if (left) Atemp = ((90.0 + ha - B1 - B2 ) * PI / 180); // Aktuator na levi strani trackerja (motor odpira tekom dneva)
      else Atemp = ((90.0 - ha - B1 - B2) * PI / 180); // Aktuator na desni strani trackerja (motor zapira tekom dneva)

      if (Atemp < 0) Atemp=0;                                    //negativni kot ne sme biti, ker povzroci pozitivne impulze, ko teh ne sme biti

      Atemp = pow(A1, 2) + pow(A2, 2) - (2*A1*A2*cos(Atemp));
      Atemp = sqrt(Atemp - pow(A3,2));
      
      return((gear_ratio*(Atemp-A4)));
//      if (destination>max_range_A) destination=max_range_A;	//west limit
}
//--------------------------------------------------
//sprotni izracun pozicije v stopinjah za prikaz na PCju
/*double geometry_2_show (int32_t position, double A1, double A2, double A3, double A4, double B1, double B2,int left, double gear_ratio) {

        double temp;

        temp=pow(((position/gear_ratio)+A4),2)+pow(A3,2);
        temp=(acos((pow(A1,2)+pow(A2,2)-temp)/(2*A1*A2))*180.0)/PI;
      
        if (left)   return(-90.0+temp+B1+B2);
	else        return(-90.0-temp+B1+B2);

        //return(temp-90.0+B1+B2);
}*/

double geometry_2_show(int32_t position ,double A1, double A2, double A3, double A4, double beta1, double beta2, int left,double gear_ratio){
	
	double azimuth;
	
	azimuth = position / gear_ratio + A4;
	azimuth = pow(azimuth, 2) + pow(A3, 2);
	azimuth = acos((-azimuth + pow(A1, 2) + pow(A2, 2))/(2 * A1 * A2));
	
	if (left)	azimuth = (180 * azimuth / PI) -90 + beta1 + beta2;
	else		azimuth = -((180 * azimuth / PI) -90 + beta1 + beta2);
	
	return azimuth;	
}

/***********************************************************
    GEOMETRY 3 - SM34 elevacijska geometrija

    A1 = stator length (61mm)
    A2 = half_axis 1 (300)
    A3 = half_axis 2 (300)             //length from elevation vertex to joint actuator-shaft (catheta)
    A4 = constant_angle ((138.953)
    A5 = actuator_holder (39.6)		//39.6mm (objemka na aktuatorju)
************************************************************/
//izracun elevacije (v stopinjah) -> destination_B (imp)
double geometry_3 (double elev, double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio){

    /*double Atemp;

    Atemp=((A4*PI)/180.0)-((elev*PI)/180.0);    //E->A

    if (Atemp<0) Atemp=0;                                  //negativni kot ne sme biti, ker povzroci pozitivne impulze, ko teh ne sme biti

    Atemp=(pow(A2,2.0)+pow(A3,2.0))-(2*A2*A3*cos(Atemp));	//length from angle c2=a2*b2-2*a*c*cos(A)
    Atemp=sqrt(Atemp-pow(A5,2.0));		                //additional angle a2=c2-b2*/
    double Atemp;

    if (bottom)	Atemp = ((elev - alphaBeta) * PI / 180); // Aktuator na spodnji strani trackerja (motor odpira z dvigom sonca)
    else Atemp = ((180 - elev - alphaBeta) * PI / 180); // Aktuator na zgornji strani trackerja (motor zapira z dvigom sonca)

    if (Atemp<0) Atemp=0;                                  //negativni kot ne sme biti, ker povzroci pozitivne impulze, ko teh ne sme biti

    Atemp=(pow(A2,2.0)+pow(A3,2.0))-(2*A2*A3*cos(Atemp));	//length from angle c2=a2*b2-2*a*c*cos(A)
    Atemp=sqrt(Atemp-pow(A5,2.0));		                //additional angle a2=c2-b2
  
    return((Atemp-A1)*gear_ratio);                              //A1 = stator_length
//    if (destination_B>max_range_B) destination_B=max_range_B;	//limit moving range
}
//--------------------------------------------------
//sprotni izracun pozicije v stopinjah za prikaz na PCju
/*double geometry_3_show (int32_t position, double A1, double A2, double A3, double A5, double alphaBeta, int bottom, double gear_ratio) {
    //return(((((A4*PI)/180.0)-acos(((pow(A2,2.0)+pow(A3,2.0))-(pow(((position/gear_ratio)+A1),2.0)+pow(A5,2.0)))/(2*A2*A3)))*180.0)/PI);
    return(((((alphaBeta*PI)/180.0)-acos(((pow(A2,2.0)+pow(A3,2.0))-(pow(((position/gear_ratio)+A1),2.0)+pow(A5,2.0)))/(2*A2*A3)))*180.0)/PI);
}*/


double geometry_3_show(int32_t position ,double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio) {
	
	double elev;
	
	elev = position / gear_ratio + A1;
	elev = pow(elev, 2) + pow(A5, 2);
	elev = acos((-elev + pow(A2, 2) + pow(A3, 2)) / (2 * A2 *A3));
	
	if (bottom) elev = (elev * 180 / PI) + alphaBeta;
	else        elev = -((elev * 180 / PI) - 180 + alphaBeta);
	
	return elev;	
}


//geometrija 13 za coordinate mode 11 elevacija
double geometry_13(double elev, double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio) {
	double Atemp;

	if (bottom) Atemp = (((90-elev)  - alphaBeta) * PI / 180);       // Aktuator na spodnji strani trackerja (motor odpira z dvigom sonca)
	else        Atemp = ((180 - (90-elev) - alphaBeta) * PI / 180); // Aktuator na zgornji strani trackerja (motor zapira z dvigom sonca)

	if (Atemp<0) Atemp=0;                                  //negativni kot ne sme biti, ker povzroci pozitivne impulze, ko teh ne sme biti

	Atemp=(pow(A2,2.0)+pow(A3,2.0))-(2*A2*A3*cos(Atemp));	//length from angle c2=a2*b2-2*a*c*cos(A)
	Atemp=sqrt(Atemp-pow(A5,2.0));		                //additional angle a2=c2-b2
	return (Atemp-A1)*gear_ratio ;                                //A1 = stator_length  
}

double geometry_13_show(int32_t encPuls ,double A1, double A2, double A3, double A5, double alphaBeta, int bottom, double gear_ratio) {
	
	double elev;
	
	elev = encPuls / gear_ratio + A1;
	elev = pow(elev, 2) + pow(A5, 2);
	elev = acos((-elev + pow(A2, 2) + pow(A3, 2)) / (2 * A2 *A3));
	
	if (bottom)	elev = -((elev * 180 / PI) - 90 + alphaBeta);
	else		elev = (elev * 180 / PI) - 180 + 90 + alphaBeta;
 	return elev;
}


int Focusing()
{
  return (Focus_avg_timeout)?0:1;
}

void init_weather_sensor(){

  LPC_INMUX->PINTSEL[1] = WIND_SENSOR_PIN + 32*WIND_SENSOR_PORT;; //pin 0_1 = wind input 
  LPC_PINT->IENR |= 1<<1;
  NVIC_EnableIRQ(PIN_INT1_IRQn);

}


void PIN_INT1_IRQHandler() {

  LPC_PINT->IST |= 1<<1;
  WindFreq = WindCnt;
  WindCnt = 0; 

}

void weather_sensor (){

//  Uwind=adc4_VAL;                 //Uwind napetost uporablja faktor od Usolar
//  Uwind=Uwind/usolar_factor;

//  Usnow=adc3_VAL;                 //Usnow napetost uporablja faktor od Usolar
//  Usnow=Usnow/usolar_factor;
//  SnowHeight=Usnow*SnowFactor;    //visina snega


  //wind sensor

    //WindSpeed=WindFreq;
    WindSpeed=(1301.0*WindFactor)/WindFreq;        //WindFreq=8000 presteje v eni sekundi (1Hz). Zato, da je lahko WindFactor stevilka koliko m/s piha pri 1Hz impulzov
    //if(WindSpeed > 200)  WindSpeed=0;
    if(WindCnt > 2000 || WindCnt < 3) WindSpeed = 0;
    if(WindFreq==NO_WIND)WindSpeed=0;


    if (((bflags&(1<<time_enable)) || mode==MODE_SLAVE_TRACKING)&&(cflags&(1<<WindSensInUse))){

        if ((WindSpeed > WindSpeedThreshold)) {
          if (WindWaitCnt > 25){                    //25 * 50 = 0,5sek delay preden gre Wind-save pozicijo - v izogib motnjam
            if (WindDestinationA!=1000.0){
                bldc_setPosition(0, geometry_selection (1,WindDestinationA,(int)geometry_mode_A), 1);
                //m_windmode_moveA();
                //m_goA(geometry_selection (1,WindDestinationA,(int)geometry_mode_A));   //pojdi na pozicijo za veter
            }
            if (WindDestinationB!=1000.0){
                bldc_setPosition(1, geometry_selection (2,WindDestinationB,(int)geometry_mode_B), 1);
                //m_windmode_moveB();
                //m_goB(geometry_selection (2,WindDestinationB,(int)geometry_mode_B));   //pozicija 1000.0 = disable
            }
            tracker_status |= SF_WIND_MODE;
            flags|=(1<<WindModeOn);
            WindWaitCnt=(int)WindWaitOff*50*60;       //za stetje nazaj bo v uporabi drug cas - v MINUTAH
          }
          else WindWaitCnt++;
        }
        else {
            if (WindWaitCnt>0)  WindWaitCnt--;
            if (WindWaitCnt==0) 
              if(mode==MODE_SLAVE_TRACKING) tracker_status &= ~SF_WIND_MODE;
              else flags&=~(1<<WindModeOn);
        }
    }
    else {
        WindWaitCnt=0;
        if(mode==MODE_SLAVE_TRACKING) tracker_status &= ~SF_WIND_MODE;
        else flags&=~(1<<WindModeOn);
    }

//snow sensor

  if (((bflags&(1<<time_enable)) || mode==MODE_SLAVE_TRACKING)&&(cflags&(1<<SnowSensInUse))){  // WindMode ima prednost pred SnowMode.

    if (Snow_Input()) {                     // TTL vhod za snow
      if (SnowWaitCnt > 500) {                                          // pocaka 10 sekund (50*10=500)
        if (SnowDestinationA!=1000.0)
          bldc_manual(0);
          usb_drive = 0;
          bldc_setPosition(0, geometry_selection (1,SnowDestinationA,(int)geometry_mode_A), 0);   //pojdi na pozicijo za sneg
        if (SnowDestinationB!=1000.0)
          bldc_manual(0);
          usb_drive = 0;
          bldc_setPosition(1, geometry_selection (2,SnowDestinationB,(int)geometry_mode_B), 0);
        flags|=(1<<SnowModeOn);
        tracker_status |= SF_SNOW_MODE;
        SnowWaitCnt=3000;
      }
      else SnowWaitCnt++;
    }
    else {
      if (SnowWaitCnt!=0) SnowWaitCnt--;                //pocaka 1 minuto (50*60=3000), da ves sneg pade dol
      if (SnowWaitCnt==0) {
       if(flags&(1<<SnowModeOn))
          bldc_Stop(1);
       if(mode==MODE_SLAVE_TRACKING)
          tracker_status &= ~SF_SNOW_MODE;
       else flags&=~(1<<SnowModeOn);
      }
    }
  }
  else {
    SnowWaitCnt=0;
    if(mode==MODE_SLAVE_TRACKING)
      tracker_status &= ~SF_SNOW_MODE;
    else flags&=~(1<<SnowModeOn);
  }
}