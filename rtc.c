
#include "LPC15xx.h"                        /* LPC11xx definitions */
#include <stdio.h>
#include "../main.h"
#include "rtc.h"

#include "suntracer.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "suntracer.h"

#define RTC_MODE_SOFT 1
#define RTC_MODE_HARD 0

volatile uint32_t rtc_ticks;


int rtcmode = RTC_MODE_HARD;

typedef struct {
  uint32_t    ticks;
  time_t time;
}time_cache;

time_cache GMTTime;
time_cache SolarTime;

time_t tmpt;

char m_chMonthTable[] = {
  0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

void rtc_init(void){

  LPC_SYSCON->SYSAHBCLKCTRL0 |= 1<<23; //enable clock to RTC

  LPC_RTC->CTRL &= ~(1<<0); //disable reset of RTC
  LPC_RTC->CTRL |=   1<<7;  //enable counters

  LPC_RTC->CTRL |= 1<<4; //enable alarm dp

  rtc_ticks = LPC_RTC->COUNT;

  //LPC_RTC->MATCH = rtc_ticks + 1;

  //NVIC_EnableIRQ(RTC_ALARM_IRQn);

}

//void RTC_softwareCorrection(int secsPerDay){
//  float add;
//  int val;
//  
//  add = (float)secsPerDay * 0.555549;
//  add = -add;
//  val= 48000 + (int)add;
//  
//  if(val > 65535) val = 65535;
//  if(val<1000)    val = 1000;   
//    
//  TIM3->ARR = (uint16_t)val;
//}



char isYearNotLeap(uint32_t uiYear) {
  if(!(uiYear % 100)) {
    return(uiYear % 400);
  }
  return(uiYear % 4);
}


void TicksToDateTime(uint32_t ulTimeTicks, time_t *dt){
unsigned int uiDateYear;
  unsigned char ucDateMonth;
  unsigned long long ulTimeTicksTmp;
  unsigned long long ulTimeTicksMem;
  

  ucDateMonth = 1;
  uiDateYear = 1970;
  
  ulTimeTicksTmp = 0;
  
  /* find year */  
  while(ulTimeTicksTmp < ulTimeTicks) {
    ulTimeTicksMem = ulTimeTicksTmp;

    ulTimeTicksTmp += (365*24*60*60);

    if(!isYearNotLeap(uiDateYear++)){
      ulTimeTicksTmp += (24*60*60);
    }
  }

  if(uiDateYear > 1970) {
    uiDateYear -= 1;
  }

  ulTimeTicks -= ulTimeTicksMem;
  ulTimeTicksTmp = 0;

  /* find month */
  while(ulTimeTicksTmp <= ulTimeTicks) {
    ulTimeTicksMem = ulTimeTicksTmp;

    ulTimeTicksTmp += (m_chMonthTable[ucDateMonth]*24*60*60);

    if(ucDateMonth == 2) {
      if(!isYearNotLeap(uiDateYear)) {
        ulTimeTicksTmp += (24*60*60);
      }
    }

    ucDateMonth += 1;
  }

  if(ucDateMonth > 1) {
    ucDateMonth -= 1;
  }

  /* find day */
  ulTimeTicks -= ulTimeTicksMem;

  dt->date = (unsigned char)((ulTimeTicks/24/60/60)+1);
  dt->month = ucDateMonth;
  dt->year = (unsigned short)uiDateYear;
  
  dt->hours = (unsigned char)((ulTimeTicks/3600)%24);
  dt->minutes = (unsigned char)((ulTimeTicks/60)%60);
  dt->seconds = (unsigned char)(ulTimeTicks%60);
}


uint32_t DateTimeToTicks(time_t *dt) {
  unsigned int uiDateYear;
  unsigned char ucDateMonth;
  
  unsigned long ulElapsedDays = 0;
  uint32_t Ticks;

  for(ucDateMonth = 1; ucDateMonth < dt->month; ucDateMonth++) {
    ulElapsedDays += m_chMonthTable[ucDateMonth];

    if(ucDateMonth == 2) {
      if(!isYearNotLeap(dt->year)) {
        ulElapsedDays += 1;
      }
    }
  }

  ulElapsedDays += (unsigned long)(dt->date-1);
  ulElapsedDays += ((dt->year-1970)*365);

  for(uiDateYear = 1970; uiDateYear < dt->year; uiDateYear++) {
    if(!isYearNotLeap(uiDateYear)) {
      ulElapsedDays += 1;
    }
  }

  Ticks =  (unsigned long)dt->seconds;
  Ticks += ((unsigned long)dt->minutes*60);
  Ticks += ((unsigned long)dt->hours*60*60);
  Ticks += (ulElapsedDays*24*60*60);
  return Ticks;
}


uint32_t getSolar_Ticks(){
  if(!(cflags&(1<<RunSoftwareRTC)))  return LPC_RTC->COUNT  + (int32_t)(longitude*240);
  else                               return LPC_RTC->COUNT  + (int32_t)(longitude*240);
}

uint32_t getGMT_Ticks(){
  if(!(cflags&(1<<RunSoftwareRTC)))  return LPC_RTC->COUNT;
  else                               return LPC_RTC->COUNT;
}


void getSolarDateTime(time_t *dt){
  uint32_t ticks = getSolar_Ticks();
  if(SolarTime.ticks == ticks){
    memcpy(dt, &SolarTime.time, sizeof(time_t));
    return;
  }
  TicksToDateTime(ticks,dt);
  memcpy(&SolarTime.time,dt,sizeof(time_t));
  SolarTime.ticks=ticks;
}

void setSolarDateTime(time_t *dt){
  uint32_t ticks =  DateTimeToTicks(dt) - (int32_t)(longitude*240);
  NVIC_DisableIRQ (RTC_ALARM_IRQn); 
  //if(RTC_WaitForLastTask(1))
      RTC_SetCounter(ticks);    //vpis v RTC
  rtc_ticks=ticks;
  NVIC_EnableIRQ (RTC_ALARM_IRQn); 
}

void setSolarTimeTics(uint32_t ticks){
  ticks -= - (int32_t)(longitude*240);
  NVIC_DisableIRQ (RTC_ALARM_IRQn); 
  //if(RTC_WaitForLastTask(1))
      RTC_SetCounter(ticks);    //vpis v RTC
  rtc_ticks=ticks;
  NVIC_EnableIRQ (RTC_ALARM_IRQn); 
}

void setGMTDateTime(time_t *dt){
  uint32_t ticks =  DateTimeToTicks(dt);
  NVIC_DisableIRQ (RTC_ALARM_IRQn); 
  //if(RTC_WaitForLastTask(1))
      RTC_SetCounter(ticks);    //vpis v RTC
  rtc_ticks=ticks;
  NVIC_EnableIRQ (RTC_ALARM_IRQn); 
}


void getGMTDateTime(time_t *dt){
   uint32_t ticks = getGMT_Ticks();
   if( GMTTime.ticks == ticks){
    memcpy(dt, &GMTTime.time, sizeof(time_t));
    return;
  }
  TicksToDateTime(ticks,dt);
  memcpy(&GMTTime.time,dt, sizeof(time_t));
  GMTTime.ticks=ticks;
}

void setSolarSeconds(uint32_t sec){
  getSolarDateTime(&tmpt);
  if(sec>59) return;
  tmpt.seconds = sec;
  setSolarDateTime(&tmpt);
}

void setSolarMinutes(uint32_t min){
  getSolarDateTime(&tmpt);
  if(min>59) return;
  tmpt.minutes = min;
  setSolarDateTime(&tmpt);
}

void setSolarHours(uint32_t hours){
  getSolarDateTime(&tmpt);
  if(hours > 23)return;
  tmpt.hours = hours;
  setSolarDateTime(&tmpt);
}

void setSolarMday(uint32_t mday){
  getSolarDateTime(&tmpt);
  if(mday>31) return;
  tmpt.date = mday;
  setSolarDateTime(&tmpt);
}

void setSolarMonth(uint32_t mon){
  getSolarDateTime(&tmpt);
  if(mon>12) return;
  tmpt.month = mon;
  setSolarDateTime(&tmpt);
}

void setSolarYear(uint32_t year){
  getSolarDateTime(&tmpt);
  if(year<2000)return;
  tmpt.year = year;
  setSolarDateTime(&tmpt);
}

void setGMTSeconds(uint32_t sec){
  getGMTDateTime(&tmpt);
  if(sec>59) return;
  tmpt.seconds = sec;
  setGMTDateTime(&tmpt);
}

void setGMTMinutes(uint32_t min){
  getGMTDateTime(&tmpt);
  if(min>59) return;
  tmpt.minutes = min;
  setGMTDateTime(&tmpt);
}

void setGMTHours(uint32_t hours){
  getGMTDateTime(&tmpt);
  if(hours > 23)return;
  tmpt.hours = hours;
  setGMTDateTime(&tmpt);
}

void setGMTMday(uint32_t mday){
  getGMTDateTime(&tmpt);
  if(mday>31) return;
  tmpt.date = mday;
  setGMTDateTime(&tmpt);
}

void setGMTMonth(uint32_t mon){
  getGMTDateTime(&tmpt);
  if(mon>12) return;
  tmpt.month = mon;
  setGMTDateTime(&tmpt);
}

void setGMTYear(uint32_t year){
  getGMTDateTime(&tmpt);
  if(year<2000)return;
  tmpt.year = year;
  setGMTDateTime(&tmpt);
}

void Time_add(time_t *dt,  int sec ){
  uint32_t newseconds = DateTimeToTicks(dt);
  newseconds+=sec;
  TicksToDateTime(newseconds, dt);
}

void Time_round(time_t *dt, unsigned int sec ){
  uint32_t newseconds;
  newseconds = DateTimeToTicks(dt);
  newseconds -= newseconds%sec;
  TicksToDateTime(newseconds, dt);
}

int Time_higherEq(time_t *dt1, time_t *dt2){
  unsigned int time1=0;
  unsigned int time2=0;  
  time1+=dt1->hours*60*60;
  time1+=dt1->minutes *60;
  time1+=dt1->seconds;

  time2+=dt2->hours*60*60;
  time2+=dt2->minutes *60;
  time2+=dt2->seconds;
  
 if(time1>=time2) return 1;
 return 0;
}

int Time_lower(time_t *dt1, time_t *dt2){
  unsigned int time1=0;
  unsigned int time2=0;  
  time1+=dt1->hours*60*60;
  time1+=dt1->minutes *60;
  time1+=dt1->seconds;

  time2+=dt2->hours*60*60;
  time2+=dt2->minutes *60;
  time2+=dt2->seconds;
  
 if(time1<time2) return 1;
 return 0;
}

uint32_t TimeToTics(time_t *dt1){
  unsigned int tics=0;  
  tics+=dt1->hours*60*60;
  tics+=dt1->minutes *60;
  tics+=dt1->seconds *60;
  return tics;
}

uint32_t TimeStrToTime(char *str, time_t * t){
  int hours;
  int minutes;
  if(strlen(str)<5) return 0 ;
  if(str[2]!=':') return 0;
  hours   =atoi(str);
  minutes =atoi(&str[3]);
  
  if(hours < 0)   hours=0;
  if(minutes < 0) minutes=0;
  
  if(hours > 23)   hours=0;
  if(minutes > 59) minutes=0;
    
  t->hours   = hours;
  t->minutes = minutes;
  t->seconds = 0;
  return 1;
}

uint32_t TimeStrToTics(char *str){
  int hours;
  int minutes;
  if(strlen(str)<5) return 0 ;
  if(str[2]!=':') return 0;
 
  hours   = atoi(str);
  minutes = atoi(&str[3]);
  
  if(hours < 0)   hours=0;
  if(minutes < 0) minutes=0;
  
  if(hours > 23)   hours=0;
  if(minutes > 59) minutes=0;
  
  return (hours*60*60) + (minutes*60);
}



int rtc_find_correction(int secsPerDay){//return nearest prescaler difference
  int val = abs(secsPerDay);
  
  val = (int)((float)val * 0.3792548);
  
  if(val > 32000)  val = 32000;
  if(val<0)       val = 0;  
  
  if(secsPerDay<0) return val;
  return -val;
}


void rtc_apply_correction(){
  uint8_t correction_min = 0;
  uint8_t correction_sec = 0;
  if(rtc_correction > 59){
    correction_min = rtc_correction / 60;
    correction_sec = (int)rtc_correction % 60;
  }else 
  correction_sec = rtc_correction;

  getSolarDateTime(&tmpt);
  tmpt.seconds += correction_sec;
  tmpt.minutes += correction_min;
  setSolarDateTime(&tmpt);

  getGMTDateTime(&tmpt);
  tmpt.seconds += correction_sec;
  tmpt.minutes += correction_min;
  //setGMTDateTime(&tmpt);

}

uint32_t getdoy(time_t *dt)
{
  int ucMonthCount;
  unsigned int uiDayCount = 0;
  
  for( ucMonthCount = 1; ucMonthCount < dt->month; ucMonthCount++) {
    uiDayCount += m_chMonthTable[ucMonthCount];
  }
  
  uiDayCount += dt->date;

  if(!isYearNotLeap(dt->year)) {
    uiDayCount += 1;
  }
  return uiDayCount;
}

uint32_t getdoyFromTics(uint32_t tics){
  time_t dt;
  TicksToDateTime(tics, &dt);
  return getdoy(&dt);
}

//software rtc timer

void RTC_ALARM_IRQHandler(void){
    //rtc_ticks++;
    if(cflags & (1<<RunSoftwareRTC)){
      if(0)
      RTC_SetCounter(rtc_ticks);             //update RTC if rtc running
    }else{
      rtc_ticks = LPC_RTC->COUNT;
    }
    //LPC_RTC->MATCH = rtc_ticks + 1;
  LPC_RTC->CTRL |= 1<<2;                  // clear UIF flag
}


void RTC_SetCounter(uint32_t ticks){

  LPC_RTC->CTRL &=  ~(1<<7);  //disable counters
  
  LPC_RTC->COUNT = ticks;

  LPC_RTC->CTRL |=   1<<7;  //enable counters
  
}

