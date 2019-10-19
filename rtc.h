#ifndef __RTC__
#define __RTC__

extern uint32_t cflags;

#define RTC_USE_INTERNAL 1 
#define TIME_TO_TICS(_hour,_min) ((_hour*60*60) + (_min*60))

typedef struct {
  uint32_t seconds;
  uint32_t minutes;
  uint32_t hours;
  uint32_t day;
  uint32_t date;
  uint32_t month;
  uint32_t year;
} time_t;

char isYearNotLeap(uint32_t uiYear);
void getDateTime(time_t *dt);
void getSolarDateTime(time_t *dt);
void getGMTDateTime(time_t *dt);
uint32_t  getSolar_Ticks();
uint32_t  getGMT_Ticks();

uint32_t  DateTimeToTicks(time_t *dt) ;
void TicksToDateTime(uint32_t ulTimeTicks, time_t *dt);

void setSolarDateTime(time_t *dt);
  
void setSolarTimeTics(uint32_t ticks);
void setSolarSeconds(uint32_t sec);
void setSolarMinutes(uint32_t min);
void setSolarHours(uint32_t hours);
void setSolarMday(uint32_t mday);
void setSolarMonth(uint32_t mon);
void setSolarYear(uint32_t year);


void setGMTSeconds(uint32_t sec);
void setGMTMinutes(uint32_t min);
void setGMTHours(uint32_t hours);
void setGMTMday(uint32_t mday);
void setGMTMonth(uint32_t mon);
void setGMTYear(uint32_t year);


void Time_add(time_t *dt,  int sec );
void Time_round(time_t *dt, unsigned int sec );
int Time_higherEq(time_t *dt1, time_t *dt2);
int Time_lower(time_t *dt1, time_t *dt2);

uint32_t TimeToTics(time_t *dt1);
uint32_t TimeStrToTime(char *str, time_t * t);
uint32_t TimeStrToTics(char *str);
uint32_t getdoyFromTics(uint32_t tics);
uint32_t getdoy(time_t *dt);

void rtc_applay_correction(int secsPerDay);
int rtc_find_correction(int secsPerDay);
void RTC_softwareCorrection(int secsPerDay);
void rtc_apply_correction();

void rtc_init(void);
void RTC_SetCounter(uint32_t ticks);
#endif