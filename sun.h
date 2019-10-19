#ifndef _SUN_
#define _SUN_

#include "rtc.h"

void sun_AziEle(double _longitude, double _latitude, double *azi, double *ele, double *hourAngle, double *declination);

//void        sun_calculate();
time_t *sun_sunrise();
time_t *sun_sunset();
time_t *sun_local_sunrise();
time_t *sun_local_sunset();

void sun_schedule_recalc();
void sun_Update();

#endif