#include <math.h>
#include "suntracer.h"
#include "sun.h"
//#include "rtc.h"



time_t dt_sunrise;
time_t dt_sunset;

time_t dt_loc_sunrise;
time_t dt_loc_sunset;

static const int monthList[12] = { 31,28,31,30,31, 30,31, 31,30, 31,30,31};

int isLeapYear(unsigned int yr){
   return ((yr % 4 == 0 && yr % 100 != 0) || yr % 400 == 0);
}

double getJD()
{
  time_t dt;
  double A,B,JD;
  int docmonth,docday,docyear;
  
  getGMTDateTime(&dt);
  
  docmonth= dt.month;
  docday  = dt.date;
  docyear = dt.year;
  
  if ( (isLeapYear(docyear)) && (docmonth == 2) ) {
    if (docday > 29) {
      docday = 29;
    } 
  } else {
    if (docday > monthList[docmonth-1]) {
      docday = monthList[docmonth-1];
    }
  }
  if (docmonth <= 2) {
    docyear -= 1;
    docmonth += 12;
  }
  A = floor((double)docyear/100);
  B = 2 - A + floor(A/4);
  JD = floor(365.25*((double)docyear + 4716)) + floor(30.6001*((double)docmonth+1)) + (double)docday + B - 1524.5;
  return JD;
}


double getGMTTime()
{
  time_t dt;
  double dochr;
  double docmn;
  double docsc;
  
  getGMTDateTime(&dt);
  
  dochr = dt.hours;
  docmn = dt.minutes;
  docsc = dt.seconds;
 
  return  dochr * 60 + docmn + docsc/60.0;
}



double degToRad(double degrees)
{
	return (degrees * PI / 180);
}

double radToDeg(double radians)
{
	return (radians * 180 / PI);
}

double calcTimeJulianCent(double jd)
{
  double T = (jd - 2451545.0)/36525.0;
  return T;
}


double calcGeomMeanLongSun(double t)
{
  double L0 = 280.46646 + t * (36000.76983 + t*(0.0003032));
  while(L0 > 360.0)
  {
    L0 -= 360.0;
  }
  while(L0 < 0.0)
  {
    L0 += 360.0;
  }
  return L0;		// in degrees
}

double calcGeomMeanAnomalySun(double t)
{
  double M = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  return M;		// in degrees
}

double  calcEccentricityEarthOrbit(double t)
{
  double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  return e;		// unitless
}

double calcSunEqOfCenter(double t)
{
  double m = calcGeomMeanAnomalySun(t);
  double mrad = degToRad(m);
  double sinm = sin(mrad);
  double sin2m = sin(mrad+mrad);
  double sin3m = sin(mrad+mrad+mrad);
  double C = sinm * (1.914602 - t * (0.004817 + 0.000014 * t)) + sin2m * (0.019993 - 0.000101 * t) + sin3m * 0.000289;
  return C;		// in degrees
}

double  calcSunTrueLong(double t)
{
  double  l0 = calcGeomMeanLongSun(t);
  double  c = calcSunEqOfCenter(t);
  double  O = l0 + c;
  return O;		// in degrees
}

double calcSunTrueAnomaly(double t)
{
  double m = calcGeomMeanAnomalySun(t);
  double c = calcSunEqOfCenter(t);
  double v = m + c;
  return v;		// in degrees
}

double calcSunRadVector(double t)
{
  double v = calcSunTrueAnomaly(t);
  double e = calcEccentricityEarthOrbit(t);
  double R = (1.000001018 * (1 - e * e)) / (1 + e * cos(degToRad(v)));
  return R;		// in AUs
}

double calcSunApparentLong(double t)
{
  double o = calcSunTrueLong(t);
  double omega = 125.04 - 1934.136 * t;
  double lambda = o - 0.00569 - 0.00478 * sin(degToRad(omega));
  return lambda;		// in degrees
}

double calcMeanObliquityOfEcliptic(double t)
{
  double seconds = 21.448 - t*(46.8150 + t*(0.00059 - t*(0.001813)));
  double e0 = 23.0 + (26.0 + (seconds/60.0))/60.0;
  return e0;		// in degrees
}

double calcObliquityCorrection(double t)
{
  double e0 = calcMeanObliquityOfEcliptic( t);
  double omega = 125.04 - 1934.136 * t;
  double e = e0 + 0.00256 * cos(degToRad(omega));
  return e;		// in degrees
}

double calcSunRtAscension(double t)
{
  double e = calcObliquityCorrection(t);
  double lambda = calcSunApparentLong(t);
  double tananum = (cos(degToRad(e)) * sin(degToRad(lambda)));
  double tanadenom = (cos(degToRad(lambda)));
  double alpha = radToDeg(atan2(tananum, tanadenom));
  return alpha;		// in degrees
}

double calcSunDeclination(double t)
{
  double e = calcObliquityCorrection(t);
  double lambda = calcSunApparentLong(t);

  double sint = sin(degToRad(e)) * sin(degToRad(lambda));
  double theta = radToDeg(asin(sint));
  return theta;		// in degrees
}

double calcEquationOfTime(double t)
{
  double sin2l0,sinm,cos2l0,sin4l0,sin2m,Etime;  
  
  double epsilon = calcObliquityCorrection(t);
  double l0 = calcGeomMeanLongSun(t);
  double e = calcEccentricityEarthOrbit(t);
  double m = calcGeomMeanAnomalySun(t);

  double y = tan(degToRad(epsilon)/2.0);
  y *= y;

  sin2l0 = sin(2.0 * degToRad(l0));
  sinm   = sin(degToRad(m));
  cos2l0 = cos(2.0 * degToRad(l0));
  sin4l0 = sin(4.0 * degToRad(l0));
  sin2m  = sin(2.0 * degToRad(m));

  Etime = y * sin2l0 - 2.0 * e * sinm + 4.0 * e * y * sinm * cos2l0 - 0.5 * y * y * sin4l0 - 1.25 * e * e * sin2m;
  return radToDeg(Etime)*4.0;	// in minutes of time
}

double calcHourAngleSunrise(double lat, double solarDec)
{
  double  latRad = degToRad(lat);
  double sdRad  = degToRad(solarDec);
  double HAarg = (cos(degToRad(90.833))/(cos(latRad)*cos(sdRad))-tan(latRad) * tan(sdRad));
  if(HAarg >180)HAarg=180;
  if(HAarg<-180)HAarg=-180;
  return  acos(HAarg);
 		// in radians (for sunset, use -HA)
}

double calcSunriseSetUTC(double rise ,double JD, double _latitude, double _longitude)
{
  double delta ,timeUTC;
  
  double t = calcTimeJulianCent(JD);
  double eqTime = calcEquationOfTime(t);
  double solarDec = calcSunDeclination(t);
  double hourAngle = calcHourAngleSunrise(_latitude, solarDec);

  if (!rise) hourAngle = -hourAngle;
  delta = _longitude + radToDeg(hourAngle);
  timeUTC = 720 - (4.0 * delta) - eqTime;	// in minutes
  return timeUTC;
}

// rise = 1 for sunrise, 0 for sunset
double calcSunriseSet(double rise,double JD,double _latitude,double _longitude,double timezone,double dst)
{
  double     newTimeUTC;
  double     timeUTC = calcSunriseSetUTC(rise, JD, _latitude, _longitude);
  if(!isnormal(timeUTC)) return 0;
  newTimeUTC  = calcSunriseSetUTC(rise, JD + timeUTC/1440.0, _latitude, _longitude); 
  if(!isnormal(newTimeUTC)) return 0;
  newTimeUTC += timezone * 60;
  newTimeUTC += dst;

    if ( (newTimeUTC >= 0.0) && (newTimeUTC < 1440.0) ) {
      return newTimeUTC ;//no seconds
    } else  {
       double jday = JD;
       double increment = ((newTimeUTC < 0) ? 1 : -1);
       //while ((newTimeUTC < 0.0)||(newTimeUTC >= 1440.0)) {
       // newTimeUTC += increment * 1440.0;
	//jday -= increment;
      //}
    }

  return newTimeUTC ;
}



time_t *sun_sunrise(){
  return &dt_sunrise;
}

time_t *sun_sunset(){
 return &dt_sunset;
}

time_t *sun_local_sunrise(){
  return &dt_loc_sunrise;
}

time_t *sun_local_sunset(){
 return &dt_loc_sunset;
}


extern double eot_offset;	
void  calcAzEl(double T, double gmttime, double _latitude, double _longitude, double *azi, double *ele, double *hour_Angle,double *declination)
{
  double hourAngle,haRad,csz,zenith,azDenom,azimuth,exoatmElevation,refractionCorrection,solarZen;
  double eqTime = calcEquationOfTime(T);
  double theta  = calcSunDeclination(T);
  
  double solarTimeFix = eqTime + 4.0 * longitude ;//- 60.0 * zone;
  double earthRadVec = calcSunRadVector(T);
  double trueSolarTime = gmttime + solarTimeFix;
  
  eot_offset = eqTime;//for external use
  
  while (trueSolarTime > 1440)
  {
    trueSolarTime -= 1440;
  }
  hourAngle = trueSolarTime / 4.0 - 180.0;
  
  if (hourAngle < -180) 
  {
    hourAngle += 360.0;
  }
  
  if(_latitude < 0){
     _latitude=-_latitude;
     hourAngle=-hourAngle;
     theta=-theta;
  }

   haRad = degToRad(hourAngle);
   csz = sin(degToRad(_latitude)) * sin(degToRad(theta)) + cos(degToRad(_latitude)) * cos(degToRad(theta)) * cos(haRad);
  if (csz > 1.0) 
  {
    csz = 1.0;
  } else if (csz < -1.0) 
  { 
    csz = -1.0;
  } 
   zenith = radToDeg(acos(csz));
   azDenom = ( cos(degToRad(_latitude)) * sin(degToRad(zenith)) );
   azimuth=0;
  if (fabs(azDenom) > 0.001) {
    double azRad = (( sin(degToRad(_latitude)) * cos(degToRad(zenith)) ) - sin(degToRad(theta))) / azDenom;
    if (fabs(azRad) > 1.0) {
      if (azRad < 0) {
	azRad = -1.0;
      } else {
	azRad = 1.0;
      }
    }
    azimuth = 180.0 - radToDeg(acos(azRad));
    if (hourAngle > 0.0) {
      azimuth = -azimuth;
    }
  } else {
    if (_latitude > 0.0) {
      azimuth = 180.0;
    } else { 
      azimuth = 0.0;
    }
  }
  if (azimuth < 0.0) {
    azimuth += 360.0;
  }
   exoatmElevation = 90.0 - zenith;

// Atmospheric Refraction correction

  if (exoatmElevation > 85.0) {
     refractionCorrection = 0.0;
  } else {
    double te = tan (degToRad(exoatmElevation));
    if (exoatmElevation > 5.0) {
       refractionCorrection = 58.1 / te - 0.07 / (te*te*te) + 0.000086 / (te*te*te*te*te);
    } else if (exoatmElevation > -0.575) {
       refractionCorrection = 1735.0 + exoatmElevation * (-518.2 + exoatmElevation * (103.4 + exoatmElevation * (-12.79 + exoatmElevation * 0.711) ) );
    } else {
       refractionCorrection = -20.774 / te;
    }
    refractionCorrection = refractionCorrection / 3600.0;
  }

   solarZen = zenith - refractionCorrection;

  *ele = 90-solarZen;
  *azi = azimuth-180;

  *hour_Angle  = hourAngle;//  (((trueSolarTime*60.0)+(60.0*eqTime))*(0.25/60.0))-180.0;//ales neki spremenjen hour angle
  *declination = degToRad(theta);
}

  
void sun_AziEle(double _longitude, double _latitude, double *azi, double *ele, double *hourAngle, double *declination)  {
  double jday = getJD();
  double tl = getGMTTime();
  double tz =  0;
  //double dst = 0;
  double total = jday + tl/1440.0 - tz/24.0;
  double T = calcTimeJulianCent(total);

  calcAzEl(T, tl, _latitude, _longitude, azi, ele, hourAngle, declination);
}


void sun_calculate(){
  double jday = getJD();
  double tl  = getGMTTime();
  double tz  = longitude / 15.0;//time_zone;
  double dst = time_dst;
  double total = jday + tl/1440.0 - tz/24.0;
  double T   = calcTimeJulianCent(total);
  double lat = latitude;
  double lng = longitude;

  double rise = calcSunriseSet(1, jday, lat, lng, tz, 0);
  double fall = calcSunriseSet(0, jday, lat, lng, tz, 0);

  TicksToDateTime((uint32_t)(rise*60.0), &dt_sunrise);
  TicksToDateTime((uint32_t)(fall*60.0), &dt_sunset);

  Time_add(&dt_sunrise,  (int)(DayMode_time   * 60 * 60));
  Time_add(&dt_sunset,   (int)(NightMode_time * 60 * 60));
  
  
  tz = time_zone;
  rise = calcSunriseSet(1, jday, lat, lng, tz, dst);
  fall = calcSunriseSet(0, jday, lat, lng, tz, dst);
  TicksToDateTime((uint32_t)(rise*60.0), &dt_loc_sunrise);
  TicksToDateTime((uint32_t)(fall*60.0), &dt_loc_sunset);
}


int _sun_update=1;
void sun_schedule_recalc(){
  _sun_update=1;
}

void sun_Update(){
  if(_sun_update){
    sun_calculate() ;
    _sun_update=0;
  }
}