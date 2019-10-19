#include <stdio.h>
#include <stdint.h>
#include "rtc.h"
#include "suntracer.h"
#include "focus.h"
#include "bldc.h"


//astronimical calc
extern double declination;              //deklinacija
extern double hour_angle;               //urni kot
extern double AE_azimuth;               //azimut po AZI/ELE izracunih
extern double AE_elevation;             //elevacija po AZI/ELE izracunih
extern double PM_azimuth;               //azimut po POLAR-MOUNT izracunih
extern double PM_elevation;             //elevacija po POLAR-MOUNT izracunih
extern double HE_azimuth;               //azimut po HELIOSTAT enacbah
extern double HE_elevation;             //elevacija po HELIOSTAT enacbah
extern double HE_AE_azimuth;               //azimut po HELIOSTAT enacbah
extern double HE_AE_elevation;             //elevacija po HELIOSTAT enacbah
extern double angle_A;                  //kot prve (vcasih:horizontalne) osi za prikaz na PCju
extern double angle_B;                  //kot druge (vcasih:vertikalne) osi za prikaz na PCju
extern double show_angle_A;             //kot prve (vcasih:horizontalne) osi za prikaz na PCju
extern double show_angle_B;             //kot druge (vcasih:vertikalne) osi za prikaz na PCju
//u32 day_of_year;
extern double eot_offset;		//"equation of time" offset in minutes
extern double latitude_2;               //samo pozitivne vrednosti (zaradi Južne poloble)

extern uint32_t seconds24;
extern uint32_t last_seconds24;

extern uint32_t buyflags;

extern uint32_t goref_Nday_cnt_B;
extern uint32_t goref_Nday_cnt_A;

extern unsigned int bflags;
extern signed int count_S;

extern time_t time;

extern unsigned int Focus_avg_timeout;
extern unsigned int flags;

extern unsigned char status_homing;


void Recalc_sun();
void init_encoder();
void change_buyFlags(unsigned int newVal);
void cfg_wind_input();
int Snow_Input();
void check_time();
void suns_position_PM (double *elevation, double *azimuth, double ha, double declination);
int anti_shadowing (double *NewAngle, double InputAngle, double space, double wide, char type);
void mirror_equations (double target_azi, double target_ele, double Aos);
void mirror_equationsAE (double target_azi, double target_ele);
double DtoR(double degrees);
double  RtoD(double radians);
double IdealRotationAngle(double azimuthD, double elevationD, double deviationD, double inclinationD);
double CorrectedRotationAngle(double idealRotationAngleD, double panelWidth, double panelSafetyWidth, double trackerDistance);
double coordinate_selection (uint32_t coordinate_mode, int axis);
unsigned int geometry_selection (uint32_t axis, double angle, uint32_t geometry_mode);
double geometry_selection_show (uint32_t axis, uint32_t geometry_mode);
double geometry_1 (int axis ,double angle, double A1, double gear_ratio);
double geometry_1_show (int axis,int32_t position, double A1, double gear_ratio);
double geometry_11(double angle, double A1,double gear_ratio);
double geometry_11_show(int32_t position, double A1, double gear_ratio);
double geometry_2 (double ha, double A1, double A2, double A3, double A4, double B1, double B2, int left, double gear_ratio);
double geometry_2_show(int32_t position ,double A1, double A2, double A3, double A4, double beta1, double beta2, int left,double gear_ratio);
double geometry_3 (double elev, double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio);
double geometry_3_show(int32_t position ,double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio);
double geometry_13(double elev, double A1, double A2, double A3, double A5, double alphaBeta, int bottom,double gear_ratio);
double geometry_13_show(int32_t encPuls ,double A1, double A2, double A3, double A5, double alphaBeta, int bottom, double gear_ratio);

void init_weather_sensor();

