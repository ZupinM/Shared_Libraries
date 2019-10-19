#ifndef __FOCUS__
#define __FOCUS__

#define FOCUS_NO_MOVE_TIMEOUT  1 * 60

float focus_angle_A();
float focus_angle_B();
void  focus_reset();
void  focus_start();

void focus_process();
void focus_update();
void focus_init();
int focus_locked();
void focus_restart();
void focus_cancel();
#endif