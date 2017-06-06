#ifndef  _fuzzy_pid_H_
#define  _fuzzy_pid_H_
#include "common.h"
float Fuzzy_Direction_P(float E,float EC);
float Fuzzy_Direction_D(float E,float EC);
extern int16 *UP;
extern int16 *UD;
extern int16 UP1[7];
extern int16 UP2[7];
extern int16 UP3[7];
extern int16 UP4[7];
extern int16 UP5[7];
extern int16 UP6[7];
extern int16 UP7[7];
extern int16 UP8[7];
extern int16 UD1[7];
extern int16 UD2[7];
extern int16 UD3[7];
extern int16 UD4[7];
extern int16 UD5[7];
extern int16 UD6[7];
extern int16 UD7[7];
extern int16 UD8[7];
extern uint8 PEC;
extern uint8 PE;
#endif