#ifndef  _fuzzy_pid2_H_
#define  _fuzzy_pid2_H_
#include "common.h"
float Fuzzy_Direction_P(float E,float EC);
float Fuzzy_Direction_D(float E,float EC);
int16 FuzzySet_Speed(int16 P, int16 D);

extern int16 *UP;
extern int16 *UD;
extern int16 *UFF;
extern int16 UP0[7];
extern int16 UP1[7];
extern int16 UP2[7];
extern int16 UP3[7];


extern int16 UD0[7];
extern int16 UD1[7];
extern int16 UD2[7];
extern int16 UD3[7];


extern int16 UFF0[7];
extern int16 UFF1[7];
extern int16 UFF2[7];
extern int16 UFF3[7];


#endif