/********************************电机驱动**********************************
*
*作者   ： 沙艺已觉
*文件名 ： FuzzySet_Speed.h
*描述   ： 模糊算法设定速度头文件
*时间   ： 2016/4/14
说明    ：参考FreeCars模糊控制例程  使用山外V5.3库
*
****************************************************************************/
#ifndef _FUZZYSET_SPEED_H_
#define _FUZZYSET_SPEED_H_

#include "include.h" 

int16 FuzzySet_Speed(int16 P, int16 D)  ;  /*模糊运算引擎*/

extern int16 *UFF ;

extern int16 UFF1[7] ;
extern int16 UFF2[7] ;
extern int16 UFF3[7] ;
extern int16 UFF4[7] ;
extern int16 UFF5[7] ;
extern int16 UFF6[7] ;

#endif