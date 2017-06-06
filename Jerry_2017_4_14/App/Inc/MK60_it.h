#include "include.h"

#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          重新定义中断向量表
 *  先取消默认的中断向量元素宏定义        #undef  VECTOR_xxx
 *  在重新定义到自己编写的中断函数      #define VECTOR_xxx    xxx_IRQHandler
 *  例如：
 *       #undef  VECTOR_003                         先取消映射到中断向量表里的中断函数地址宏定义
 *       #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
 */


extern void PIT0_IRQHandler();
extern void PORTA_IRQHandler();
void PORTE_IRQHandler();
extern int16 speed;
extern int32 chaoshengboTime;
extern uint16 ABDistance;
extern uint8 CSB_lost;
extern uint8 CSB_lost_count;
extern uint16 CurrentABDistance;
extern uint16  ABDistance_filter[3];

#endif  //__MK60_IT_H__