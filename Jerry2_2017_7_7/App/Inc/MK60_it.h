#include "include.h"

#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          ���¶����ж�������
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��        #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���      #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003                         ��ȡ��ӳ�䵽�ж�����������жϺ�����ַ�궨��
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 */


extern void PIT0_IRQHandler();
extern void PORTA_IRQHandler();
void PORTE_IRQHandler();
extern int16 speed;
extern int32 RunTime;
extern int32 chaoshengboTime;
extern uint16 ABDistance;
extern uint8 CSB_lost;
extern uint8 CSB_lost_count;
extern uint16 CurrentABDistance;
extern uint16  ABDistance_filter[3];

#endif  //__MK60_IT_H__