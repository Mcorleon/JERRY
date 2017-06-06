#ifndef	_MOTOR_H_
#define _MOTOR_H_

 #define Motor_FTM_A   FTM0
 #define Motor_FTM_B   FTM0
 #define Motor_CH_A   FTM_CH4
 #define Motor_CH_B   FTM_CH5
 #define Motor_FREQ    6000
#define Motor_Speed_MAX   8000
#define Motor_Speed_MIN  -8000
extern uint8 Straight_flag; //直道标志 
extern uint8 ruwan_flag;   
extern uint8 wandao_flag; 
extern uint8 stop_flag;
extern uint8 start_flag;
extern uint8 longStraight_flag; //长直道标志
extern uint8 LongS_to_W_flag; //长直道入弯
extern int32 Motor_Duty;
extern void Motor_GetSpeed();
extern void Motor_SetSpeed(int16 speed_duty);
extern void Motor_Control();
extern void CalSpeed();
extern int16  realSpeed;
extern int16 TargetSpeed;
extern int16 ABD_err;
extern uint16 ABDistance_set ;
extern float Motor_P;
extern float Motor_I;
extern float Motor_D;
#endif