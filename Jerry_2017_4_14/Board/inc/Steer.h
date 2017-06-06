#ifndef	_STEER_H_
#define _STEER_H_


#define   SD5_FREQ    300       //�������Ƶ��
#define   SD5_FTM     FTM2       //FTMģ��
#define   SD5_CH      FTM_CH0    //ͨ�� 
#define   SD5_mid     442   //��ֵ   434   440
#define   SD5_Lmax    345   // ����  354   360
#define   SD5_Rmax    540   // ����  514   520

typedef struct AdData{

  
   uint8 ResultCenter[6];
   
   uint8 ResultLeft[6];
   
   uint8 ResultRight[6];
   
   uint8 V_Left[6];
   
    uint8 V_Mid[6];
   
   uint8 V_Right[6];
    
   uint16 ResultCenter_ToOne;
   
   uint16 ResultLeft_ToOne;
   
   uint16 ResultRight_ToOne;
   
   uint16 Bazi_Left_ToOne;
   
   uint16 Bazi_Right_ToOne;
   
   //����ADƫ��
   float L_C_CHA;

   //���AD��
   float L_C_HE;
  
   float R_C_CHA;
   
   float R_C_HE;
   
   float L_R_CHA;
   
   float L_R_HE;
   
   float VerL_C_HE;
   
   float VerL_C_CHA;
   
   float VerR_C_HE;
   
   float VerR_C_CHA;
   
   float CurrentError;
   

}AdData;
extern struct AdData ADdata;
extern uint8 count;
extern float dianya;
extern void Read_AD();
extern void CalculateCurrentError();
extern void DirectionControl();
extern void Sensor_To_One();
extern float Steer_P ;    //����
extern float Steer_D ;  //΢��
extern float  DirectionPianCha[10] ;
extern float DoubleError;
extern s16 direction_offset;
extern uint16 circle_flag_count;
extern uint8 circle_flag;
extern uint8 dir_change;
extern uint8 dir_flag;
extern uint8 circle_ready_tx;
extern uint8 circle_ready_rx;
extern uint8 back_car_dir_tx;
extern uint8 back_car_dir_rx;
extern uint8  takeoff_over_tx;
extern uint8  takeoff_over_rx;
extern uint8  no_takeoff_tx;
extern uint8  no_takeoff_rx;
extern uint8  stop_dajiao;
#endif