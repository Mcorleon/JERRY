#include "include.h"


int16 realSpeed=0;
int16 realspeed[3]={0};
int16 TargetSpeed=0;
int32 Motor_Duty=0;

 int16 speed_save[10];
int16  Speed_err[3];
float Track_complexity ;                //赛道复杂程度
float Prospect_See ;                    //前瞻 
uint8 zhidao_num;
uint8 BBcontrol;
uint8 Straight_flag; //直道标志 
uint8 ruwan_flag;   
uint8 wandao_flag; 
uint8 longStraight_flag; //长直道标志
uint8 LongS_to_W_flag; //长直道入弯
int16 speed_filter_error[3];
uint8 stop_flag=0;
uint8 start_flag=0;
u16 W_time = 0;                       //弯道时间
uint8 Instraightjishu;
uint16 ABDistance_set =450;
int16 ABD_err=0;
int16 ABD_last_err=0;
int16 ABD_err_err=0;
uint8 CSB_lost_count=0;
uint8 CSB_lost=0;
float ABD_p = 0.2;
float ABD_d = 0.0;
int16 Maxspeed =0;
int16 Minspeed =0;
float Motor_P=100;//12
float Motor_I=80;//18
float Motor_D=0;
void Motor_SetSpeed(int16 speed_duty)
{    
  
   if(speed_duty>=0)    
    {
     ftm_pwm_duty(Motor_FTM_A,Motor_CH_A,speed_duty);
     ftm_pwm_duty(Motor_FTM_B,Motor_CH_B,0);      
    }
   else
   {        
     ftm_pwm_duty(Motor_FTM_A,Motor_CH_A,0);
     ftm_pwm_duty(Motor_FTM_B,Motor_CH_B,-speed_duty);          
                
   }

}

void Motor_GetSpeed()
{
   int a,xishu;
   int16 speed_error;
   realSpeed=ftm_quad_get(FTM1);
   ftm_quad_clean(FTM1);
   ftm_quad_clean(FTM1);
   
     speed_error =realSpeed - speed_save[0];
  
  xishu = 80+realSpeed/10;
  
  a = speed_error/xishu ;                //a是系数，
  if(a < 0)
    a = -a;
  if(a > 0.85)
    a = 0.85 ;


  //速度滤波，十阶滞后滤波算法
 realSpeed = realSpeed* (1-a) + (speed_save[1]+speed_save[2]+speed_save[3]+speed_save[4]+speed_save[5]+speed_save[6]+speed_save[7]+speed_save[8]+speed_save[9])*a/9;// car.speed_new * 0.1 +  
  //car.speed_new = (car.speed_new+car.speed_save[0]+car.speed_save[1])/3;
  speed_save[9] = speed_save[8] ;
  speed_save[8] = speed_save[7] ;
  speed_save[7] = speed_save[6] ;
  speed_save[6] = speed_save[5] ;
  speed_save[5] = speed_save[4] ;
  speed_save[4] = speed_save[3] ;
  speed_save[3] = speed_save[2] ;
  speed_save[2] = speed_save[1] ;
  speed_save[1] = speed_save[0] ;
  speed_save[0] = realSpeed ;

//   realSpeed=realSpeed*0.7+realspeed[0]*0.2+realspeed[1]*0.1;
//
//   realspeed[1]= realspeed[0];
//   realspeed[0]= realSpeed;
}




void Motor_Control()
{
   if(ADdata.ResultCenter[0]>=80&&Ramp_flag==0) 
   {
      zhidao_num++;
      if(zhidao_num>30)
      zhidao_num=30;
   }
   else  
   {
     zhidao_num=0; 
     Straight_flag=0;
   }
   if(zhidao_num>=8 && Straight_flag==0) 
   {       
     Straight_flag=1; //直道标志 
   }   
  
  if(host_flag == 0)//前车
  {
//        if(BSP1==1)
//        {
//           UFF=UFF0;
//        }
//        else if(BSP2==1)
//        {
//          UFF=UFF1;
//        }
//        else if(BSP3==1)
//        {
//          UFF=UFF2;
//        }
//        else if(BSP4==1)
//        {
//          UFF=UFF3; 
//        }
//         else if(BSP5==1)
//        {
//          UFF=UFF4; 
//        }
       
       UFF=UFF1;
       Track_complexity=ABS(DirectionPianCha[0]*10) ;                //赛道复杂程度
       Prospect_See =ABS(DoubleError*100);  
       TargetSpeed = FuzzySet_Speed(Track_complexity,Prospect_See);
  }
  else if(host_flag==1)//后车
  {
       Track_complexity=ABS(DirectionPianCha[0]*10) ;                //赛道复杂程度
       Prospect_See =ABS(DoubleError*100); 
       UFF=UFF1;
        TargetSpeed = FuzzySet_Speed(Track_complexity,Prospect_See);
        UFF=UFF2;
        Maxspeed =FuzzySet_Speed(Track_complexity,Prospect_See);
         Minspeed =65;
//        UFF=UFF0;
//        Minspeed =FuzzySet_Speed(Track_complexity,Prospect_See);
     
    if(ABDistance>1200)
      ABDistance = CurrentABDistance;
    if(ABDistance<0)
      ABDistance=0;
    
    ABDistance_filter[2] = ABDistance_filter[1];
    ABDistance_filter[1] = ABDistance_filter[0];
    ABDistance_filter[0] = ABDistance;
    ABDistance = (uint16)(ABDistance_filter[0]*0.6+ABDistance_filter[1]*0.3+ABDistance_filter[2]*0.1);
   
    CurrentABDistance = ABDistance;
     if((CurrentABDistance==ABDistance_filter[1]&&ABDistance==ABDistance_filter[2]))
    {
      CSB_lost_count++;
    }
    else
    {
      CSB_lost_count=0;
      CSB_lost=0;
    }
    if(CSB_lost_count>=8)
    {
      CSB_lost_count=8;
      CSB_lost=1;
    }
  }
  
  if(host_flag==1&&CSB_lost!=1)//后车控距
  {
   // if((ABDistance<ABDistance_set)||(ABDistance>ABDistance_set+100))
    {
       ABD_err = ABDistance - ABDistance_set;
       ABD_err_err = ABD_err - ABD_last_err;  
        TargetSpeed += ABD_p*(ABDistance - ABDistance_set)+ABD_d*ABD_err_err;
       ABD_last_err = ABD_err;
              
      if(TargetSpeed>Maxspeed) TargetSpeed=Maxspeed;
      else if(TargetSpeed<Minspeed) TargetSpeed=Minspeed;
    }
  } 

     
       if(circle_ready_rx==1&&circle_flag==0&&host_flag==1)
         TargetSpeed=60;
       else if(circle_flag==1&&dir_change==0)
         TargetSpeed=60;
       else if(circle_flag==1&&dir_change!=0)
         TargetSpeed=75;
       
       if(Ramp_flag==1)
         TargetSpeed=60;
       
       if(takeoff_over_rx==1&&host_flag==2)//超车完成了
         {
          stop_flag=0;
          takeoff_over_rx=0;
         }
      if(stop_flag==1)    
        TargetSpeed=0;
  
//   speed_filter_error[2] = speed_filter_error[1];
//   speed_filter_error[1] = speed_filter_error[0];
//   speed_filter_error[0] = TargetSpeed;
//   TargetSpeed = (int16)(0.7*speed_filter_error[0]+0.2*speed_filter_error[1]+0.1*speed_filter_error[2]);
  
  Speed_err[2]=Speed_err[1];
  Speed_err[1]=Speed_err[0];
  Speed_err[0]=TargetSpeed-(int16)(realSpeed);
  
  if( Speed_err[0]<0)
     Speed_err[0]=2*Speed_err[0];
  
  Motor_Duty= Motor_Duty+ (int32)(Motor_P*(Speed_err[0]-Speed_err[1])+Motor_I*Speed_err[0]+Motor_D*(Speed_err[0]-2*Speed_err[1]+Speed_err[2]));


    if(Motor_Duty>Motor_Speed_MAX)Motor_Duty=Motor_Speed_MAX;
    if(Motor_Duty<Motor_Speed_MIN) Motor_Duty=Motor_Speed_MIN;
    
  if(start_flag==0)
     Motor_Duty=0;
  
    Motor_SetSpeed(Motor_Duty);
}
