#include "include.h"
uint8 count;
struct AdData ADdata;
float dianya;

float Steer_P = 9.5 ; //比例
float Steer_D =120;  //微分
float DoubleError;
s16 direction_offset=0;
s16 SteeringZkb=0;
s16 Last_SteeringZkb=0;
uint8 dir_flag;
float  DirectionPianCha[10] = {0.0}; //取三次偏差数组
float ad_max=100 ;
float ad_min=0;
//其中，ad_min 是给定的下限值，ad_max 是实测的上限值。
uint8 circle_flag=0;
uint8 circle_left=0;
uint8 circle_right=0;
uint8 already_in_circle=0;
uint8 dir_change=0;
uint8 circle_ready_tx;
uint8 circle_ready_rx;
uint8 back_car_dir_tx;
uint8 back_car_dir_rx;
uint8  takeoff_over_tx;
uint8  takeoff_over_rx;
uint8  no_takeoff_tx;
uint8  no_takeoff_rx;
uint8  stop_dajiao;
uint8 circle_level=0;
uint8 chaoche=0;
uint8 Ramp_flag=0;
uint8 Ramp_count=0;
int Ramp_time=0;
uint8 Ramp_start=0;
uint8 Ramp_Down_flag=0;



uint16 circle_flag_count=0;

void Read_AD()
{

  for(count = 5 ; count >= 1 ; count --)
  {
     
      //更新ad采样数组
      ADdata.ResultCenter[count] = ADdata.ResultCenter[count-1]; 
  
      ADdata.ResultLeft[count] = ADdata.ResultLeft[count-1];
      
      ADdata.ResultRight[count] = ADdata.ResultRight[count-1];
      
      ADdata.V_Left[count] = ADdata.V_Left[count-1];
      
      ADdata.V_Right[count] = ADdata.V_Right[count-1];
      
      ADdata.V_Mid[count] = ADdata.V_Mid[count-1];
      
  }
   //out6-out1  
  //10 11 13 12 14 15
    
       ADdata.ResultLeft[0] = ad_ave(ADC1_SE10, ADC_8bit,15); //左
       ADdata.ResultCenter[0] = ad_ave(ADC1_SE11, ADC_8bit,15); //  中
       ADdata.ResultRight[0] = ad_ave(ADC1_SE13,ADC_8bit,15); //右

       ADdata.V_Left[0]=ad_ave(ADC1_SE12,ADC_8bit,15);//八字左 
       ADdata.V_Mid[0]=ad_ave(ADC1_SE14,ADC_8bit,15);//八字左 
       ADdata.V_Right[0]=ad_ave(ADC1_SE15,ADC_8bit,15);//八字右
       dianya=ad_ave(ADC0_SE13, ADC_8bit,10)*0.04031749;

     ADdata.ResultCenter[0]=ADdata.ResultCenter[0]*0.7+ADdata.ResultCenter[1]*0.2+ADdata.ResultCenter[2]*0.1;
     ADdata.ResultLeft[0]=ADdata.ResultLeft[0]*0.7+ADdata.ResultLeft[1]*0.2+ADdata.ResultLeft[2]*0.1;
     ADdata.ResultRight[0]=ADdata.ResultRight[0]*0.7+ADdata.ResultRight[1]*0.2+ADdata.ResultRight[2]*0.1;
     ADdata.V_Left[0]=ADdata.V_Left[0]*0.7+ADdata.V_Left[1]*0.2+ADdata.V_Left[2]*0.1;
     ADdata.V_Mid[0]=ADdata.V_Mid[0]*0.7+ADdata.V_Mid[1]*0.2+ADdata.V_Mid[2]*0.1;
     ADdata.V_Right[0]=ADdata.V_Right[0]*0.7+ADdata.V_Right[1]*0.2+ADdata.V_Right[2]*0.1;
     
    if( ADdata.ResultLeft[0]<10) ADdata.ResultLeft[0]=10;
     if(ADdata.ResultCenter[0]<10)ADdata.ResultCenter[0]=10;
      if( ADdata.ResultRight[0]<10) ADdata.ResultRight[0]=10;
       if(ADdata.V_Left[0]<10)ADdata.V_Left[0]=10;
        if(ADdata.V_Mid[0]<10)ADdata.V_Mid[0]=10;
         if( ADdata.V_Right[0]<10) ADdata.V_Right[0]=10;
   

}
void Circle_Cal()
{
  u16 ad_left,ad_right,ad_mid,ad_vl,ad_vm,ad_vr;
   
   ad_left=ADdata.ResultLeft[0];
   ad_mid = ADdata.ResultCenter[0];
   ad_right=ADdata.ResultRight[0];
   ad_vl=ADdata.V_Left[0];
   ad_vm=ADdata.V_Mid[0];
   ad_vr=ADdata.V_Right[0];
   
   //圆环判断
   if(ad_left<=55&&ad_mid<=65&&ad_right<=55&&Ramp_flag==0&&Ramp_time==0)
    {
      if(ad_left>=20&&ad_mid>=20&&ad_right>=20)
      {
        if(ABS(ad_left-ad_mid)<=20&&ABS(ad_right-ad_mid)<=20&&ABS(ad_left-ad_right)<=20)
        {  
           if(ad_vr<=50&&ad_vm<=12&&ad_vl<=50)
           {              
              if(ABS(ad_vr-ad_vl)<=20)
              {
                if(circle_flag==0)
                {
                    circle_flag=1;
                    circle_level++;
                    if((BSP1==1&&circle_level==1)||(BSP2==1&&circle_level==2)||(BSP3==1&&circle_level==3))
                    {
                      chaoche=1;
                    }
                    else
                    {
                     chaoche=0;
                    }
                }
              }
           }
        } 
        
      }
      
    }
  if((ad_mid>=55||ad_vm>=55)&&circle_flag==1&&circle_flag_count>=80)
    {
      circle_flag=0;
      chaoche=0;
      already_in_circle=0;
      circle_flag_count=0;
      dir_change=0;
      circle_right=0;
      circle_left=0;
      circle_ready_tx=0;
      circle_ready_rx=0;
      back_car_dir_tx=0;
      back_car_dir_rx=0;
      if(host_flag==1&&overtake_mode==1&&no_takeoff_rx==0)//后车出去了 变前车
      {
       host_flag=0;
       takeoff_over_tx=1;
      }
       else if(host_flag==1&&overtake_mode==1&&no_takeoff_rx==1)//后车出去了 没变前车
      {
       no_takeoff_rx=0;
      }
      else if(host_flag==2)//前车出去了 变后车
      {
       host_flag=1;
      }
      else if(host_flag==0&&overtake_mode==1)//前车直接过了，后车不超车
      {
       no_takeoff_tx=1;
      }
    }

   if(Ramp_flag==0&&ad_mid<=60&&ad_left<=60&&ad_right<=60)
  {
   Ramp_Down_flag=0;
  }
  if((ad_mid>=120||ad_left>=120||ad_right>=120)&&Ramp_flag==0&&Ramp_Down_flag==0&&DirectionPianCha[0]<=3&&circle_flag==0)//坡道
  {
      Ramp_count++;
  }
  else
  {
    Ramp_count=0;
  }
  if(Ramp_count>=5&&Ramp_flag==0)
  {
   Ramp_flag=1;
   Ramp_start=1;
   Ramp_count=0;
  }
  if(Ramp_flag==1&&ad_mid<=100&&ad_left<=100&&ad_right<=100)
  {
   Ramp_Down_flag=1;
  }
  if(Ramp_Down_flag==1&&(ad_mid>=120||ad_left>=120||ad_right>=120))
  {
    Ramp_flag=0;
  }
  if(Ramp_start==1)//坡道计时，防止下坡判圆环
  {
   Ramp_time++;
   if(Ramp_time>=400)
   {
     Ramp_time=0;
     Ramp_start=0;
   }
  }
 
    if(circle_flag==0)
    {
      if(ad_mid>100)
      {
          DirectionPianCha[0] =0;
      }
      else
      {
          DirectionPianCha[0] = 15.0*ad_mid/ad_max - 15.0;

      }
     
     if(Straight_flag==1)
     {
       DirectionPianCha[0]=0.7*DirectionPianCha[0];
     }
     else if(Ramp_flag==1)
     {
      DirectionPianCha[0]=0.3*DirectionPianCha[0];
     }
    }
   else if(circle_flag==1)
   {
      if(ad_mid>50)
      {
          DirectionPianCha[0] =15.0*ad_mid/ad_max - 15.0;
      }
      else
      {
          DirectionPianCha[0] = 15.0*ad_mid/50 - 15.0;
      }
   }


    //负的左转
}

void CalculateCurrentError()
{
    u16 ad_left,ad_right,ad_mid,ad_vl,ad_vm,ad_vr;
   
   ad_left=ADdata.ResultLeft[0];
   ad_mid = ADdata.ResultCenter[0];
   ad_right=ADdata.ResultRight[0];
   ad_vl=ADdata.V_Left[0];
   ad_vm=ADdata.V_Mid[0];
   ad_vr=ADdata.V_Right[0];


   if(circle_flag==0)
   {
      if(ABS(DirectionPianCha[1])>=7)
     {
        if(DirectionPianCha[1]>0)
        {
          DirectionPianCha[0] = -DirectionPianCha[0];
          dir_flag=1;
        }
       
     }    
     else   if(ad_right>ad_left)
     {
       DirectionPianCha[0] = -DirectionPianCha[0];
       dir_flag=1;
     }
    else   if(ad_right==ad_left&&dir_flag==1)
     {
       DirectionPianCha[0] = -DirectionPianCha[0];
       dir_flag=0;
     }
   }
   
    
   else if(circle_flag==1)//到圆环了
   { 

     if( host_flag==0&&overtake_mode==1&&chaoche==1)//前车通知后车有圆环
     {
      circle_ready_tx=1;
      takeoff_over_tx=0;
      no_takeoff_tx=0;
     }
     else if(host_flag==1&&overtake_mode==1&&chaoche==1)
     {
       if(back_car_dir_rx==1)
       {
        circle_left=1;
       }
       else if(back_car_dir_rx==2)
       {
        circle_right=1; 
       }
     }
     
   
     
     if(circle_left==0&&circle_right==0&& host_flag==0) //判断姿态
     {
       if(DirectionPianCha[1]>0)
       {
          circle_right=1;    //前车右过 通知后车左过
          if(chaoche==1&&overtake_mode==1)
          back_car_dir_tx=1;
       }
       else if(DirectionPianCha[1]<0)
       {
          circle_left=1;          //前车左过 通知后车右过
          if(chaoche==1&&overtake_mode==1)
          back_car_dir_tx=2;
       }
     }
     else  if(circle_left==0&&circle_right==0&& host_flag==1&&(overtake_mode==0||chaoche==0||no_takeoff_rx==1)) //后车在不超车模式下判断姿态
     {
       if(DirectionPianCha[1]>0)
       {
          circle_right=1;
          back_car_dir_rx=0;
       }
       else if(DirectionPianCha[1]<0)
       {
          circle_left=1;
          back_car_dir_rx=0;          
       }
     }
          circle_flag_count++;//圆环计数 5MS一次
          if(circle_flag_count>30000)
            circle_flag_count=30000;
          
    if(circle_left==1||back_car_dir_rx==1)//左路过圆环
    {
      if(chaoche==1&&overtake_mode==1&&host_flag==0)
       back_car_dir_tx=2;
       if(circle_flag_count<=50) 
       {
          DirectionPianCha[0] = DirectionPianCha[0] ;
       }   
       else if(circle_flag_count>50)
       {
          
           if(ad_right>=ad_left&&dir_change==0)
           {
            dir_change=2;
             if( host_flag==0&&overtake_mode==1&&chaoche==1)//前车停下 超车准备
            {
             stop_flag=1;
             host_flag=2;
            }
           }
         
        if(dir_change==2)
        {
          
//           if(circle_flag_count>=100)
//          {
//            if(ad_right>=ad_left)
//            {
//              DirectionPianCha[0] = -DirectionPianCha[0];
//            }
//            else   if(ad_left>ad_right)
//            {
//              DirectionPianCha[0] = DirectionPianCha[0];
//            }
//          }
//          else if(circle_flag_count<100)
//          {
           DirectionPianCha[0] = -DirectionPianCha[0] ;
         // }
        }
        else if(dir_change==0)
          {
             DirectionPianCha[0] = DirectionPianCha[0] ;
          }
    }
   }
    else if(circle_right==1||back_car_dir_rx==2)//右路过圆环
    {
      if(chaoche==1&&overtake_mode==1&&host_flag==0)
       back_car_dir_tx=1;
         if(circle_flag_count<=50) 
       {
          DirectionPianCha[0] = -DirectionPianCha[0];
       }   
       else if(circle_flag_count>50)
       {
           
           if(ad_left>=ad_right&&dir_change==0)
           {
            dir_change=1;
           if( host_flag==0&&overtake_mode==1&&chaoche==1)//前车停下 超车准备
            {
             stop_flag=1;
             host_flag=2;
            }
           }
         
        if(dir_change==1)
         {
          
//           if(circle_flag_count>=100)
//           {
//             if(ad_left>=ad_right)
//            {
//              DirectionPianCha[0] = DirectionPianCha[0];
//            }
//            else   if(ad_right>ad_left)
//            {
//              DirectionPianCha[0] = -DirectionPianCha[0];
//            }
//           }
//           else if(circle_flag_count<100)
//           {
            DirectionPianCha[0] = DirectionPianCha[0];
           //}
         }
          else if(dir_change==0)
          {
             DirectionPianCha[0] =-DirectionPianCha[0];
          }
       
      }
    }
    
   }
    
}

    
   
  
    

void DirectionControl()
{
    
     
     DoubleError = DirectionPianCha[0] - DirectionPianCha[1];

      if(DoubleError>0.65)
      {
        DoubleError=0.65;
      }
      else if(DoubleError<-0.65)
      {
        DoubleError=-0.65;
      }
       DirectionPianCha[0]=DirectionPianCha[1]+DoubleError;
  

     DirectionPianCha[3] = DirectionPianCha[2];
     DirectionPianCha[2] = DirectionPianCha[1];        
     DirectionPianCha[1] = DirectionPianCha[0];
     
     UP = UP2;
     UD = UD3;
     Steer_P= (Fuzzy_Direction_P(ABS(DirectionPianCha[0]*10),ABS(DoubleError*100)))/100.0;
     //Steer_D= Fuzzy_Direction_D(ABS(DirectionPianCha[0]*10),ABS(DoubleError*100));

 
     
     SteeringZkb =SD5_mid + (s16)((Steer_P*DirectionPianCha[0])+Steer_D*(DoubleError));
    direction_offset=(s16)((Steer_P*DirectionPianCha[0])+Steer_D*(DoubleError));
    
     if(SteeringZkb>SD5_Rmax)   //舵机打角限幅
        SteeringZkb = SD5_Rmax;
     else  if(SteeringZkb<SD5_Lmax)
        SteeringZkb = SD5_Lmax;
     

     ftm_pwm_duty(SD5_FTM,SD5_CH, SteeringZkb);   
    
}