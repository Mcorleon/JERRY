#include "include.h"
uint8 count;
struct AdData ADdata;
float dianya;

float Steer_P = 9.5 ; //����
float Steer_D =120;  //΢��
float DoubleError;
s16 direction_offset=0;
s16 SteeringZkb=0;
s16 Last_SteeringZkb=0;
uint8 dir_flag;
float  DirectionPianCha[10] = {0.0}; //ȡ����ƫ������
float ad_max=100 ;
float ad_min=0;
//���У�ad_min �Ǹ���������ֵ��ad_max ��ʵ�������ֵ��
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
uint8 Ramp_start=0;
uint8 circle_start=0;
int Ramp_time=0;
int circle_time=0;
uint8 Ramp_Down_flag=0;



uint16 circle_flag_count=0;

void Read_AD()
{

  for(count = 5 ; count >= 1 ; count --)
  {
     
      //����ad��������
      ADdata.ResultCenter[count] = ADdata.ResultCenter[count-1]; 
  
      ADdata.ResultLeft[count] = ADdata.ResultLeft[count-1];
      
      ADdata.ResultRight[count] = ADdata.ResultRight[count-1];
      
      ADdata.V_Left[count] = ADdata.V_Left[count-1];
      
      ADdata.V_Right[count] = ADdata.V_Right[count-1];
      
      ADdata.V_Mid[count] = ADdata.V_Mid[count-1];
      
  }
   //out6-out1  
  //10 11 13 12 14 15
    
       ADdata.ResultLeft[0] = ad_ave(ADC1_SE10, ADC_8bit,15); //��
       ADdata.ResultCenter[0] = ad_ave(ADC1_SE11, ADC_8bit,15); //  ��
       ADdata.ResultRight[0] = ad_ave(ADC1_SE13,ADC_8bit,15); //��

       ADdata.V_Left[0]=ad_ave(ADC1_SE12,ADC_8bit,15);//������ 
       ADdata.V_Mid[0]=ad_ave(ADC1_SE14,ADC_8bit,15);//������ 
       ADdata.V_Right[0]=ad_ave(ADC1_SE15,ADC_8bit,15);//������
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
   
   //Բ���ж�
//   if(ad_left<=50&&ad_mid<=60&&ad_right<=50&&Ramp_flag==0&&Ramp_time==0&&RunTime>0)
//    {
//      if(ad_left>=20&&ad_mid>=30&&ad_right>=20)
//      {
//        if(ABS(ad_left-ad_mid)<=18&&ABS(ad_right-ad_mid)<=18&&ABS(ad_left-ad_right)<=18)
//        {  
//           if(ad_vr<=40&&ad_vm<=15&&ad_vl<=40)
//           {              
//              if(ABS(ad_vr-ad_vl)<=20)
//              {
//                if(circle_flag==0)
//                {
//                    circle_flag=1;
//                    circle_level++;
//                    if((BSP1==1&&circle_level==1)||(BSP2==1&&circle_level==2)||(BSP3==1&&circle_level==3))
//                    {
//                      chaoche=1;//�ò��뿪��ѡ���ļ���Բ������
//                    }
//                    else
//                    {
//                     chaoche=0;
//                    }
//                }
//              }
//           }
//        } 
//        
//      }
//      
//    }
    if(ad_left<=40&&ad_mid<=50&&ad_right<=40&&Ramp_flag==0&&Ramp_time==0)
    {
      if(ad_left>=20&&ad_mid>=25&&ad_right>=20)
      {    
           if(ad_vr<=40&&ad_vm<=14&&ad_vl<=40)
           {    
             
                if(circle_flag==0)
                {
                    circle_flag=1;
                    circle_start=1;
                    circle_level++;
                    if((BSP1==1&&circle_level==1)||(BSP2==1&&circle_level==2)||(BSP3==1&&circle_level==3))
                    {
                      chaoche==1;
                    }
                    else
                    {
                     chaoche=0;
                    }
                }
            
           }
      }
    }
    if(circle_start==1)
    {
     circle_time++;
     if(circle_time>500)
     {
      circle_time=0;
      circle_start=0;
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
      if(host_flag==1&&overtake_mode==1&&no_takeoff_rx==0)//�󳵳�ȥ�� ��ǰ��
      {
       host_flag=0;
       takeoff_over_tx=1;
      }
       else if(host_flag==1&&overtake_mode==1&&no_takeoff_rx==1)//�󳵳�ȥ�� û��ǰ��
      {
       no_takeoff_rx=0;
      }
      else if(host_flag==2)//ǰ����ȥ�� ���
      {
       host_flag=1;
      }
      else if(host_flag==0&&overtake_mode==1)//ǰ��ֱ�ӹ��ˣ��󳵲�����
      {
       no_takeoff_tx=1;
      }
    }
   //�µ�ʶ��

  if((ad_mid>=120||ad_left>=120||ad_right>=120)&&RunTime>500&&ad_vm<=15&&circle_time==0&&Ramp_flag==0&&Straight_flag==1&&circle_flag==0)
  {
      Ramp_count++;
  }
  else
  {
    Ramp_count=0;
  }
  if(Ramp_count>=3&&Ramp_flag==0&&Ramp_time==0)
  {
   Ramp_flag=1;
   Ramp_start=1;
   Ramp_count=0;
  }

   if(Ramp_time>=230)
   {
     Ramp_flag=0;
   }
  if(Ramp_start==1)//�µ���ʱ����ֹ������Բ��
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
     if(zhidao_OT==1)//ֱ������
     {
     if(RunTime<=650&&host_flag==0)
       DirectionPianCha[0]=0;
     if(RunTime>650&&RunTime<=1500&&host_flag==0)
        DirectionPianCha[0]=0.5;
      if(RunTime<=900&&host_flag==1)
        DirectionPianCha[0]=0;
     }
    }
   else if(circle_flag==1)//Բ���ı��׼ֵ
   {
      if(ad_mid>45)
      {
          DirectionPianCha[0] =15.0*ad_mid/ad_max - 15.0;
      }
      else
      {
          DirectionPianCha[0] = 15.0*ad_mid/45 - 15.0;
      }
   }


    //������ת
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
   
    
   else if(circle_flag==1)//��Բ����
   { 

     if( host_flag==0)
     {
      takeoff_over_tx=0;
      no_takeoff_tx=0;
      if(overtake_mode==1&&chaoche==1)//ǰ��֪ͨ����Բ��
         circle_ready_tx=1;
     }
     else if(host_flag==1&&overtake_mode==1&&chaoche==1)//���ڳ���ģʽ�½��յ���������һ��·
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
     
   
     
     if(circle_left==0&&circle_right==0&& host_flag==0) //ǰ���ж���̬
     {
       if(DirectionPianCha[1]>0)
       {
          circle_right=1;    //ǰ���ҹ� ֪ͨ�����
          if(chaoche==1&&overtake_mode==1)
          back_car_dir_tx=1;
       }
       else if(DirectionPianCha[1]<0)
       {
          circle_left=1;          //ǰ����� ֪ͨ���ҹ�
          if(chaoche==1&&overtake_mode==1)
          back_car_dir_tx=2;
       }
     }
     else  if(circle_left==0&&circle_right==0&& host_flag==1&&(overtake_mode==0||chaoche==0||no_takeoff_rx==1)) 
     {//���ڲ�����ģʽ���ж���̬
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
          circle_flag_count++;//Բ������ 5MSһ��
          if(circle_flag_count>30000)
            circle_flag_count=30000;
          
    if(circle_left==1||back_car_dir_rx==1)//��·��Բ��
    {
      if(chaoche==1&&overtake_mode==1&&host_flag==0)//ǰ������������
       back_car_dir_tx=2;
      if(back_car_dir_rx==1)//������ʱ��30
      {
       if(circle_flag_count<=50) 
       {
          DirectionPianCha[0]=-13.5;
       }   
       else if(circle_flag_count>50)
       {
           if(ad_right>=ad_left&&dir_change==0)
           {
            dir_change=2;
           }
         
        if(dir_change==2)
        {
           DirectionPianCha[0] = -DirectionPianCha[0] ;
        }
        else if(dir_change==0)
          {
             DirectionPianCha[0] = DirectionPianCha[0] ;
          }
     }
      }
      else
      {
       if(circle_flag_count<=25) 
       {
          DirectionPianCha[0]=-13.5;
       }   
       else if(circle_flag_count>25)
       {
           if( host_flag==0&&overtake_mode==1&&chaoche==1)//ǰ��ͣ�� ����׼��
            {
             stop_flag=1;
             host_flag=2;
            }
           if(ad_right>=ad_left&&dir_change==0)
           {
            dir_change=2;
           }
         
        if(dir_change==2)
        {
           DirectionPianCha[0] = -DirectionPianCha[0] ;
        }
        else if(dir_change==0)
          {
             DirectionPianCha[0] = DirectionPianCha[0] ;
          }
      }
     }
    }
    else if(circle_right==1||back_car_dir_rx==2)//��·��Բ��
    {
      if(chaoche==1&&overtake_mode==1&&host_flag==0)
       back_car_dir_tx=1;
      if(back_car_dir_rx==2)
      {
        if(circle_flag_count<=50) 
       {
          DirectionPianCha[0]=13.5;
       }   
       else if(circle_flag_count>50)
       {
           if(ad_left>=ad_right&&dir_change==0)
           {
            dir_change=1;
           }
         
        if(dir_change==1)
         {
            DirectionPianCha[0] = DirectionPianCha[0];
         }
        else if(dir_change==0)
        {
            DirectionPianCha[0] =-DirectionPianCha[0];
        }
       
      }
     }
      else
      {
       if(circle_flag_count<=25) 
       {
        //  DirectionPianCha[0] = -DirectionPianCha[0];
          DirectionPianCha[0]=13.5;
       }   
       else if(circle_flag_count>25)
       {
           if( host_flag==0&&overtake_mode==1&&chaoche==1)//ǰ��ͣ�� ����׼��
            {
             stop_flag=1;
             host_flag=2;
            }    
           if(ad_left>=ad_right&&dir_change==0)
           {
            dir_change=1;
//           if( host_flag==0&&overtake_mode==1&&chaoche==1)//ǰ��ͣ�� ����׼��
//            {
//             stop_flag=1;
//             host_flag=2;
//            }
           }
         
        if(dir_change==1)
         {
            DirectionPianCha[0] = DirectionPianCha[0];
         }
          else if(dir_change==0)
          {
             DirectionPianCha[0] =-DirectionPianCha[0];
          }
       
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
     if(circle_flag==1)
     {
      Steer_P=5.9;
     }
     else
     {
      switch(P_level)
     {
     case 0: {UP = UP0;break;}
     case 1: {UP = UP1;break;}
     case 2: {UP = UP2;break;}
     case 3: {UP = UP3;break;}
     case 4: {UP = UP4;break;}
     case 5: {UP = UP5;break;}
     }
     
   
     Steer_P= (Fuzzy_Direction_P(ABS(DirectionPianCha[0]*10),ABS(DoubleError*100)))/100.0;
     }
     

     SteeringZkb =SD5_mid + (s16)((Steer_P*DirectionPianCha[0])+Steer_D*(DoubleError));
    direction_offset=(s16)((Steer_P*DirectionPianCha[0])+Steer_D*(DoubleError));
    
     if(SteeringZkb>SD5_Rmax)   //�������޷�
        SteeringZkb = SD5_Rmax;
     else  if(SteeringZkb<SD5_Lmax)
        SteeringZkb = SD5_Lmax;
     

     ftm_pwm_duty(SD5_FTM,SD5_CH, SteeringZkb);   
    
}