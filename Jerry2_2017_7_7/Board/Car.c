#include "include.h"

uint8 page=1;//oled页面
uint8 Wait_to_go_flag=0;
uint8 Go_flag=0;//发车标志
uint8 overtake_mode=0;//超车模式
uint8 zhidao_OT=0;
uint8 P_level=0;
uint8 sp_level=0;
//电机初始化
void Motor_init()
{   
       ftm_pwm_init(Motor_FTM_A, Motor_CH_A,Motor_FREQ,0);
       ftm_pwm_init(Motor_FTM_B, Motor_CH_B,Motor_FREQ,0);
}

//舵机初始化
void Steer_init()
{
    ftm_pwm_init(SD5_FTM, SD5_CH,SD5_FREQ,0);
}


void BSP_init()
{
        //按键
        gpio_init (PTC3, GPI,1);
        gpio_init (PTC4, GPI,1);
        gpio_init (PTC5, GPI,1);
        gpio_init (PTC7, GPI,1);
        port_init_NoALT (PTC3,  PULLUP );     
        port_init_NoALT (PTC4,  PULLUP );     
        port_init_NoALT (PTC5,  PULLUP );     
        port_init_NoALT (PTC7,  PULLUP );      //内部上拉
        //拨码开关
        gpio_init (PTB18, GPI,0);
        gpio_init (PTB19, GPI,0);
        gpio_init (PTB20, GPI,0);
        gpio_init (PTB21, GPI,0);        
        gpio_init (PTB22, GPI,0);
        gpio_init (PTB23, GPI,0);

         port_init_NoALT (PTB18,  PULLUP );     
         port_init_NoALT (PTB19,  PULLUP );      //内部上拉
         port_init_NoALT (PTB20,  PULLUP );     
         port_init_NoALT (PTB21,  PULLUP );     
         port_init_NoALT (PTB22,  PULLUP );     
         port_init_NoALT (PTB23,  PULLUP );      //内部上拉
        //LED
        gpio_init (PTD6, GPO,1);
        gpio_init (PTD8, GPO,1);
        
        gpio_init (PTD15, GPO,1);//k60上的
        gpio_init (PTA17, GPO,1);
        
        
        //超车模式选择
        if(BSP3==1)
        {
         zhidao_OT=1;
        }
        else  if(BSP3==0)
        {
         zhidao_OT=0;
        }
         if(BSP5==1)
        {
         host_flag=1;
        }
        else  if(BSP5==0)
        {
         host_flag=0;
        }
         if(BSP4==1)
        {
          overtake_mode=1;
          ABDistance_set=600;
        }
        else  if(BSP4==0)
        {
         overtake_mode=0;
          ABDistance_set=550;
        }
}

//超声波初始化
void Chao_init()
{
        //超声波
        pit1Init();
        gpio_init(PTA11,GPI,0);      //超声波接收模块的DO接口使用PTA11 stateA9
         gpio_init(PTA9,GPI,0);
          gpio_init(PTC6,GPO,0);
        port_init(PTA11, ALT1  | IRQ_EITHER |PULLUP);  //   GPIO模式，上下边沿触发，上拉       
}

void Sensor_AD_init()
{
            adc_init (ADC1_SE10 );//B4
            adc_init (ADC1_SE11 );//B5
            adc_init (ADC1_SE12 );//B6
            adc_init (ADC1_SE13 );//B7
            adc_init (ADC1_SE14 );//B10  
            adc_init (ADC1_SE15);//B11
             for(count = 0 ; count <= 5 ; count ++)
             {      
                ADdata.ResultCenter[count]=0;
  
                ADdata.ResultLeft[count]=0;
        
                ADdata.ResultRight[count]=0;
                
                ADdata.V_Left[count]=0;
                
                ADdata.V_Mid[count]=0;
               
                ADdata.V_Right[count]=0;
       
             }
}

void Oled_Display()
{          
 if(page==1)
 {
  LED_PrintValueF(0, 0,ADdata.ResultLeft[0],1);  //用此显示函数要关掉FPU，否则跑飞
  LED_PrintValueF(45, 0,ADdata.ResultCenter[0],1);  
  LED_PrintValueF(90, 0,ADdata.ResultRight[0], 1);
  LED_PrintValueF(0, 1,ADdata.V_Left[0], 1);
    LED_PrintValueF(45, 1,ADdata.V_Mid[0], 1);
  LED_PrintValueF(90, 1,ADdata.V_Right[0], 1);
  
  LCD_P6x8Str(0,2,"Err:");
  LED_PrintValueF(25, 2, DirectionPianCha[0],1);
  LCD_P6x8Str(78,2,"Y:");
  LED_PrintValueF(95, 2, circle_level,1);//Ramp_flag circle_level 
  LCD_P6x8Str(0,3,"OT:");  
  LED_PrintValueF(20, 3,overtake_mode,1);
  LED_PrintValueF(75, 3,zhidao_OT,1);
  LCD_P6x8Str(0,4,"BMQ:");
  LED_PrintValueF(25,4,realSpeed, 1);
  LCD_P6x8Str(65,4,"Dis:");
   LED_PrintValueF(90, 4,ABDistance,1);
  LCD_P6x8Str(0,5,"Power:");
  LED_PrintValueF(35,5,dianya, 2);
   LCD_P6x8Str(70,5,"host:");  
  LED_PrintValueF(100, 5,host_flag,1); 
  LCD_P6x8Str(0,6,"Steer:");
  LED_PrintValueF(40,6,P_level, 1);
  LED_PrintValueF(80,6,Steer_P, 1);
  LCD_P6x8Str(0,7,"Speed:");
  LED_PrintValueF(40,7,sp_level, 1);
  LED_PrintValueF(80,7,TargetSpeed, 1);
 }
else if(page==2)
{
 
 LCD_P6x8Str(0,0,"zhi:");
 LED_PrintValueF(30,0,Straight_flag, 1);
  LCD_P6x8Str(0,1,"ru:");
 LED_PrintValueF(30,1,ruwan_flag, 1);
  LCD_P6x8Str(0,2,"wan:");
 LED_PrintValueF(30,2,wandao_flag, 1);
 
  LED_PrintValueF(0,3,circle_ready_rx, 1);
 LED_PrintValueF(0,4,back_car_dir_rx, 1);
 LED_PrintValueF(0,5,takeoff_over_rx, 1);
 LED_PrintValueF(0,6,rxbuf[4], 1);
}
//   if(KEY1==0)
//   { 
//     LCD_Fill(0x00);
//    page++;
//    if(page>2)
//      page=1;
//    DELAY_MS(200);
//   }
   if(KEY4==0)
   {
     if(host_flag==0)//前车
     {
      DELAY_MS(1000);
      Go_flag=1;
      DELAY_MS(200);
     }
     else if(host_flag==1&&zhidao_OT==0)
     {
         Wait_to_go_flag=1;
     }
     else  if(host_flag==1&&zhidao_OT==1)//后
     {
      DELAY_MS(2000);
      Go_flag=1;
      DELAY_MS(200);
     } 
   }
//   if(KEY3==0)
//   {
//     DELAY_MS(10);
//     if(KEY3==0)
//     { 
//      P_level++;
//      if(P_level>5)
//        P_level=0;
//     }
//     DELAY_MS(200);
//   }
  if(KEY1==0)
   {
     DELAY_MS(10);
     if(KEY1==0)
     { 
      sp_level++;
      if(sp_level>2)
        sp_level=0;
     }
     DELAY_MS(200);
   }
}

float var[8];
void send_data()
{
        var[0] =ADdata.ResultLeft[0];
        var[1] = ADdata.ResultCenter[0];
        var[2] =ADdata.ResultRight[0];
        var[3] = ADdata.V_Left[0];
        var[4] =ADdata.V_Mid[0];//DoubleError*100;circle_flag_count ABDistance_set
        var[5] = ADdata.V_Right[0];//ABD_err realSpeed host_flag
        var[6] = circle_flag*100;//Steer_P*10 TargetSpeed circle_flag
        var[7] = DirectionPianCha[0]*10;//direction_offset  Motor_Duty ABDistance dir_change

      vcan_sendware(var, sizeof(var));
}