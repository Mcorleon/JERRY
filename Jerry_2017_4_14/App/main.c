
/*************************************
Jerry The Mouse
Jerry The Mouse
Jerry The Mouse
Jerry The Mouse
Jerry The Mouse
Jerry The Mouse
**************************************/
#include "include.h"


void main()
{  
      DisableInterrupts;
      Motor_init();//电机
      Steer_init();//舵机
      BSP_init();//开关按键
      Chao_init();//超声波
      Sensor_AD_init();//电感AD                                 
      gpio_init (PTE12, GPO,1);//蜂鸣器
      adc_init (ADC0_SE13 );//电压检测
      ftm_quad_init(FTM1);//正交解码  
      LCD_Init();//液晶
      pit_init_ms(PIT0,1);//1MS中断

      while(!nrf_init())                  //初始化NRF24L01+ ,等待初始化成功为止
      {
       
      }
      LED1=0;
      set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
      set_vector_handler(PORTA_VECTORn,PORTA_IRQHandler);
      set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    			


       /************************ 配置 K60 的优先级  ***********************/
    //K60 的默认优先级 都为 0
    //参考帖子：急求中断嵌套的例程 - 智能车讨论区 - 山外论坛
    //          http://www.vcan123.com/forum.php?mod=viewthread&tid=499&page=1#pid3270
      NVIC_SetPriorityGrouping(4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级
      NVIC_SetPriority(PORTE_IRQn,0);         //配置优先级           
      NVIC_SetPriority(PORTA_IRQn,1);         //配置优先级
      NVIC_SetPriority(PIT0_IRQn,2);          //配置优先级
      EnableInterrupts;
      enable_irq (PIT0_IRQn);
      enable_irq (PORTA_IRQn); 
      enable_irq(PORTE_IRQn);
       
      
     while(1)
     {  
       PTE12_OUT=1;
         
        Oled_Display();//液晶显示
     //  send_data();//发送数据给上位机
       if(circle_flag==1)
       {
        PTD15_OUT=0;
        PTA17_OUT=0;
       }
       else
       {
        PTD15_OUT=1;
        PTA17_OUT=1;
       }
       if(Ramp_flag==1)
         LED2=0;
       else
         LED2=1;
        if(host_flag==0)//前车只发送超声波 后车只接收
       {
        PTC6_OUT=1;
       }
       else if(host_flag==1)
       {
        PTC6_OUT=0;
       }

                        
                          
  
        if(host_flag==0) //前车
        {
            enable_irq (PORTE_IRQn);
            nrftxbuf();//发送数据
        }
        else //后车
        {
            nrf_rx_mode();
        }
        
        if(rx_nrf_flag)
        {                      
             rx_nrf_flag = 0;
             if(rxbuf[26]==0xf7&&rxbuf[27]==0xf7&&rxbuf[28]==0xf7&&rxbuf[29]==0xf7&&rxbuf[30]==0xf7&&rxbuf[31]==0xf7)//帧尾确认
             { 
               rx_Calculate(rxbuf);//对收到的数据进行处理
             }
               
             enable_irq (PORTE_IRQn);
        }

 
     }
}
