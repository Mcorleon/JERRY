#include "include.h"
uint16 TimeCount=0;
int32 chaoshengboTime=0;
int32 RunTime=0;
uint16 ABDistance=0; //两车距离 mm
uint16 CurrentABDistance=0;
uint16  ABDistance_filter[3];

void PIT0_IRQHandler() // 每1MS执行1次
{
  if( start_flag==1)
  {
     RunTime++;
     if(RunTime>60000)
       RunTime=60000;
  }
   TimeCount++;
   if(TimeCount==1)
   {
    if(Go_flag==1)
    {
      DELAY_MS(500);
      start_flag=1;
      Go_flag=0;
    }
    if(Wait_to_go_flag==1)
    {
     if(ABDistance>550)
     { 
       start_flag=1;
       Wait_to_go_flag=0;
     }
    }
    
   }
   else if(TimeCount==2)
   {
      Read_AD();//读取AD
    
   }
   else if(TimeCount==3)
   {    
      Circle_Cal();
     
    
   }
   else if(TimeCount==4)
   {
     CalculateCurrentError();//计算偏差
     DirectionControl();//方向控制
   }
 
   else if(TimeCount==5)
   {

     Motor_GetSpeed();//读编码器
     Motor_Control();//速度控制
   // Motor_SetSpeed(2000);

     if(realSpeed<=15&&overtake_mode==0&&RunTime>2000)
       Motor_SetSpeed(0);
     
     TimeCount=0;

   }
  
     PIT_Flag_Clear(PIT0);       //清中断标志位
     
}


//超声波中断
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位
        n = 11;	
        if(flag & (1<<n))
        {
          if(gpio_get(PTA11) == 1)  //上升沿
          {
              PIT_TCTRL(PIT1) &= ~PIT_TCTRL_TEN_MASK;
              PIT_LDVAL(PIT1) = 0xFFFFFFFF;
              PIT_TCTRL(PIT1) |=  PIT_TCTRL_TEN_MASK;
          }
          else                     //下降沿
          {
            if(gpio_get(PTA9) == 1)
            {
              chaoshengboTime = (0xFFFFFFFF - PIT_CVAL(PIT1))/MAX_BUS_CLK;//根据BUS调整
              ABDistance =  chaoshengboTime * 340 / 1000;  //mm
              PIT_TCTRL(PIT1) &= ~PIT_TCTRL_TEN_MASK;
            }
          }
          
        }

} 


//NRF2401中断 建议最优先 
void PORTE_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;
    
     uint8 relen=0 ;
    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //清中断标志位

    n = 0;
    if(flag & (1 << n))                                 //PTE0触发中断
    {
        nrf_handler();
        if(host_flag==1||host_flag==2)
        {
            relen = nrf_rx(rxbuf,DATA_PACKET);               //等待接收一个数据包，数据存储在buff里
            if(relen != 0)
            { 
                  rx_nrf_flag = 1;
                  disable_irq (PORTE_IRQn);
                   //rx_value(rxbuf);
            }
        }
        
    }
}