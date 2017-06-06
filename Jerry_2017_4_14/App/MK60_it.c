#include "include.h"
uint16 TimeCount=0;
int32 chaoshengboTime=0;
int32 RunTime=0;
uint16 ABDistance=0; //�������� mm
uint16 CurrentABDistance=0;
uint16  ABDistance_filter[3];

void PIT0_IRQHandler() // ÿ1MSִ��1��
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
     if(ABDistance>400)
     { 
       start_flag=1;
       Wait_to_go_flag=0;
     }
    }
   }
   else if(TimeCount==2)
   {

   }
   else if(TimeCount==3)
   {    
     Read_AD();//��ȡAD
   }
   else if(TimeCount==4)
   {
     CalculateCurrentError();//����ƫ��
     DirectionControl();//�������
   }
 
   else if(TimeCount==5)
   {

     Motor_GetSpeed();//��������
     //CalSpeed();
     Motor_Control();//�ٶȿ���
   // Motor_SetSpeed(2000);
     //protect();
     if(realSpeed<=15&&overtake_mode==0&&RunTime>2000)
       Motor_SetSpeed(0);
     
     TimeCount=0;

   }
  
     PIT_Flag_Clear(PIT0);       //���жϱ�־λ
     
}


//�������ж�
void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ
        n = 11;	
        if(flag & (1<<n))
        {
          if(gpio_get(PTA11) == 1)  //������
          {
              PIT_TCTRL(PIT1) &= ~PIT_TCTRL_TEN_MASK;
              PIT_LDVAL(PIT1) = 0xFFFFFFFF;
              PIT_TCTRL(PIT1) |=  PIT_TCTRL_TEN_MASK;
          }
          else                     //�½���
          {
            if(gpio_get(PTA9) == 1)
            {
              chaoshengboTime = (0xFFFFFFFF - PIT_CVAL(PIT1))/MAX_BUS_CLK;//����BUS����
              ABDistance =  chaoshengboTime * 340 / 1000;  //mm
              PIT_TCTRL(PIT1) &= ~PIT_TCTRL_TEN_MASK;
            }
          }
          
        }

} 


//NRF2401�ж� ���������� 
void PORTE_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;
    
     uint8 relen=0 ;
    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //���жϱ�־λ

    n = 0;
    if(flag & (1 << n))                                 //PTE0�����ж�
    {
        nrf_handler();
        if(host_flag==1||host_flag==2)
        {
            relen = nrf_rx(rxbuf,DATA_PACKET);               //�ȴ�����һ�����ݰ������ݴ洢��buff��
            if(relen != 0)
            { 
                  rx_nrf_flag = 1;
                  disable_irq (PORTE_IRQn);
                   //rx_value(rxbuf);
            }
        }
        
    }
}