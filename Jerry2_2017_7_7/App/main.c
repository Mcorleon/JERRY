
/*************************************
Jerry2
Jerry2
Jerry2
Jerry2
Jerry2        
Jerry2  �̻������������ ��95 95 95 �� 85 85 85
**************************************/
#include "include.h"


void main()
{  
      DisableInterrupts;
      Motor_init();//���
      Steer_init();//���
      BSP_init();//���ذ���
      Chao_init();//������
      Sensor_AD_init();//���AD                                 
      gpio_init (PTE12, GPO,1);//������
      gpio_init (PTE27, GPI,1);//�ɻɹ�
      adc_init (ADC0_SE13 );//��ѹ���
      ftm_quad_init(FTM1);//��������  
      LCD_Init();//Һ��
      pit_init_ms(PIT0,1);//1MS�ж�

//      while(!nrf_init())                  //��ʼ��NRF24L01+ ,�ȴ���ʼ���ɹ�Ϊֹ
//      {
//       
//      }
      LED1=0;
      set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
      set_vector_handler(PORTA_VECTORn,PORTA_IRQHandler);
      set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    			


       /************************ ���� K60 �����ȼ�  ***********************/
    //K60 ��Ĭ�����ȼ� ��Ϊ 0
    //�ο����ӣ������ж�Ƕ�׵����� - ���ܳ������� - ɽ����̳
    //          http://www.vcan123.com/forum.php?mod=viewthread&tid=499&page=1#pid3270
      NVIC_SetPriorityGrouping(4);            //�������ȼ�����,4bit ��ռ���ȼ�,û�������ȼ�
      NVIC_SetPriority(PORTE_IRQn,2);         //�������ȼ�           
      NVIC_SetPriority(PORTA_IRQn,1);         //�������ȼ�
      NVIC_SetPriority(PIT0_IRQn,0);          //�������ȼ�
      EnableInterrupts;
      enable_irq (PIT0_IRQn);
      enable_irq (PORTA_IRQn); 
      enable_irq(PORTE_IRQn);
       
      
     while(1)
     {  
       PTE12_OUT=1;
         
        Oled_Display();//Һ����ʾ
        send_data();//�������ݸ���λ��
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
        if(host_flag==0&&circle_flag==0)//ǰ��ֻ���ͳ����� ��ֻ����
       {
        PTC6_OUT=1;
       }
       else if(host_flag==1)
       {
        PTC6_OUT=0;
       }
       if(PTE27_IN==0)//�յ�ͣ��
       {
         if(RunTime>2000)
         {
           if(host_flag==0)
           {
            stop_flag=2;
           }
          else if(host_flag==1)
            stop_flag=1;
         }

       }               
                          
  
//        if(host_flag==0) //ǰ��
//        {
//            enable_irq (PORTE_IRQn);
//            nrftxbuf();//��������
//        }
//        else //��
//        {
//            nrf_rx_mode();
//        }
//        
//        if(rx_nrf_flag)
//        {                      
//             rx_nrf_flag = 0;
//             if(rxbuf[26]==0xf7&&rxbuf[27]==0xf7&&rxbuf[28]==0xf7&&rxbuf[29]==0xf7&&rxbuf[30]==0xf7&&rxbuf[31]==0xf7)//֡βȷ��
//             { 
//               rx_Calculate(rxbuf);//���յ������ݽ��д���
//             }
//               
//             enable_irq (PORTE_IRQn);
//        }
//      DELAY_MS(10);
     }
}
