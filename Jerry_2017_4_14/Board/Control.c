#include "include.h"
uint8 overtakingstatues=0;//������־  0����
uint8 host_flag=0;// ǰ����0   ����1 2����
uint8 rx_nrf_flag = 0;//�յ�������1
uint8 txbuf[32];//���͵�����
uint8 rxbuf[32];//�յ�������

void  nrftxbuf()
{

                txbuf[0] = ABDistance;
                txbuf[1] = circle_ready_tx;
                txbuf[2] = back_car_dir_tx;
                txbuf[3] = takeoff_over_tx;
                txbuf[4] = no_takeoff_tx;


                txbuf[26] = 0xf7;
                txbuf[27] = 0xf7;
                txbuf[28] = 0xf7;
                txbuf[29] = 0xf7;
                txbuf[30] = 0xf7;
                txbuf[31] = 0xf7;                

      nrf_tx(txbuf, 32); 

}

void rx_Calculate(uint8 *value)
{
  circle_ready_rx=rxbuf[1];
  back_car_dir_rx=rxbuf[2];
  takeoff_over_rx=rxbuf[3];
   no_takeoff_rx=rxbuf[4];

}