#ifndef	_CONTROL_H_
#define _CONTROL_H_
extern uint8 overtakingstatues;
extern uint8 host_flag ;//ǰ����0   ����1
extern uint8 rx_nrf_flag ;//�յ�������1
extern uint8 txbuf[32];//���͵�����
extern uint8 rxbuf[32];//�յ�������
void  nrftxbuf();
void rx_Calculate(uint8 *value);
 
#endif