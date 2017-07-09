#ifndef	_CONTROL_H_
#define _CONTROL_H_
extern uint8 overtakingstatues;
extern uint8 host_flag ;//前车是0   后车是1
extern uint8 rx_nrf_flag ;//收到数据置1
extern uint8 txbuf[32];//发送的数据
extern uint8 rxbuf[32];//收到的数据
void  nrftxbuf();
void rx_Calculate(uint8 *value);
 
#endif