#ifndef	_CAR_H_
#define _CAR_H_
#define LED1   PTD6_OUT
#define LED2   PTD8_OUT
#define BSP1   PTB18_IN
#define BSP2   PTB19_IN
#define BSP3   PTB20_IN
#define BSP4   PTB21_IN
#define BSP5   PTB22_IN 
#define BSP6   PTB23_IN
#define KEY1   PTC3_IN
#define KEY2   PTC4_IN
#define KEY3   PTC5_IN
#define KEY4   PTC7_IN
extern uint8 Go_flag;//发车标志
extern uint8 overtake_mode;
extern uint8 Wait_to_go_flag;
void Motor_init();
void Steer_init();
void BSP_init();
void Chao_init();
void Sensor_AD_init();
void Oled_Display();
void send_data();
#endif