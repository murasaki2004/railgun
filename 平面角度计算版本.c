#include "stm32f10x.h"
#include "math.h"
#include "delay.h"
#include "usart.h"
#include "sys.h"

#define du 0.0111111111111111
#define M_PI		3.14159265358979323846

#define Tim_Arr 999    // 重装载值
#define Tim_Psc 719    // 预分频系数

uint16_t angle_use = 950, angle_input = 0;

void tim_init(unsigned int, unsigned int);
void nvic_init(void);
void servo_init(void);
void led_init(void);

uint16_t new_type(uint16_t);    // 通过角度计算占空比
uint16_t Galanz(uint16_t, uint16_t);    // 输入两边长计算瞄准点需要的平面角度

int main (){
	// 初始化
	led_init();
	delay_init();
	servo_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	nvic_init();
	uart_init(115200);
	tim_init(Tim_Arr, Tim_Psc);
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);
	delay_ms(500);
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
	
	
	// 开始信号
	USART_SendData(USART1, 1);
	delay_ms(10);
	USART_SendData(USART1, USART_ReceiveData(USART1));
	TIM_SetCompare2(TIM3, 950);
	
	while (1);
}

void USART1_IRQHandler(void){                	//串口1中断服务程序
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		static uint16_t input_x = 0, input_y = 0;
		if (input_x == input_y && input_x == 0){
			input_x = USART_ReceiveData(USART1);
			delay_ms(5);
			USART_SendData(USART1, 7);
		} else if (input_y == 0){
			input_y = USART_ReceiveData(USART1);
			delay_ms(5);
		}
		if (input_x != 0 && input_y != 0){
			volatile uint16_t tmp = new_type(Galanz(input_x, input_y));
			TIM_SetCompare2(TIM3, tmp);
		}
	}
}

uint16_t new_type(uint16_t angle){
	volatile double tmp = du;
	tmp = tmp * angle + 0.5;
	tmp = (tmp / 10) * (Tim_Arr + 1);
	
	USART_SendData(USART1, angle);
	delay_ms(5);
	USART_SendData(USART1, (int)tmp);
	
	// 判断是否正常
	if (tmp <= 250 && tmp >= 50){
		return (Tim_Arr + 1) - tmp;
	}else{
		return 0;
	}
}

uint16_t Galanz(uint16_t x, uint16_t y){
	volatile double c = 0, angle_plane = 0;
	if (y > 0){
		int tmp = 0;
		// 去符号
		//if (x < 0){
      //x = sqrt(pow(x, 2.0));
			//tmp = 1;
		//}
		
		// 计算
		c = sqrt(x*x + y*y);
		angle_plane = asin(y / c);
		
		// 弧度转角度
		if (tmp)
			return (angle_plane * (180 / M_PI)) + 90;
		else
			return angle_plane * (180 / M_PI);
	}
	return 0;
}

void tim_init(unsigned int arr, unsigned int psc){
	TIM_TimeBaseInitTypeDef Tim3;
	TIM_OCInitTypeDef Tim3_OC;
	
	
	// 使能TIM3、AFIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	// 初始化TIM3的参数
	Tim3.TIM_Prescaler = psc;
	Tim3.TIM_Period = arr;
	Tim3.TIM_CounterMode = TIM_CounterMode_Up;
	Tim3.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &Tim3);
	
	// 初始化TIM3 Channe12 PWM模式
	Tim3_OC.TIM_OCMode = TIM_OCMode_PWM2;
	Tim3_OC.TIM_OutputState = TIM_OutputState_Enable;
	Tim3_OC.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &Tim3_OC);
	
	// 使能TIM3 在CCR2上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	// 使能TIM3
	TIM_Cmd(TIM3, ENABLE);
}

void nvic_init(){
	NVIC_InitTypeDef TIMx;
	
	// 初始化TIM3的中断优先级参数
	TIMx.NVIC_IRQChannel = TIM3_IRQn;
	TIMx.NVIC_IRQChannelPreemptionPriority = 3;
	TIMx.NVIC_IRQChannelSubPriority = 0;
	TIMx.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&TIMx);
}

void servo_init(){
	GPIO_InitTypeDef PA7;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	PA7.GPIO_Mode = GPIO_Mode_AF_PP;
	PA7.GPIO_Pin = GPIO_Pin_7;
	PA7.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &PA7);
}

void led_init(){
	GPIO_InitTypeDef led;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Pin = GPIO_Pin_5;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &led);
	
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
}
