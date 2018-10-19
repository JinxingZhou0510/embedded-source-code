/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_VARIAL_H
#define __GLOBAL_VARIAL_H

/*以下包含必要的头文件，包括数据格式文件*/
/****************Start*******************/
//#include "STM32F10x_System.h"
//#include "stm32f10x_type.h"
#include "stm32f10x.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "Macro.h" 


#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
//
//
#include  "stm32f10x_conf.h"
//#include  <stm32f10x.h>

//#include  <stm32f10x_lib.h>
//


#define MIN(a,b) (a<b) ? (a) : (b)
#define MAX(a,b) (a>b) ? (a) : (b)
#define rt_int8_t	   int8_t
#define rt_int16_t	   int16_t
#define rt_int32_t	   int32_t

#define rt_uint8_t	   uint8_t
#define rt_uint16_t	   uint16_t
#define rt_uint32_t	   uint32_t

#define IDLE		   0
#define ACCELERATING   1
#define AT_MAX		   2
#define DECELERATING   3

typedef __packed struct 
{
	unsigned char en;	        //使能
	unsigned char dir;			//方向
	unsigned char running;		//转动完成标志 
	unsigned char rstflg;		//复位标志
	unsigned char divnum;		//分频数
	unsigned char speedenbale;	//是否使能速度控制	
	unsigned char clockwise;	//顺时针方向对应的值
	unsigned char id;			//电机id

	uint32_t pulsecount;
	uint16_t *Counter_Table;  	//指向启动时，时间基数计数表
	uint16_t *Step_Table;    	//指向启动时，每个频率脉冲个数表
	uint16_t CurrentIndex;    	//当前表的位置
	uint16_t TargetIndex;    	//目标速度在表中位置
	uint16_t StartTableLength;  //启动数据表
	uint16_t StopTableLength;   //启动数据表
	uint32_t StartSteps;		//电机启动步数
	uint32_t StopSteps;		    //电机停止步数
	uint32_t RevetDot;			//电机运动的减速点
	uint32_t PulsesGiven;		//电机运动的总步数
	uint32_t PulsesHaven;		//电机已经运行的步数
	uint32_t CurrentPosition;	//当前位置
	uint32_t MaxPosition;		//最大位置，超过该位置置0
	uint32_t CurrentPosition_Pulse;	//当前位置
	uint32_t MaxPosition_Pulse;		//当前位置
	unsigned long long Time_Cost_Act;	//实际运转花费的时间
	unsigned long long Time_Cost_Cal;	//计算预估运转花费的时间
	TIM_TypeDef* TIMx;	
} MOTOR_CONTROL_S ;

typedef __packed struct 
{
	unsigned char en;	        //使能
	unsigned char dir;		    //方向
	unsigned char running;	    //转动完成标志
	unsigned char rstflg;	    //复位标志，为1时，限位开关强停。
	unsigned char divnum;	    //分频数	 
	unsigned char speedenbale;	//是否使能速度控制	
	unsigned char clockwise;	//顺时针方向对应的值
	unsigned char id;			//电机id	

	uint32_t PulsesGiven;		//电机运动的总步数
	uint32_t PulsesHaven;		//电机已经运行的步数
	uint32_t step_move     ;	//total move requested
	uint32_t step_spmax    ;	//maximum speed
	uint32_t step_accel    ;	//accel/decel rate, 8.8 bit format
	uint32_t step_acced    ;	//steps in acceled stage

	uint32_t step_middle   ;	//mid-point of move, = (step_move - 1) >> 1
	uint32_t step_count    ;	//step counter
	uint32_t step_frac     ;	//step counter fraction
	uint32_t step_speed    ;	//current speed, 16.8 bit format (HI byte always 0)
	uint32_t speed_frac    ;	//speed counter fraction
	uint8_t step_state    ;		//move profile state
	uint8_t step_dur      ;		//counter for duration of step pulse HI

	uint32_t CurrentPosition;	//当前位置
	uint32_t MaxPosition;		//最大位置，超过该位置置0
	uint32_t CurrentPosition_Pulse;	//当前位置
	uint32_t MaxPosition_Pulse;		//当前位置

	TIM_TypeDef* TIMx;	
	GPIO_TypeDef * GPIOBASE;
	int32_t PWMGPIO;
} MOTOR_CONTROL_SPTA ;

#define M1_CLOCKWISE	 0
#define M1_UNCLOCKWISE	 1
#define M2_CLOCKWISE	 0
#define M2_UNCLOCKWISE	 1
#define M3_CLOCKWISE	 0
#define M3_UNCLOCKWISE	 1
#define M4_CLOCKWISE	 0
#define M4_UNCLOCKWISE	 1

#define PWM1_PreemptionPriority  1             //阶级
#define PWM1_SubPriority         0			   //阶层
#define PWM2_PreemptionPriority  1             //阶级
#define PWM2_SubPriority         1			   //阶层
#define PWM3_PreemptionPriority  2             //阶级
#define PWM3_SubPriority         0			   //阶层
#define PWM4_PreemptionPriority  2             //阶级
#define PWM4_SubPriority         1			   //阶层
/***********************END********************************************/

//电机
extern MOTOR_CONTROL_S    motor1;
extern MOTOR_CONTROL_S    motor2;
extern MOTOR_CONTROL_S    motor3;
extern MOTOR_CONTROL_SPTA motor4; 

extern volatile uint8_t USART1_TxBuffer[USART1TXSIZE];  //串口1发送缓冲区
extern volatile uint16_t PTxBufferUSART11;				//串口1发送前向位置
extern volatile uint16_t PTxBufferUSART12;				//串口1发送后向位置，后向-前向=未发送的数据
extern volatile uint16_t USART1_TxCounter ;				//串口1发送计数
extern volatile uint16_t USART1_RxCounter ; 			//串口1接收计数
extern volatile uint16_t USART1_NbrOfDataToTransfer;	//串口1要发送的数据个数
extern volatile uint8_t USART1_RxBuffer[USART1RXSIZE];	//串口1接收缓冲区
extern volatile uint16_t PRxBufferUSART11;
extern volatile uint16_t PRxBufferUSART12;
extern volatile uint8_t USART1_NbrOfDataReceived;		//串口1要接收的数据个数
extern volatile uint8_t USART1_Received_Flag;
extern struct rt_ringbuffer rb_recv;

extern volatile uint8_t USART2_TxBuffer[USART2TXSIZE];	//串口2发送缓冲区
extern volatile uint16_t PTxBufferUSART21;				//串口2发送前向位置
extern volatile uint16_t PTxBufferUSART22;				//串口2发送后向位置，后向-前向=未发送的数据
extern volatile uint16_t USART2_TxCounter ;				//串口2发送计数
extern volatile uint16_t USART2_RxCounter ; 			//串口2接收计数
extern volatile uint16_t USART2_NbrOfDataToTransfer;	//串口2要发送的数据个数
extern volatile uint8_t USART2_RxBuffer[USART2RXSIZE];	//串口2接收缓冲区
extern volatile uint16_t PRxBufferUSART21;
extern volatile uint16_t PRxBufferUSART22;
extern volatile uint8_t USART2_NbrOfDataReceived;	     //串口2要接收的数据个数
extern volatile uint8_t USART2_Received_Flag;


extern volatile uint8_t USART3_TxBuffer[USART3TXSIZE];	 //串口3发送缓冲区
extern volatile uint16_t PTxBufferUSART31;				 //串口3发送前向位置
extern volatile uint16_t PTxBufferUSART32;				 //串口3发送后向位置，后向-前向=未发送的数据
extern volatile uint16_t USART3_TxCounter;				 //串口3发送计数
extern volatile uint16_t USART3_RxCounter ; 			 //串口3接收计数
extern volatile uint16_t USART3_NbrOfDataToTransfer;	 //串口3要发送的数据个数
extern volatile uint8_t USART3_RxBuffer[USART3RXSIZE];	 //串口3接收缓冲区
extern volatile uint16_t PRxBufferUSART31;
extern volatile uint16_t PRxBufferUSART32;
extern volatile uint8_t USART3_NbrOfDataReceived;		 //串口3要接收的数据个数
extern volatile uint8_t USART3_Received_Flag;			 //串口3收到数据标志，1：接收到数据，0：没有接收到数据


void USART1_Initial(void);
void USART1_Printf(unsigned char *q,unsigned char len);
void USART1_Printfstr(unsigned char *p);
void USART2_Initial(void);
void USART2_Printf(unsigned char *q,unsigned char len);
void USART2_Printfstr(unsigned char *p);
void USART3_Initial(void);
void USART3_Printf(unsigned char *q,unsigned char len);
void USART3_Printfstr(unsigned char *p);

void Initial_MotorIO(void);
void Initial_Motor(unsigned char MotorID, unsigned char StepDive,unsigned int maxposition);
void MotorRunParaInitial(void);
void Start_Motor12(unsigned char dir1,unsigned int Degree1,unsigned char dir2,unsigned int Degree2);
void Start_Motor_S(unsigned char MotorID,unsigned char dir,unsigned int Degree);
void Start_Motor_SPTA(unsigned char MotorID,unsigned char dir,unsigned int Degree);
void SetSpeed(unsigned char MotorID, signed char speedindex);
void Do_Reset(unsigned char MotorID);
void Deal_Cmd(void);
void Initial_PWM_Motor1(void);
void Initial_PWM_Motor2(void);
void Initial_PWM_Motor3(void);
void Initial_PWM_Motor4(void);
void EXTI_Configuration(void);



#endif
