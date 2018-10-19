/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_VARIAL_H
#define __GLOBAL_VARIAL_H

/*���°�����Ҫ��ͷ�ļ����������ݸ�ʽ�ļ�*/
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
	unsigned char en;	        //ʹ��
	unsigned char dir;			//����
	unsigned char running;		//ת����ɱ�־ 
	unsigned char rstflg;		//��λ��־
	unsigned char divnum;		//��Ƶ��
	unsigned char speedenbale;	//�Ƿ�ʹ���ٶȿ���	
	unsigned char clockwise;	//˳ʱ�뷽���Ӧ��ֵ
	unsigned char id;			//���id

	uint32_t pulsecount;
	uint16_t *Counter_Table;  	//ָ������ʱ��ʱ�����������
	uint16_t *Step_Table;    	//ָ������ʱ��ÿ��Ƶ�����������
	uint16_t CurrentIndex;    	//��ǰ���λ��
	uint16_t TargetIndex;    	//Ŀ���ٶ��ڱ���λ��
	uint16_t StartTableLength;  //�������ݱ�
	uint16_t StopTableLength;   //�������ݱ�
	uint32_t StartSteps;		//�����������
	uint32_t StopSteps;		    //���ֹͣ����
	uint32_t RevetDot;			//����˶��ļ��ٵ�
	uint32_t PulsesGiven;		//����˶����ܲ���
	uint32_t PulsesHaven;		//����Ѿ����еĲ���
	uint32_t CurrentPosition;	//��ǰλ��
	uint32_t MaxPosition;		//���λ�ã�������λ����0
	uint32_t CurrentPosition_Pulse;	//��ǰλ��
	uint32_t MaxPosition_Pulse;		//��ǰλ��
	unsigned long long Time_Cost_Act;	//ʵ����ת���ѵ�ʱ��
	unsigned long long Time_Cost_Cal;	//����Ԥ����ת���ѵ�ʱ��
	TIM_TypeDef* TIMx;	
} MOTOR_CONTROL_S ;

typedef __packed struct 
{
	unsigned char en;	        //ʹ��
	unsigned char dir;		    //����
	unsigned char running;	    //ת����ɱ�־
	unsigned char rstflg;	    //��λ��־��Ϊ1ʱ����λ����ǿͣ��
	unsigned char divnum;	    //��Ƶ��	 
	unsigned char speedenbale;	//�Ƿ�ʹ���ٶȿ���	
	unsigned char clockwise;	//˳ʱ�뷽���Ӧ��ֵ
	unsigned char id;			//���id	

	uint32_t PulsesGiven;		//����˶����ܲ���
	uint32_t PulsesHaven;		//����Ѿ����еĲ���
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

	uint32_t CurrentPosition;	//��ǰλ��
	uint32_t MaxPosition;		//���λ�ã�������λ����0
	uint32_t CurrentPosition_Pulse;	//��ǰλ��
	uint32_t MaxPosition_Pulse;		//��ǰλ��

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

#define PWM1_PreemptionPriority  1             //�׼�
#define PWM1_SubPriority         0			   //�ײ�
#define PWM2_PreemptionPriority  1             //�׼�
#define PWM2_SubPriority         1			   //�ײ�
#define PWM3_PreemptionPriority  2             //�׼�
#define PWM3_SubPriority         0			   //�ײ�
#define PWM4_PreemptionPriority  2             //�׼�
#define PWM4_SubPriority         1			   //�ײ�
/***********************END********************************************/

//���
extern MOTOR_CONTROL_S    motor1;
extern MOTOR_CONTROL_S    motor2;
extern MOTOR_CONTROL_S    motor3;
extern MOTOR_CONTROL_SPTA motor4; 

extern volatile uint8_t USART1_TxBuffer[USART1TXSIZE];  //����1���ͻ�����
extern volatile uint16_t PTxBufferUSART11;				//����1����ǰ��λ��
extern volatile uint16_t PTxBufferUSART12;				//����1���ͺ���λ�ã�����-ǰ��=δ���͵�����
extern volatile uint16_t USART1_TxCounter ;				//����1���ͼ���
extern volatile uint16_t USART1_RxCounter ; 			//����1���ռ���
extern volatile uint16_t USART1_NbrOfDataToTransfer;	//����1Ҫ���͵����ݸ���
extern volatile uint8_t USART1_RxBuffer[USART1RXSIZE];	//����1���ջ�����
extern volatile uint16_t PRxBufferUSART11;
extern volatile uint16_t PRxBufferUSART12;
extern volatile uint8_t USART1_NbrOfDataReceived;		//����1Ҫ���յ����ݸ���
extern volatile uint8_t USART1_Received_Flag;
extern struct rt_ringbuffer rb_recv;

extern volatile uint8_t USART2_TxBuffer[USART2TXSIZE];	//����2���ͻ�����
extern volatile uint16_t PTxBufferUSART21;				//����2����ǰ��λ��
extern volatile uint16_t PTxBufferUSART22;				//����2���ͺ���λ�ã�����-ǰ��=δ���͵�����
extern volatile uint16_t USART2_TxCounter ;				//����2���ͼ���
extern volatile uint16_t USART2_RxCounter ; 			//����2���ռ���
extern volatile uint16_t USART2_NbrOfDataToTransfer;	//����2Ҫ���͵����ݸ���
extern volatile uint8_t USART2_RxBuffer[USART2RXSIZE];	//����2���ջ�����
extern volatile uint16_t PRxBufferUSART21;
extern volatile uint16_t PRxBufferUSART22;
extern volatile uint8_t USART2_NbrOfDataReceived;	     //����2Ҫ���յ����ݸ���
extern volatile uint8_t USART2_Received_Flag;


extern volatile uint8_t USART3_TxBuffer[USART3TXSIZE];	 //����3���ͻ�����
extern volatile uint16_t PTxBufferUSART31;				 //����3����ǰ��λ��
extern volatile uint16_t PTxBufferUSART32;				 //����3���ͺ���λ�ã�����-ǰ��=δ���͵�����
extern volatile uint16_t USART3_TxCounter;				 //����3���ͼ���
extern volatile uint16_t USART3_RxCounter ; 			 //����3���ռ���
extern volatile uint16_t USART3_NbrOfDataToTransfer;	 //����3Ҫ���͵����ݸ���
extern volatile uint8_t USART3_RxBuffer[USART3RXSIZE];	 //����3���ջ�����
extern volatile uint16_t PRxBufferUSART31;
extern volatile uint16_t PRxBufferUSART32;
extern volatile uint8_t USART3_NbrOfDataReceived;		 //����3Ҫ���յ����ݸ���
extern volatile uint8_t USART3_Received_Flag;			 //����3�յ����ݱ�־��1�����յ����ݣ�0��û�н��յ�����


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
