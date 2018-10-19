//����ͷ�ļ�
#include "include.h" 
#include "ringbuffer.h"

MOTOR_CONTROL_S motor1;	     	 
MOTOR_CONTROL_S motor2;	     	 
MOTOR_CONTROL_S motor3;	      
MOTOR_CONTROL_SPTA motor4;

/*�򵥵���ʱ����*/
void TimerDly(unsigned int Time)
{
	unsigned int i=0;
	while(Time)
	{
		for(i=0;i<8000;i++);
		Time--;
	}
} 

/*ϵͳʱ�ӳ�ʼ��*/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	//RCC system reset(for debug purpose) //
	RCC_DeInit();
	//RCC_HSICmd(ENABLE);					//ʹ���ڲ���RC����

  //Enable HSE //
	RCC_HSEConfig(RCC_HSE_ON);

	//Wait till HSE is ready
 	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
    // Enable Prefetch Buffer
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    //Flash 2 wait state
    FLASH_SetLatency(FLASH_Latency_2);

    //HCLK = SYSCLK
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    //PCLK2 = HCLK
    RCC_PCLK2Config(RCC_HCLK_Div1);

    //PCLK1 = HCLK/2
    RCC_PCLK1Config(RCC_HCLK_Div2);

    //PLLCLK = 8MHz/2 * 18 = 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    //Enable PLL
    RCC_PLLCmd(ENABLE);

    //Wait till PLL is ready
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

    //Select PLL as system clock source
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // Wait till PLL is used as system clock source
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
	}
}

/*ϵͳ��ʼ��*/
void System_Init(void)
{
	RCC_Configuration();
}

/*�����������*/
int motortest(void)
{	
	System_Init(); 				//ϵͳʱ�ӳ�ʼ�� 72M��Ƶ��HCLK 72M   PCLK2 72M  PCLK1 36M
	
	/*��ʼ��������Ƶ�GPIO�ڣ�һ��������ͨ��GPIO��һ����������PWM�����GPIO��*/
	Initial_MotorIO();    		//���IO�ڳ�ʼ�� 
	
	//���ø��������PWM����
	Initial_PWM_Motor1();				//��ʼ�����1��PWM 
	Initial_PWM_Motor2();				//��ʼ�����2��PWM
	Initial_PWM_Motor3();				//��ʼ�����3��PWM
	Initial_PWM_Motor4();				//��ʼ�����4��TIM
	
	/*��ʼ��������в�������Ҫ�Ǹ���S�����߲������ɱ��*/
	MotorRunParaInitial();
	
	/*�����������ϸ�֣�ͨ����������THB6128��GPIO���ŵĵ�ƽʵ��
	���⻹��ָ���������ʹ�õĶ�ʱ����GPIO�����˳ʱ�뷽����ֵ�Ȳ���*/
	Initial_Motor(1,M1DIV,73600);   	
    Initial_Motor(2,M2DIV,73600);   		
	Initial_Motor(3,M3DIV,73600); 	
    Initial_Motor(4,M4DIV,73600);  	
	
	/*�ⲿ�жϳ�ʼ���������λʱʹ��*/
	EXTI_Configuration();	
	
	/*���ڳ�ʼ��������ʹ��*/
	USART1_Initial(); 
	USART3_Initial();
	USART1_Printfstr("APP Start-USART1\r\n");
	USART3_Printfstr("APP Start-USART3\r\n");
	rt_ringbuffer_init(&rb_recv,(unsigned char *)USART1_RxBuffer,USART1RXSIZE);

	/*���������������*/
#if 1
	
	Start_Motor_S(1,M1_CLOCKWISE,100);  		 
	while(motor1.running==1);
	TimerDly(500); 
	Start_Motor_S(1,M1_CLOCKWISE,200*2);  		 
	while(motor1.running==1);
	TimerDly(500); 
	Start_Motor_S(1,M1_UNCLOCKWISE,200*20);  		 
	while(motor1.running==1);
	TimerDly(500); 
	
	Start_Motor_S(1,M1_CLOCKWISE,200*20);  		 
	while(motor1.running==1);
	TimerDly(500); 
	
	
	Start_Motor_S(2,M2_CLOCKWISE,200*20);  		 
	while(motor2.running==1);
	TimerDly(500); 
	Start_Motor_S(2,M2_UNCLOCKWISE,200*20);  		 
	while(motor2.running==1);
	TimerDly(500); 

	Start_Motor_S(3,M3_CLOCKWISE,200*20);  		 
	while(motor3.running==1);
	TimerDly(500); 
	Start_Motor_S(3,M3_UNCLOCKWISE,200*20);  	 
	while(motor3.running==1); 
	TimerDly(500);
#endif	
	Start_Motor_SPTA(4,M4_CLOCKWISE,200*20);  		 
	while(motor4.running==1);
	TimerDly(500); 
	Start_Motor_SPTA(4,M4_UNCLOCKWISE,200*20); 	 
	while(motor4.running==1); 
	TimerDly(500);

#if 0
	/*���1���2ͬ������*/
	Start_Motor12(1,500,1,10000); 
	while(motor1.running==1); 
	while(motor2.running==1); 
	/*���1���2ͬ������*/
	Start_Motor12(1,5000,1,10000); 
	while(motor1.running==1); 
	while(motor2.running==1); 
#endif	
	/*��������������*/
	while(1)
	{		
		Deal_Cmd();		
	}
}


