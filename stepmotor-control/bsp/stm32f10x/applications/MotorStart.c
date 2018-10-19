//包含头文件
#include "include.h" 
#include "ringbuffer.h"

MOTOR_CONTROL_S motor1;	     	 
MOTOR_CONTROL_S motor2;	     	 
MOTOR_CONTROL_S motor3;	      
MOTOR_CONTROL_SPTA motor4;

/*简单的延时函数*/
void TimerDly(unsigned int Time)
{
	unsigned int i=0;
	while(Time)
	{
		for(i=0;i<8000;i++);
		Time--;
	}
} 

/*系统时钟初始化*/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	//RCC system reset(for debug purpose) //
	RCC_DeInit();
	//RCC_HSICmd(ENABLE);					//使能内部的RC振荡器

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

/*系统初始化*/
void System_Init(void)
{
	RCC_Configuration();
}

/*电机驱动测试*/
int motortest(void)
{	
	System_Init(); 				//系统时钟初始化 72M主频，HCLK 72M   PCLK2 72M  PCLK1 36M
	
	/*初始化电机控制的GPIO口，一部分是普通的GPIO，一部分是用于PWM输出的GPIO口*/
	Initial_MotorIO();    		//电机IO口初始化 
	
	//设置各个输出的PWM参数
	Initial_PWM_Motor1();				//初始化电机1的PWM 
	Initial_PWM_Motor2();				//初始化电机2的PWM
	Initial_PWM_Motor3();				//初始化电机3的PWM
	Initial_PWM_Motor4();				//初始化电机4的TIM
	
	/*初始化电机运行参数，主要是根据S型曲线参数生成表格*/
	MotorRunParaInitial();
	
	/*将电机的驱动细分，通过控制连接THB6128的GPIO引脚的电平实现
	此外还有指定各个电机使用的定时器，GPIO，电机顺时针方向数值等参数*/
	Initial_Motor(1,M1DIV,73600);   	
    Initial_Motor(2,M2DIV,73600);   		
	Initial_Motor(3,M3DIV,73600); 	
    Initial_Motor(4,M4DIV,73600);  	
	
	/*外部中断初始化，电机复位时使用*/
	EXTI_Configuration();	
	
	/*串口初始化，调试使用*/
	USART1_Initial(); 
	USART3_Initial();
	USART1_Printfstr("APP Start-USART1\r\n");
	USART3_Printfstr("APP Start-USART3\r\n");
	rt_ringbuffer_init(&rb_recv,(unsigned char *)USART1_RxBuffer,USART1RXSIZE);

	/*单个电机启动控制*/
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
	/*电机1电机2同步控制*/
	Start_Motor12(1,500,1,10000); 
	while(motor1.running==1); 
	while(motor2.running==1); 
	/*电机1电机2同步控制*/
	Start_Motor12(1,5000,1,10000); 
	while(motor1.running==1); 
	while(motor2.running==1); 
#endif	
	/*电机串口命令控制*/
	while(1)
	{		
		Deal_Cmd();		
	}
}


