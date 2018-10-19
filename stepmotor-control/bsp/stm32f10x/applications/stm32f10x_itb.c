/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "STM32F10X_System.h"
#include "global_varial.h"
#include "math.h"
#include "Macro.h"
//#include <ucos_ii.h>

/* Private define ------------------------------------------------------------*/
/* CAN Master Control Register bits */
#define MCR_INRQ     ((u32)0x00000001) /* Initialization request */
#define MCR_SLEEP    ((u32)0x00000002) /* Sleep mode request */
#define MCR_TXFP     ((u32)0x00000004) /* Transmit FIFO priority */
#define MCR_RFLM     ((u32)0x00000008) /* Receive FIFO locked mode */
#define MCR_NART     ((u32)0x00000010) /* No automatic retransmission */
#define MCR_AWUM     ((u32)0x00000020) /* Automatic wake up mode */
#define MCR_ABOM     ((u32)0x00000040) /* Automatic bus-off management */
#define MCR_TTCM     ((u32)0x00000080) /* time triggered communication */

/* CAN Master Status Register bits */
#define MSR_INAK     ((u32)0x00000001)    /* Initialization acknowledge */
#define MSR_WKUI     ((u32)0x00000008)    /* Wake-up interrupt */
#define MSR_SLAKI    ((u32)0x00000010)    /* Sleep acknowledge interrupt */

/* CAN Transmit Status Register bits */
#define TSR_RQCP0    ((u32)0x00000001)    /* Request completed mailbox0 */
#define TSR_TXOK0    ((u32)0x00000002)    /* Transmission OK of mailbox0 */
#define TSR_ABRQ0    ((u32)0x00000080)    /* Abort request for mailbox0 */
#define TSR_RQCP1    ((u32)0x00000100)    /* Request completed mailbox1 */
#define TSR_TXOK1    ((u32)0x00000200)    /* Transmission OK of mailbox1 */
#define TSR_ABRQ1    ((u32)0x00008000)    /* Abort request for mailbox1 */
#define TSR_RQCP2    ((u32)0x00010000)    /* Request completed mailbox2 */
#define TSR_TXOK2    ((u32)0x00020000)    /* Transmission OK of mailbox2 */
#define TSR_ABRQ2    ((u32)0x00800000)    /* Abort request for mailbox2 */
#define TSR_TME0     ((u32)0x04000000)    /* Transmit mailbox 0 empty */
#define TSR_TME1     ((u32)0x08000000)    /* Transmit mailbox 1 empty */
#define TSR_TME2     ((u32)0x10000000)    /* Transmit mailbox 2 empty */

/* CAN Receive FIFO 0 Register bits */
#define RF0R_FULL0   ((u32)0x00000008)    /* FIFO 0 full */
#define RF0R_FOVR0   ((u32)0x00000010)    /* FIFO 0 overrun */
#define RF0R_RFOM0   ((u32)0x00000020)    /* Release FIFO 0 output mailbox */

/* CAN Receive FIFO 1 Register bits */
#define RF1R_FULL1   ((u32)0x00000008)    /* FIFO 1 full */
#define RF1R_FOVR1   ((u32)0x00000010)    /* FIFO 1 overrun */
#define RF1R_RFOM1   ((u32)0x00000020)    /* Release FIFO 1 output mailbox */

/* CAN Error Status Register bits */
#define ESR_EWGF     ((u32)0x00000001)    /* Error warning flag */
#define ESR_EPVF     ((u32)0x00000002)    /* Error passive flag */
#define ESR_BOFF     ((u32)0x00000004)    /* Bus-off flag */

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ  ((u32)0x00000001) /* Transmit mailbox request */

/* CAN Filter Master Register bits */
#define FMR_FINIT    ((u32)0x00000001) /* Filter init mode */

/* ---------------------- TIM registers bit mask ------------------------ */
#define CR1_CEN_Set                 ((u16)0x0001)
#define CR1_CEN_Reset               ((u16)0x03FE)

void SysTickHandler(void)
{
    u8 i=0;
	systime.sysmsecond++;
	  //�������ǰ�������񿽱�
	if(StartCopyCmd==TRUE)
	{
		if(systime.sysmsecond>StartCopyCmdTime)
		{
			if((systime.sysmsecond-StartCopyCmdTime)>800)
			{
				StartCopyCmd=FALSE;	
				StartCopy=TRUE;
				//CopyNewInf();							
			}
		}
		else
		{
			if((StartCopyCmdTime+1000-systime.sysmsecond)>800)
			{
				StartCopyCmd=FALSE;
				//CopyNewInf();	
				StartCopy=TRUE;						
			}	
		}	
	}			
	 //��������ʱ���Ĺ��ܣ���ʱ���Ĺ�����ר�õģ�������ʱ���Ǳ�����������ʹ��delay
	for(i=0;i<5;i++)
	{
	  	if(TaskTimer.m_timeron[i]==1)
	  	{
	     	TaskTimer.m_timercnt[i]++;
			if(TaskTimer.m_timercnt[i]>TaskTimer.m_timerval[i])
		 	{
		    	TaskTimer.m_timerout[i]=1;
				TaskTimer.m_timercnt[i]=0;
		 	}  
	  	}
	}
	//�������д򿪱úͷ���һ��ʱ���ִ����һ������
	if(TaskTimer.m_timerout[0]==1)
	{
	 	TaskTimer.m_timerout[0]=0;
	 	TaskTimer.m_timeron[0]=0;

		//�رշ���
	 	//TurnOffPump(1); GPIO_ResetBits(GPIOE,GPIO_Pin_10);
	 	//TurnOffValve(2);  GPIO_ResetBits(GPIOE,GPIO_Pin_13);
	 	//TurnOffValve(1);	GPIO_ResetBits(GPIOE,GPIO_Pin_12);
		//set GPIO	GPIOx->BSRR = GPIO_Pin;
		//reset GPIOx->BRR = GPIO_Pin;
		GPIOE->BRR=GPIO_Pin_10;	
		GPIOE->BRR=GPIO_Pin_13;
		GPIOE->BRR=GPIO_Pin_12;

		//TaskTimer.m_timercnt[0]=0;
	 	if(AddSubstrateCmd==TRUE)
	 	{
	 		AddSubstrateHandState++;
	 	}
	}
	//��������ͣ����ʱ��
	if(TaskTimer.m_timerout[1]==1)
	{
	 	TaskTimer.m_timerout[1]=0;
	 	TaskTimer.m_timeron[1]=0; 
		//TaskTimer.m_timercnt[1]=0;	 	
	 	if(TestProbeDownCmd==TRUE)
	 	{
	 		TestHandState++;
	 	}
	}
	//���ڵ����Ͽ��Ƶ���ʱ
	if(TaskTimer.m_timerout[2]==1)
	{
	 	TaskTimer.m_timerout[2]=0;
	 	TaskTimer.m_timeron[2]=0;
		TaskTimer.m_timercnt[2]=0;
		motor4.flag=0;
	 	if(AddSubstrateCmd==TRUE)
	 	{
	 		AddSubstrateHandState++;
	 	}
	}
	//��������һ�η�Ӧ���жϺ󣬼��һ��ʱ����ж�ʹ��
	if(TaskTimer.m_timerout[3]==1)
	{
	 	TaskTimer.m_timerout[3]=0;
	 	TaskTimer.m_timeron[3]=0;
		TaskTimer.m_timercnt[3]=0;
	 	//���ⲿ�ж�
	 	Switch5Enable=TRUE;
	} 
    if(systime.sysmsecond<1000)  return;

	 if(systime.sysmsecond>=1000)
	   {
	    totalsecond++;
	    systime.syssecond++;
		SystickFlag=1;
		systime.sysmsecond=0;
	   }
	 if(systime.syssecond>=60)
	   {
	     systime.syssecond=0;
	     systime.sysminute++;
		}
	 if(systime.sysminute>=60)
	   {
	    systime.sysminute=0;
	     systime.syshour++;
	   }
	 if(systime.syshour>=60)
	   systime.syshour=0;
}
void TIM1_UP_IRQHandler(void)
{
   	//���ж�
   	TIM1->SR = (u16)~TIM_FLAG_Update;
   	//TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); //���ж�
   
 	//********************************************************���4*****************************/ 
 	if(1==motor4.en)
	{ 
	   motor4.PulsesHaven++; //���������
	   motor4.pulsecount++;  //�Ը�Ƶ������������������	
	   if(motor4.pulsecount>=motor4.Step_Table[motor4.CurrentPostion])
	   { 
		    if(motor4.PulsesHaven<=motor4.StartSteps)//�𲽽׶�
			{
			   if(motor4.CurrentPostion<motor4.StartTableLength)
			   {
			       motor4.CurrentPostion++;
			       motor4.pulsecount=0;
			       if(motor4.CurrentPostion>=motor4.StartTableLength)motor4.CurrentPostion=motor4.StartTableLength-1;  
			   }		  
			}
			if(motor4.PulsesGiven-motor4.PulsesHaven<motor4.StopSteps) //ֹͣ�׶�
			{
			   if(motor4.CurrentPostion<motor4.StartTableLength)
			   {
			     motor4.CurrentPostion=36-motor4.CurrentPostion;  
			   }
			   motor4.CurrentPostion++;
			   motor4.pulsecount=0;
			   if(motor4.CurrentPostion>=motor4.StartTableLength+motor4.StopTableLength)motor4.CurrentPostion=motor4.StartTableLength+motor4.StopTableLength-1;
			} 	
			TIM1->ARR = motor4.Counter_Table[motor4.CurrentPostion] ; //��������
			TIM1->CCR1 =( motor4.Counter_Table[motor4.CurrentPostion])>>1;       //����ռ�ձ�	    
	  }	 	  
	  //��תԤ����������ֹͣ��flag=0�����Խ�����һ����ת
	  if(motor4.PulsesHaven>=motor4.PulsesGiven)
	  {
		     motor4.en=0;
		     motor4.flag=0;
			 TIM1->CR1 &= CR1_CEN_Reset;
			 if(AddSubstrateCmd==TRUE)
			 {
			    AddSubstrateHandState++;
			 } 
			 if(motor4.rstflg==1)         //��λ��־û�б���λ�����жϣ���λ�쳣
			 {
			    MotorRstException|=0x08;
			 }
	  }	
	  	  		  	 
	}//���ʹ������µ����۽���   

}
void TIM2_IRQHandler(void)
{    
   //TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update); 
   //���ж� 
   TIM2->SR = (u16)~TIM_FLAG_Update;     

   //********************************************************���2*****************************/
  if(1==motor2.en)
	{ 
	   motor2.PulsesHaven++; //���������
	   StepsOfDown_TRUE++;
	   if(upstepcntcmd)
	   StepsOfDownExtra_TRUE++;
	   if(upstepcntcmd1)
	   StepsOfDownExtra_TRUE1++;
	   motor2.pulsecount++;  //�Ը�Ƶ������������������	
	  if(motor2.pulsecount>=motor2.Step_Table[motor2.CurrentPostion])
	  { 
	    if(motor2.PulsesHaven<=motor2.StartSteps)//�𲽽׶�
		{
		   if(motor2.CurrentPostion<motor2.StartTableLength)
		   {
		  	 motor2.CurrentPostion++;
		  	 motor2.pulsecount=0;
		  	 if(motor2.CurrentPostion>=motor2.StartTableLength)motor2.CurrentPostion=motor2.StartTableLength-1;
		   }
		}
		if(motor2.PulsesGiven-motor2.PulsesHaven<motor2.StopSteps) //ֹͣ�׶�
		{
		   if(motor2.CurrentPostion<motor2.StartTableLength)
		   {
		     motor2.CurrentPostion=36-motor2.CurrentPostion;  
		   }
		   motor2.CurrentPostion++;
		   motor2.pulsecount=0;
		   if(motor2.CurrentPostion>=motor2.StartTableLength+motor2.StopTableLength)
		     motor2.CurrentPostion=motor2.StartTableLength+motor2.StopTableLength-1;
		}  		
	
		TIM2->ARR = motor2.Counter_Table[motor2.CurrentPostion] ; //��������
		TIM2->CCR1 =( motor2.Counter_Table[motor2.CurrentPostion])>>1;       //����ռ�ձ�	    
	  }	 	      
	  
	  //��תԤ����������ֹͣ��flag=0�����Խ�����һ����ת
	  if(motor2.PulsesHaven>=motor2.PulsesGiven)
	  {
	     motor2.en=0;
	     motor2.flag=0;
	     TIM2->CR1 &= CR1_CEN_Reset;
		 if(AddSubstrateCmd==TRUE)
		 {
		    AddSubstrateHandState++;
		 } 
		
		 if(motor2.rstflg==1)         //��λ��־û�б���λ�����жϣ���λ�쳣
		 {
		    MotorRstException|=0x02;
		 }
	  }	
	  	  		  	 
	}//���ʹ������µ����۽���
}

void TIM3_IRQHandler(void)
{    
   //TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update); 
   //���ж� 
   TIM3->SR = (u16)~TIM_FLAG_Update;     

   //********************************************************���2*****************************/
  if(1==motor3.en)
	{ 
	   motor3.PulsesHaven++; //���������
	   motor3.pulsecount++;  //�Ը�Ƶ������������������	
	  if(motor3.pulsecount>=motor3.Step_Table[motor3.CurrentPostion])
	  { 
	    if(motor3.PulsesHaven<=motor3.StartSteps)//�𲽽׶�
		{
		   if(motor3.CurrentPostion<motor3.StartTableLength)
		   {
		  	 motor3.CurrentPostion++;
		  	 motor3.pulsecount=0;
		  	 if(motor3.CurrentPostion>=motor3.StartTableLength)motor3.CurrentPostion=motor3.StartTableLength-1;
		   }
		}
		if(motor3.PulsesGiven-motor3.PulsesHaven<motor3.StopSteps) //ֹͣ�׶�
		{
		   if(motor3.CurrentPostion<motor3.StartTableLength)
		   {
		     motor3.CurrentPostion=36-motor3.CurrentPostion;  
		   }
		   motor3.CurrentPostion++;
		   motor3.pulsecount=0;
		   if(motor3.CurrentPostion>=motor3.StartTableLength+motor3.StopTableLength)
		     motor3.CurrentPostion=motor3.StartTableLength+motor3.StopTableLength-1;
		}  		
	
		TIM3->ARR = motor3.Counter_Table[motor3.CurrentPostion] ; //��������
		TIM3->CCR1 =( motor3.Counter_Table[motor3.CurrentPostion])>>1;       //����ռ�ձ�	    
	  }	 	      
	  
	  //��תԤ����������ֹͣ��flag=0�����Խ�����һ����ת
	  if(motor3.PulsesHaven>=motor3.PulsesGiven)
	  {
	     motor3.en=0;
	     motor3.flag=0;
	     TIM3->CR1 &= CR1_CEN_Reset;
		 if(TestProbeDownCmd==TRUE)
		 {
		    TestHandState++;
		 } 
		 if(motor3.rstflg==1)         //��λ��־û�б���λ�����жϣ���λ�쳣
		 {
		    MotorRstException|=0x04;
		 }
	  }	
	  	  		  	 
	}//���ʹ������µ����۽���
}

#if 0
void TIM3_IRQHandler(void)
{    
    TIM3->SR = (u16)~TIM_FLAG_Update;
    //GPIO_ResetBits(GPIOA,GPIO_Pin_6);                                                        //��λʱ���ź�
    GPIOA->BRR = GPIO_Pin_6;
	if(motor3.ActualVelocity!=motor3.TargetVelocity)
    {
      	motor3.VelAccumulator+=motor3.ActualAcceleration;       
      	motor3.VelAdd = motor3.VelAccumulator >> 17;            
      	motor3.VelAccumulator-=motor3.VelAdd << 17;
      	if(motor3.ActualVelocity<motor3.TargetVelocity)
       	{
       		motor3.ActualVelocity=MIN(motor3.ActualVelocity+motor3.VelAdd, motor3.TargetVelocity);
       	}
      	else if(motor3.ActualVelocity>motor3.TargetVelocity)
       	{
         	motor3.ActualVelocity=MAX(motor3.ActualVelocity-motor3.VelAdd, motor3.TargetVelocity);
       	}
    }
   	else
    {
      	motor3.VelAccumulator=0;
      	motor3.VelAdd=0;
    }
    switch(motor3.RampState)
    { 
      	case RAMP_IDLE:                                                    
	        if(motor3.ActualPosition<motor3.TargetPosition)        
	        {
	          motor3.TargetVelocity=motor3.MaxPositioningSpeed;
	          motor3.RampState=RAMP_ACCELERATE;
	          motor3.AccelerationSteps=motor3.ActualPosition;
	        }
	        else if(motor3.ActualPosition>motor3.TargetPosition)   
	        {
	          motor3.TargetVelocity=-motor3.MaxPositioningSpeed;
	          motor3.RampState=RAMP_ACCELERATE;
	          motor3.AccelerationSteps=motor3.ActualPosition;
	        }
        	break;
      case RAMP_ACCELERATE:                                            
	        if(abs(motor3.ActualVelocity)==motor3.MaxPositioningSpeed)
	        {                                                              
	          motor3.AccelerationSteps=abs(motor3.ActualPosition-motor3.AccelerationSteps)+1;
	          motor3.RampState=RAMP_DRIVING;
	        }
	        else if(abs(motor3.ActualPosition-motor3.AccelerationSteps)>=abs(motor3.TargetPosition-motor3.ActualPosition))   
	        {                                                              
	          motor3.TargetVelocity=0;
	          motor3.RampState=RAMP_DECELERATE;
	        }
	        break;	 
      case RAMP_DRIVING:                                               
	       if((abs(motor3.TargetPosition-motor3.ActualPosition) <= motor3.AccelerationSteps) ||
	           ((motor3.TargetVelocity<0) && (motor3.ActualPosition<motor3.TargetPosition)) ||
	           ((motor3.TargetVelocity>0) && (motor3.ActualPosition>motor3.TargetPosition)))
	        {                                                              
	          motor3.TargetVelocity=0;
	          motor3.RampState=RAMP_DECELERATE;
	        }
	        break;
      case RAMP_DECELERATE:   
	        if(motor3.ActualPosition==motor3.TargetPosition)
	        {                                                              
	          motor3.TargetVelocity=motor3.ActualVelocity=0;
	          motor3.RampState=RAMP_IDLE;
	        }
	        if(motor3.ActualVelocity==0)                              
	        {
	          motor3.RampState=RAMP_IDLE;
	          if(motor3.ActualPosition==motor3.TargetPosition) 
	          {
	             motor3.TargetReachedFlag=TRUE;
	             motor3.flag=0;
				 TIM3->CR1 &= CR1_CEN_Reset;
				 if(TestProbeDownCmd==TRUE)
				 {
				    TestHandState++;
				 } 
				
				 if(motor3.rstflg==1)         //��λ��־û�б���λ�����жϣ���λ�쳣
				 {
				    MotorRstException|=0x04;
				 }
	             
	          }
	        }
	        break;
    }
    motor3.PosAccumulator += motor3.ActualVelocity;            
	motor3.PosAdd = motor3.PosAccumulator >> 17;               
	motor3.PosAccumulator -= motor3.PosAdd << 17;
  	if(motor3.PosAdd==0) 
    	return;                                                            
    else
    {
    	motor3.ActualPosition+=motor3.PosAdd;
    	//GPIO_SetBits(GPIOA,GPIO_Pin_6);                                                //��������������
		GPIOA->BSRR = GPIO_Pin_6;
		/*
    	if((Motor_Sample_flag)&(Motor_Sample.type_state==1))                                        //��λ
	      {
	        Motor_Sample.ActualPosition=Motor_Sample.TargetPosition=0;
	        Motor_Sample.RampState=RAMP_IDLE;
	        Motor_Sample.TargetVelocity=Motor_Sample.ActualVelocity=0;
	        Motor_Sample.flag=0;
	        TIM_Cmd(TIM3, DISABLE);                                                                  //���ֹͣ���� 
	      }
		  */
    }
 }
 #endif

 void TIM4_IRQHandler(void)
{
 
   //TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);      
   TIM4->SR = (u16)~TIM_FLAG_Update;
   //********************************************************���1*****************************/
  if(1==motor1.en)
	{ 
	   motor1.PulsesHaven++; //���������
	   motor1.pulsecount++;  //�Ը�Ƶ������������������	
	  if(motor1.pulsecount>=motor1.Step_Table[motor1.CurrentPostion])
	  { 
	    
	    if(motor1.PulsesHaven<=motor1.StartSteps)//�𲽽׶�
		{
		   if(motor1.CurrentPostion<motor1.StartTableLength)
		   {
		   	  motor1.CurrentPostion++;
		      motor1.pulsecount=0;
		      if(motor1.CurrentPostion>=motor1.StartTableLength)motor1.CurrentPostion=motor1.StartTableLength-1;
		   }
		}
		if(motor1.PulsesGiven-motor1.PulsesHaven<motor1.StopSteps) //ֹͣ�׶�
		{
		   if(motor1.CurrentPostion<motor1.StartTableLength)
		   {
		     motor1.CurrentPostion=142-motor1.CurrentPostion;
		   }
		  motor1.CurrentPostion++;
		  motor1.pulsecount=0;
		  if(motor1.CurrentPostion>=motor1.StartTableLength+motor1.StopTableLength)
		     motor1.CurrentPostion=motor1.StartTableLength+motor1.StopTableLength-1;
		}  		
	
		TIM4->ARR = motor1.Counter_Table[motor1.CurrentPostion] ; //��������
		TIM4->CCR1 =( motor1.Counter_Table[motor1.CurrentPostion])>>1;       //����ռ�ձ�	    
	  }	 	      
	  
	  //��תԤ����������ֹͣ��flag=0�����Խ�����һ����ת
	  if(motor1.PulsesHaven>=motor1.PulsesGiven)
	  {
	     motor1.en=0;
	     motor1.flag=0;
	     TIM4->CR1 &= CR1_CEN_Reset;
		 if(AddSubstrateCmd==TRUE)
		 {
		    AddSubstrateHandState++;
		 } 		
		 if(motor1.rstflg==1)         //��λ��־û�б���λ�����жϣ���λ�쳣
		 {
		    MotorRstException|=0x01;
		 }
	  }	
	  	  		  	 
	}//���ʹ������µ����۽���
  
}
void TIM8_UP_IRQHandler(void)
{
 	TIM8->SR = (u16)~TIM_FLAG_Update;
	testtimer5=1;
}	 
void EXTI0_IRQHandler(void)
{
  u32 enablestatus = 0;   
  enablestatus =  EXTI->IMR & EXTI_Line0;
  if (((EXTI->PR & EXTI_Line0) != (u32)RESET) && (enablestatus != (u32)RESET))
  {       
	 //EXTI_ClearITPendingBit(EXTI_Line0);   //�Ƚ��ж����㣬������BUG
	 EXTI->PR = EXTI_Line0;
	 if(FALSE==Switch1Enable)
	    return;
	 
	 Switch1Enable=FALSE;   //�����жϴ����ر�ʹ�ܣ���Systick����һ��ʱ����
     EXTIFlag++;
	 //if(EXTIFlag==1)TurnOnLED(1);
	 //if(EXTIFlag==2)TurnOnLED(2);
     EXTINum=1;
     if(motor1.rstflg==1)
     {   
	    motor1.PulsesHaven=motor1.PulsesGiven-PCMCVarial.PCMCmotor1.PCMCM1RstAfter; 		 	     
		motor1.rstflg=0;
	 } 		 	 
  }	 
}

void EXTI1_IRQHandler(void)
{
   u32 enablestatus = 0;    
   enablestatus =  EXTI->IMR & EXTI_Line1;
   if (((EXTI->PR & EXTI_Line1) != (u32)RESET) && (enablestatus != (u32)RESET))
   { 
      //EXTI_ClearITPendingBit(EXTI_Line1);
	  EXTI->PR = EXTI_Line1;
	  if(FALSE==Switch2Enable)
	    return;
	  Switch2Enable=FALSE;   //�����жϴ����ر�ʹ�ܣ���Systick����һ��ʱ����
      EXTIFlag++;
      EXTINum=2;
      if(motor2.rstflg==1)
     {
	    motor2.PulsesHaven=motor2.PulsesGiven-PCMCVarial.PCMCmotor2.PCMCM2RstAfter;
		 //TIM_Cmd(TIM2, DISABLE);		  //DISABLE
       // motor2.en=0; //ֹͣ���2
       // motor2.flag=0; //���ǵ����ת��־Ҫ���㣬����������;
	   motor2.rstflg=0;
	  
     }
   }  
}

void EXTI2_IRQHandler(void)
{
    u32 enablestatus = 0;   
   	enablestatus =  EXTI->IMR & EXTI_Line2;
   	if (((EXTI->PR & EXTI_Line2) != (u32)RESET) && (enablestatus != (u32)RESET))
    { 
	   
	   //EXTI_ClearITPendingBit(EXTI_Line2);
	   EXTI->PR = EXTI_Line2;
	   if(FALSE==Switch3Enable)
	    return;
	   Switch3Enable=FALSE;   //�����жϴ����ر�ʹ�ܣ���Systick����һ��ʱ����
       EXTIFlag++;
       EXTINum=3;
       if(motor3.rstflg==1)
	   {
	       motor3.PulsesHaven=motor3.PulsesGiven-(PCMCVarial.PCMCmotor3.PCMCM3RstAfter);//20000;//PCMRVarial.PCMRmotor3.PCMRM3RstAfter;;	 //��Χ25000-32000
		   TurnOnLED(1);
	       //motor3.en=0; //ֹͣ���3
	       //motor3.flag=0; //���ǵ����ת��־Ҫ���㣬����������;
		   motor3.rstflg=0;
	   }
    }
}

void EXTI4_IRQHandler(void)
{
   u32 enablestatus = 0;   
   enablestatus =  EXTI->IMR & EXTI_Line4;
   if (((EXTI->PR & EXTI_Line4) != (u32)RESET) && (enablestatus != (u32)RESET))
   { 
       
	   //EXTI_ClearITPendingBit(EXTI_Line4);
	   EXTI->PR = EXTI_Line4;
	   if(FALSE==Switch4Enable)
	    return;
	 Switch4Enable=FALSE;   //�����жϴ����ر�ʹ�ܣ���Systick����һ��ʱ����
     EXTIFlag++;
     EXTINum=4;  
	 if(motor4.rstflg==1)
	 {
	     motor4.PulsesHaven=motor4.PulsesGiven-128;
	     //motor4.en=0; //ֹͣ���4
	     //motor4.flag=0; //���ǵ����ת��־Ҫ���㣬����������;
		 motor4.rstflg=0;
	 }  
  }
}

void EXTI9_5_IRQHandler(void)
{
   //���̴���
   u32 enablestatus = 0;   
   enablestatus =  EXTI->IMR & EXTI_Line5;
   if (((EXTI->PR & EXTI_Line5) != (u32)RESET) && (enablestatus != (u32)RESET))
   { 
	   //EXTI_ClearITPendingBit(EXTI_Line5);
	   EXTI->PR = EXTI_Line5;
	   if(FALSE==Switch5Enable)
	    return;
      Switch5Enable=FALSE;   //�����жϴ����ر�ʹ�ܣ���Systick����һ��ʱ����
      EXTIFlag++;
      EXTINum=5; 
	 // if(StartTest==TRUE)
	  {
	   ReactionPlateCome=TRUE; 
	  }   
   }
}
void EXTI3_IRQHandler(void)
{
   unsigned int tmp=0,tmp1=0,i;
   	u32 enablestatus = 0;   
   	enablestatus =  EXTI->IMR & EXTI_Line3;
   	if (((EXTI->PR & EXTI_Line3) != (u32)RESET) && (enablestatus != (u32)RESET))
   	{ 		 
	      //EXTI_ClearITPendingBit(EXTI_Line3);
		  //���ж�
		  EXTI->PR = EXTI_Line3;
		  if(FALSE==Switch6Enable)
		  {
		     	return; 
		  }		    
		  Switch6Enable=FALSE;        //�����жϴ����ر�ʹ�ܣ���Systick����һ��ʱ����
	
		  if((LiquidIntEnable==FALSE)&&(CollisionIntEnable==FALSE))  //��е�۲������½�ʱ�������жϣ�ֱ�ӷ���
		  {
		     	return;
		  }
		  if(LiquidIntEnable==TRUE)
		  {
		      //�رն�ʱ��2
		  	  TIM2->CR1 &= CR1_CEN_Reset;
		      //LiquidInt++;
			  LiquidIntEnable=FALSE;      //��Ӧһ���жϺ�����ر�
			  if(motor2.en==0)
			  {
			  	 TIM2->CR1 |= CR1_CEN_Set;	
			     return;		
			  }	 
			 
			  StepsOfDown=motor2.PulsesHaven;  //��¼�½��Ĳ���	
			  StepsOfDown1= motor2.PulsesHaven; 
			  if(StepsOfDown<=MAXSUBSHIGH)
			  {
				 TIM_Cmd(TIM3, DISABLE);	
				 TIM_Cmd(TIM4, DISABLE);
				 TIM_Cmd(TIM1, DISABLE);
				 TIM_Cmd(TIM2, DISABLE);
				 TurnOnLED(1);
				 while(1);
			  }
			  //StepsOfDown_TRUE=motor2.PulsesHaven;
			   
			  StepsOfDown_TRUE1=StepsOfDown_TRUE;
			  StepsOfDown_TRUE=0;
			  StepsOfDownExtra_TRUE=0;
			  upstepcntcmd=1;
			  tmp=9*PCMCVarial.PCMCmotor2.PCMCM2ReaLiqAfterDown;
			  StepsOfDown+=tmp;
			  StepsOfDown1+=tmp;
			  motor2.PulsesHaven=motor2.PulsesGiven-tmp; 
			  pos1=motor2.CurrentPostion; 
			 
			  //�����Ƿ�ֹҺ��ܵͣ��������Һ��ʱ�Ѿ����٣���ʱ��Ҫ���٣�����������ٶȾ�����
			  if(tmp>motor2.StopSteps)     //��Ҫ���еĲ������ڼ��ٲ���
			  {
					if(motor2.CurrentPostion>18) //��ʱ�������״̬
					{
						//motor2.CurrentPostion=36-motor2.CurrentPostion; //�����������CurrentPostion=11,�ٶȺ���������ʱ�Ȳ��������������Ҳ�����������������һֱ�Ե�������
						motor2.CurrentPostion=18;
					}
			  }
			  else
			  {
					tmp1=0;
					for(i=24;i>0;i--)
					{  
					  	tmp1+=motor2.Step_Table[i];
					   	if(tmp1>tmp)
					    {
					     	motor2.CurrentPostion=i;
						 	break; 
					    }
					}
					if(i<18)
					{
					   	motor2.CurrentPostion=18;
					}
					else
					{
					  	motor2.CurrentPostion=i; 
					} 
			  }
			  pos2=motor2.CurrentPostion;
			  TIM2->CR1 |= CR1_CEN_Set;	
		  }
		  if(CollisionIntEnable==TRUE)	  
		  {
		     CollisionIntEnable=FALSE;
//			 motor1.en=0;
//			 motor2.en=0;
//			 motor3.en=0;
//			 motor4.en=0;
//			 TIM_Cmd(TIM3, DISABLE);	
//			 TIM_Cmd(TIM4, DISABLE);
//			 TIM_Cmd(TIM1, DISABLE);
//			 TIM_Cmd(TIM2, DISABLE);
//			 while(1);	
		  }    
	      EXTINum=6;  
   	}
}



/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
    //CanRxMsg RxMessage;
    uint8_t i,TransmitMailbox;
    unsigned int SendAddr=0;
    unsigned int Serial=0;

    //CAN_Receive(CAN_FIFO0, &RxMessage);
  	/* Get the Id */
	RxMessage.IDE = (u8)0x04 & CAN->sFIFOMailBox[CAN_FIFO0].RIR;
	if (RxMessage.IDE == CAN_ID_STD)
	{
		RxMessage.StdId = (u32)0x000007FF & (CAN->sFIFOMailBox[CAN_FIFO0].RIR >> 21);
	}
	else
	{
		RxMessage.ExtId = (u32)0x1FFFFFFF & (CAN->sFIFOMailBox[CAN_FIFO0].RIR >> 3);
	}   
  	RxMessage.RTR = (u8)0x02 & CAN->sFIFOMailBox[CAN_FIFO0].RIR;  
	/* Get the DLC */
	RxMessage.DLC = (u8)0x0F & CAN->sFIFOMailBox[CAN_FIFO0].RDTR;	
	/* Get the FMI */
	RxMessage.FMI = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDTR >> 8);

	/* Get the data field */
	RxMessage.Data[0] = (u8)0xFF & CAN->sFIFOMailBox[CAN_FIFO0].RDLR;
	RxMessage.Data[1] = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDLR >> 8);
	RxMessage.Data[2] = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDLR >> 16);
	RxMessage.Data[3] = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDLR >> 24);
	
	RxMessage.Data[4] = (u8)0xFF & CAN->sFIFOMailBox[CAN_FIFO0].RDHR;
	RxMessage.Data[5] = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDHR >> 8);
	RxMessage.Data[6] = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDHR >> 16);
	RxMessage.Data[7] = (u8)0xFF & (CAN->sFIFOMailBox[CAN_FIFO0].RDHR >> 24);

  	/* Release the FIFO */
  	//CAN_FIFORelease(CAN_FIFO0); 
  	CAN->RF0R = RF0R_RFOM0;
    SendAddr=RxMessage.ExtId&0x3F;          //��6λΪĿ�ĵص�ַ
    USART1_Printfstr("\r\nCAN RCV \r\n"); 
    if(SendAddr!=LOCALMODEL)
    {
        USART1_Printfstr("\r\nCAN RCV Wrong ID_1\r\n");   
    }
    if(RxMessage.Data[0]!=LOCALMODEL)
    {
        USART1_Printfstr("\r\nCAN RCV Wrong ID_2\r\n");   
    }          
    Serial=(RxMessage.ExtId&0x3FFFC0)>>6;   //�õ����к�
    if(Serial==LastCANSeries)
    {
        //�ظ���������
        USART1_Printfstr("\r\nCAN RCV Repeated packets\r\n"); 
        //return;
    }
    LastCANSeries=Serial;
    CurrentCANRxSeries++;
    if(Serial!=CurrentCANRxSeries)
    {
        //��������
         USART1_Printfstr("\r\nCAN Lost packets\r\n"); 
        //�˴�����
        
        CurrentCANRxSeries=Serial;    
    }   
    if( (RxMessage.IDE==CAN_ID_EXT)&& (RxMessage.DLC==8) ) //SampleModel 
    {  
	   canrx++;	
	   //����������Ӧ���������Ӧ��֡��ֱ�ӷ���
	    if(RxMessage.Data[1]==0x03)
		{
			AckFram.ModuleNum=RxMessage.Data[0];
			AckFram.DataNature=RxMessage.Data[1];
			AckFram.Cmd=RxMessage.Data[2];
			return ; 
		}   	   
		for(i=0;i<8;i++)
		{
		  CanRevBuff[PReceiveBufferCAN2+i]=RxMessage.Data[i];
		}
		CanRevFlag++;	
        CanRevLen=RxMessage.DLC;
		PReceiveBufferCAN2+=RxMessage.DLC;
		if(PReceiveBufferCAN2>=CANRXBUFSIZE)
		{
		   PReceiveBufferCAN2=0;  
		}
		if(RxMessage.Data[1]==0x02)	           //����д������Ӧ�𣬶��ڶ������ﲻ����������Ӧ�ò�
		{
		     if((RxMessage.Data[2]==0x05)||(RxMessage.Data[2]==0x15)||(RxMessage.Data[2]==0x10)||(RxMessage.Data[2]==0x12)||(RxMessage.Data[2]==0x08)||(RxMessage.Data[2]==0x18)||(RxMessage.Data[2]==0xF0))	   //����5��15������ǻض�У��,���ݿ���������ȡ�������ﲻ��Ӧ��
			    return;
            RxMessage.Data[0]=localaddr;  
	 	    RxMessage.Data[1]=0x11;
		    CAN_SendData(RxMessage.Data,8,PACKETS_TYPE_NORMAL);		
		}
	    if(RxMessage.Data[1]==0x03)	           //�������Ӧ����
		{
		     if((RxMessage.Data[2]==0x13)||(RxMessage.Data[2]==0x23))	   //����3��Ӧ����������ͽ������������
			 {
			   CuptoTest[RxMessage.Data[3]-1].m_AckFlag++;
			 }		    
		}
	 }//end if	 
}
void USB_HP_CAN_TX_IRQHandler(void)	      
{
   	  uint8_t i = 0;
	  u8 TransmitMailbox = 0;

	   if(PTxBufferCAN1 == PTxBufferCAN2) //���ݴ�����Ϻ���ж�
	   {
		  //PTxBufferCAN1=PTxBufferCAN2=0;
    	  CAN->IER &= ~CAN_IT_TME;
		  return;
	   } 
	   //����3�仰�������������жϣ�ò�Ʋ���������������ֻ�з�������жϣ�û�취����������
	  if(RESET!=(CAN->TSR&TSR_RQCP0))
	  {
	  	 CAN->TSR = TSR_RQCP0;
	  }
	  if(RESET!=(CAN->TSR&TSR_RQCP1))
	  {
	  	 CAN->TSR = TSR_RQCP1;
	  }
	  if(RESET!=(CAN->TSR&TSR_RQCP2))
	  {
	  	 CAN->TSR = TSR_RQCP2;
	  }
	  CurrentCANTxSeries++;   
	  TxMessage.ExtId=(CanTxBuff[PTxBufferCAN1+8]<<22)|(CurrentCANTxSeries<<6)|MainModel|localaddr;//MainModel;//LOCALMODEL;		  
	  for(i=0;i<8;i++)
	  {		   
		TxMessage.Data[i]=CanTxBuff[PTxBufferCAN1+i]; 	
	  }	
	  	
	  /* Select one empty transmit mailbox */
	  if ((CAN->TSR&TSR_TME0) == TSR_TME0)
	  {
	    TransmitMailbox = 0;
	  }
	  else if ((CAN->TSR&TSR_TME1) == TSR_TME1)
	  {
	    TransmitMailbox = 1;
	  }
	  else if ((CAN->TSR&TSR_TME2) == TSR_TME2)
	  {
	    TransmitMailbox = 2;
	  }
	  else
	  {
	    TransmitMailbox = CAN_NO_MB;
	  }
	
	  if (TransmitMailbox != CAN_NO_MB)
	  {
	    /* Set up the Id */
	    CAN->sTxMailBox[TransmitMailbox].TIR &= TMIDxR_TXRQ;
	    if (TxMessage.IDE == CAN_ID_STD)
	    {
	      TxMessage.StdId &= (u32)0x000007FF;
	      TxMessage.StdId = TxMessage.StdId << 21;
	      
	      CAN->sTxMailBox[TransmitMailbox].TIR |= (TxMessage.StdId | TxMessage.IDE |
	                                               TxMessage.RTR);
	    }
	    else
	    {
	      TxMessage.ExtId &= (u32)0x1FFFFFFF;
	      TxMessage.ExtId <<= 3;
	
	      CAN->sTxMailBox[TransmitMailbox].TIR |= (TxMessage.ExtId | TxMessage.IDE | 
	                                               TxMessage.RTR);
	    }
	    
	    /* Set up the DLC */
	    TxMessage.DLC &= (u8)0x0000000F;
	    CAN->sTxMailBox[TransmitMailbox].TDTR &= (u32)0xFFFFFFF0;
	    CAN->sTxMailBox[TransmitMailbox].TDTR |= TxMessage.DLC;
	
	    /* Set up the data field */
	    CAN->sTxMailBox[TransmitMailbox].TDLR = (((u32)TxMessage.Data[3] << 24) | 
	                                             ((u32)TxMessage.Data[2] << 16) |
	                                             ((u32)TxMessage.Data[1] << 8) | 
	                                             ((u32)TxMessage.Data[0]));
	    CAN->sTxMailBox[TransmitMailbox].TDHR = (((u32)TxMessage.Data[7] << 24) | 
	                                             ((u32)TxMessage.Data[6] << 16) |
	                                             ((u32)TxMessage.Data[5] << 8) |
	                                             ((u32)TxMessage.Data[4]));
	
	    /* Request transmission */
		PTxBufferCAN1+=9; 
		if(PTxBufferCAN1>=CANTXBUFSIZE)
		{
			PTxBufferCAN1=0;	
		}
	    CAN->sTxMailBox[TransmitMailbox].TIR |= TMIDxR_TXRQ;
	  }
   
}
void USART1_IRQHandler(void)
{
 	  u32 usartxbase = 0x00;
	  u32 bitpos = (u32)0x01 << (USART_IT_RXNE >> 0x08), itmask = (u32)0x01 << (USART_IT_RXNE & 0x001F);//usartreg =(((u8)USART_IT_RXNE) >> 0x05);; 
	  
	  //�����жϵĴ���	 
	  itmask &= USART1->CR1;  
	  bitpos &= USART1->SR;	
	  if ((itmask != (u16)RESET)&&(bitpos != (u16)RESET))
	  {
	    
	    	USART1_RxBuffer[USART1_RxCounter++] = USART1->DR&(u16)0x01FF;
	    	USART1_NbrOfDataReceived++;	
			if(USART1_RxCounter>1&&(USART1_RxBuffer[USART1_RxCounter-1]==0x0a)&&(USART1_RxBuffer[USART1_RxCounter-2]==0x0d))	 
			{
		 		 USART1_Received_Flag=1;
			}  
	  }
  
     //�����жϵĴ���	 	 	  							  
	  itmask = (u32)0x01 << (USART_IT_TXE & ((u16)0x001F));	 	 
	  itmask &= USART1->CR1;  	
	  bitpos = (u32)0x01 << (USART_IT_TXE >> 0x08);	  
	  bitpos &= USART1->SR;	 

	  if ((itmask != (u16)0)&&(bitpos != (u16)0))
	  { 	   
			USART1->DR =USART1_TxBuffer[PTxBufferUSART11++];
	
	    	if(PTxBufferUSART11 >= PTxBufferUSART12)
	    	{
	     
		  		PTxBufferUSART11=PTxBufferUSART12=0;
				usartxbase=USART1_BASE+0x0C;
				itmask = (((u32)0x01) << (USART_IT_TXE & 0x001F)); 					 				
				*(vu32*)usartxbase &= ~itmask; 		  
	    	} 
		  
	  }  
}
/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
     u32 usartxbase=0x0000; 
     u32 bitpos = (u32)0x01 << (USART_IT_RXNE >> 0x08), itmask = (u32)0x01 << (USART_IT_RXNE & 0x001F);
   
     itmask &= USART3->CR1;	
	 bitpos &= USART3->SR;
	
	if(USART_GetITStatus(USART3, USART_IT_PE) != RESET)
    {
    /* Clear the USART1 Receive interrupt */
        USART_ClearITPendingBit(USART3, USART_IT_PE);
		USART3_Received_Error=1;
    }


	 if ((itmask != (u16)RESET)&&(bitpos != (u16)RESET))
	 {
	    USART3_RxBuffer[USART3_RxCounter++] =  USART3->DR&(u16)0x01FF;
        USART3_NbrOfDataReceived++;	
		if(USART3_RxCounter>1&&(USART3_RxBuffer[USART3_RxCounter-1]==0x0d))//&&(USART3_RxBuffer[USART3_RxCounter-2]==0x0d))	 
		{
		  USART3_Received_Flag=1;
		}  
	 }
 
	  //�����жϵĴ���	 	 	  							  
	  itmask = (u32)0x01 << (USART_IT_TXE & ((u16)0x001F));	 	 
	  itmask &= USART3->CR1;  	
	  bitpos = (u32)0x01 << (USART_IT_TXE >> 0x08);	  
	  bitpos &= USART3->SR;

	 if ((itmask != (u16)RESET)&&(bitpos != (u16)RESET))
	 {   
	    // Write one byte to the transmit data register //
	    //USART_SendData(USART3, USART3_TxBuffer[USART3_TxCounter++]);
		USART3->DR =USART3_TxBuffer[USART3_TxCounter++];
	
	    if(USART3_TxCounter >= USART3_NbrOfDataToTransfer)
	    {		  
		   usartxbase=USART3_BASE+0x0C;
		   itmask = (((u32)0x01) << (USART_IT_TXE & 0x001F)); 
		   *(vu32*)usartxbase &= ~itmask;
	    }    
	 } 
}
/*******************************************************************************
* Function Name  : NMIException
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMIException(void)
{
}

/*******************************************************************************
* Function Name  : HardFaultException
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFaultException(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManageException
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManageException(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFaultException
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFaultException(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFaultException
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFaultException(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : DebugMonitor
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMonitor(void)
{
}

/*******************************************************************************
* Function Name  : SVCHandler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVCHandler(void)
{
}

/*******************************************************************************
* Function Name  : PendSVC
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSVC(void)
{
}

/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : PVD_IRQHandler
* Description    : This function handles PVD interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PVD_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TAMPER_IRQHandler
* Description    : This function handles Tamper interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TAMPER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FLASH_IRQHandler
* Description    : This function handles Flash interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RCC_IRQHandler
* Description    : This function handles RCC interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{

}

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{

}

/*******************************************************************************
* Function Name  : DMA1_Channel3_IRQHandler
* Description    : This function handles DMA1 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel4_IRQHandler
* Description    : This function handles DMA1 Channel 4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC4))
	{
    	DMA_ClearITPendingBit(DMA1_IT_TC4);
	}
}

/*******************************************************************************
* Function Name  : DMA1_Channel5_IRQHandler
* Description    : This function handles DMA1 Channel 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel6_IRQHandler
* Description    : This function handles DMA1 Channel 6 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel 7 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : CAN_RX1_IRQHandler
* Description    : This function handles CAN RX1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_RX1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : CAN_SCE_IRQHandler
* Description    : This function handles CAN SCE interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_SCE_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
}



/*******************************************************************************
* Function Name  : TIM1_TRG_COM_IRQHandler
* Description    : This function handles TIM1 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
}	   

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI2_IRQHandler
* Description    : This function handles SPI2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_IRQHandler(void)
{
}


/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
}



/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RTCAlarm_IRQHandler
* Description    : This function handles RTC Alarm interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USBWakeUp_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBWakeUp_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_BRK_IRQHandler
* Description    : This function handles TIM8 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_BRK_IRQHandler(void)
{
	testtimer5=1;
}

/*******************************************************************************
* Function Name  : TIM8_UP_IRQHandler
* Description    : This function handles TIM8 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_TRG_COM_IRQHandler
* Description    : This function handles TIM8 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_TRG_COM_IRQHandler(void)
{
testtimer5=1;
}

/*******************************************************************************
* Function Name  : TIM8_CC_IRQHandler
* Description    : This function handles TIM8 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_CC_IRQHandler(void)
{
 testtimer5=1;
}

/*******************************************************************************
* Function Name  : ADC3_IRQHandler
* Description    : This function handles ADC3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FSMC_IRQHandler
* Description    : This function handles FSMC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM5_IRQHandler
* Description    : This function handles TIM5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


/*******************************************************************************
* Function Name  : SPI3_IRQHandler
* Description    : This function handles SPI3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART4_IRQHandler
* Description    : This function handles UART4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART5_IRQHandler
* Description    : This function handles UART5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM6_IRQHandler
* Description    : This function handles TIM6 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM6_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : This function handles TIM7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{
	
}

/*******************************************************************************
* Function Name  : DMA2_Channel1_IRQHandler
* Description    : This function handles DMA2 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel2_IRQHandler
* Description    : This function handles DMA2 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel3_IRQHandler
* Description    : This function handles DMA2 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel4_5_IRQHandler
* Description    : This function handles DMA2 Channel 4 and DMA2 Channel 5
*                  interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel4_5_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
