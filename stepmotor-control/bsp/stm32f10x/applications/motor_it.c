/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_it.h"
#include "STM32F10X.h"
#include "include.h"


#ifdef OUTPUT_DATA
char tmp[64];
float timecnt=0;
float periodcnt=0;
#endif
  
/*see 
http://picprog.strongedge.net/step_prof/step-profile.html
*/

/*���SPTA�㷨����������*/
void TIMX_IRQHandler_SPTA(MOTOR_CONTROL_SPTA *pmotor)
{
	int carry=0;
#ifdef OUTPUT_DATA
	timecnt++;
	periodcnt++;
#endif
	/*���������ź�*/
	pmotor->GPIOBASE->BRR = pmotor->PWMGPIO; 
	
	/*�����ٶ��ۼ����Ƿ�����������Ƿ����һ����������*/
	pmotor->step_frac += pmotor->step_speed;            
	carry = pmotor->step_frac >> 16;               
	pmotor->step_frac -= carry << 16;
	if(carry!=0)
	{ 
		pmotor->step_count+=1;
		/*���������źŲ���һ����������*/
		pmotor->GPIOBASE->BSRR = pmotor->PWMGPIO; 
		
		//λ�ü���
		if(pmotor->clockwise==pmotor->dir)
		{
			pmotor->CurrentPosition_Pulse++;
			if(pmotor->CurrentPosition_Pulse>=pmotor->MaxPosition_Pulse)
			{
				pmotor->CurrentPosition_Pulse=0;
			}
		}
		else
		{
			pmotor->CurrentPosition_Pulse--;
			if(pmotor->CurrentPosition_Pulse==0xffffffff)
			{
				pmotor->CurrentPosition_Pulse=pmotor->MaxPosition_Pulse-1;
			}
		}
		pmotor->CurrentPosition=pmotor->CurrentPosition_Pulse/pmotor->divnum;
#ifdef OUTPUT_DATA
		
		//ÿ��ʱ�̶�Ӧ�ĸߵ�ƽ��͵�ƽ
		sprintf(tmp,"%f,0\r\n%f,1\r\n",timecnt-1,timecnt);
		
		USART1_Printfstr(tmp);	
		periodcnt=0;
#endif
	}	
	
	//�ٶȿ���
	if(pmotor->speedenbale)
	{
		if( (pmotor->step_speed>=pmotor->step_spmax&&pmotor->step_speed-pmotor->step_spmax<=3)||
				(pmotor->step_speed<=pmotor->step_spmax&&pmotor->step_spmax-pmotor->step_speed<=3))
		{
			return;
		}
	}
	/*���ݵ����״̬����״̬ת���Լ������任*/
	switch(pmotor->step_state)
	{ 		
		case ACCELERATING:
			if(carry){				
				pmotor->step_acced++;
			}
			pmotor->speed_frac+=pmotor->step_accel;		
			carry=pmotor->speed_frac>>17;
			pmotor->speed_frac-=carry<<17;
			if(carry){
				pmotor->step_speed+=carry;
			}
			if(!pmotor->speedenbale)
			{
				/*although we are ACCELERATING,but middle point reached,we need DECELERATING*/
				if(pmotor->step_middle!=0){
					if(pmotor->step_count==pmotor->step_middle)   
					{               
							pmotor->step_state=DECELERATING;
					}		
				}else if(pmotor->step_count>0){
					pmotor->step_state=DECELERATING;
				}
			}
			if(pmotor->step_speed>=pmotor->step_spmax)
			{        
				pmotor->step_speed=pmotor->step_spmax;
				pmotor->step_state=AT_MAX;
			}
			break;	 
		case AT_MAX:                                               
		  if(pmotor->step_move-pmotor->step_count<=pmotor->step_acced)   
			{               
				pmotor->step_state=DECELERATING;
			}
			break;
		case DECELERATING:  
			if(carry&&pmotor->step_acced>0){				
				pmotor->step_acced--;
			}
			pmotor->speed_frac+=pmotor->step_accel;		
			carry=pmotor->speed_frac>>17;
			pmotor->speed_frac-=carry<<17;
			if(carry&&pmotor->step_speed>carry){
				pmotor->step_speed-=carry;
			}	
			if(!pmotor->speedenbale)
			{
				if(pmotor->step_count>=pmotor->step_move)
				{  
						pmotor->step_state=IDLE;  
						pmotor->running=0;
						pmotor->step_spmax=0;
						TIM_Cmd(pmotor->TIMx, DISABLE);
	#ifdef OUTPUT_DATA
						timecnt=0;
	#endif
				}
			}
			break;
	}
}
void TIM4_IRQHandler(void)
{   
	/*����ж�*/
	TIM4->SR = (u16)~TIM_FLAG_Update;
	TIMX_IRQHandler_SPTA(&motor4);
}
 
 /*���S�������㷨����������*/
void TIMX_UP_IRQHandler_S(MOTOR_CONTROL_S* pmotor)
{   
	if(1==pmotor->en)
	{ 
		//λ�ü���
		if(pmotor->clockwise==pmotor->dir)
		{
			pmotor->CurrentPosition_Pulse++;
			if(pmotor->CurrentPosition_Pulse>=pmotor->MaxPosition_Pulse)
			{
				pmotor->CurrentPosition_Pulse=0;
			}
		}
		else
		{
			pmotor->CurrentPosition_Pulse--;
			if(pmotor->CurrentPosition_Pulse==0xffffffff)
			{
				pmotor->CurrentPosition_Pulse=pmotor->MaxPosition_Pulse-1;
			}
		}
		pmotor->CurrentPosition=pmotor->CurrentPosition_Pulse/pmotor->divnum;
		
		//�ٶȿ���
		if(pmotor->speedenbale&&(pmotor->CurrentIndex==pmotor->TargetIndex||pmotor->TargetIndex+pmotor->CurrentIndex==pmotor->StartTableLength+pmotor->StopTableLength-1))
		{
			return;
		}
		pmotor->PulsesHaven++; //���������
		pmotor->pulsecount++;  //�Ը�Ƶ������������������ 
		
		//�ԳƷ�ת
		if(pmotor->RevetDot==pmotor->PulsesHaven)
		{
			pmotor->pulsecount=pmotor->Step_Table[pmotor->CurrentIndex];
		}
		if(pmotor->pulsecount>=pmotor->Step_Table[pmotor->CurrentIndex])
		{ 
			if(pmotor->PulsesHaven<=pmotor->StartSteps)
			{
				//�𲽽׶�
				if(pmotor->CurrentIndex<pmotor->StartTableLength-1)
				{
					pmotor->CurrentIndex++;
					pmotor->pulsecount=0;
					if(pmotor->CurrentIndex>=pmotor->StartTableLength)pmotor->CurrentIndex=pmotor->StartTableLength;
				}
			}
			//�����ٶȿ��ƣ��˴������ж�pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1)
			//if(pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1))
			if((pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->speedenbale==1)||
				(pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->speedenbale==0&&pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1))) 
			{
				//ֹͣ�׶�
				if(pmotor->CurrentIndex<pmotor->StartTableLength-1)
				{
					pmotor->CurrentIndex=pmotor->StartTableLength+pmotor->StopTableLength-pmotor->CurrentIndex;  
				}
				pmotor->CurrentIndex++;
				pmotor->pulsecount=0;
				if(pmotor->CurrentIndex>=pmotor->StartTableLength+pmotor->StopTableLength)
					pmotor->CurrentIndex=pmotor->StartTableLength+pmotor->StopTableLength-1;
			}  		
			pmotor->TIMx->ARR = pmotor->Counter_Table[pmotor->CurrentIndex] ; //��������
			pmotor->TIMx->CCR1 =( pmotor->Counter_Table[pmotor->CurrentIndex])>>1;       //����ռ�ձ�	    
		}	  
		//��תԤ����������ֹͣ��running=0�����Խ�����һ����ת
		if(pmotor->PulsesHaven>=pmotor->PulsesGiven&&pmotor->PulsesHaven>3)
		{
			pmotor->en=0;
			pmotor->running=0;
			pmotor->CurrentIndex=0;
			TIM_Cmd(pmotor->TIMx, DISABLE);		  //DISABLE 	
			USART1_Printfstr("1\r\n");
#ifdef OUTPUT_DATA
			timecnt=0;		
#endif			
		}
		else
		{			
			pmotor->Time_Cost_Act+=pmotor->TIMx->ARR;
		}
	}
#ifdef OUTPUT_DATA
	
	//ÿ��ʱ�̶�Ӧ��Ƶ��
	//sprintf(tmp,"%f\t%f\r\n",timecnt,1.0/TIM1->CCR1);
	
	//ÿ��ʱ�̶�Ӧ�ĸߵ�ƽ��͵�ƽ
	sprintf(tmp,"%f,0\r\n%f,1\r\n",timecnt,timecnt+pmotor->TIMx->CCR1);
	
	USART1_Printfstr(tmp);
	timecnt+=pmotor->TIMx->ARR;
#endif
}

void TIM1_UP_IRQHandler(void)
{ 
	TIM1->SR = (u16)~TIM_FLAG_Update;   
	TIMX_UP_IRQHandler_S(&motor1);
}

void TIM2_IRQHandler(void)
{
	TIM2->SR = (u16)~TIM_FLAG_Update;   
	TIMX_UP_IRQHandler_S(&motor2);
}
void TIM3_IRQHandler(void)
{
	TIM3->SR = (u16)~TIM_FLAG_Update;   
	TIMX_UP_IRQHandler_S(&motor3);
}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!= RESET)
	{        
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(motor3.rstflg==1)
		{
			motor3.speedenbale=0;
			motor3.PulsesHaven=motor3.PulsesGiven-128;	
			motor3.dir=M3_UNCLOCKWISE;
			motor3.CurrentPosition_Pulse=128;		
			motor3.rstflg=0;
		}  
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)!= RESET)
	{        
		EXTI_ClearITPendingBit(EXTI_Line1);
		if(motor1.rstflg==1)
		{
			motor1.speedenbale=0;
			motor1.PulsesHaven=motor1.PulsesGiven-128;	
			motor1.dir=M1_UNCLOCKWISE;
			motor1.CurrentPosition_Pulse=128;		
			motor1.rstflg=0;
		}  
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!= RESET)
	{        
		EXTI_ClearITPendingBit(EXTI_Line2);
		if(motor4.rstflg==1)
		{
			motor4.speedenbale=0;
			motor4.PulsesHaven=motor4.PulsesGiven-128;	
			motor4.dir=M4_UNCLOCKWISE;
			motor4.CurrentPosition_Pulse=128;		
			motor4.rstflg=0;
		}  
	}
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!= RESET)
	{        
		EXTI_ClearITPendingBit(EXTI_Line4);
		if(motor2.rstflg==1)
		{
			motor2.speedenbale=0;
			motor2.PulsesHaven=motor2.PulsesGiven-128;	
			motor2.dir=M2_UNCLOCKWISE;
			motor2.CurrentPosition_Pulse=128;		
			motor2.rstflg=0;
		}  
	}
}
