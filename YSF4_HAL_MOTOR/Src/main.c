/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-5-31
  * ��    ��: 57&42����������μӼ���ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "key/bsp_key.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
#define  FASTSEEK_SPEED   50		//ԭ��ع��ٶ�
#define  SLOWSEEK_SPEED   20		//ԭ��ع������ٶ�
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
// �ٶ����ֵ���������͵����������Щ�����1800����Щ���Դﵽ4000
__IO uint32_t set_speed  = 200;         // �ٶ� ��λΪ0.05rad/sec
// ���ٶȺͼ��ٶ�ѡȡһ�����ʵ����Ҫ��ֵԽ���ٶȱ仯Խ�죬�Ӽ��ٽ׶αȽ϶���
// ���Լ��ٶȺͼ��ٶ�ֵһ������ʵ��Ӧ���жೢ�Գ����Ľ��
__IO uint32_t step_accel = 20;         // ���ٶ� ��λΪ0.1rad/sec^2
__IO uint32_t step_decel = 10;         // ���ٶ� ��λΪ0.1rad/sec^2
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // ���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // ��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
//	uint8_t i = 0;
	/* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
	HAL_Init();
	/* ����ϵͳʱ�� */
	SystemClock_Config();
	/* ����GPIO��Ϊ����ʹ�� */
	KEY_GPIO_Init();
	/* ���ö�ʱ��������� */
	STEPMOTOR_TIMx_Init();
	
	STEPMOTOR_AllHome();
	
	/* ����ѭ�� */
	while (1)
	{
		if (KEY1_StateRead()==KEY_DOWN) {
			STEPMOTOR_Fixed_Point_Movement(5);
//			HAL_Delay(200);
			STEPMOTOR_Lift_Slice();
//			HAL_Delay(200);
			STEPMOTOR_Fixed_Point_Movement(10);
//			HAL_Delay(200);
			STEPMOTOR_Placing_Slice();
//			HAL_Delay(200);
		}
		
		if (KEY2_StateRead()==KEY_DOWN) {
			STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, -3200, step_accel, step_decel, set_speed);
			while(stmr.run_state != STOP);
			STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, 3200, step_accel, step_decel, set_speed);
			while(stmr.run_state != STOP);
		}
		
		if (KEY3_StateRead()==KEY_DOWN) {
			STEPMOTOR_AxisMoveRel(STEPMOTOR_X, 1000, 10, 10, 100);
			while(stmr.run_state != STOP);
		}
	}
}


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
