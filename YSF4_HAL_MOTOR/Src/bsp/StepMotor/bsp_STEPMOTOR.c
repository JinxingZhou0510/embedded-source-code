/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-06-03
  * ��    ��: �����������������ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "key/bsp_key.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
static speedRampData srd               = {STOP,CW,0,0,0,0,0};         // �Ӽ������߱���
static __IO uint8_t  ZeroStep = IDLE ;	 //����ԭ��״̬��
static __IO uint8_t  LimPosi = FALSE ;   //�������ޱ�־λ  True:���Ｋ��λ  False:δ���Ｋ��λ
static __IO uint8_t  LimNega = FALSE ;   //�������ޱ�־λ
static __IO uint8_t  DOG         	   = FALSE;   	// �����ź�
static __IO uint8_t  HomeCapture       = FALSE;		// ԭ�㲶���־
static __IO int8_t   HomeDir           = CCW;	    // �����ԭ�㷽��
static __IO int32_t  step_position     = 0;         // ��ǰλ��
static __IO uint8_t  MotionStatus      = 0;         //�Ƿ����˶���0��ֹͣ��1���˶�
static uint32_t STEPMOTOR_TIM[6] = {STEPMOTOR_Y_TIM_CHANNEL_x, STEPMOTOR_TIM_CHANNEL_x, \
									STEPMOTOR_Y_TIM_FLAG_CCx,  STEPMOTOR_TIM_FLAG_CCx, \
									STEPMOTOR_Y_TIM_IT_CCx,    STEPMOTOR_TIM_IT_CCx};
static uint16_t STEPMOTOR_CurrentCoord = 0;
static const uint16_t STEPMOTOR_Piont[22] = {582,   1164,  1746,  2328, 2910, 3492, \
											 4074,  4656,  5238,  5820, 6402, 6984, \
											 7566,  8148,  8730,  9312, 9894, 10476, \
											 11058, 11640, 12222, 12804};
__IO stepmotorRun stmr = {1, 1};
__IO uint8_t stepmotor_status = 1;
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct; 

	/* ���Ŷ˿�ʱ��ʹ�� */
	STEPMOTOR_TIM_GPIO_CLK_ENABLE();
	STEPMOTOR_DIR_GPIO_CLK_ENABLE();
	STEPMOTOR_ENA_GPIO_CLK_ENABLE();
	ORIGIN_X_GPIO_CLK_ENABLE();
	
	STEPMOTOR_Y_TIM_GPIO_CLK_ENABLE();
	STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE();
	STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE();
	ORIGIN_Y_GPIO_CLK_ENABLE();

	/* �����������������IO��ʼ�� */
	GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN | STEPMOTOR_Y_TIM_PUL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;        // GPIO��������TIM���ù���
	HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT, &GPIO_InitStruct);

	/* �����������������IO��ʼ�� */
	GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN | STEPMOTOR_Y_DIR_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(STEPMOTOR_DIR_PORT, &GPIO_InitStruct);

	/* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
	GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(STEPMOTOR_ENA_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = STEPMOTOR_Y_ENA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(STEPMOTOR_Y_ENA_PORT, &GPIO_InitStruct);
	
	/* �������ԭ��������*/
	GPIO_InitStruct.Pin  = ORIGIN_X_PIN | ORIGIN_Y_PIN ;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(ORIGIN_X_PORT, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(ORIGIN_X_EXTI_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(ORIGIN_X_EXTI_IRQn);
	HAL_NVIC_SetPriority(ORIGIN_Y_EXTI_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(ORIGIN_Y_EXTI_IRQn);

	STEPMOTOR_X_DIR_FORWARD();
	STEPMOTOR_X_OUTPUT_DISABLE();
	
	STEPMOTOR_Y_DIR_FORWARD();
	STEPMOTOR_Y_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
	TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����

	STEPMOTOR_TIM_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	STEPMOTOR_GPIO_Init();

	/* ��ʱ�������������� */
	htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                          // ��ʱ�����
	htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;           // ��ʱ��Ԥ��Ƶ��
	htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;              // �����������ϼ���
	htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                 // ��ʱ������
	htimx_STEPMOTOR.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;        // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htimx_STEPMOTOR);

	/* ��ʱ��ʱ��Դ���� */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       	// ʹ���ڲ�ʱ��Դ
	HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

	/* ��ʱ���Ƚ�������� */
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
	sConfigOC.Pulse = 0xFFFF;                            // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;           // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
	HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_TIM_CHANNEL_x);
	HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_Y_TIM_CHANNEL_x);
	
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_Y_TIM_CHANNEL_x, TIM_CCx_DISABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);

	__HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_Y_TIM_FLAG_CCx);
	/* ʹ�ܶ�ʱ���Ƚ���� */
//	__HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
//	__HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_Y_TIM_IT_CCx);
	/* Enable the main output */
	__HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR);  
	HAL_TIM_Base_Start(&htimx_STEPMOTOR);  // ʹ�ܶ�ʱ��
}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance==STEPMOTOR_TIMx) {
		/* ������ʱ������ʱ��ʹ�� */
		STEPMOTOR_TIM_RCC_CLK_ENABLE();
	}
}

/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance==STEPMOTOR_TIMx) {
		/* ������ʱ������ʱ�ӽ��� */
		STEPMOTOR_TIM_RCC_CLK_DISABLE();
		HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT, STEPMOTOR_TIM_PUL_PIN);
		HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT, STEPMOTOR_DIR_PIN);
		HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT, STEPMOTOR_ENA_PIN);
		HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT, STEPMOTOR_Y_TIM_PUL_PIN);
		HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT, STEPMOTOR_Y_DIR_PIN);
		HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT, STEPMOTOR_Y_ENA_PIN);

		HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
	}
} 
/**
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: step���ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
              accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2
              decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
              speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ�����������ȼ��ٵ�����ٶȣ�Ȼ���ں���λ�ÿ�ʼ
  *           ������ֹͣ��ʹ�������˶�����Ϊָ���Ĳ���������Ӽ��ٽ׶κ̲ܶ���
  *           �ٶȺ������ǻ�û�ﵽ����ٶȾ�Ҫ��ʼ����
  */
void STEPMOTOR_AxisMoveRel(int8_t stepmotor, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{  
	__IO uint16_t tim_count;
	// �ﵽ����ٶ�ʱ�Ĳ���
	__IO uint32_t max_s_lim;
	// ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
	__IO uint32_t accel_lim;

	if (MotionStatus != STOP)    // ֻ�����������ֹͣ��ʱ��ż���
		return;
	srd.stepmotor = stepmotor;
	if (step < 0) {    // ����Ϊ����
		srd.dir = CCW; // ��ʱ�뷽����ת
		STEPMOTOR_DIR_REVERSAL(stepmotor);
		step = -step;  // ��ȡ��������ֵ
	} else {
		srd.dir = CW;  // ˳ʱ�뷽����ת
		STEPMOTOR_DIR_FORWARD(stepmotor);
	}
	
	if (stepmotor) {
		__HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
		__HAL_TIM_DISABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_Y_TIM_IT_CCx);
	} else {
		__HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_Y_TIM_IT_CCx);
		__HAL_TIM_DISABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
	}

	if (step == 1) {  // ����Ϊ1
		srd.accel_count = -1;   // ֻ�ƶ�һ��
		srd.run_state = DECEL;  // ����״̬
		srd.step_delay = 1000;	// ����ʱ	
	} else if (step != 0) {     // ���Ŀ���˶�������Ϊ0
		// ���ǵĵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

		// ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
		// min_delay = (alpha / tt) / w
		srd.min_delay = (int32_t)(A_T_x10/speed);

		// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
		srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);

		// ������ٲ�֮��ﵽ����ٶȵ�����
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
		// ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
		// ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
		if (max_s_lim == 0)
			max_s_lim = 1;

		// ������ٲ�֮�����Ǳ��뿪ʼ����
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = (uint32_t)(step*decel/(accel+decel));
		// ���Ǳ����������1�����ܲ��ܿ�ʼ����.
		if (accel_lim == 0) 
			accel_lim = 1;

		// ʹ�������������ǿ��Լ�������ٽ׶β���
		if (accel_lim <= max_s_lim)
			srd.decel_val = accel_lim - step;
		else 
			srd.decel_val = -(max_s_lim*accel/decel);
		
		// ��ֻʣ��һ�����Ǳ������
		if (srd.decel_val == 0) 
			srd.decel_val = -1;

		// ���㿪ʼ����ʱ�Ĳ���
		srd.decel_start = step + srd.decel_val;

		// �������ٶȺ��������ǾͲ���Ҫ���м����˶�
		if (srd.step_delay <= srd.min_delay) {
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		} else {
			srd.run_state = ACCEL;
		}    
		// ��λ���ٶȼ���ֵ
		srd.accel_count = 0;
	}
	stmr.stepmotor = stepmotor;
	stmr.run_state = RUN;
	MotionStatus = 1; // ���Ϊ�˶�״̬
	tim_count = __HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR, STEPMOTOR_TIM[stepmotor], tim_count+srd.step_delay); // ���ö�ʱ���Ƚ�ֵ
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM[stepmotor], TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
}

/** ��������: �ƶ�������λ��:�����ƶ�
  * �������: targert_step:Ŀ���λ��
  *			      accel:���ٶ�
  *			      decel:���ٶ�
  *			      speed:����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ֶ����ƶ�,����Ŀ��λ�������ԭ��Ĳ���,
  *  		      �Լ��ٶȼ��ٵ�����ٶȺ�����Ѱ��Ŀ��λ��,
  *			      Ȼ���ں��ʵ�λ�ü���,����Ŀ��λ��.
  */
void STEPMOTOR_AxisMoveAbs(__IO int8_t stepmotor, __IO int32_t targert_step, \
	          __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
	__IO int32_t rel_step = 0;
	__IO int8_t dir = -1;
	rel_step = step_position - targert_step ; 	// ��ȡ��ǰλ�ú�Ŀ��λ��֮��Ĳ���ֵ
	
	if (rel_step == 0)
		dir = 0;
	else 
		dir = -1;
	STEPMOTOR_AxisMoveRel(stepmotor, dir*rel_step, accel, decel, speed);//
}

/** 
  * ��������: ����ԭ��
  * �������: FASTSEEK_SPEED ԭ��ع��ٶ�
  *   		    SLOWSEEK_SPEED ԭ��ع������ٶ�
  *			      accel   ���ٶ�
  *			      decel   ���ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ʵ��Ѱ��ԭ��,����ԭ��λ��ͣ����,�����źź�ԭ���ź��� KEY1 ���½���
  *  	        ��������ģ��,��⵽�½��ر�ʾ���񵽽����ź�,��ʼ��slowseek_speed�ƶ�,����
  *			      �ر�ʾ����ԭ��,����ֹͣ.
  */
void STEPMOTOR_AxisHome(__IO int8_t stepmotor, __IO int32_t fastseek_speed, \
	__IO uint32_t slowseek_speed, __IO uint32_t accel, __IO uint32_t decel)
{
//	__IO static int32_t  decel_val;   		// ���ٽ׶β���
    __IO uint32_t max_fs_lim = 0;	        // ���ٵ�fs_speed����Ĳ���

	switch (ZeroStep) {
	case FASTSEEK :										
		STEPMOTOR_AxisMoveRel(stepmotor, -HomeDir*MAX_NUM_LAP, accel, decel, fastseek_speed);	// �Իع��ٶ�����
		srd.decel_start = MAX_NUM_STEP;	// �޸�srd����ֵ,ʹ�õ�����˶������в������,���ָ�������STEPMOTOR_AxisMoveRel����֮��ִָ��
		ZeroStep = SLOWSEEK;
		break;
	
	case SLOWSEEK : // ��⵽�����ź�DOG,��ʼ���ٵ������ٶ�
		if (DOG == TRUE) { // ��⵽DOG�źžͿ�ʼ����
			HAL_Delay(100);                 // ʹ��I/O��ģ��ԭ���ź�,���л�е����,��Ҫ����
			HomeCapture = FALSE;
			/* ���������Ҫ�Ĳ��� */
			srd.decel_start = 0;	// ���ÿ�ʼ���ٵ�λ��	  
			srd.min_delay = (int32_t)(A_T_x10/slowseek_speed);	// �ڶ��ٶ�
			ZeroStep = MOVETOZERO;
		}
		break ;
		
	case MOVETOZERO:  // ��⵽ԭ���ź�,����ֹͣ
		if (DOG == TRUE ) {
			if (HomeCapture == TRUE) {
				DOG = FALSE;
				HomeCapture = FALSE;
				srd.run_state = STOP;
				stepmotor_status = STOP;
				step_position = HOME_POSITION;	 // ��ǰλ��λ��ԭ��
				ZeroStep = IDLE;				 // ����ԭ�����
			}
		}
		break;
		
	case IDLE:
		if((DOG==FALSE) && ( HomeCapture==FALSE))	
			ZeroStep = FASTSEEK ;
		break;
	default :break;
	}
}

void STEPMOTOR_AllHome(void)
{
	HAL_GPIO_WritePin(LED0_ENA_PORT, LED0_ENA_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_ENA_PORT, LED1_ENA_PIN, GPIO_PIN_RESET);
	while (stepmotor_status != STOP) {
		STEPMOTOR_AxisHome(STEPMOTOR_Y, 50, 20, 20, 10);	//����ԭ��
	}
	stepmotor_status = 1;
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED0_ENA_PORT, LED0_ENA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_ENA_PORT, LED1_ENA_PIN, GPIO_PIN_SET);
	while (stepmotor_status != STOP) {
		STEPMOTOR_AxisHome(STEPMOTOR_X, 50, 10, 10, 10);	//����ԭ��
	}
	stepmotor_status = 1;
	HAL_Delay(100);
	STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, 9600, 10, 10, 200);
	while(stmr.run_state != STOP);
	HAL_GPIO_WritePin(LED0_ENA_PORT, LED0_ENA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_ENA_PORT, LED1_ENA_PIN, GPIO_PIN_RESET);
	STEPMOTOR_CurrentCoord = 0;
}

/**
  * ��������: �˶���ָ��λ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void STEPMOTOR_Fixed_Point_Movement(uint8_t point)
{
	int16_t  step = STEPMOTOR_Piont[point-1] - STEPMOTOR_CurrentCoord;
	
	if (point < 1)
		point = 1;
	else if (point > 22)
		point = 22;
	
	STEPMOTOR_AxisMoveRel(STEPMOTOR_X, step, 10, 10, 200);
	STEPMOTOR_CurrentCoord = STEPMOTOR_Piont[point-1];
	HAL_GPIO_TogglePin(LED1_ENA_PORT, LED1_ENA_PIN);
	while(stmr.run_state != STOP);
}

/**
  * ��������: ����ϸ����Ƭ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void STEPMOTOR_Lift_Slice(void)
{
	STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, -6400, 20, 10, 100);
	while(stmr.run_state != STOP);
	STEPMOTOR_AxisMoveRel(STEPMOTOR_X, -100, 10, 10, 50);
	STEPMOTOR_CurrentCoord += (-100);
	while(stmr.run_state != STOP);
	HAL_Delay(1000);
	STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, 6400, 20, 10, 100);
	while(stmr.run_state != STOP);
}

/**
  * ��������: ����ϸ����Ƭ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void STEPMOTOR_Placing_Slice(void)
{
	STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, -6400, 20, 10, 100);
	while(stmr.run_state != STOP);
	STEPMOTOR_AxisMoveRel(STEPMOTOR_X, 100, 10, 10, 50);
	STEPMOTOR_CurrentCoord += (100);
	while(stmr.run_state != STOP);
	STEPMOTOR_AxisMoveRel(STEPMOTOR_Y, 6400, 20, 10, 100);
	while(stmr.run_state != STOP);
}

/**
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void STEPMOTOR_TIMx_IRQHandler(void)// ��ʱ���жϴ���
{ 
	__IO uint16_t tim_count=0;
	// �����£��£�һ����ʱ����
	uint16_t new_step_delay=0;
	// ���ٹ��������һ����ʱ���������ڣ�.
	__IO static uint16_t last_accel_delay=0;
	// ���ƶ�����������
	__IO static uint32_t step_count = 0;
	// ��¼new_step_delay�е������������һ������ľ���
	__IO static int32_t rest = 0;
	// ��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
	__IO static uint8_t i=0;

	if (__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM[srd.stepmotor+4]) !=RESET) {
		// �����ʱ���ж�
		__HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM[srd.stepmotor+4]);

		// ���ñȽ�ֵ
		tim_count = __HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
		__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR, STEPMOTOR_TIM[srd.stepmotor], tim_count+srd.step_delay);

		i++;     // ��ʱ���жϴ�������ֵ
		if (i == 2) { // 2�Σ�˵���Ѿ����һ����������
			i = 0;    // ���㶨ʱ���жϴ�������ֵ
			switch (srd.run_state) { // �Ӽ������߽׶�
			case STOP:
				step_count = 0;  // ���㲽��������
				rest = 0;        // ������ֵ
				// �ر�ͨ��
				TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM[srd.stepmotor], TIM_CCx_DISABLE);        
				__HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM[srd.stepmotor+2]);
				MotionStatus = 0;  //  ���Ϊֹͣ״̬ 		
				stmr.run_state = STOP;			
				break;

			case ACCEL:
				step_count++;        // ������1
				if (srd.dir == CW)  	
					step_position++; // ����λ�ü�1
				else
					step_position--; // ����λ�ü�1

				srd.accel_count++;   // ���ټ���ֵ��1
				new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));// ������(��)һ����������(ʱ����)
				rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
				if (step_count >= srd.decel_start) { // ����ǹ�Ӧ�ÿ�ʼ����
					srd.accel_count = srd.decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
					srd.run_state = DECEL;           // �¸����������ٽ׶�
				} else if (new_step_delay <= srd.min_delay) { // ����Ƿ񵽴�����������ٶ�
					last_accel_delay = new_step_delay; // ������ٹ��������һ����ʱ���������ڣ�
					new_step_delay = srd.min_delay;    // ʹ��min_delay����Ӧ����ٶ�speed��
					rest = 0;                          // ������ֵ
					srd.run_state = RUN;               // ����Ϊ��������״̬
				}
				break;

			case RUN:
				step_count++;        // ������1
				if (srd.dir == CW)	  	
					step_position++; // ����λ�ü�1
				else 
					step_position--; // ����λ�ü�1
				new_step_delay = srd.min_delay;        // ʹ��min_delay����Ӧ����ٶ�speed��
				if (step_count >= srd.decel_start) {   // ��Ҫ��ʼ����
					srd.accel_count = srd.decel_val;   // ���ٲ�����Ϊ���ټ���ֵ
					new_step_delay = last_accel_delay; // �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
					srd.run_state = DECEL;             // ״̬�ı�Ϊ����
				}
				break;

			case DECEL:
				step_count++;  // ������1
				if (srd.dir == CW)  	
					step_position++; // ����λ�ü�1
				else
					step_position--; // ����λ�ü�1
				srd.accel_count++;
				new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); // ������(��)һ����������(ʱ����)
				rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1); // �����������´μ��㲹���������������

				// ����Ƿ�Ϊ���һ��
				if (srd.accel_count >= 0)
					srd.run_state = STOP;
				break;
			}      
			srd.step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
		}
	}
}

/**
  * ��������: �ⲿ�жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ���ж��Ƿ񵽴Ｋ�޺ͼ��ԭ���ź�
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ORIGIN_X_PIN) {
		__HAL_GPIO_EXTI_CLEAR_IT(ORIGIN_X_PIN);	
		if(HAL_GPIO_ReadPin(ORIGIN_X_PORT, ORIGIN_X_PIN) == ORI_DOWNLEVEL) // ��ȡKEY1״̬,���¾��������� �ɿ������½���,�պþ���һ�������ź�
			DOG = TRUE;				// ��⵽�����ź�
		else 				
			HomeCapture = TRUE;		// ����ԭ��
	} 
	
	if (GPIO_Pin == ORIGIN_Y_PIN) {
		__HAL_GPIO_EXTI_CLEAR_IT(ORIGIN_Y_PIN);	
		if(HAL_GPIO_ReadPin(ORIGIN_Y_PORT, ORIGIN_Y_PIN) == ORI_DOWNLEVEL)// ��ȡKEY1״̬,���¾��������� �ɿ������½���,�պþ���һ�������ź�
			DOG = TRUE;				// ��⵽�����ź�
		else 				
			HomeCapture = TRUE;		// ����ԭ��
	} 
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
