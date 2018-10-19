/**
  ******************************************************************************
  * �ļ�����: bsp_key.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-05-31
  * ��    ��: ���ض��������ײ���������
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
#include "key/bsp_key.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ���ذ���IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_key.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void KEY_GPIO_Init(void)
{
	/* ����IOӲ����ʼ���ṹ����� */
	GPIO_InitTypeDef GPIO_InitStruct;

	/* ʹ��(����)KEY���Ŷ�ӦIO�˿�ʱ�� */  
	KEY1_RCC_CLK_ENABLE();
	KEY2_RCC_CLK_ENABLE();

	/* ����KEY1 GPIO:��������ģʽ */
	GPIO_InitStruct.Pin = KEY1_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct);  

	/* ����KEY2 GPIO:��������ģʽ */
	GPIO_InitStruct.Pin = KEY2_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);

	/* ����KEY3 GPIO:��������ģʽ */
	GPIO_InitStruct.Pin = KEY3_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);

	/* ����KEY4 GPIO:��������ģʽ */
	GPIO_InitStruct.Pin = KEY4_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(KEY4_GPIO, &GPIO_InitStruct);

	/* ����KEY5 GPIO:��������ģʽ */
	GPIO_InitStruct.Pin = KEY5_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(KEY5_GPIO, &GPIO_InitStruct);
  
	LED0_ENA_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = LED0_ENA_PIN | LED1_ENA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(LED0_ENA_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LED0_ENA_PORT, LED0_ENA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_ENA_PORT, LED1_ENA_PIN, GPIO_PIN_RESET);

}

/**
  * ��������: ��ȡ����KEY1��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY1_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY2��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY2_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY3��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY3_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY4��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY4_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY5��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY5_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
