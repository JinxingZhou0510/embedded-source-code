#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
  KEY_UP   = 0,
  KEY_DOWN = 1,
}KEYState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/
#define KEY1_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY1_GPIO_PIN                 GPIO_PIN_0
#define KEY1_GPIO                     GPIOE
#define KEY1_DOWN_LEVEL               0  /* 根据原理图设计，KEY1按下时引脚为高电平，所以这里设置为1 */

#define KEY2_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY2_GPIO_PIN                 GPIO_PIN_1
#define KEY2_GPIO                     GPIOE
#define KEY2_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY3_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY3_GPIO_PIN                 GPIO_PIN_2
#define KEY3_GPIO                     GPIOE
#define KEY3_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY4_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY4_GPIO_PIN                 GPIO_PIN_3
#define KEY4_GPIO                     GPIOE
#define KEY4_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY5_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY5_GPIO_PIN                 GPIO_PIN_4
#define KEY5_GPIO                     GPIOE
#define KEY5_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define LED0_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define LED0_ENA_PORT                    GPIOE                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define LED0_ENA_PIN                     GPIO_PIN_5                       // 而ENA+直接接开发板的VCC

#define LED1_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define LED1_ENA_PORT                    GPIOE                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define LED1_ENA_PIN                     GPIO_PIN_6                       // 而ENA+直接接开发板的VCC

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void KEY_GPIO_Init(void);
KEYState_TypeDef KEY1_StateRead(void);
KEYState_TypeDef KEY2_StateRead(void);
KEYState_TypeDef KEY3_StateRead(void);
KEYState_TypeDef KEY4_StateRead(void);
KEYState_TypeDef KEY5_StateRead(void);

#endif  // __BSP_KEY_H__

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
