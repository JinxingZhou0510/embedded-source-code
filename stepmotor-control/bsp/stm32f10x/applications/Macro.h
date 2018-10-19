#ifndef __MACRO_H
#define __MACRO_H

#define M1DIV               128   //定义电机1的细分数
#define M2DIV               32   //定义电机2的细分数
#define M3DIV               32   //定义电机3的细分数
#define M4DIV               32   //定义电机4的细分数

#define M_FRE_START					10000 //电机的启动频率
#define M_FRE_AA						6000	//电机频率的加加速度
#define M_T_AA							2			//电机频率的加加速时间
#define M_T_UA							6     //电机频率的匀加速时间
#define M_T_RA							2		  //电机频率的减加速时间 

#define USART1TXSIZE 1200
#define USART1RXSIZE 128
#define USART2TXSIZE 512
#define USART2RXSIZE 128
#define USART3TXSIZE 512
#define USART3RXSIZE 128


#endif
