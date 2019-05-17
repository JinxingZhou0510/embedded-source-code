/*************************************************************************************

  * @file   soft_iic.c

  * @brief Thisis a brief description.

  * @author ZJX

  * @par   Copyright (c):

  *          ZJX.

  *         All Rights Reserved

  * @date   2019/05/06 

  *  @note   mattersneeding attention

  *  @version <version  number>
  
  *************************************************************************************/ 
#include "soft_iic.h"
#include "stm32_common.h"
#include "error_report.h"
//拉高sda
#define iic_sda_h(siic) HAL_GPIO_WritePin(siic->sda_gpio, siic->sda_pin, GPIO_PIN_SET)
//拉低sda
#define iic_sda_l(siic) HAL_GPIO_WritePin(siic->sda_gpio, siic->sda_pin, GPIO_PIN_RESET)
//读取sda，检测是否高电平
#define is_iic_sda_h(siic) (GPIO_PIN_SET==HAL_GPIO_ReadPin(siic->sda_gpio,siic->sda_pin))

//拉高scl
#define iic_scl_h(siic) HAL_GPIO_WritePin(siic->scl_gpio, siic->scl_pin, GPIO_PIN_SET)
//拉低scl
#define iic_scl_l(siic) HAL_GPIO_WritePin(siic->scl_gpio, siic->scl_pin, GPIO_PIN_RESET)

/*************************************************************************************
 * 延时值，用来简单控制iic通讯速率
 * @note 这个值不能过小，stm32F407开漏上拉需要一定的上升时间，过小从器件无法识别
 *
 * @author ZJX
 * @date 2019/5/6
 *************************************************************************************/
#define  iic_t  10

/*************************************************************************************
 * 延时函数
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param cnt 延时时间（单位：us）.
 *************************************************************************************/
static inline void delay(unsigned int cnt)
{
	delay_us(cnt);
}

/*************************************************************************************
 * Sda in sda设置为输入模式
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return An int.
 *************************************************************************************/
static inline int sda_in(soft_iic *siic)
{
	if (!siic) {
		return kIllgalArg;
	}
	iic_sda_h(siic);
}

/*************************************************************************************
 * Sda out sda设置为输出模式
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *************************************************************************************/
static inline void sda_out(soft_iic *siic)
{

}

/*************************************************************************************
 * 软件IIC初始化.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return 参数错误返回kIllgalArg.
 *************************************************************************************/
int soft_iic_init(soft_iic *siic)
{
	if (!siic){
		return kIllgalArg;
	}
	
	iic_scl_h(siic);
	iic_sda_h(siic);
	sda_out(siic);
	
	return kOK;
}

/*************************************************************************************
 * 产生IIC起始信号 起始信号：当SCL为高期间，SDA由高到低的跳变.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return 参数错误返回kIllgalArg.
 *************************************************************************************/
int soft_iic_start(soft_iic *siic)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	sda_out(siic);     //sda线输出
	iic_sda_h(siic);
	iic_scl_h(siic);
	delay(iic_t);
	iic_sda_l(siic);//START:when CLK is high,DATA change form high to low 
	delay(iic_t);
	iic_scl_l(siic);//钳住I2C总线，准备发送或接收数据
	delay(iic_t/2);
	
	return kOK;
}

/*************************************************************************************
 * 产生IIC停止信号 停止信号：当SCL为高期间，SDA由低到高的跳变.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return 参数错误返回kIllgalArg.
 *************************************************************************************/
int soft_iic_stop(soft_iic *siic)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	sda_out(siic);//sda线输出
	iic_scl_l(siic);
	iic_sda_l(siic);
	delay(iic_t);
	iic_scl_h(siic);//STOP:when CLK is high DATA change form low to high
	delay(iic_t / 2);
	iic_sda_h(siic);//发送I2C总线结束信号
	
	return kOK;
}

/*************************************************************************************
 * 等待应答信号到来,如果超时没有应答就自动发送一个stop信号.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return 参数错误返回kIllgalArg，未接收到应答返回kFailed， 接收到应答信号返回kOK.
 *************************************************************************************/
int soft_iic_wait_ack(soft_iic *siic)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	int ucErrTime = 0;
	sda_in(siic);      //SDA设置为输入
	delay(iic_t);
	iic_scl_h(siic);
	delay(iic_t);
	while (is_iic_sda_h(siic)) {
		ucErrTime++;
		if (ucErrTime > 1024) {
			soft_iic_stop(siic);
			return kFailed;
		}
	}
	iic_scl_l(siic);//时钟输出0
	delay(iic_t);
	sda_out(siic); //SDA恢复为输出 
	
	return kOK;
}

/*************************************************************************************
 * 软件IIC发送一字节数据.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 * @param 		   txd  待发送的数据.
 *
 * @return 参数错误返回kIllgalArg，否则返回应答信息.
 *************************************************************************************/
int soft_iic_send_byte(soft_iic *siic, uint8_t txd)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	uint8_t t;
	for (t = 0; t < 8; t++) {
		if ((txd & 0x80) >> 7) {
			iic_sda_h(siic);
		}
		else {
			iic_sda_l(siic);
		}
		txd <<= 1;
		delay(iic_t/2);   
		iic_scl_h(siic);
		delay(iic_t);
		iic_scl_l(siic);
		delay(iic_t/2);
	}
	
	return soft_iic_wait_ack(siic);
}

/*************************************************************************************
 * 软件IIC发送一字节数据不接收应答信号.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 * @param 		   txd  待发送的数据.
 *
 * @return 参数错误返回kIllgalArg，否则返回应答信息.
 *************************************************************************************/
int soft_iic_send_byte_nack(soft_iic *siic, uint8_t txd)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	uint8_t t;
	for (t = 0; t < 8; t++) {
		if ((txd & 0x80) >> 7) {
			iic_sda_h(siic);
		}
		else {
			iic_sda_l(siic);
		}
		txd <<= 1;
		delay(iic_t/2);   
		iic_scl_h(siic);
		delay(iic_t);
		iic_scl_l(siic);
		delay(iic_t/2);
	}
	
	return kOK;
}

/*************************************************************************************
 * 产生ACK应答.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return 参数错误返回kIllgalArg.
 *************************************************************************************/
int soft_iic_ack(soft_iic *siic)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	iic_scl_l(siic);
	sda_out(siic);
	iic_sda_l(siic);
	delay(iic_t);
	iic_scl_h(siic);
	delay(iic_t);
	iic_scl_l(siic);
	
	return kOK;
}

/*************************************************************************************
 * 不产生ACK应答.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 *
 * @return 参数错误返回kIllgalArg.
 *************************************************************************************/
int soft_iic_nack(soft_iic *siic)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	iic_scl_l(siic);
	sda_out(siic);
	iic_sda_h(siic);
	delay(iic_t);
	iic_scl_h(siic);
	delay(iic_t);
	iic_scl_l(siic);
	
	return kOK;
}

/*************************************************************************************
 * 读1个字节数据.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic 软件IIC句柄.
 * @param 		   ack  ack=1时，发送ACK，ack=0，发送nACK.
 *
 * @return 读取到的1字节数据.
 *************************************************************************************/
uint8_t soft_iic_read_byte(soft_iic *siic, unsigned char ack)
{
	if (!siic) {
		return kIllgalArg;
	}
	
	unsigned char receive = 0;
	sda_in(siic);//SDA设置为输入
	for (int i = 0; i < 8; i++) {
		iic_scl_l(siic);
		delay(iic_t);
		iic_scl_h(siic);
		receive <<= 1;
		if (is_iic_sda_h(siic))
			receive++;
		delay(iic_t);
	}
	if (!ack)
		soft_iic_nack(siic);//发送nACK
	else
		soft_iic_ack(siic); //发送ACK   
	
	return receive;
}
