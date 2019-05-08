/*************************************************************************************

  * @file   soft_iic.h

  * @brief Thisis a brief description.

  * @author ZJX

  * @par   Copyright (c):

  *          ZJX.

  *         All Rights Reserved

  * @date   2019/05/06 

  *  @note   mattersneeding attention

  *  @version <version  number>
  
  *************************************************************************************/ 
#ifndef SOFT_IIC_H
#define SOFT_IIC_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "stm32_typedef.h"
//@note 注意IO要配置成开漏上拉模式

typedef struct  _soft_iic {
	gpio_t scl_gpio;
	gpio_pin_t scl_pin;
	gpio_t sda_gpio;
	gpio_pin_t sda_pin;
}soft_iic;

/*************************************************************************************
 * Iic initialize 令iic进入空闲状态 　I2C总线总线的SDA和SCL两条信号线同时处于高电平时，规定为总线的空闲状态。
 * 此时各个器件的输出级场效应管均处在截止状态，即释放总线，由两条信号线各自的上拉电阻把电平拉高。
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 *
 * @return An int.
 *************************************************************************************/
int soft_iic_init(soft_iic *siic);     

/*************************************************************************************
 * 产生IIC起始信号 起始信号：当SCL为高期间，SDA由高到低的跳变；
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 *
 * @return An int.
 *************************************************************************************/
int soft_iic_start(soft_iic *siic);	

/*************************************************************************************
 * 产生IIC停止信号 停止信号：当SCL为高期间，SDA由低到高的跳变；
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 *
 * @return An int.
 *************************************************************************************/
int soft_iic_stop(soft_iic *siic);	

/*************************************************************************************
 * 等待应答信号到来,如果超时没有应答就自动发送一个stop信号
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 *
 * @return An int.
 *************************************************************************************/
int soft_iic_wait_ack(soft_iic *siic); 	

/*************************************************************************************
 * 产生ACK应答
 * 
 * 发送器每发送一个字节，就在时钟脉冲9期间释放数据线，由接收器反馈一个应答信号。 
 * 应答信号为低电平时，规定为有效应答位（ACK简称应答位），表示接收器已经成功地接收了该字节；
 * 应答信号为高电平时，规定为非应答位（NACK），一般表示接收器接收该字节没有成功。 
 * 对于反馈有效应答位ACK的要求是，接收器在第9个时钟脉冲之前的低电平期间将SDA线拉低，
 * 并且确保在该时钟的高电平期间为稳定的低电平。 如果接收器是主控器，则在它收到最后一个字节后，
 * 发送一个NACK信号，以通知被控发送器结束数据发送，并释放SDA线，以便主控接收器发送一个停止信号P
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 *
 * @return An int.
 *************************************************************************************/
int soft_iic_ack(soft_iic *siic);		

/*************************************************************************************
 * 不产生ACK应答
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 *
 * @return An int.
 *************************************************************************************/
int soft_iic_nack(soft_iic *siic);	

/*************************************************************************************
 * IIC发送一个字节
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 * @param 		   txd  Information describing the transmit.
 *
 * @return An int.  kOK 表示从机应答了.
 *************************************************************************************/
int soft_iic_send_byte(soft_iic *siic, uint8_t txd);

/*************************************************************************************
 * 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] siic The siic.
 * @param 		   ack  The acknowledge.
 *
 * @return An uint8_t.
 *************************************************************************************/
uint8_t soft_iic_read_byte(soft_iic *siic, unsigned char ack);


#ifdef __cplusplus
}
#endif
#endif //SOFT_IIC_H
















