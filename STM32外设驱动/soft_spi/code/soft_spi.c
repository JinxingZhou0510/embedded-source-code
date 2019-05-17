/*************************************************************************************

  * @file   soft_spi.c

  * @brief Thisis a brief description.

  * @author ZJX

  * @par   Copyright (c):

  *          ZJX.

  *         All Rights Reserved

  * @date   2019/05/06 

  *  @note   mattersneeding attention

  *  @version <version  number>
  
  *************************************************************************************/ 
#include "soft_spi.h"
#include "error_report.h"
#include "log.h"
#include "stm32_common.h"

/*************************************************************************************
 * 延时值，用来简单控制spi通讯速率
 * @note
 *
 * @author ZJX
 * @date 2019/5/6
 *************************************************************************************/
#define  spi_t  10

/*************************************************************************************
 * 延时函数.
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
 * 软件SPI初始化.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi 软件SPI句柄.
 *
 * @return An int.
 *************************************************************************************/
int soft_spi_init(soft_spi *sspi)
{
	if (!sspi) {
		return kIllgalArg;
	}
	
	clk_high(sspi);
	mosi_low(sspi);
	
	return kOK;
}

/*************************************************************************************
 * 软件SPI写数据.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi 软件SPI句柄.
 * @param [in,out] data 待发送的数据地址.
 * @param 		   len  发送的数据长度.
 *
 * @return 参数错误返回kIllgalArg，否则返回0.
 *************************************************************************************/
int soft_spi_write(soft_spi *sspi, uint8_t *data, int len) 
{
	if (!sspi||!data||len<0){
		return kIllgalArg;
	}
	
	uint8_t mask;
	for (int i = 0; i < len;i++) {//逐字节
		for (int j = 0, mask = 0x80; j < 8; j++) {//逐位
			if ((mask & data[i]) == 0) {
				mosi_low(sspi);
			} else {
				mosi_high(sspi);
			}
			clk_low(sspi);
			delay(spi_t);
			clk_high(sspi);
			delay(spi_t);
			mask = mask >> 1;
		}
	}
	mosi_high(sspi);
	
	return kOK;
}

/*************************************************************************************
 * 软件SPI读数据.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi 软件SPI句柄.
 * @param [in,out] data 数据接收地址.
 * @param 		   len  接收数据的长度.
 *
 * @return 参数错误返回kIllgalArg，否则返回0.
 *************************************************************************************/
int soft_spi_read(soft_spi *sspi, uint8_t *data, int len)
{
	if (!sspi || !data || len < 0) {
		return kIllgalArg;
	}
	
	mosi_high(sspi);
	miso_high(sspi);
	uint8_t read_byte = 0;
	for (int i = 0; i < len; i++)	{
		for (int j = 0; j < 8; j++)		{
			clk_low(sspi);
			if (is_miso_low(sspi))		{
				read_byte = read_byte << 1;
			}	else			{
				read_byte = read_byte << 1;
				read_byte = read_byte + 0x01;
			}
			delay(spi_t);
			clk_high(sspi);
			delay(spi_t);
		}
		data[i] = read_byte;
		read_byte = 0;
	}
	
	return kOK;
}
