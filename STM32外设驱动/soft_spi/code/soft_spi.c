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
 * ��ʱֵ�������򵥿���spiͨѶ����
 * @note
 *
 * @author ZJX
 * @date 2019/5/6
 *************************************************************************************/
#define  spi_t  10

/*************************************************************************************
 * ��ʱ����.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param cnt ��ʱʱ�䣨��λ��us��.
 *************************************************************************************/
static inline void delay(unsigned int cnt)
{
	delay_us(cnt);
}

/*************************************************************************************
 * ���SPI��ʼ��.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi ���SPI���.
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
 * ���SPIд����.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi ���SPI���.
 * @param [in,out] data �����͵����ݵ�ַ.
 * @param 		   len  ���͵����ݳ���.
 *
 * @return �������󷵻�kIllgalArg�����򷵻�0.
 *************************************************************************************/
int soft_spi_write(soft_spi *sspi, uint8_t *data, int len) 
{
	if (!sspi||!data||len<0){
		return kIllgalArg;
	}
	
	uint8_t mask;
	for (int i = 0; i < len;i++) {//���ֽ�
		for (int j = 0, mask = 0x80; j < 8; j++) {//��λ
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
 * ���SPI������.
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi ���SPI���.
 * @param [in,out] data ���ݽ��յ�ַ.
 * @param 		   len  �������ݵĳ���.
 *
 * @return �������󷵻�kIllgalArg�����򷵻�0.
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
