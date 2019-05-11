/*************************************************************************************

  * @file   soft_spi.h

  * @brief Thisis a brief description.

  * @author ZJX

  * @par   Copyright (c):

  *          ZJX.

  *         All Rights Reserved

  * @date   2019/05/06 

  *  @note   mattersneeding attention

  *  @version <version  number>
  
  *************************************************************************************/ 
#ifndef SOFT_SPI_H
#define SOFT_SPI_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>
#include "stm32_typedef.h"


typedef struct  _soft_spi{
	gpio_t clk_gpio;	
	gpio_pin_t clk_pin;
	gpio_t mosi_gpio;	
	gpio_pin_t mosi_pin; 
	gpio_t miso_gpio;	
	gpio_pin_t miso_pin;
}soft_spi;

/*************************************************************************************
 * Soft spi initialize
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi If non-null, the sspi.
 *
 * @return An int.
 *************************************************************************************/
int soft_spi_init(soft_spi *sspi);

/*************************************************************************************
 * Soft spi write
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi If non-null, the sspi.
 * @param [in,out] data If non-null, the data.
 * @param 		   len  The length.
 *
 * @return An int.
 *************************************************************************************/
int soft_spi_write(soft_spi *sspi, uint8_t *data, int len);

/*************************************************************************************
 * Soft spi read
 *
 * @author ZJX
 * @date 2019/5/6
 *
 * @param [in,out] sspi If non-null, the sspi.
 * @param [in,out] data If non-null, the data.
 * @param 		   len  The length.
 *
 * @return An int.
 *************************************************************************************/
int soft_spi_read(soft_spi *sspi, uint8_t *data, int len);

#define  clk_high(sspi)	HAL_GPIO_WritePin(sspi->clk_gpio, sspi->clk_pin, GPIO_PIN_SET)
#define  clk_low(sspi)	HAL_GPIO_WritePin(sspi->clk_gpio, sspi->clk_pin, GPIO_PIN_RESET)
#define  mosi_high(sspi) HAL_GPIO_WritePin(sspi->mosi_gpio, sspi->mosi_pin, GPIO_PIN_SET)
#define  mosi_low(sspi)	HAL_GPIO_WritePin(sspi->mosi_gpio, sspi->mosi_pin, GPIO_PIN_RESET)
#define  miso_high(sspi) HAL_GPIO_WritePin(sspi->miso_gpio,sspi->miso_pin,GPIO_PIN_SET)
#define  miso_low(sspi) HAL_GPIO_WritePin(sspi->miso_gpio,sspi->miso_pin,GPIO_PIN_RESET)
#define  is_miso_low(sspi) (GPIO_PIN_RESET==HAL_GPIO_ReadPin(sspi->miso_gpio, sspi->miso_pin))


#ifdef __cplusplus
}
#endif
#endif /* ifndef SOFT_SPI_H */