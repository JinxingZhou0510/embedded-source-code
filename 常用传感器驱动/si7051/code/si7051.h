#ifndef SI7051_H__
#define SI7051_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32_typedef.h"
#include "soft_iic.h"
#include "bit_opreate.h"
#include "delay.h"

/* I2C slave address of Si705x */
#define SI705X_ADDR_R      0x81
#define SI705X_ADDR_W      0x80

/* Commands */
#define CMD_MEASURE_TEMPERATURE_HOLD    0xE3
#define CMD_MEASURE_TEMPERATURE_NO_HOLD 0xF3
#define CMD_RESET                       0xFE
#define CMD_WRITE_REGISTER              0xE6
#define CMD_READ_REGISTER               0xE7
#define CMD_READ_ID_1ST_LSB             0XFA
#define CMD_READ_ID_1ST_HSB             0X0F
#define CMD_READ_ID_2ST_LSB             0XFC
#define CMD_READ_ID_2ST_HSB             0XC9
#define CMD_READ_FIRMWARE_LSB           0X84
#define CMD_READ_FIRMWARE_HSB           0XB8


/** Values that represent 705x precisions */
typedef enum _si705x_precision {
	one_percent = 11, // 温度精度0.01度
	two_percent，     // 温度精度0.02度
	four_percent，    // 温度精度0.04度
	eight_percent     // 温度精度0.08度
}si705x_precision;

/**************************************************************************************************
 * A 7051 register.
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *************************************************************************************************/
#pragma anon_unions
typedef union {
	uint8_t rawData;
	struct {
		uint8_t resolution0 : 1;
		uint8_t reserve1 : 4;
		uint8_t vdds : 1; // vdds = 1 if and only if VDD between 1.8V and 1.9V
		uint8_t reserved2 : 1;
		uint8_t resolution7 : 1;
	};
} si7051_register;

/** Values that represent 705x Identifiers */
typedef enum _si705x_id {
	Si7050 = 50,
	Si7051,
	Si7052,
	Si7053,
	Si7054,
	Si7055
}si705x_id;

/**************************************************************************************************
 * A 7051.
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *************************************************************************************************/
typedef struct _si7051 {
	soft_iic *siic;
	si705x_id serial_number;
	si705x_precision precision;
	float temp_value;
} si705x_typedef;

/**************************************************************************************************
 * 705x initialize
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *
 * @param [in,out] si705x		 If non-null, the 705x.
 * @param 		   serial_number The serial number.
 * @param 		   precision	 The precision.
 *
 * @return An int.
 *************************************************************************************************/
int si705x_init(si705x_typedef *si705x, si705x_id serial_number, si705x_precision precision);

/**************************************************************************************************
 * 705x read t
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *
 * @param [in,out] si705x If non-null, the 705x.
 *
 * @return A float.
 *************************************************************************************************/
float si705x_readT(si705x_typedef *si705x);

#ifdef __cplusplus
}
#endif
#endif