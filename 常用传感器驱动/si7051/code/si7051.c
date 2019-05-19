#include "si7051.h"

static uint8_t si705x_read_serial_number(si705x_typedef *si705x);
static int si705x_set_resolution(si705x_typedef *si705x, uint8_t resolution);

/**************************************************************************************************
 * 7051 initialize
 *
 * @author Jinxing Zhou
 * @date 2019/5/7
 *
 * @param [in,out] si705x The resolution.
 *
 * @return An int.
 *************************************************************************************************/
int si705x_init(si705x_typedef *si705x, si705x_id serial_number, si705x_precision precision)
{
	if (!si705x) {
        return kIllgalArg;
    }
    if (!si705x->siic) {
        return kIllgalArg;
    }
	
	soft_iic_init(si705x->siic);
	si705x->serial_number = serial_number;
	si705x->precision = precision;
	
	uint8_t si705x_serial_number = si705x_read_serial_number(si705x);
	if (si705x->serial_number != si705x_serial_number) {
		return kFailed;
	}
	if (kOK != si705x_set_resolution(si705x, si705x->precision)) {
		return kFailed;
	}
	
	return kOK;
}

/**************************************************************************************************
 * 7051 start.
 *
 * @author Jinxing Zhou
 * @date 2019/4/28
 *
 * @param [in,out] si705x If non-null, the 705x.
 *
 * @return An int.
 *************************************************************************************************/
int si705x_start(si705x_typedef *si705x)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);

	return kOK;
}

/**************************************************************************************************
 * 7051 reset.
 *
 * @author Jinxing Zhou
 * @date 2019/4/28
 *
 * @param [in,out] si705x If non-null, the 705x.
 *
 * @return An int.
 *************************************************************************************************/
int si705x_reset(si705x_typedef *si705x)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, CMD_RESET)) {
        return kFailed;
    }
	soft_iic_stop(siic);

	return kOK;
}

/**************************************************************************************************
 * 7051 sleep.
 *
 * @author Jinxing Zhou
 * @date 2019/4/28
 *
 * @param [in,out] si705x If non-null, the 705x.
 *
 * @return An int.
 *************************************************************************************************/
int si705x_Sleep(si705x_typedef *si705x)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
	soft_iic_stop(siic);

	return kOK;
}

/**************************************************************************************************
 * 705x delay milliseconds
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *
 * @param cnt Number of.
 *************************************************************************************************/
static void si705x_delay_ms(uint16_t cnt)
{
	delay_ms(cnt);
}

/**************************************************************************************************
 * 705x read serial number
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *
 * @param [in,out] si705x If non-null, the 705x.
 * @param 		   id_lsb The identifier LSB.
 * @param 		   id_hsb The identifier hsb.
 *
 * @return An int.
 *************************************************************************************************/
static int _si705x_read_serial_number(si705x_typedef *si705x, uint8_t id_lsb, uint8_t id_hsb)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_W)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, id_lsb)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, id_hsb)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_R)) {
        return kFailed;
    }
	
	return kOK;
}

/**************************************************************************************************
 * 705x read serial number
 *
 * @author Jinxing Zhou
 * @date 2019/5/11
 *
 * @param [in,out] si705x If non-null, the 705x.
 *
 * @return An uint8_t.
 *************************************************************************************************/
static uint8_t si705x_read_serial_number(si705x_typedef *si705x)
{
	if (!si705x) {
        return kIllgalArg;
    }

	 soft_iic *siic = si705x->siic;
   soft_iic_start(siic);
	if (kOK != _si705x_read_serial_number(si705x, CMD_READ_ID_1ST_LSB, CMD_READ_ID_1ST_HSB)) {
		return kFailed;
	}
	uint8_t sna   = soft_iic_read_byte(siic, ACK);  // SNA_3
	uint8_t crc   = soft_iic_read_byte(siic, ACK);
	        sna   = soft_iic_read_byte(siic, ACK);  // SNA_2
			crc   = soft_iic_read_byte(siic, ACK);
	        sna   = soft_iic_read_byte(siic, ACK);  // SNA_1
			crc   = soft_iic_read_byte(siic, ACK);
	        sna   = soft_iic_read_byte(siic, ACK);  // SNA_0
			crc   = soft_iic_read_byte(siic, NACK);
	soft_iic_stop(si705x->siic);

	if (kOK != _si705x_read_serial_number(si705x, CMD_READ_ID_2ST_LSB, CMD_READ_ID_2ST_HSB)) {
		return kFailed;
	}
	uint8_t snb_3 = soft_iic_read_byte(siic, ACK); // SNB_3
	        sna   = soft_iic_read_byte(siic, ACK); // SNB_2
			crc   = soft_iic_read_byte(siic, ACK); 
	        sna   = soft_iic_read_byte(siic, ACK); // SNB_1
	        sna   = soft_iic_read_byte(siic, ACK); // SNB_0
			crc   = soft_iic_read_byte(siic, NACK);
	soft_iic_stop(si705x->siic);

	return snb_3;
}

/**************************************************************************************************
 * 7051 read user register.
 *
 * @author Jinxing Zhou
 * @date 2019/4/28
 *
 * @return An uint8_t.
 *************************************************************************************************/
static uint8_t si705x_read_user_register(si705x_typedef *si705x)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_W)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, CMD_READ_REGISTER)) {
        return kFailed;
    }
	
	soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_R)) {
        return kFailed;
    }
	uint8_t sb = soft_iic_read_byte(siic, NACK);
	soft_iic_stop(siic);
	
	return sb;
}

/**************************************************************************************************
 * 7051 write user register.
 *
 * @author Jinxing Zhou
 * @date 2019/4/28
 *
 * @param data The data.
 *************************************************************************************************/
static int si705x_write_user_register(si705x_typedef *si705x, uint8_t data)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_W)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, CMD_WRITE_REGISTER)) {
        return kFailed;
    }
	if (kIllgalArg == soft_iic_send_byte_nack(siic, data)) {
        return kFailed;
    }
	soft_iic_ack(siic);
	soft_iic_stop(siic);

	return kOK;
}

/**************************************************************************************************
 * 7051 read firmware version.
 *
 * @author Jinxing Zhou
 * @author Jinxing Zhou
 * @date 2019/4/28
 * @date 2019/4/28
 *
 * @return An uint8_t. //** 7051 read firmware version.
 * @return An uint8_t.
 *************************************************************************************************/
static uint8_t si705x_read_firmware_version(si705x_typedef *si705x)
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_W)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, CMD_READ_FIRMWARE_LSB)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, CMD_READ_FIRMWARE_HSB)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_R)) {
        return kFailed;
    }
	uint8_t sb = soft_iic_read_byte(siic, NACK);
	soft_iic_stop(siic);

	return sb;
}

/**************************************************************************************************
 * 7051 set resolution.
 *
 * @author Jinxing Zhou
 * @date 2019/4/28
 *
 * @param resolution The resolution.
 *************************************************************************************************/
static int si705x_set_resolution(si705x_typedef *si705x, uint8_t resolution)
{
	si7051_register reg, reg_read_data;

	switch (resolution) {
	case 11:
		reg.resolution0 = 1;
		reg.resolution7 = 1;
		break;
	case 12:
		reg.resolution0 = 1;
		reg.resolution7 = 0;
		break;	
	case 13:
		reg.resolution0 = 0;
		reg.resolution7 = 1;
		break;
	case 14:
		reg.resolution0 = 0;
		reg.resolution7 = 0;
		break;
	default: 
		break;
	}
	
	si705x_write_user_register(si705x, reg.rawData);

	reg_read_data.rawData = si705x_read_user_register(si705x);

	if ((reg.resolution0 == reg_read_data.resolution0) && 
			(reg.resolution7 == reg_read_data.resolution7)) {
		return kOK;
	}

	return kFailed;
}

/**************************************************************************************************
 * 7051 read temperature
 *
 * @author Jinxing Zhou
 * @date 2019/5/7
 *
 * @return A float.
 *************************************************************************************************/
float si705x_read_temperature(si705x_typedef *si705x) 
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_W)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, CMD_MEASURE_TEMPERATURE_NO_HOLD)) {
        return kFailed;
    }
    si705x_delay_ms(10);
	
	soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_R)) {
        return kFailed;
    }
	uint8_t msb = soft_iic_read_byte(siic, ACK);
	uint8_t lsb = soft_iic_read_byte(siic, NACK);
	soft_iic_stop(siic);
	uint16_t val = msb << 8 | lsb;

	return (175.72*val) / 65536 - 46.85;
}

/**************************************************************************************************
 * 7051 read temperature master
 *
 * @author Jinxing Zhou
 * @date 2019/5/7
 *
 * @return A float.
 *************************************************************************************************/
float si705x_read_temperature_master(si705x_typedef *si705x) 
{
	if (!si705x) {
        return kIllgalArg;
    }

    soft_iic *siic = si705x->siic;
    soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_W)) {
        return kFailed;
    }
	if (kOK != soft_iic_send_byte(siic, CMD_MEASURE_TEMPERATURE_HOLD)) {
        return kFailed;
    }
    si705x_delay_ms(10);
	
	soft_iic_start(siic);
	if (kOK != soft_iic_send_byte(siic, SI705X_ADDR_R)) {
        return kFailed;
    }
	uint8_t msb = soft_iic_read_byte(siic, ACK);
	uint8_t lsb = soft_iic_read_byte(siic, NACK);
	soft_iic_stop(siic);
	uint16_t val = msb << 8 | lsb;

	return (175.72*val) / 65536 - 46.85;
}

/**************************************************************************************************
 * 7051 read t
 *
 * @author Jinxing Zhou
 * @date 2019/5/7
 *
 * @return A float.
 *************************************************************************************************/
float si705x_readT(si705x_typedef *si705x) 
{
	return si705x_read_temperature(si705x);
}
