#include "apds9960.h"

uint8_t apds9960_init(apds9960_data_type *apds9960)
{
    if (!apds9960) {
        return false;
    }
    if (!apds9960->siic) {
        return false;
    }
	
    uint8_t id;

	soft_iic_init(apds9960->siic);


}

uint8_t apds9960_get_mode()
{

}

uint8_t apds9960_set_mode()
{

}

uint8_t apds9960_enable_light_sensor()
{

}

uint8_t apds9960_disable_light_sensor()
{

}

uint8_t apds9960_enable_proximity_sensor()
{

}

uint8_t apds9960_disable_proximity_sensor()
{

}

uint8_t apds9960_enable_gesture_sensor()
{

}

uint8_t apds9960_disable_gesture_sensor()
{

}

uint8_t apds9960_is_gesture_available()
{

}

uint8_t apds9960_read_gesture()
{

}

uint8_t apds9960_enable_power()
{

}

uint8_t apds9960_disable_power()
{

}

uint8_t apds9960_read_ambient_light()
{

}

uint8_t apds9960_read_red_light()
{

}

uint8_t apds9960_read_green_light()
{

}

uint8_t apds9960_read_blue_light()
{

}

uint8_t apds9960_read_proximity()
{

}

uint8_t apds9960_reset_gesture_parameters()
{

}

uint8_t apds9960_process_gesture_data()
{

}

uint8_t apds9960_decode_gesture()
{

}

uint8_t apds9960_get_prox_int_low_thresh()
{

}

uint8_t apds9960_set_prox_int_low_thresh()
{

}

uint8_t apds9960_get_prox_int_high_thresh()
{

}

uint8_t apds9960_set_prox_int_high_thresh()
{
    
}

uint8_t apds9960_get_led_drive()
{

}

uint8_t apds9960_set_led_drive()
{

}

uint8_t apds9960_get_proximity_gain()
{

}

uint8_t apds9960_set_proximity_gain()
{

}

uint8_t apds9960_get_ambient_light_gain()
{

}

uint8_t apds9960_set_ambient_light_gain()
{

}

uint8_t apds9960_get_led_boost()
{

}

uint8_t apds9960_set_led_boost()
{

}

uint8_t apds9960_get_prox_gain_comp_enable()
{

}

uint8_t apds9960_set_prox_gain_comp_enable()
{

}

uint8_t apds9960_get_prox_photo_mask()
{

}

uint8_t apds9960_set_prox_photo_mask()
{

}

uint8_t apds9960_get_gesture_enter_thresh()
{

}

uint8_t apds9960_set_gesture_enter_thresh()
{

}

uint8_t apds9960_get_gesture_exit_thresh()
{

}

uint8_t apds9960_get_sesture_exit_thresh()
{

}

uint8_t apds9960_get_gesture_gain()
{

}

uint8_t apds9960_set_gesture_gain()
{

}

uint8_t apds9960_get_gesture_led_drive()
{

}

uint8_t apds9960_set_gesture_led_drive()
{
    
}

uint8_t apds9960_get_gesture_wait_time()
{

}

uint8_t apds9960_set_gesture_wait_time()
{

}

uint8_t apds9960_get_light_int_low_threshold()
{

}

uint8_t apds9960_set_light_int_low_threshold()
{

}

uint8_t apds9960_get_light_int_high_threshold()
{

}

uint8_t apds9960_set_light_int_high_threshold()
{

}

uint8_t apds9960_get_proximity_int_low_threshold()
{

}

uint8_t apds9960_set_proximity_int_low_threshold()
{
    
}

uint8_t apds9960_get_proximity_int_high_threshold()
{
    
}

uint8_t apds9960_set_proximity_int_high_threshold()
{
    
}

uint8_t apds9960_get_ambient_light_int_enable()
{

}

uint8_t apds9960_set_ambient_light_int_enable()
{

}

uint8_t apds9960_get_proximity_int_enable()
{

}

uint8_t apds9960_set_proximity_int_enable()
{

}

uint8_t apds9960_get_gesture_int_enable()
{

}

uint8_t apds9960_set_gesture_int_enable()
{

}

uint8_t apds9960_clear_ambient_light_int()
{

}

uint8_t apds9960_clear_proximity_int()
{

}

uint8_t apds9960_get_gesture_mode()
{

}

uint8_t apds9960_set_gesture_mode()
{

}
