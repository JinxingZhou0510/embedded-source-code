/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
/*!
** @file   timer.c
** @author Edouard TISSERANT and Francis DUPIN
** @date   Tue Jun  5 09:32:32 2007
**
** @brief
**
**
*/

#include "soft_timer.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "stm32_common.h"
#include "log.h"
#include <stdio.h>
///软定时器表
soft_timer_t timers[kMaxNumTimer] = {
    { kTimeFree, NULL, 0, 0 },
};

///上一次定时事件触发现在到下一个定时事件触发的休眠时间
static timer_val_t total_sleep_time = kTimeValMax;

static int last_timer_raw = -1;
///已使用的定时器个数
static unsigned int using_timers_num = 0;

static soft_timer_is_on = false;	/// 软定时器是否在工作

timer_handle_t set_alarm(timer_callback_t callback, void* arg, timer_val_t value, timer_val_t period)
{
	
    if (!callback) {
        return kTimeNone;
    }
    ///定时时间小于最小值会造成实际定时时间为kTimeValMax+value;
    if (value < kTimeValMin) {
        value = kTimeValMin;
    }
	if (value > kTimeValMax) {
		value = kTimeValMax;
	}
    if (period != 0 && period < kTimeValMin) {
        period = kTimeValMin;
    }
    timer_handle_t row_number;
    soft_timer_t* row;
	if (soft_timer_is_on)	{
		HAL_TIM_Base_Stop_IT(&htim7);
	}
    /* in order to decide new timer setting we have to run over all timer rows */
    for (row_number = 0, row = timers; row_number <= last_timer_raw + 1 && row_number < kMaxNumTimer; row_number++, row++) {
        if (row->state == kTimeFree) { 
            timer_val_t real_timer_value;
            timer_val_t elapsed_time;

            if (row_number == last_timer_raw + 1)
                last_timer_raw++;

            elapsed_time = get_elapsed_time();
            /* set next wakeup alarm if new entry is sooner than others, or if it is alone */
            ///实际定时时间不能超过最大定时值
            real_timer_value = value;
            real_timer_value = real_timer_value < kTimeValMax ? real_timer_value : kTimeValMax;
            ///还没到下一个定时事件触发的时间
            ///且该次定时的时间小于从现在到触发下一个定时事件的时间
            if (total_sleep_time > elapsed_time && total_sleep_time - elapsed_time > real_timer_value) {
                ///把这次定时设为下一次触发的定时事件
                total_sleep_time = elapsed_time + real_timer_value;
                ///设定从现在到下一个定时事件触发的定时值
                setTimer(real_timer_value);
            }
            row->callback = callback;
            row->arg = arg;
            row->val = real_timer_value + elapsed_time;
            row->interval = period;
            row->state = kTimeArmed;
			if (using_timers_num < 1) {
				soft_timer_is_on = true;
			}
			using_timers_num++;
			HAL_TIM_Base_Start_IT(&htim7);
            return row_number;
        }
    }
	if (soft_timer_is_on) {
		HAL_TIM_Base_Start_IT(&htim7);
	}
    return kTimeNone;
}


timer_handle_t del_alarm(timer_handle_t handle)
{
	HAL_TIM_Base_Stop_IT(&htim7);
    /* Quick and dirty. system timer will continue to be trigged, but no action will be preformed. */
    //MSG_WAR(0x3320, "del_alarm. handle = ", handle);
    if (handle > kTimeNone && handle < kMaxNumTimer) {
        if (handle == last_timer_raw)
            last_timer_raw--;
        timers[handle].state = kTimeFree;
		using_timers_num--;
		if (using_timers_num<1)	{
			soft_timer_is_on = false;
			__HAL_TIM_SET_COUNTER(&htim7, 0);
		}
		if (soft_timer_is_on) {
			HAL_TIM_Base_Start_IT(&htim7);
		}
        return handle;
    }
	if (soft_timer_is_on) {
		HAL_TIM_Base_Start_IT(&htim7);
	}
    return kTimeNone;
}


void time_dispatch(void)
{
	tim_flag_0();
    timer_handle_t i;
    timer_val_t next_wakeup = kTimeValMax; /* used to compute when should normaly occur next wakeup */
    /* First run : change timer state depending on time */
    /* Get time since timer signal */

    //累计误差 在执行time_dispatch时被其他高优先级的中断抢占而没有及时更新定时器的值，这是为了减小误差而设计的
    uint32_t overrun = (uint32_t)get_elapsed_time();

    timer_val_t real_total_sleep_time = total_sleep_time + overrun;

    soft_timer_t* row;

    for (i = 0, row = timers; i <= last_timer_raw; i++, row++) {
		//Logging(DDEBUG, "row:%u,val:%u\n",i, row->val);
        if (row->state & kTimeArmed) { /* if row is active */

            if (row->val <= real_total_sleep_time) { /* to be trigged */

                //单次定时
                if (!row->interval) { /* if simply outdated */

                    row->state = kTimeTrig; /* ask for trig */
                } else {                    /* or period have expired */

                    //周期定时 计算距离下一次周期到来还要多久
                    /* set val as interval, with 32 bit overrun correction, */
                    /* modulo for 64 bit not available on all platforms     */
                    row->val = row->interval - (overrun % (uint32_t)row->interval);
                    row->state = kTimeTirgPeriod; /* ask for trig, periodic */
                    /* Check if this new timer value is the soonest */
                    if (row->val < next_wakeup)
                        next_wakeup = row->val;
                }
            } else {
                /* Each armed timer value in decremented. */
                row->val -= real_total_sleep_time;

                /* Check if this new timer value is the soonest */
                if (row->val < next_wakeup)
                    next_wakeup = row->val;
            }
        }
    }

    /* Remember how much time we should sleep. */
    total_sleep_time = next_wakeup;

    /* Set timer to soonest occurence */
    setTimer(next_wakeup);

    /* Then trig them or not. */
    for (i = 0, row = timers; i <= last_timer_raw; i++, row++) {
        if (row->state & kTimeTrig) {
            row->state &= ~kTimeTrig; /* reset trig state (will be free if not periodic) */
            if (row->callback)
                (*row->callback)(i, row->arg); /* trig ! */
        }
    }
}

/**
@fn	void setTimer(timer_val_t value);

@brief	采用硬件定时器进行一次定时

@author	CLX
@date	2018/6/6

@param	value	The value.
*/
void setTimer(timer_val_t value)
{
    __HAL_TIM_SET_AUTORELOAD(&htim7, value&0xFFFF);
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    return;
}

inline timer_val_t get_elapsed_time(void)
{
	//timer_val_t elapsed_time = (timer_val_t)(__HAL_TIM_GET_COUNTER(&htim7) & 0xFFFF);
    return (timer_val_t)(__HAL_TIM_GET_COUNTER(&htim7) & 0xFFFF);
}
