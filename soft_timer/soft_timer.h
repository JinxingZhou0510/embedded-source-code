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

#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------- types and constants definitions --------- */
/*! \enum timer_state_t
 *
 *  软定时器状态
 */
typedef enum {
    kTimeNone = -1, ///没有定时器可以使用
    kTimeFree = 0,  ///定时器未使用
    kTimeArmed = 1, ///定时器已使用
    kTimeTrig = 2,  ///定时器被触发
    kTimeTirgPeriod = 3,
} timer_state_t;

//定时值单位: 0.5msec
typedef uint32_t timer_val_t;

#define  kTimeValMax  (65535U)   ///最大定时值
#define   kTimeValMin  (2U)	///最小定时值

#define MS_TO_TIMEVAL(ms) ((timer_val_t)(ms * 2)) ///单位转换宏 毫秒转换为定时器的计数值
#define US_TO_TIMEVAL(us) ((timer_val_t)(us / 500))///单位转换宏 微秒转换为定时器的计数值

typedef int timer_handle_t;///定时器句柄
typedef void (*timer_callback_t)(timer_handle_t tim_handle, void* arg);///定时时间到时调用的回调函数


#define  kMaxNumTimer 128 ///最大定时器个数
typedef struct {
	timer_state_t state;       ///定时器当前状态
	timer_callback_t callback; ///定时时间到时的回调函数
	timer_val_t val;            ///定时时间
	timer_val_t interval;   ///非0是代表是周期定时的定时值 
	void* arg;///定时时间到时回调函数调用的参数
} soft_timer_t;


/* ---------  prototypes --------- */
/*#define set_alarm(d, id, callback, value, period) printf("%s, %d, set_alarm(%s, %s, %s, %s, %s)\n",__FILE__, __LINE__, #d, #id, #callback, #value, #period); _SetAlarm(d, id, callback, value, period)*/

/**
 @fn	timer_handle_t set_alarm(timer_callback_t callback, void* arg, timer_val_t value, timer_val_t period);

 @brief	设置一个软定时器

 @author	CLX
 @date	2018/6/6

 @param 			callback	软定时器触发时调用的回调函数
 @param [in,out]	arg			If non-null, the argument.回调函数使用的参数
 @param 			value   	定时时间，回调函数将在value时间后被调用
 @param 			period  	定时周期，非0则说明这个软定时器会按这个定时周期进行重复促发

 @return	A timer_handle_t.
 */

timer_handle_t set_alarm(timer_callback_t callback, void* arg, timer_val_t value, timer_val_t period);

/**
 @fn	timer_handle_t del_alarm(timer_handle_t handle);

 @brief	删除一个软定时器

 @author	CLX
 @date	2018/6/6

 @param	handle	The handle.

 @return	A timer_handle_t.
 */

timer_handle_t del_alarm(timer_handle_t handle);

/**
 @fn	void time_dispatch(void);

 @brief	Time dispatch 软定时器调度函数，必须在硬件定时器定时中断中调用

 @author	CLX
 @date	2018/6/6
 */

void time_dispatch(void);





/**
 @fn	timer_val_t get_elapsed_time(void);

 @brief	获取从上一次硬件定时被触发到现在过去的时间

 @author	CLX
 @date	2018/6/6

 @return	The elapsed time.
 */

timer_val_t get_elapsed_time(void);

#ifdef __cplusplus
}
#endif

#endif /* #define SOFT_TIMER_H */
