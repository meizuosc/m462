/* mach/touch_booster.h
 *
 * Copyright (C) 2012 ZhuHai MEIZU Technology Co., Ltd.
 *	  http://www.meizu.com
 *
 * Meizu touch booster interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __H_TOUCH_BOOSTER__
#define __H_TOUCH_BOOSTER__

#include <linux/input.h>
struct tb_private_info {
	struct class tb_class;
	unsigned long boost_time;	/* us */
	unsigned long boost_hmp_time;	/* ms */
	unsigned int boost_debug;
	unsigned int event_type;
	unsigned int boost_type;
	unsigned int boost_request;
	struct input_handle handle;
	struct task_struct *boost_task;
	unsigned int task_started;
	wait_queue_head_t wait_queue;
	atomic_t is_ondemand;
	atomic_t is_start;
	int enabled;
};
#endif
