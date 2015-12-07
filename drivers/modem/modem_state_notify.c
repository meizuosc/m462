/******************************************************************************
*(C) Copyright 2014 Marvell International Ltd.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>

#include "modem_state_notify.h"

static enum modemState modem_state = UNKNOWN_STATE;
static spinlock_t modem_state_lock;

static RAW_NOTIFIER_HEAD(modem_state_chain);
int register_modem_state_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&modem_state_chain, nb);
}

EXPORT_SYMBOL(register_modem_state_notifier);

int unregister_modem_state_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&modem_state_chain, nb);
}

EXPORT_SYMBOL(unregister_modem_state_notifier);

static int modem_state_notifier_call_chain(unsigned long val, void *v)
{
	return raw_notifier_call_chain(&modem_state_chain, val, v);
}

EXPORT_SYMBOL(modem_state_notifier_call_chain);

void init_modem_notify_chain(void)
{
	spin_lock_init(&modem_state_lock);
}

static char *modem_state_name[] = {
	"HOLD",
	"RELEASE",
	"BOOTUP",
	"ASSERT",
	"WDT",
	"UNKNOWN_STATE"
};

void set_modem_state(enum modemState state, char *env[])
{
	if (env != NULL) {
		switch (state) {
		case HOLD:
			env[0] = "STATE=HOLD";
			break;
		case RELEASE:
			env[0] = "STATE=RELEASE";
			break;
		case BOOTUP:
			env[0] = "STATE=BOOTUP";
			break;
		case ASSERT:
			env[0] = "ERROR=ASSERT";
			break;
		case WDT:
			env[0] = "ERROR=WDTIMEOUT";
			break;
		default:
			env[0] = "STATE=UNKNOWN";
			break;
		}
		env[1] = NULL;
	}
	spin_lock(&modem_state_lock);
	modem_state = state;
	modem_state_notifier_call_chain((unsigned long)modem_state, NULL);
	spin_unlock(&modem_state_lock);
}

enum modemState get_modem_state(void)
{
	return modem_state;
}

EXPORT_SYMBOL(get_modem_state);

char *get_modem_state_name(int state)
{
	if (state <= UNKNOWN_STATE && state >= HOLD)
		return modem_state_name[state];
	else if (modem_state <= UNKNOWN_STATE && modem_state >= HOLD)
		return modem_state_name[modem_state];
	else
		return "UNKNOW_STATE";
}

EXPORT_SYMBOL(get_modem_state_name);
