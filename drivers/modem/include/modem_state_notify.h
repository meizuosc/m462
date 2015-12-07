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
#ifndef __MODEM_STATE_NOTIFY_H_
#define __MODEM_STATE_NOTIFY_H_

enum modemState {
	HOLD = 0,
	RELEASE,
	BOOTUP,
	ASSERT,
	WDT,
	UNKNOWN_STATE
};

void set_modem_state(enum modemState state, char *env[]);
enum modemState get_modem_state(void);
char *get_modem_state_name(int state);

void init_modem_notify_chain(void);
int register_modem_state_notifier(struct notifier_block *nb);
int unregister_modem_state_notifier(struct notifier_block *nb);

#endif
