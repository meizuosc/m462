#ifndef LINUX_SPI_IX_H
#define LINUX_SPI_IX_H

#if defined(CONFIG_IX_BTP_NAVIGATION) || defined(CONFIG_IX_BTP_MOUSE)
#define IX_NAVI_ENABLE
struct ix_nav_settings {
	u8 p_sensitivity_key;
	u8 p_sensitivity_ptr;
	u8 p_multiplier_x;
	u8 p_multiplier_y;

	u8 multiplier_key_accel;
	u8 multiplier_ptr_accel;
	int sum_x;
	int sum_y;
	u8 threshold_key_start;
	u8 threshold_key_accel;
	u8 threshold_key_dispatch;
	u8 threshold_ptr_start;
	u8 threshold_ptr_accel;
	u8 duration_key_clear;
	u8 duration_ptr_clear;

	u8 mouse_enable;
	u8 mouse_scroll;
	u8 mouse_scroll_skip_frame;

	u8 btp_input_mode;
};
#endif

struct ix_platform_data {
	int irq_gpio;
	int reset_gpio;
#if defined(IX_NAVI_ENABLE)
	struct ix_nav_settings nav;
#endif
};

#endif