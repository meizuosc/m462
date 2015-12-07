#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h> /* BUS_I2C */
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/wakelock.h>
#include <linux/mfd/mx-hub.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/meizu-sys.h>

/* GPIO for MCU control */

#define ACK		0x79
#define NACK		0x1F

#define DPS_MAX			(1 << (16 - 1))
#define MAX_IIC_ERROR	5	

/* Input poll interval in milliseconds */
#define CWMCU_POLL_INTERVAL	10
#define CWMCU_POLL_MAX		200
#define CWMCU_POLL_MIN		10

#define CWMCU_MAX_OUTPUT_ID		(CW_SNAP+1)
#define CWMCU_MAX_OUTPUT_BYTE		(CWMCU_MAX_OUTPUT_ID * 7)
#define CWMCU_MAX_DRIVER_OUTPUT_BYTE		256

/* turn on gpio interrupt if defined */
#define CWMCU_INTERRUPT
#define LINK_KOBJ_NAME	"mx_hub"

struct CWMCU_data *sensor;
//static volatile int iic_error = 0;

static struct mfd_cell mx7_hub_devs[] = {
	{ .name = "mx7-hub-led", },
};

static void cwmcu_powermode_switch(SWITCH_POWER_ID id, int onoff);
static int CWMCU_i2c_write_upf(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int i;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(sensor->client, reg_addr++, data[i]);
		if (dummy < 0) {
		pr_err("%s: i2c write error =%d, reg_addr = %d\n", __func__, dummy, reg_addr);
			return dummy;
		}
	}
	return 0;
}

/* Returns the number of read bytes on success */
static int CWMCU_i2c_read_upf(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	return i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data);
}

int CWMCU_i2c_write(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int i;
	int rty = 0;

	if(sensor->is_update)
		return -EBUSY;

	mutex_lock(&sensor->iolock);

	for (i = 0; i < len; i++) {
retry:
		dummy = i2c_smbus_write_byte_data(sensor->client, reg_addr++, data[i]);
		if (dummy < 0) {
			if(rty < 5 ){
				//cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT,1);
				gpio_set_value(sensor->board_data->wake_up_gpio, 0);
				usleep_range(100, 300);
				gpio_set_value(sensor->board_data->wake_up_gpio, 1);
				usleep_range(300, 500);
				rty++;
				goto retry;
			}else{

				pr_err("%s: i2c write error =%d, reg_addr = %d\n", __func__, dummy, reg_addr);
				goto err_write;
			}
		}
	}

	mutex_unlock(&sensor->iolock);
	//iic_error = 0;
	return 0;

err_write:
	mutex_unlock(&sensor->iolock);
	//iic_error ++;
	//mx_hub_reinit();
	return dummy;
}

/* Returns the number of read bytes on success */
int CWMCU_i2c_read(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int ret;
	int rty = 0;

	if(sensor->is_update)
		return -EBUSY;

	mutex_lock(&sensor->iolock);
retry:
	ret = i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data);
	if(ret < 0)
	{
		if(rty < 5){
			//cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT,1);
			gpio_set_value(sensor->board_data->wake_up_gpio, 0);
			usleep_range(100, 300);
			gpio_set_value(sensor->board_data->wake_up_gpio, 1);
			usleep_range(300, 500);
			rty++;
			goto retry;
		}else{
			printk("%s : is error, the ret is %d, regJ_addr = %d\n", __func__, ret, reg_addr);
			goto err_read;
		}
	}
	mutex_unlock(&sensor->iolock);
	//iic_error = 0;
	return ret;

err_read:
	mutex_unlock(&sensor->iolock);
	//iic_error ++;
	//mx_hub_reinit();
	return ret;
}

/* write format    1.slave address  2.data[0]  3.data[1] 4.data[2] */
static int CWMCU_write_i2c_block(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int rty = 0;
	if(sensor->is_update)
		return -EBUSY;

retry:
	dummy = i2c_smbus_write_i2c_block_data(sensor->client, reg_addr, len, data);
	if (dummy < 0) {
		if(rty < 5 ){
			//cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT,1);
			gpio_set_value(sensor->board_data->wake_up_gpio, 0);
			usleep_range(100, 300);
			gpio_set_value(sensor->board_data->wake_up_gpio, 1);
			usleep_range(300, 500);
			rty++;
			goto retry;
		}else{
			pr_err("%s :i2c write error =%d, reg_addr = %d \n",__func__, dummy, reg_addr);
			return dummy;
		}
	}
	return 0;
}

static int CWMCU_write_serial(u8 *data, int len)
{
	int dummy;
	dummy = i2c_master_send(sensor->client, data, len);
	if (dummy < 0) {
		pr_err("%s :i2c write error =%d\n",__func__, dummy);
		return dummy;
	}
	return 0;
}

static int CWMCU_read_serial(u8 *data, int len)
{
	int dummy;
	dummy = i2c_master_recv(sensor->client, data, len);
	if (dummy < 0) {
		pr_err("i2c read error =%d\n", dummy);
		return dummy;
	}
	return 0;
}

int mx_hub_read_addr(struct CWMCU_data *sensor,u8 addr,
				  int bytes, void *dest)
{
	struct i2c_client *client = sensor->client;
	struct i2c_msg xfer[2];
	int ret;

	if(sensor->is_update)
		return -EBUSY;

	mutex_lock(&sensor->iolock);

	cwmcu_powermode_switch(SWITCH_POWER_LED, 1);
	xfer[0].addr =  client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &addr;

	xfer[1].addr =  client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = bytes;
	xfer[1].buf = (char *)dest;

	ret = i2c_transfer(client->adapter, xfer, 2);
	cwmcu_powermode_switch(SWITCH_POWER_LED, 0);
	mutex_unlock(&sensor->iolock);
	if (ret < 0){
		pr_err("%s :i2c read error =%d, addr = %d \n",__func__, ret, addr);
	}
	return ret;


}

int mx_hub_write_addr(struct CWMCU_data *sensor, u8 reg, int bytes, const void *src)
{
	struct i2c_client *client = sensor->client;
	int ret;	
	unsigned char buf[256];
	struct i2c_msg xfer;

	if(sensor->is_update)
		return -EBUSY;

	mutex_lock(&sensor->iolock);

	cwmcu_powermode_switch(SWITCH_POWER_LED, 1);
	if(bytes > 255 )
		bytes = 255;

	buf[0] = reg;
	memcpy(&buf[1],src,bytes);

	xfer.addr = client->addr;
	xfer.flags = 0;
	xfer.len = bytes + 1;
	xfer.buf = (char *)buf;

	ret = i2c_transfer(client->adapter, &xfer, 1);
	cwmcu_powermode_switch(SWITCH_POWER_LED, 0);
	mutex_unlock(&sensor->iolock);
	if (ret < 0)
		goto err_write;
	if (ret != 1) {
		ret = -EIO;
		goto err_write;
	}
	//iic_error = 0;
	return 0;

err_write:
	//iic_error ++;
	//mx_hub_reinit();
	return ret;

}


static int CWMCU_Set_Mcu_Mode(int mode)
{
	switch (sensor->mcu_mode) {
	case CW_NORMAL:
		sensor->mcu_mode = mode;
		
		break;
	case CW_SLEEP:
		sensor->mcu_mode = mode;
		
		break;
	case CW_NO_SLEEP:
		sensor->mcu_mode = mode;
		break;
	case CW_BOOT:
		sensor->mcu_mode = mode;
		break;
	default:
		return -EAGAIN;
	}
	return 0;
}

static void cwmcu_debuglog(void)
{
	u8 data[40] = {0};
	s16 data_buff[3] = {0};
	int i = 0;

	if (CWMCU_i2c_read(sensor, CW_ERRORCOUNT, data, 1) >= 0) {
		data_buff[0] = (s16)data[0];
		/* printk(KERN_DEBUG "errorcount = %d\n",data_buff[0]); */
		for (i = 0; i < data_buff[0]; i++) {
			if (CWMCU_i2c_read(sensor, CW_ERRORLOG, data, 30) >= 0)
				printk(KERN_DEBUG "%s\n", data);
		}
	}
}

static void cwmcu_powermode_switch(SWITCH_POWER_ID id, int onoff)
{
	if (onoff) {
		if (sensor->power_on_list == 0) {
			gpio_set_value(sensor->board_data->wake_up_gpio, onoff);
		}
		sensor->power_on_list |= ((uint32_t)(1) << id);
		usleep_range(500, 1000);
	} else {
		sensor->power_on_list &= ~(1 << id);
		if (sensor->power_on_list == 0) {
			gpio_set_value(sensor->board_data->wake_up_gpio, onoff);
		}
	}
	/* printk(KERN_DEBUG "--CWMCU--%s id = %d, onoff = %d\n", __func__, id, onoff); */
}

static void cwmcu_batch_read(struct CWMCU_data *sensor)
{
	int i = 0;
	int event_count = 0;
	uint8_t data[20] = {0};
	uint8_t data_buff = 0;
	uint32_t data_event[4] = {0};

	/* read the count of batch queue */
	if (CWMCU_i2c_read(sensor, CW_BATCHCOUNT, data, 2) >= 0) {
		data_event[0] = ((u32)data[1] << 8) | (u32)data[0];
		event_count = data_event[0];
		printk(KERN_DEBUG "--CWMCU-- batch count %d\n", event_count);
	} else {
		printk(KERN_DEBUG "--CWMCU-- check batch count failed~!!\n");
	}

	for (i = 0; i < event_count; i++) {
		if (CWMCU_i2c_read(sensor, CW_BATCHEVENTDATA, data, 9) >= 0) {
			/* check if there are no data from queue */
			if (data[0] != CWMCU_NODATA) {
				if (data[0] == CW_META_DATA) {
					data_event[1] = ((u32)data[0] << 16) | ((u32)data[4] << 8) | (u32)data[3];
					printk(KERN_DEBUG "--CWMCU-- CW_META_DATA return flush event_id = %d complete~!!\n", data[3]);
					input_report_abs(sensor->input, CW_ABS_Z, data_event[1]);
					input_sync(sensor->input);
				} else if (data[0] == CW_MAGNETIC_UNCALIBRATED_BIAS) {
					data_buff = CW_MAGNETIC_UNCALIBRATED;
					data_event[1] = ((u32)data_buff << 16) | ((u32)data[4] << 8) | (u32)data[3];
					data_event[2] = ((u32)data_buff << 16) | ((u32)data[6] << 8) | (u32)data[5];
					data_event[3] = ((u32)data_buff << 16) | ((u32)data[8] << 8) | (u32)data[7];

					printk(KERN_DEBUG "--CWMCU-- Batch data: total count = %d, current count = %d, event_id = %d, data_x = %d, data_y = %d, data_z = %d\n"
									, event_count
									, i
									, data[0]
									, ((int16_t)(((u32)data[4] << 8) | (u32)data[3]))
									, ((int16_t)(((u32)data[6] << 8) | (u32)data[5]))
									, ((int16_t)(((u32)data[8] << 8) | (u32)data[7]))
									);
					input_report_abs(sensor->input, CW_ABS_X1, data_event[1]);
					input_report_abs(sensor->input, CW_ABS_Y1, data_event[2]);
					input_report_abs(sensor->input, CW_ABS_Z1, data_event[3]);
				} else if (data[0] == CW_GYROSCOPE_UNCALIBRATED_BIAS) {
					data_buff = CW_GYROSCOPE_UNCALIBRATED;
					data_event[1] = ((u32)data_buff << 16) | ((u32)data[4] << 8) | (u32)data[3];
					data_event[2] = ((u32)data_buff << 16) | ((u32)data[6] << 8) | (u32)data[5];
					data_event[3] = ((u32)data_buff << 16) | ((u32)data[8] << 8) | (u32)data[7];

					printk(KERN_DEBUG "--CWMCU-- Batch data: total count = %d, current count = %d, event_id = %d, data_x = %d, data_y = %d, data_z = %d\n"
									, event_count
									, i
									, data[0]
									, ((int16_t)(((u32)data[4] << 8) | (u32)data[3]))
									, ((int16_t)(((u32)data[6] << 8) | (u32)data[5]))
									, ((int16_t)(((u32)data[8] << 8) | (u32)data[7]))
									);
					input_report_abs(sensor->input, CW_ABS_X1, data_event[1]);
					input_report_abs(sensor->input, CW_ABS_Y1, data_event[2]);
					input_report_abs(sensor->input, CW_ABS_Z1, data_event[3]);
				} else {
					data_event[0] = ((u32)data[0] << 16) | ((u32)data[2] << 8) | (u32)data[1];
					data_event[1] = ((u32)data[0] << 16) | ((u32)data[4] << 8) | (u32)data[3];
					data_event[2] = ((u32)data[0] << 16) | ((u32)data[6] << 8) | (u32)data[5];
					data_event[3] = ((u32)data[0] << 16) | ((u32)data[8] << 8) | (u32)data[7];
					/*
					printk(KERN_DEBUG "--CWMCU-- Batch data: total count = %d, current count = %d, event_id = %d, data_x = %d, data_y = %d, data_z = %d\n"
									, event_count
									, i
									, data[0]
									, ((int16_t)(((u32)data[4] << 8) | (u32)data[3]))
									, ((int16_t)(((u32)data[6] << 8) | (u32)data[5]))
									, ((int16_t)(((u32)data[8] << 8) | (u32)data[7]))
									);
					*/
					/* check flush event */
					input_report_abs(sensor->input, CW_ABS_X, data_event[1]);
					input_report_abs(sensor->input, CW_ABS_Y, data_event[2]);
					input_report_abs(sensor->input, CW_ABS_Z, data_event[3]);
					input_report_abs(sensor->input, CW_ABS_TIMEDIFF, data_event[0]);
					input_sync(sensor->input);
				}
			}
		}
	}
}

static void cwmcu_gesture_read(struct CWMCU_data *sensor)
{
	uint8_t data[2] = {0};
	uint32_t data_event;
	int data_count = 0;
	int i = 0;

	if (CWMCU_i2c_read(sensor, CW_READ_GESTURE_EVENT_COUNT, data, 1) >= 0) {
		data_count = data[0];
		for (i = 0; i < data_count; i++) {
			/* read 2byte */
			if (CWMCU_i2c_read(sensor, CW_READ_GESTURE_EVENT_DATA, data, 2) >= 0) {
				if (data[0] != CWMCU_NODATA) {
					data_event = ((u32)data[0] << 16) | (((u32)data[1]));

					printk(KERN_DEBUG "--CWMCU--Normal gesture %d data -> x = %d\n"
							, data[0]
							, data[1]
							);
					input_report_abs(sensor->input, CW_ABS_X, data_event);
					input_sync(sensor->input);
				}
			} else {
				printk(KERN_DEBUG "--CWMCU-- read gesture failed~!!\n");
			}
		}
	}
}
static void cwmcu_send_flush(int id)
{
	uint32_t flush_data = 0;

	flush_data = ((u32)CW_META_DATA << 16) | id;
	printk(KERN_DEBUG "--CWMCU-- flush sensor: %d auto return~!!\n", id);
	input_report_abs(sensor->input, CW_ABS_Z, flush_data);
	input_sync(sensor->input);
	/* reset flush event for ABS_Z*/
	input_report_abs(sensor->input, CW_ABS_Z, 0);
	input_sync(sensor->input);
}

#if 0
static void cwmcu_check_sensor_update(void)
{
	int id = 0;
	int64_t tvusec, temp = 0;
	unsigned int tvsec;

	do_gettimeofday(&sensor->now);
	tvsec = sensor->now.tv_sec;
	tvusec = sensor->now.tv_usec;

	temp = (int64_t)(tvsec * 1000000LL) + tvusec;

	/* printk(KERN_DEBUG "--CWMCU-- time(u) = %llu, tv_sec = %u, v_usec = %llu\n",temp, tvsec, tvusec); */

	for (id = 0; id < CW_SENSORS_ID_END; id++) {
	//	sensor->time_diff[id] = temp - sensor->sensors_time[id];
		if( sensor->enabled_list & (1<<id) && (sensor->sensor_timeout[id] == 0)) {
			 //printk("--CWMCU-- id = %d sensor->sensor_timeout = %lld \n", id, sensor->sensor_timeout[id]); 
			sensor->time_diff[id] = temp - sensor->sensors_time[id];
			//printk("sensor->time_diff = %lld , sensor->report_period = %d\n", sensor->time_diff[id], sensor->report_period[id]);
			if (sensor->time_diff[id] >= (sensor->report_period[id] * 1000)) {
				
//	printk("--CWMCU-- sensor->update_list = %d\n", sensor->update_list);
				sensor->update_list |= 1<<id;
				sensor->sensors_time[id] = temp;
			} else {
				sensor->update_list &= ~(1<<id);
			}
		} else {
			sensor->update_list &= ~(1<<id);
		}
	}
	//printk("--CWMCU-- sensor->update_list = %d\n", sensor->update_list);
}
#endif

static int CWMCU_read(struct CWMCU_data *sensor)
{
	int id_check = 0;
	uint8_t data[20] = {0};
	uint32_t data_event[7] = {0};

	if (sensor->mcu_mode == CW_BOOT) {
		/* it will not get data if status is bootloader mode */
		return 0;
	}

	//cwmcu_check_sensor_update();

	if (sensor->update_list) {
		for (id_check = 0; id_check < CW_SENSORS_ID_END; id_check++) {
			if ((sensor->update_list & (1<<id_check)) && (sensor->sensor_timeout[id_check] == 0)) {
					switch (id_check) {
					//case CW_ACCELERATION:
					case CW_MAGNETIC:
					case CW_GYRO:
					case CW_LIGHT:
					case CW_PROXIMITY:
					case CW_PRESSURE:
					case CW_ORIENTATION:
					case CW_ROTATIONVECTOR:
					case CW_LINEARACCELERATION:
					case CW_GRAVITY:
					case CW_AIRMOUSE:
					case CW_GAME_ROTATION_VECTOR:
					case CW_GEOMAGNETIC_ROTATION_VECTOR:
					case CW_TILT:
					case CW_PDR:
							/* read 6byte */
							if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 6) >= 0) {
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);

								/*	printk( "--CWMCU--Normal %d data -> x = %d, y = %d, z = %d\n"
												, id_check
												, (int16_t)((u32)data[1] << 8) | (u32)data[0]
												, (int16_t)((u32)data[3] << 8) | (u32)data[2]
												, (int16_t)((u32)data[5] << 8) | (u32)data[4]
												);*/
									if (id_check == CW_MAGNETIC || id_check == CW_ORIENTATION) {
										if (CWMCU_i2c_read(sensor, CW_ACCURACY, data, 1) >= 0) {
											data_event[6] = ((u32)id_check << 16) | (u32)data[0];
										}
										//printk(KERN_DEBUG "--CWMCU--MAG ACCURACY = %d\n", data[0]);
										input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
										input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
										input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
										input_report_abs(sensor->input, CW_ABS_ACCURACY, data_event[6]);
										input_sync(sensor->input);

										/*reset */
										input_report_abs(sensor->input, CW_ABS_X,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_Y,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_Z,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_ACCURACY,  0xFF0000);
										input_sync(sensor->input);
									} else {
										input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
										input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
										input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
										input_sync(sensor->input);

										/*reset */
										input_report_abs(sensor->input, CW_ABS_X,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_Y,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_Z,  0xFF0000);
										input_sync(sensor->input);
									}
							} else {
								printk( "--CWMCU-- CW_ACCELERATION CWMCU_i2c_read error 0x%x~ update_list = 0x%x!!!\n", CWMCU_I2C_SENSORS_REG_START+id_check, sensor->update_list);
							}
							break;
					case CW_ACCELERATION:
							/* read 6byte */
							if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 6) >= 0) {
							    if(sensor->acc_non_wake){
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);

									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_sync(sensor->input);

										/*reset */
									input_report_abs(sensor->input, CW_ABS_X,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z,  0xFF0000);
									input_sync(sensor->input);
								}
								if(sensor->phone_enable){
								    id_check = 30;
								    data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);

									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_sync(sensor->input);

										/*reset */
									input_report_abs(sensor->input, CW_ABS_X,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z,  0xFF0000);
									input_sync(sensor->input);
								}
							} else {
								printk( "--CWMCU-- CW_ACCELERATION CWMCU_i2c_read error 0x%x~ update_list = 0x%x!!!\n", CWMCU_I2C_SENSORS_REG_START+id_check, sensor->update_list);
							}
							break;
					case CW_RGB:
							/* read 8byte */
							if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 8) >= 0) {
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);
									data_event[3] = ((u32)id_check << 16) | (((u32)data[7] << 8) | (u32)data[6]);

									printk(KERN_DEBUG "--CWMCU--Normal %d data -> x = %d, y = %d, z = %d, ct = %d\n"
												, id_check
												, (int16_t)((u32)data[1] << 8) | (u32)data[0]
												, (int16_t)((u32)data[3] << 8) | (u32)data[2]
												, (int16_t)((u32)data[5] << 8) | (u32)data[4]
												, (int16_t)((u32)data[7] << 8) | (u32)data[6]
												);
									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_report_abs(sensor->input, CW_ABS_X1, data_event[3]);
									input_sync(sensor->input);

									/*reset */
									input_report_abs(sensor->input, CW_ABS_X,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z,  0xFF0000);
									input_report_abs(sensor->input, CW_ABS_X1,  0xFF0000);
									input_sync(sensor->input);
						//	} else {
						//		printk("--CWMCU-- CWMCU_i2c_read error 0x%x~!!!\n", CWMCU_I2C_SENSORS_REG_START+id_check);
							}
							break;

					case CW_MAGNETIC_UNCALIBRATED:
					case CW_GYROSCOPE_UNCALIBRATED:
								/* read 12byte */
							if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 12) >= 0) {
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);
									data_event[3] = ((u32)id_check << 16) | (((u32)data[7] << 8) | (u32)data[6]);
									data_event[4] = ((u32)id_check << 16) | (((u32)data[9] << 8) | (u32)data[8]);
									data_event[5] = ((u32)id_check << 16) | (((u32)data[11] << 8) | (u32)data[10]);

									/*printk(KERN_DEBUG "--CWMCU--Normal %d data -> x = %d, y = %d, z = %d, x_bios = %d, y_bios = %d, z_bios = %d,\n"
												, id_check
												, (int16_t)((u32)data[1] << 8) | (u32)data[0]
												, (int16_t)((u32)data[3] << 8) | (u32)data[2]
												, (int16_t)((u32)data[5] << 8) | (u32)data[4]
												, (int16_t)((u32)data[7] << 8) | (u32)data[6]
												, (int16_t)((u32)data[9] << 8) | (u32)data[8]
												, (int16_t)((u32)data[11] << 8) | (u32)data[10]
												); */
									if (CWMCU_i2c_read(sensor, CW_ACCURACY, data, 1) >= 0) {
											data_event[6] = ((u32)id_check << 16) | (u32)data[0];
										}
									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_report_abs(sensor->input, CW_ABS_X1, data_event[3]);
									input_report_abs(sensor->input, CW_ABS_Y1, data_event[4]);
									input_report_abs(sensor->input, CW_ABS_Z1, data_event[5]);
									input_report_abs(sensor->input, CW_ABS_ACCURACY, data_event[6]);
									input_sync(sensor->input);

									/*reset */
									input_report_abs(sensor->input, CW_ABS_X, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_X1, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y1, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z1, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_ACCURACY, 0xFF0000);
									input_sync(sensor->input);
							}
							break;
					case CW_PEDOMETER:
								/* read 6byte */
								if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 6) >= 0) {
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);

									/*printk("--CWMCU--Normal %d data -> x = %d, y = %d, z = %d\n"
												, id_check
												, (int16_t)((u32)data[1] << 8) | (u32)data[0]
												, (int16_t)((u32)data[3] << 8) | (u32)data[2]
												, (int16_t)((u32)data[5] << 8) | (u32)data[4]
												); */
									if (data_event[2] != 0) {
										input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
										input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
										input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
										input_sync(sensor->input);

										/*reset */
										input_report_abs(sensor->input, CW_ABS_X,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_Y,  0xFF0000);
										input_report_abs(sensor->input, CW_ABS_Z,  0xFF0000);
										input_sync(sensor->input);
									}
								} else {
									printk("--CWMCU-- CW_PEDOMETER CWMCU_i2c_read error 0x%x~  update_list = 0x%x!!!\n", CWMCU_I2C_SENSORS_REG_START+id_check, sensor->update_list);
								}
								break;
					default:
								break;
					}
				}
			}
		}

#ifndef CWMCU_INTERRUPT
		sensor->debug_count++;
		/* show debug log if there are error form mcu */
		if (sensor->debug_count == 20) {
			cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT, 1);
			cwmcu_debuglog();
			cwmcu_batch_read(sensor);
			/* read gesture event */
			cwmcu_gesture_read(sensor);

			cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT, 0);
			sensor->debug_count = 0;
		}
#endif
	return 0;
}

/*==========sysfs node=====================*/

static int active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int enabled = 0;
	int sensors_id = 0;
	int error_msg = 0;
	u8 data = 0;
	u8 i = 0;
	//uint8_t delay_ms = 0;

	sscanf(buf, "%d %d\n", &sensors_id, &enabled);

	if(sensors_id == 30 || sensors_id == 0){
	    if(sensors_id == 30){
	        sensor->phone_enable = enabled;
	        printk("%s: the sensor id is 30.\n", __func__);
	    }else{
            sensor->acc_non_wake = enabled;
	    }

	    if((sensor->phone_enable || sensor->acc_non_wake) == (sensor->enabled_list & 0x01))
	        return count;

	    sensors_id = 0;
	}

	sensor->enabled_list &= ~(1<<sensors_id);
	sensor->enabled_list |= ((uint32_t)enabled)<<sensors_id;


	if (sensor->mcu_mode == CW_BOOT) {
		return count;
	}

	/* clean timeout value if sensor turn off */
	if (enabled == 0) {
		sensor->sensor_timeout[sensors_id] = 0;
		sensor->sensors_time[sensors_id] = 0;
	} else {
		do_gettimeofday(&sensor->now);
		sensor->sensors_time[sensors_id] = (sensor->now.tv_sec * 1000000LL) + sensor->now.tv_usec;
	}

	i = sensors_id / 8;
	data = (u8)(sensor->enabled_list>>(i*8));
	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);

	error_msg += CWMCU_i2c_write(sensor, CW_ENABLE_REG+i, &data, 1);

	printk( "--CWMCU-- data =%d, i = %d, sensors_id=%d enable=%d  enable_list=%d\n", data, i, sensors_id, enabled, sensor->enabled_list);

	//delay_ms = (uint8_t)sensor->report_period[sensors_id];

	//CWMCU_i2c_write(sensor, CW_DELAY_ACC+sensors_id, &delay_ms, 1);

	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);

	return count;
}

static int active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(sensor->enabled_list), "%u\n", sensor->enabled_list);
}

static ssize_t phone_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sensor->phone_enable);
}

static ssize_t phone_set(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	sensor->phone_enable= simple_strtol(buf, NULL, 10);

	return count;
}

static int interval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(unsigned int), "%d\n", CWMCU_POLL_INTERVAL);
}

static int interval_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int sensors_id = 0;
	uint8_t delay_ms = 0;
	if (sensor->mcu_mode == CW_BOOT) {
		return count;
	}

	sscanf(buf, "%d %d\n", &sensors_id , &val);

	if(sensors_id == 30)
	    sensors_id = 0;

	if (val < CWMCU_POLL_MIN)
		val = CWMCU_POLL_MIN;

	//sensor->report_period[sensors_id] = val*1000;
	sensor->report_period[sensors_id] = val;

	delay_ms = (uint8_t)val;
	cwmcu_powermode_switch(SWITCH_POWER_DELAY, 1);
	CWMCU_i2c_write(sensor, CW_DELAY_ACC+sensors_id, &delay_ms, 1);
	cwmcu_powermode_switch(SWITCH_POWER_DELAY, 0);
	printk(KERN_DEBUG "--CWMCU-- sensors_id=%d delay_ms=%d\n", sensors_id, delay_ms);
	return count;
}

static int batch_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sensors_id = 0;
	int delay_ms = 0;
	int64_t timeout = 0;
	int batch_mode = -1;
	uint8_t data[5] = {0};
	uint8_t delay = 0;
	if (sensor->mcu_mode == CW_BOOT) {
		return count;
	}

	printk(KERN_DEBUG "--CWMCU-- %s in~!!\n", __func__);

	sscanf(buf, "%d %d %d %lld\n", &sensors_id, &batch_mode, &delay_ms, &timeout);

	printk(KERN_DEBUG "--CWMCU-- sensors_id = %d, timeout = %lld\n", sensors_id, timeout);
    if(sensors_id == 30)
        sensors_id = 0;

	sensor->sensor_timeout[sensors_id] = timeout;

	sensor->report_period[sensors_id] = delay_ms;

	data[0] = (uint8_t)sensors_id;
	data[1] = (uint8_t)(timeout);
	data[2] = (uint8_t)(timeout >> 8);
	data[3] = (uint8_t)(timeout >> 16);
	data[4] = (uint8_t)(timeout >> 24);

	cwmcu_powermode_switch(SWITCH_POWER_BATCH, 1);
	CWMCU_write_i2c_block(sensor, CW_BATCHTIMEOUT, data, 5);
	delay = (uint8_t)delay_ms;
	CWMCU_i2c_write(sensor, CW_DELAY_ACC+sensors_id, &delay, 1);
	cwmcu_powermode_switch(SWITCH_POWER_BATCH, 0);

	printk(KERN_DEBUG "--CWMCU-- sensors_id = %d, current_timeout = %lld, delay_ms = %d\n", sensors_id, timeout, delay_ms);

	return count;
}

static int batch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "sensor->batched_list = %d, sensor->current_timeout = %lld\n"
					,sensor->batched_list, sensor->current_timeout);
}

static int flush_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sensors_id = 0;
	int error_msg = 0;
	uint8_t data = 0;
	if (sensor->mcu_mode == CW_BOOT) {
		return count;
	}

	printk(KERN_DEBUG "--CWMCU-- %s in\n", __func__);

	sscanf(buf, "%d\n", &sensors_id);
	if(sensors_id == 30){
        data = 0;
	}else{
	    data = (uint8_t)sensors_id;
    }

	printk(KERN_DEBUG "--CWMCU-- flush sensors_id = %d~!!\n", sensors_id);

	/* check flush event */
	if (sensor->current_timeout == 0) {
		cwmcu_send_flush(sensors_id);
	} else {
		cwmcu_powermode_switch(SWITCH_POWER_BATCH, 1);

		error_msg = CWMCU_i2c_write(sensor, CW_BATCHFLUSH, &data, 1);

		if (error_msg < 0)
			printk(KERN_DEBUG "--CWMCU-- flush i2c error~!!\n");
		cwmcu_powermode_switch(SWITCH_POWER_BATCH, 0);
	}
	return count;
}

static int flush_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "--CWMCU-- %s in\n", __func__);
	return  0;
}

static int CWMCU_Erase_Mcu_Memory(void)
{
	/* page should be 1~N */
	uint8_t send[300];
	uint8_t received[10];
	uint8_t XOR = 0;
	uint8_t page;
	uint16_t i = 0;
	page = 128;

	send[0] = 0x44;
	send[1] = 0xBB;
	if (CWMCU_write_serial((uint8_t *)send, 2) < 0) {
		return -EAGAIN;
		}
	if (CWMCU_read_serial((uint8_t *)received, 1) < 0) {
		return -EAGAIN;
		}
	if (received[0] != ACK) {
		return -EAGAIN;
		}

	send[0] = (uint8_t) ((page-1)>>8);
	send[1] = (uint8_t) (page-1);
	send[2] = send[0] ^ send[1];
	if (CWMCU_write_serial((uint8_t *)send, 3) < 0) {
		return -EAGAIN;
		}
	if (CWMCU_read_serial((uint8_t *)received, 1) < 0) {
		return -EAGAIN;
		}
	if (received[0] != ACK) {
		return -EAGAIN;
		}

	for (i = 0; i < page; i++) {
		send[2*i] = (uint8_t)(i>>8);
		send[(2*i)+1] = (uint8_t)i;
		XOR = XOR ^ send[2*i] ^ send[(2*i)+1];
	}
	send[(2*page)] = XOR;
	if (CWMCU_write_serial((uint8_t *)send, ((2*page)+1)) < 0) {
		return -EAGAIN;
		}
	return 0;

}

static int CWMCU_Write_Mcu_Memory(const char *buf)
{
	uint8_t WriteMemoryCommand[2];
	uint8_t data[300];
	uint8_t received[10];
	uint8_t XOR = 0;
	uint16_t i = 0;
	WriteMemoryCommand[0] = 0x31;
	WriteMemoryCommand[1] = 0xCE;
	if (CWMCU_write_serial((uint8_t *)WriteMemoryCommand, 2) < 0) {
		return -EAGAIN;
		}
	if (CWMCU_read_serial((uint8_t *)received, 1) < 0) {
		return -EAGAIN;
		}
	if (received[0] != ACK) {
		return -EAGAIN;
		}

	/* Set Address + Checksum */
	data[0] = (uint8_t) (sensor->addr >> 24);
	data[1] = (uint8_t) (sensor->addr >> 16);
	data[2] = (uint8_t) (sensor->addr >> 8);
	data[3] = (uint8_t) sensor->addr;
	data[4] = data[0] ^ data[1] ^ data[2] ^ data[3];
	if (CWMCU_write_serial((uint8_t *)data, 5) < 0) {
		return -EAGAIN;
		}
	if (CWMCU_read_serial((uint8_t *)received, 1) < 0) {
		return -EAGAIN;
		}
	if (received[0] != ACK) {
		return -EAGAIN;
		}

	/* send data */
	data[0] = sensor->len - 1;
	XOR = sensor->len - 1;
	for (i = 0; i < sensor->len; i++) {
		data[i+1] = buf[i];
		XOR ^= buf[i];
	}
	data[sensor->len+1] = XOR;

	if (CWMCU_write_serial((uint8_t *)data, (sensor->len + 2)) < 0) {
		return -EAGAIN;
		}
	return 0;
}

static int cwmcu_reinit(void)
{
	int id;
	int part;
	int error_msg = 0;
	uint8_t delay_ms = 0;
	u8 list;
	uint8_t data[5] = {0};

	printk( "--CWMCU-- in, sensor->enabled_list = %d*******************************%s\n", sensor->enabled_list,__func__);
	for (id = 0; id < CW_SENSORS_ID_END; id++) {
		//printk("--CWMCU--%d\n", id);
		if (sensor->enabled_list & (1<<id)) {
			part = id / 8;
			list = (u8)(sensor->enabled_list>>(part*8));

			data[0] = (uint8_t)id;
			data[1] = (uint8_t)(sensor->sensor_timeout[id]);
			data[2] = (uint8_t)(sensor->sensor_timeout[id] >> 8);
			data[3] = (uint8_t)(sensor->sensor_timeout[id] >> 16);
			data[4] = (uint8_t)(sensor->sensor_timeout[id] >> 24);
			delay_ms = (uint8_t)sensor->report_period[id];

			error_msg += CWMCU_i2c_write(sensor, CW_ENABLE_REG+part, &list, 1);
			CWMCU_write_i2c_block(sensor, CW_BATCHTIMEOUT, data, 5);
			CWMCU_i2c_write(sensor, CW_DELAY_ACC+id, &delay_ms, 1);

			printk(KERN_DEBUG "--CWMCU-- sensors_id=%d , enable_list=%d, sensor->sensor_timeout= %lld\n", id, sensor->enabled_list, sensor->sensor_timeout[id]);
		}
	}

	/*reload the acc and gyro cal value*/
	if(sensor->acc_cal[31] != 0){
		if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_ACC, sensor->acc_cal, 30) >= 0){
			printk("--CWMCU cwmcu reinit-- CW_ACCELERATION_CALIBRATOR write data\n");
		}
		if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_GYRO, sensor->gyro_cal, 30) >= 0){
			printk("--CWMCU cwmcu reinit-- CWMCU_GYRO_CALIBRATOR write data\n");
		}
	}
	return 0;
}

static int set_firmware_update_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 data[40] = {0};
	s16 data_buff[3] = {0};
	s16  _bData[8];
	s16  _bData2[9];
	s16  m_hdata[3];
	s16  m_asa[3];
	u8 test = 0x01;

	sscanf(buf, "%d %d %d\n", &sensor->cmd, &sensor->addr, &sensor->len);
	printk(KERN_DEBUG "CWMCU cmd=%d addr=%d len=%d\n", sensor->cmd, sensor->addr, sensor->len);

	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);

	switch (sensor->cmd) {
	case CHANGE_TO_BOOTLOADER_MODE:
			printk("CWMCU CHANGE_TO_BOOTLOADER_MODE\n");
			/* boot enable : put high , reset: put low */
#if 1
			if(gpio_is_valid(sensor->board_data->reset_gpio))
			    gpio_request(sensor->board_data->reset_gpio,"MCU_RESET");
			gpio_direction_output(sensor->board_data->reset_gpio, 1);
			gpio_set_value(sensor->board_data->boot_gpio, 1);
			//msleep(500);
			usleep_range(500, 1000);
			gpio_set_value(sensor->board_data->reset_gpio, 1);
			//msleep(500);
			usleep_range(500, 1000);
			gpio_set_value(sensor->board_data->reset_gpio, 0);
			//msleep(500);
			usleep_range(10000, 11000);
			gpio_set_value(sensor->board_data->reset_gpio, 1);
			//msleep(1000);
			msleep(20);
#else

			gpio_direction_output(GPIO_CW_MCU_RESET, 1);
			gpio_set_value(GPIO_CW_MCU_BOOT, 1);
			msleep(500);
			gpio_set_value(GPIO_CW_MCU_RESET, 1);
			msleep(500);
			gpio_set_value(GPIO_CW_MCU_RESET, 0);
			msleep(500);
			gpio_set_value(GPIO_CW_MCU_RESET, 1);
			msleep(1000);

#endif
			sensor->mcu_mode = CW_BOOT;
			sensor->mcu_slave_addr = sensor->client->addr;
			sensor->client->addr = 0x72 >> 1;
			sensor->is_update = 1;
			break;

	case CHANGE_TO_NORMAL_MODE:
			printk("CWMCU CHANGE_TO_NORMAL_MODE\n");

			sensor->firmwave_update_status = 1;
			sensor->client->addr = 0x74 >> 1;

			/* boot low  reset high */
			gpio_set_value(sensor->board_data->boot_gpio, 0);
			gpio_set_value(sensor->board_data->reset_gpio, 1);
			//msleep(500);
			usleep_range(500, 1000);
			gpio_set_value(sensor->board_data->reset_gpio, 0);
			//msleep(500);
			usleep_range(10000, 11000);
			gpio_set_value(sensor->board_data->reset_gpio, 1);
			//msleep(1000);
			msleep(20);
			gpio_direction_input(sensor->board_data->reset_gpio);

			sensor->mcu_mode = CW_NORMAL;
			sensor->is_update = 0;

			break;

	case ERASE_MCU_MEMORY:
			printk(KERN_DEBUG "CWMCU ERASE_MCU_MEMORY\n");
			sensor->firmwave_update_status = 1;
			sensor->firmwave_update_status = CWMCU_Erase_Mcu_Memory();
			break;

	case WRITE_MCU_MEMORY:
			printk(KERN_DEBUG "CWMCU Set Addr=%d\tlen=%d\n", sensor->addr, sensor->len);
			break;

	case MCU_GO:
			printk(KERN_DEBUG "CWMCU MCU_GO\n");
			break;

	case CHECK_FIRMWAVE_VERSION:
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_FWVERSION, data, 1) >= 0) {
				printk(KERN_DEBUG "CHECK_FIRMWAVE_VERSION %d\n", (int)data[0]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;

	case CHECK_ACC_DATA:
			printk(KERN_DEBUG "CWMCU CHECK_ACC_DATA\n");
			if (CWMCU_i2c_read_upf(sensor, CW_READ_ACCELERATION, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk("x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;

	case CHECK_MAG_DATA:
			printk(KERN_DEBUG "CWMCU CHECK_MAG_DATA\n");
		   //	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_READ_MAGNETIC, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk(KERN_DEBUG "x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;

	case CHECK_GYRO_DATA:
			printk(KERN_DEBUG "CWMCU CHECK_GYRO_DATA\n");
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_READ_GYRO, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk("x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;
	case CHECK_GAME_R_DATA:
			printk(KERN_DEBUG "CWMCU CW_GAME_ROTATION_VECTOR\n");
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_READ_GAME_ROTATION_VECTOR, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk(KERN_DEBUG "x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;
	case CHECK_GEMO_R_DATA:
			printk(KERN_DEBUG "CWMCU CW_GEOMAGNETIC_ROTATION_VECTOR\n");
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_READ_GEOMAGNETIC_ROTATION_VECTOR, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk(KERN_DEBUG "x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			break;
	case CHECK_UNCALMAG_DATA:
			printk(KERN_DEBUG "CWMCU CW_MAGNETIC_UNCALIBRATED\n");
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_READ_MAGNETIC_UNCALIBRATED, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk(KERN_DEBUG "x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;
	case CHECK_UNCALGYRO_DATA:
			printk(KERN_DEBUG "CWMCU CW_GYROSCOPE_UNCALIBRATED\n");
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			if (CWMCU_i2c_read_upf(sensor, CW_READ_GYROSCOPE_UNCALIBRATED, data, 6) >= 0) {
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

				printk(KERN_DEBUG "x = %d, y = %d, z = %d\n",
					data_buff[0], data_buff[1], data_buff[2]);
			}
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;
#if 0
	case CHECK_HWID:
			printk(KERN_DEBUG "CWMCU CHECK_HWID\n");
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
			cwmcu_set_hwinfo(CW_ACCELERATION);
			cwmcu_set_hwinfo(CW_MAGNETIC);
			cwmcu_set_hwinfo(CW_GYRO);
			//cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			break;
#endif
	case CHECK_INFO:
			printk(KERN_DEBUG "CWMCU check info\n");
			cwmcu_debuglog();
			break;

	case CHECK_MAG1_INFO:
			if (CWMCU_i2c_read_upf(sensor, CW_MAGINFO, data, 30) >= 0) {
				memcpy(_bData,&data[0],sizeof(_bData));
				memcpy(m_hdata,&data[16],sizeof(m_hdata));
				memcpy(m_asa,&data[22],sizeof(m_asa));
			printk("Disp_AKMDEBUG_1:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \n",
                                _bData[0],
                                (s16)(((uint16_t)_bData[2])<<8|_bData[1]),
                                (s16)(((uint16_t)_bData[4])<<8|_bData[3]),
                                (s16)(((uint16_t)_bData[6])<<8|_bData[5]),
                                _bData[7],
                                m_hdata[0], m_hdata[1], m_hdata[2],
                                m_asa[0], m_asa[1], m_asa[2]);
			}
			break;

	case CHECK_MAG2_INFO:
			if (CWMCU_i2c_read_upf(sensor, CW_MAGINFO, data, 30) >= 0) {
				memcpy(_bData2,&data[0],sizeof(_bData));
				memcpy(m_hdata,&data[18],sizeof(m_hdata));
				memcpy(m_asa,&data[24],sizeof(m_asa));
			printk("Disp_AKMDEBUG_2:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \n",
                                _bData2[0],
                                (s16)(((uint16_t)_bData2[2])<<8|_bData2[1]),
                                (s16)(((uint16_t)_bData2[4])<<8|_bData2[3]),
                                (s16)(((uint16_t)_bData2[6])<<8|_bData2[5]),
                                _bData2[8],
                                m_hdata[0], m_hdata[1], m_hdata[2],
                                m_asa[0], m_asa[1], m_asa[2]);
			}
			break;

	case TEST:
			printk(KERN_DEBUG "CWMCU watch dog timeout\n");
			test = 0x02;
			CWMCU_i2c_write_upf(sensor, CW_WATCHDOG, &test, 1);
			break;

	case TEST2:
			printk(KERN_DEBUG "CWMCU TEST2\n");
			cwmcu_reinit();
			break;

	default:
			break;
	}
	return count;
}

static int set_firmware_update_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_DEBUG "CWMCU Write Data\n");
	printk(buf);
	sensor->firmwave_update_status = 1;
	sensor->firmwave_update_status = CWMCU_Write_Mcu_Memory(buf);
	return count;
}

static int get_firmware_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "CWMCU firmwave_update_status = %d\n", sensor->firmwave_update_status);
	return snprintf(buf, sizeof(sensor->firmwave_update_status), "%d\n", sensor->firmwave_update_status);
}

static int set_firmware_update_i2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int intsize = sizeof(int);
	memcpy(&sensor->cw_i2c_rw, buf, intsize);
	memcpy(&sensor->cw_i2c_len, &buf[4], intsize);
	memcpy(sensor->cw_i2c_data, &buf[8], sensor->cw_i2c_len);

	return count;
}

static int get_firmware_update_i2(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;
	if (sensor->cw_i2c_rw) {
		//printk(" -----Cywee -----if sensor-> cw_i2c_rw is  TRUE\n");
		if (CWMCU_write_serial(sensor->cw_i2c_data, sensor->cw_i2c_len) < 0) {
		printk(" -----Cywee -----if write_serial() < 0 is TRUE\n");
			status = -1;
		}
		memcpy(buf, &status, sizeof(int));
		return 4;
	} else {
		if (CWMCU_read_serial(sensor->cw_i2c_data, sensor->cw_i2c_len) < 0) {
			//printk("-----Cywee -----if read_serial() < 0 is TRUE\n");
			status = -1;
			memcpy(buf, &status, sizeof(int));
			return 4;
		}
		memcpy(buf, &status, sizeof(int));
		memcpy(&buf[4], sensor->cw_i2c_data, sensor->cw_i2c_len);
		return 4+sensor->cw_i2c_len;
	}
	return  0;
}

static int mcu_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(sensor->mcu_mode), "%d\n", sensor->mcu_mode);
}

static int mcu_model_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode = 0;
	sscanf(buf, "%d\n", &mode);
	CWMCU_Set_Mcu_Mode(mode);
	return count;
}

/* get calibrator data */
static int get_calibrator_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;
	uint8_t data[33] = {0};

	data[0] = sensor->cal_cmd;
	data[1] = sensor->cal_type;
	data[2] = sensor->cal_id;

	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);

	switch (sensor->cal_cmd) {

	case CWMCU_CALIBRATOR_STATUS:
			printk(KERN_DEBUG "--CWMCU-- CWMCU_CALIBRATOR_STATUS\n");
			if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_STATUS, &data[3], 1) >= 0) {
				printk(KERN_DEBUG "--CWMCU-- calibrator status = %d\n", data[3]);
				return sprintf(buf, "%d\n",data[3]);
			} else {
				printk(KERN_DEBUG "--CWMCU-- fuck i2c calibrator status = %d\n", data[3]);
				return sprintf(buf, "fuck: %d\n",data[3]);
			}
			break;
	case CWMCU_ACCELERATION_CALIBRATOR:
			printk(KERN_DEBUG "--CWMCU-- CWMCU_ACCELERATION_CALIBRATOR read data\n");
			if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_BIAS_ACC, &data[3], 30) <= 0) {
				printk(KERN_ERR "--CWMCU-- i2c calibrator read fail!!! [ACC]\n");
			}
			for(i = 0; i < 30; i++){
				sensor->acc_cal[i] = data[i+3];
			}
			sensor->acc_cal[31] = 1;
			break;
	case CWMCU_MAGNETIC_CALIBRATOR:
			printk(KERN_DEBUG "--CWMCU-- CWMCU_MAGNETIC_CALIBRATOR read data\n");
			if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_BIAS_MAG, &data[3], 30) <= 0) {
				printk(KERN_ERR "--CWMCU-- i2c calibrator read fail!!! [MAG]\n");
			}
			break;
	case CWMCU_GYRO_CALIBRATOR:
			printk(KERN_DEBUG "--CWMCU-- CWMCU_GYRO_CALIBRATOR read data\n");
			if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_BIAS_GYRO, &data[3], 30) <= 0) {
				printk(KERN_ERR "--CWMCU-- i2c calibrator read fail!!! [GYRO]\n");
			}
			for(i = 0; i < 30; i++){
				sensor->gyro_cal[i] = data[i+3];
			}
			break;
	case CWMCU_LIGHT_CALIBRATOR:
			printk(KERN_DEBUG "--CWMCU-- CWMCU_LIGHT_CALIBRATOR read data\n");
			if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_BIAS_LIGHT, &data[3], 30) <= 0) {
				printk(KERN_ERR "--CWMCU-- i2c calibrator read fail!!! [LIGHT]\n");
			}
			break;
	case CWMCU_PROXIMITY_CALIBRATOR:
			printk(KERN_DEBUG "--CWMCU-- CWMCU_PROXIMITY_CALIBRATOR read data\n");
			if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_BIAS_PROXIMITY, &data[3], 30) <= 0) {
				printk(KERN_ERR "--CWMCU-- i2c calibrator read fail!!! [PROX]\n");
			}
			break;
	}

	for (i = 0; i < 33; i++) {
		printk(KERN_DEBUG "--CWMCU-- castor read data[%d] = %u\n", i, data[i]);
	}

	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 0);

	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			data[0], data[1], data[2],
			data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12],
			data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22],
			data[23], data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31], data[32]);

}

static int set_calibrator_data(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int i = 0;
	uint8_t data[33] = {0};
	int temp[33] = {0};
	char source[512];
	char *pch;
	int buf_count=0;

	char *myBuf= source;

	strcpy(source,buf);

	printk( "--CWMCU-- source = %s | count:%d\n", source, count);

	while ((pch = strsep(&myBuf, ", ")) != NULL) {
		buf_count++;
	}

	printk( "--CWMCU-- buf = %s | bufcount:%d\n", buf, buf_count);

	if (buf_count == 3) {
		sscanf(buf, "%d %d %d",&temp[0], &temp[1], &temp[2]);
		sensor->cal_cmd = (uint8_t)temp[0];
		sensor->cal_type = (uint8_t)temp[1];
		sensor->cal_id = (uint8_t)temp[2];
		printk(KERN_DEBUG "--CWMCU-- cmd:%d type:%d id:%d\n", sensor->cal_cmd, sensor->cal_type, sensor->cal_id);
		if (sensor->cal_cmd == CWMCU_CALIBRATOR_INFO) {
			cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);
			printk(KERN_DEBUG "--CWMCU-- set calibrator info\n");
			CWMCU_i2c_write(sensor, CW_CALIBRATOR_TYPE, &sensor->cal_type, 1);
			CWMCU_i2c_write(sensor, CW_CALIBRATOR_SENSORLIST, &sensor->cal_id, 1);
		} else {
			printk(KERN_DEBUG "--CWMCU-- set command\n");
			return count;
		}
	} else if (buf_count >= 33) {
		sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
			&temp[0], &temp[1], &temp[2],
			&temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8], &temp[9], &temp[10], &temp[11], &temp[12],
			&temp[13], &temp[14], &temp[15], &temp[16], &temp[17], &temp[18], &temp[19], &temp[20], &temp[21], &temp[22],
			&temp[23], &temp[24], &temp[25], &temp[26], &temp[27], &temp[28], &temp[29], &temp[30], &temp[31], &temp[32]);

		for (i = 0; i < 33; i++) {
			data[i] = (uint8_t)temp[i];
		}

		sensor->cal_cmd = data[0];
		sensor->cal_type = data[1];
		sensor->cal_id = data[2];

		printk(KERN_DEBUG "--CWMCU-- set command=%d , type=%d, sensorid=%d\n", sensor->cal_cmd, sensor->cal_type, sensor->cal_id);

		switch (sensor->cal_cmd) {
		case CWMCU_ACCELERATION_CALIBRATOR:
				for(i = 0; i < 30; i++){
					sensor->acc_cal[i] = data[i+3];
				}
				cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);
				if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_ACC, &data[3], 30) >= 0) {
					printk("--CWMCU-- CW_ACCELERATION_CALIBRATOR write data\n");
				}
				sensor->acc_cal[31] = 1;
				break;
		case CWMCU_MAGNETIC_CALIBRATOR:
				cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);
				if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_MAG, &data[3], 30) >= 0) {
					printk(KERN_DEBUG "--CWMCU-- CWMCU_MAGNETIC_CALIBRATOR write data\n");
				}
				break;
		case CWMCU_GYRO_CALIBRATOR:
				for(i = 0; i < 30; i++){
					sensor->gyro_cal[i] = data[i+3];
				}
				cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);
				if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_GYRO, &data[3], 30) >= 0) {
					printk("--CWMCU-- CWMCU_GYRO_CALIBRATOR write data\n");
				}
				break;
		case CWMCU_LIGHT_CALIBRATOR:
				printk(KERN_DEBUG "--CWMCU-- CWMCU_GYRO_CALIBRATOR write data\n");
				if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_LIGHT, &data[3], 30) >= 0) {
					for (i = 0; i < 33; i++) {
						printk(KERN_DEBUG "--CWMCU-- data[%d] = %d\n", i, data[i]);
					}
				}
				break;
		case CWMCU_PROXIMITY_CALIBRATOR:
				printk(KERN_DEBUG "--CWMCU-- CWMCU_GYRO_CALIBRATOR write data\n");
				if (CWMCU_write_i2c_block(sensor, CW_CALIBRATOR_BIAS_PROXIMITY, &data[3], 30) >= 0) {
					for (i = 0; i < 33; i++) {
						printk(KERN_DEBUG "--CWMCU-- data[%d] = %d\n", i, data[i]);
					}
				}
				break;
		}
	} else {
		printk(KERN_DEBUG "--CWMCU-- input parameter incorrect !!! | %d\n",count);
		return count;
	}

	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 0);

	return count;
}

static int version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t data = 0;

	printk("--CWMCU-- %s\n", __func__);

	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
	if (CWMCU_i2c_read(sensor, CW_FWVERSION, &data, 1) >= 0) {
		printk(KERN_DEBUG "CHECK_FIRMWAVE_VERSION : %d\n", data);
	}
	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);

	return snprintf(buf, 5, "%d\n", data);
}

static DEVICE_ATTR(enable, 0664, active_show, active_set);
static DEVICE_ATTR(sensor_phone_calling, 0664, phone_show, phone_set);
static DEVICE_ATTR(delay_ms, 0664, interval_show, interval_set);
/* static DEVICE_ATTR(poll, 0666, poll_show, NULL); */
static DEVICE_ATTR(batch, 0664, batch_show, batch_set);
static DEVICE_ATTR(flush, 0664, flush_show, flush_set);
static DEVICE_ATTR(mcu_mode, 0664, mcu_mode_show, mcu_model_set);

static DEVICE_ATTR(firmware_update_i2c, 0664, get_firmware_update_i2, set_firmware_update_i2);
static DEVICE_ATTR(firmware_update_cmd, 0664, NULL, set_firmware_update_cmd);
static DEVICE_ATTR(firmware_update_data, 0664, NULL, set_firmware_update_data);
static DEVICE_ATTR(firmware_update_status, 0664, get_firmware_update_status, NULL);

static DEVICE_ATTR(calibrator_cmd, 0664, get_calibrator_data, set_calibrator_data);
static DEVICE_ATTR(version, 0664, version_show, NULL);

static struct attribute *sysfs_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_sensor_phone_calling.attr,
	&dev_attr_delay_ms.attr,
	/* &dev_attr_poll.attr, */
	&dev_attr_batch.attr,
	&dev_attr_flush.attr,

	&dev_attr_mcu_mode.attr,
	&dev_attr_firmware_update_i2c.attr,
	&dev_attr_firmware_update_cmd.attr,
	&dev_attr_firmware_update_data.attr,
	&dev_attr_firmware_update_status.attr,

	&dev_attr_calibrator_cmd.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group sysfs_attribute_group = {
	.attrs = sysfs_attributes
};

/*=======input device==========*/

static void  CWMCU_init_input_device(struct CWMCU_data *sensor, struct input_dev *idev)
{
	idev->name = CWMCU_I2C_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &sensor->client->dev;
	idev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_ABS);

	set_bit(EV_ABS, idev->evbit);
	input_set_abs_params(idev, CW_ABS_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_X1, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Y1, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Z1, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_TIMEDIFF, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_ACCURACY, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, REL_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, REL_Y, -DPS_MAX, DPS_MAX, 0, 0);
}

#ifndef CWMCU_INTERRUPT
/*=======polling device=========*/
static void CWMCU_poll(struct input_polled_dev *dev)
{
printk("%s:\n", __func__);
	CWMCU_read(dev->private);
}

static int CWMCU_open(struct CWMCU_data *sensor)
{
	int error;
	error = pm_runtime_get_sync(&sensor->client->dev);
	if (error && error != -ENOSYS)
		return error;
	return 0;
}

static void CWMCU_close(struct CWMCU_data *sensor)
{
	pm_runtime_put_sync(&sensor->client->dev);
}

static void CWMCU_poll_open(struct input_polled_dev *ipoll_dev)
{
	struct CWMCU_data *sensor = ipoll_dev->private;
	CWMCU_open(sensor);
}

static void CWMCU_poll_close(struct input_polled_dev *ipoll_dev)
{
	struct CWMCU_data *sensor = ipoll_dev->private;
	CWMCU_close(sensor);
}
#endif
#ifndef CWMCU_INTERRUPT
static int CWMCU_register_polled_device(struct CWMCU_data *sensor)
{
	int error = -1;
	struct input_polled_dev *ipoll_dev;

	/* poll device */
	ipoll_dev = input_allocate_polled_device();
	if (!ipoll_dev)
		return -ENOMEM;

	ipoll_dev->private = sensor;
	ipoll_dev->open = CWMCU_poll_open;
	ipoll_dev->close = CWMCU_poll_close;
	ipoll_dev->poll = CWMCU_poll;
	ipoll_dev->poll_interval = CWMCU_POLL_INTERVAL;
	ipoll_dev->poll_interval_min = CWMCU_POLL_MIN;
	ipoll_dev->poll_interval_max = CWMCU_POLL_MAX;

	CWMCU_init_input_device(sensor, ipoll_dev->input);

	error = input_register_polled_device(ipoll_dev);
	if (error) {
		input_free_polled_device(ipoll_dev);
		return error;
	}

	sensor->input_polled = ipoll_dev;
	sensor->input = ipoll_dev->input;

	return 0;
}
#else
static int CWMCU_register_polled_device(struct CWMCU_data *sensor)
{
	int error;

	sensor->input = input_allocate_device();
	sensor->input_polled = NULL;

	if(!sensor->input) {
		printk("sensor hub allocate input device failed!\n");
		return -ENOMEM;
	}
	
	CWMCU_init_input_device(sensor, sensor->input);

	error = input_register_device(sensor->input);
	if(error){
		printk("sensor hub register input device failed!\n");	
		goto err_register_input;
	}

	return 0;
	
err_register_input:
	input_free_device(sensor->input);
	return error;
}
#endif

static int CWMCU_suspend(struct device *dev)
{
	if(!sensor->phone_enable)
		disable_irq(sensor->client->irq);
	return 0;
}

static int CWMCU_resume(struct device *dev)
{
	if(!sensor->phone_enable)
		enable_irq(sensor->client->irq);
	return 0;
}

#ifdef CWMCU_INTERRUPT
static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
	/* printk(KERN_DEBUG "--CWMCU--%s in\n", __func__); */
	if (sensor->mcu_mode == CW_BOOT) {
		printk("--CWMCU--%s sensor->mcu_mode = %d\n", __func__, sensor->mcu_mode);
		return IRQ_HANDLED;
	}
	schedule_work(&sensor->work);

	return IRQ_HANDLED;
}

static void cwmcu_work_report(struct work_struct *work)
{
	uint8_t temp[6] = {0};
	u8 test = 0x01;
	/*
	struct CWMCU_data *sensor =
	    container_of(work, struct CWMCU_data, work);
	*/
	if(sensor->phone_enable){
		wake_lock_timeout(&sensor->sensorhub_wakelock, HZ);
	}

	if (sensor->mcu_mode == CW_BOOT) {
		printk(KERN_DEBUG "--CWMCU--%s sensor->mcu_mode = %d\n", __func__, sensor->mcu_mode);
		return;
	}

	cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT, 1);

	/* check mcu interrupt status */
	if (CWMCU_i2c_read(sensor, CW_INTERRUPT_STATUS, temp, 6) >= 0) {
		sensor->interrupt_status = (u32)temp[1] << 8 | (u32)temp[0];
		sensor->update_list = (u32)temp[5] << 24 | (u32)temp[4] << 16 | (u32)temp[3] << 8 | (u32)temp[2];
		//printk(KERN_DEBUG "--CWMCU-- sensor->interrupt_status~ = %d\n", sensor->interrupt_status);
	} else {
		printk(KERN_DEBUG "--CWMCU-- check interrupt_status failed~!!\n");
	}

	if (sensor->interrupt_status & (1<<INTERRUPT_INIT)) {
		CWMCU_i2c_write(sensor, CW_WATCHDOG, &test, 1);
		cwmcu_reinit();
	}

	/* check interrupt until status is clean */
	if (sensor->interrupt_status & (1<<INTERRUPT_GESTURE)
		|| sensor->interrupt_status & (1<<INTERRUPT_BATCHTIMEOUT)
		|| sensor->interrupt_status & (1<<INTERRUPT_BATCHFULL)) {
		/* send home key to wake up system */
		/*
		input_report_key(sensor->input, 102, 1);
		input_sync(sensor->input);
		input_report_key(sensor->input, 102, 0);
		input_sync(sensor->input);
		*/
		/*
		input_report_key(sensor->input, 88, 1);
		input_sync(sensor->input);
		input_report_key(sensor->input, 88, 0);
		input_sync(sensor->input);
		*/
	}
	/* read sensor data of batch mode*/
	if (sensor->interrupt_status & (1<<INTERRUPT_BATCHTIMEOUT) || sensor->interrupt_status & (1<<INTERRUPT_BATCHFULL)) {
		cwmcu_batch_read(sensor);
	}

	/* read gesture event */
	if (sensor->interrupt_status & (1<<INTERRUPT_GESTURE)) {
		cwmcu_gesture_read(sensor);
	}
	/* error log */
	if (sensor->interrupt_status & (1<<INTERRUPT_ERROR)) {
		cwmcu_debuglog();
	}
	/* read sensor data of normal mode*/
	if (sensor->interrupt_status & (1<<INTERRUPT_DATAREADY)) {
		CWMCU_read(sensor);
	}
	cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT, 0);
}
#endif

static int parse_dt(struct device *dev, struct CWMCU_platform_data *bdata)
{
	int retval;
	u32 value;
	const char *name;
//	struct pinctrl *pinctrl;
	struct device_node *np = dev->of_node;
#if 0
	pinctrl = devm_pinctrl_get_select(dev, "sensor-hub-irq");
	if(IS_ERR(pinctrl))
		pr_err("failed to get sensor hub irq pinctrl - ON");
#endif
	
	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"mx-hub,irq-gpio", 0, NULL);
	
	retval = of_property_read_u32(np, "mx-hub,irq-flags", &value);
	if(retval < 0)
		return retval;
	else
		bdata->irq_flags = value;

	retval = of_property_read_string(np, "mx-hub,pwr28-reg-name", &name);
	if(retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "mx-hub,pwr18-reg-name", &name);
	if(retval == -EINVAL)
		bdata->pwr18_reg_name = NULL;
	else
		bdata->pwr18_reg_name = name;

	if(of_property_read_bool(np, "mx-hub,wkup-gpio")) {
		bdata->wake_up_gpio = of_get_named_gpio_flags(np,
				"mx-hub,wkup-gpio", 0, NULL);
	}else {
		bdata->wake_up_gpio = -1;
	}

	if(of_property_read_bool(np, "mx-hub,reset-gpio")) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"mx-hub,reset-gpio", 0, NULL);
	}else {
		bdata->reset_gpio = -1;
	}

	if(of_property_read_bool(np, "mx-hub,boot-gpio")) {
		bdata->boot_gpio = of_get_named_gpio_flags(np,
				"mx-hub,boot-gpio", 0, NULL);
	}else {
		bdata->boot_gpio = -1;
	}

	if(of_property_read_bool(np, "mx-hub,sleep-gpio")) {
		bdata->sleep_mcu_gpio = of_get_named_gpio_flags(np,
				"mx-hub,sleep-gpio", 0, NULL);
	}else {
		bdata->sleep_mcu_gpio = -1;
	}

	if(of_property_read_bool(np, "mx-hub,busy-gpio")) {
		bdata->mcu_busy_gpio = of_get_named_gpio_flags(np,
				"mx-hub,busy-gpio", 0, NULL);
	}else {
		bdata->mcu_busy_gpio = -1;
	}

	return 0;

}

static int mx_hub_gpio_setup(int gpio, unsigned char *buf, bool config, int dir, int state)
{
	int retval = 0;

	if(config) {
		retval = gpio_request(gpio, buf);
		if(retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
				__func__, gpio, retval);
			return retval;
			} 
		if(dir == 0)
			retval = gpio_direction_input(gpio);
		else 
			retval = gpio_direction_output(gpio, state);
		if(retval) {
			pr_err("%s: Failed to set gpio %d direction",
				__func__, gpio);
			return retval;
			}
	}else {
				gpio_free(gpio);
	}
	
	return 0;

}

static int mx_hub_set_gpio(struct CWMCU_platform_data *bdata )
{
	int retval;

	retval = mx_hub_gpio_setup(bdata->irq_gpio, "MCU_IRQ", 1, 0, 0);
	if (retval < 0) {
		printk("%s: Failed to configure attention GPIO	1 %d\n",
				__func__ , bdata->irq_gpio);
		goto err_gpio_irq;
	}

	if(bdata->wake_up_gpio >=0) {
		retval = mx_hub_gpio_setup(bdata->wake_up_gpio, "MCU_WAKEUP", 1, 1, 0);
		if (retval < 0) {
			printk("%s: Failed to configure attention GPIO	2 %d\n",
					__func__ , bdata->wake_up_gpio);
			goto err_gpio_wakeup;
		}
	}

	if(bdata->reset_gpio >=0) {
		retval = mx_hub_gpio_setup(bdata->reset_gpio, "MCU_RESET", 1, 1, 1);
		if (retval < 0) {
			printk("%s: Failed to configure attention GPIO 	3 %d\n",
					__func__ , bdata->reset_gpio);
			goto err_gpio_reset;
		}
	}

	if(bdata->boot_gpio >=0) {
		retval = mx_hub_gpio_setup(bdata->boot_gpio, "MCU_BOOT", 1, 1, 1);
		if (retval < 0) {
			printk("%s: Failed to configure attention GPIO  4 %d\n",
					__func__ , bdata->boot_gpio);
			goto err_gpio_boot;
		}
	}
	
		printk("boot_gpio value is %d\n", gpio_get_value(bdata->boot_gpio));
	//msleep(100);
	usleep_range(500, 1000);
	if(bdata->boot_gpio >= 0) 
		gpio_set_value(bdata->boot_gpio, 0);

	
	//msleep(100);
	usleep_range(10000, 15000);
	if(bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, 0);
		//msleep(10);
		usleep_range(10000, 11000);
		gpio_set_value(bdata->reset_gpio, 1);
		msleep(20);
		//usleep_range(500,1000);
		gpio_direction_input(bdata->reset_gpio);
	}



err_gpio_boot:
	if(bdata->reset_gpio >= 0)
		gpio_free(bdata->reset_gpio);

err_gpio_reset:
	if(bdata->wake_up_gpio >= 0)
		gpio_free(bdata->wake_up_gpio);

err_gpio_wakeup:
	gpio_free(bdata->irq_gpio);

err_gpio_irq:
	return retval;
}

//static void mx_hub_reinit(struct CWMCU_data *hub)
#if 0
static void mx_hub_reinit(void)
{
	int i, j, retval = 0;
	u8 data = 0;
	u8 delay_ms = 0;
	int error_msg = 0;

	if(iic_error > MAX_IIC_ERROR)
	{
		iic_error = 0;

		retval = mx_hub_set_gpio(sensor->board_data);
	        if(retval < 0) {
		printk(" Failed to set up GPIO's \n");
		};
	
		for (i = 0; i < CW_SENSORS_ID_END; i++) {
			if(sensor->enabled_list & (1<<i))
			{
				j = i / 8;
				data = (u8)(sensor->enabled_list>>(i*8));
				cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);
				error_msg += CWMCU_i2c_write(sensor, CW_ENABLE_REG+j, &data, 1);
				printk("--CWMCU-- %s:data =%d, i = %d, sensors_id=%d  enable_list=%d\n",__func__, data, j, i, sensor->enabled_list);
				delay_ms = (uint8_t)sensor->report_period[i];
				CWMCU_i2c_write(sensor, CW_DELAY_ACC+i, &delay_ms, 1);
				cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);
			}
				
		}
	}
}
#endif

static int CWMCU_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0, retval;
	int i = 0;
	struct regulator *regulator = NULL;

	printk("--CWMCU-- %s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "--CWMCU-- i2c_check_functionality error\n");
		return -EIO;
	}
	
	sensor = kzalloc(sizeof(struct CWMCU_data), GFP_KERNEL);
	if (!sensor) {
		printk(KERN_DEBUG "--CWMCU-- kzalloc error\n");
		return -ENOMEM;
	}
	
	if(client->dev.of_node) {
		sensor->board_data = devm_kzalloc(&client->dev, sizeof(struct CWMCU_platform_data), GFP_KERNEL);
		if(!sensor->board_data) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for board data\n",
					__func__);
			goto exit_free_mem;
		}
	
		parse_dt(&client->dev, sensor->board_data);
	};
	mutex_init(&sensor->iolock);
	
	/* power on */
	if((sensor->board_data->pwr18_reg_name != NULL) && (sensor->board_data->pwr18_reg_name != 0)) {
		regulator = regulator_get(&client->dev, sensor->board_data->pwr18_reg_name);
		if(!IS_ERR(regulator)) {
        	error = regulator_enable(regulator);
        	dev_info(&client->dev, "regulator_enable: %d\n", error);
    	}else{
    		dev_err(&client->dev, "failed to get regulator (0x%p).\n",regulator);
		}
	}
		
	if((sensor->board_data->pwr_reg_name != NULL) && (sensor->board_data->pwr_reg_name != 0)) {
		sensor->sensor_pwr = regulator_get(&client->dev, sensor->board_data->pwr_reg_name);
		if(!IS_ERR(sensor->sensor_pwr)) {
        	error = regulator_enable(sensor->sensor_pwr);
        	dev_info(&client->dev, "regulator_enable: %d\n", error);
    	}else{
    		dev_err(&client->dev, "failed to get regulator (0x%p).\n",sensor->sensor_pwr);
		}
	}
	usleep_range(2000, 3000);
	/* mcu reset */
#if 0 
	gpio_direction_output(GPIO_CW_MCU_RESET, 1);
	gpio_direction_output(GPIO_CW_MCU_BOOT, 1);
	msleep(100);
	gpio_set_value(GPIO_CW_MCU_BOOT, 0);
	gpio_set_value(GPIO_CW_MCU_RESET, 1);
	msleep(100);
	gpio_set_value(GPIO_CW_MCU_RESET, 0);
	msleep(100);
	gpio_set_value(GPIO_CW_MCU_RESET, 1);
	msleep(1000);

	gpio_direction_input(GPIO_CW_MCU_RESET);
#else
	retval = mx_hub_set_gpio(sensor->board_data);	
	if(retval < 0) {
		printk(" Failed to set up GPIO's \n");
	};

#endif

	sensor->client = client;
	i2c_set_clientdata(client, sensor);

//#ifndef CWMCU_INTERRUPT
	error = CWMCU_register_polled_device(sensor);
	if (error) {
		printk(KERN_DEBUG "--CWMCU-- CWMCU_register_polled_device error\n");
		goto err_free_lock;
	}
//#endif
#ifdef CWMCU_INTERRUPT
	sensor->client->irq = gpio_to_irq(sensor->board_data->irq_gpio);
	printk("--CWMCU--sensor->client->irq  =%d~!!\n", sensor->client->irq);
	
	if (sensor->client->irq > 0) {
		INIT_WORK(&sensor->work, cwmcu_work_report);
		error = request_threaded_irq(sensor->client->irq, NULL,
			CWMCU_interrupt_thread,IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							   "cwmcu", sensor);
		if (error < 0) {
			pr_err("request irq %d failed\n", sensor->client->irq);
			goto exit_destroy_mutex;
		}
		disable_irq(sensor->client->irq);
	}
#endif
	wake_lock_init(&sensor->sensorhub_wakelock, WAKE_LOCK_SUSPEND,"sensorhub_wake_lock");
	error = sysfs_create_group(&sensor->input->dev.kobj,
					&sysfs_attribute_group);
	if (error)
		goto exit_free_input;

	for (i = 0; i < CW_SENSORS_ID_END; i++) {
		sensor->sensors_time[i] = 0;
		sensor->report_period[i] = 200000;
		sensor->time_diff[i] = 0;
	}

	for (i = 0; i < 32; i++){
		sensor->acc_cal[i] = 0;
		sensor->gyro_cal[i] = 0;
	}
	sensor->mcu_mode = CW_NORMAL;
	sensor->current_timeout = 0;
	sensor->timeout_count = 0;
	sensor->is_update = 0;

	pm_runtime_enable(&client->dev);

	error = mfd_add_devices(&client->dev, -1, mx7_hub_devs, ARRAY_SIZE(mx7_hub_devs),NULL,sensor->client->irq, NULL);
	if(error < 0)
		printk("mx7 sensor hub add devices failed\n");
	enable_irq(sensor->client->irq);

	if(meizu_sysfslink_register(&sensor->input->dev, LINK_KOBJ_NAME) < 0)
		printk("sensor hub create sys link failed\n");

	printk(KERN_DEBUG "--CWMCU-- CWMCU_i2c_probe success!\n");

	return 0;

exit_free_input:
	input_free_device(sensor->input);
	wake_lock_destroy(&sensor->sensorhub_wakelock);
exit_destroy_mutex:
	free_irq(sensor->client->irq, sensor);
err_free_lock:	
	mutex_destroy(&sensor->iolock);
exit_free_mem:	
	kfree(sensor);
	return error;
}

static int CWMCU_i2c_remove(struct i2c_client *client)
{
	struct CWMCU_data *sensor = i2c_get_clientdata(client);
	mutex_destroy(&sensor->iolock);
	meizu_sysfslink_unregister(LINK_KOBJ_NAME);
	kfree(sensor);
	return 0;
}
static void cwmcu_shutdown(struct i2c_client *client)
{
	int ret = 0;
	
	struct CWMCU_data *sensor = i2c_get_clientdata(client);
	if((!IS_ERR(sensor->sensor_pwr)) &&(regulator_is_enabled(sensor->sensor_pwr))){
		ret = regulator_disable(sensor->sensor_pwr);
		dev_info(&client->dev, "regulator_disable: %d\n", ret);
	}else{
		dev_err(&client->dev, "failed to disable sensor hub regulator (0x%p).\n",sensor->sensor_pwr);
	}
}

static const struct dev_pm_ops CWMCU_pm_ops = {
	.suspend = CWMCU_suspend,
	.resume = CWMCU_resume
};

static const struct i2c_device_id CWMCU_id[] = {
	{ CWMCU_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, CWMCU_id);

static struct of_device_id mx_hub_of_match_table[] = {
	{
		.compatible = "st,sensor_hub",
	},
	{},
};

static struct i2c_driver CWMCU_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &CWMCU_pm_ops,
		.of_match_table = mx_hub_of_match_table,
	},
	.probe    = CWMCU_i2c_probe,
	.remove   = CWMCU_i2c_remove,
	.shutdown = cwmcu_shutdown,
	.id_table = CWMCU_id,
};

static int __init CWMCU_i2c_init(void){
	printk(KERN_DEBUG "CWMCU_i2c_init\n");
	return i2c_add_driver(&CWMCU_driver);
}

static void __exit CWMCU_i2c_exit(void){
	i2c_del_driver(&CWMCU_driver);
}

module_init(CWMCU_i2c_init);
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
