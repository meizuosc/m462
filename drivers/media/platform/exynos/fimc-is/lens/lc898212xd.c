/*
 * TDK tvclb850lba voice coil motor driver IC LC898212XD.
 *
 * Modified to comply with SS architecture by qudao.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "../meizu_camera_special.h"
#include "AfInit.h"
#include "AfSTMV.h"

#define LENS_I2C_BUSNUM 0
#define E2PROM_WRITE_ID 0xA8
#define E2PROM_SHARP_WRITE_ID 0xA0

#define LITEON_MODULE	0x01
#define SHARP_MODULE	0x02

#define Min_Pos		0
#define Max_Pos		1023

#define MAX_INFI	0x6400
#define MAX_MACRO	0x9C00

signed short Hall_Max = 0x0000; // Please read INF position from EEPROM or OTP
signed short Hall_Min = 0x0000; // Please read MACRO position from EEPROM or OTP

static bool af_active_progress = false;
static char af_active_result[1024];
#define LC898212XD_DRVNAME "LC898212XD"

#define LC898212XD_DEBUG
#ifdef LC898212XD_DEBUG
#define LC898212XDDB printk
#else
#define LC898212XDDB(x,...)
#endif

static spinlock_t g_LC898212XD_SpinLock;
static struct i2c_client * g_pstLC898212XD_I2Cclient = NULL;
static unsigned long g_u4LC898212XD_INF = 0;
static unsigned long g_u4LC898212XD_MACRO = 1023;
static unsigned long g_u4CurrPosition   = 0;

extern void RamReadA(unsigned short addr, unsigned short *data);

/*******************************************************************************
* WriteRegI2C
********************************************************************************/
int LC898212XD_WriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId)
{
	int  i4RetValue = 0;
	int retry = 3;
	u16 i2c_origi_addr;

	spin_lock(&g_LC898212XD_SpinLock);
	i2c_origi_addr = g_pstLC898212XD_I2Cclient->addr;
	g_pstLC898212XD_I2Cclient->addr = (i2cId >> 1);
	spin_unlock(&g_LC898212XD_SpinLock);

	do {
		i4RetValue = i2c_master_send(g_pstLC898212XD_I2Cclient,
			a_pSendData, a_sizeSendData);
		if (i4RetValue != a_sizeSendData) {
			LC898212XDDB("[LC898212XD] I2C send failed!!, Addr = 0x%x, Data = 0x%x \n",
				a_pSendData[0], a_pSendData[1] );
		} else {
			break;
		}
		udelay(50);
	} while ((retry--) > 0);

	spin_lock(&g_LC898212XD_SpinLock);
	g_pstLC898212XD_I2Cclient->addr = i2c_origi_addr;
	spin_unlock(&g_LC898212XD_SpinLock);
	return 0;
}

/*******************************************************************************
* ReadRegI2C
********************************************************************************/
int LC898212XD_ReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int  i4RetValue = 0;
	u16 i2c_origi_addr;

	spin_lock(&g_LC898212XD_SpinLock);
	i2c_origi_addr = g_pstLC898212XD_I2Cclient->addr;
	g_pstLC898212XD_I2Cclient->addr = (i2cId >> 1);
	spin_unlock(&g_LC898212XD_SpinLock);

	i4RetValue = i2c_master_send(g_pstLC898212XD_I2Cclient,
		a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		LC898212XDDB("[LC898212XD] I2C send failed!!, Slave Addr = 0x%x\n",
			g_pstLC898212XD_I2Cclient->addr);
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstLC898212XD_I2Cclient, (u8 *)a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		LC898212XDDB("[LC898212XD] I2C read failed!! \n");
		return -1;
	}

	spin_lock(&g_LC898212XD_SpinLock);
	g_pstLC898212XD_I2Cclient->addr = i2c_origi_addr;
	spin_unlock(&g_LC898212XD_SpinLock);

	return 0;
}

void E2PROMReadA_sharp(unsigned short addr, u8 *data)
{
	int ret = 0;

	u8 puSendCmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};
	ret = LC898212XD_ReadRegI2C(puSendCmd , sizeof(puSendCmd), data, 1, E2PROM_SHARP_WRITE_ID);     
	if (ret < 0)
		LC898212XDDB("[LC898212XD] I2C read e2prom failed!! \n");

	return;
}

void E2PROMReadA(unsigned short addr, u8 *data)
{
	int ret = 0;

	u8 puSendCmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};
	ret = LC898212XD_ReadRegI2C(puSendCmd , sizeof(puSendCmd), data, 1, E2PROM_WRITE_ID);     
	if (ret < 0)
		LC898212XDDB("[LC898212XD] I2C read e2prom failed!! \n");

	return;
}

int AF_reverse_convert(signed short position)
{
#if 0       // 1: INF -> Macro =  0x8001 -> 0x7FFF
	return (((unsigned short)(position - Hall_Min) * (Max_Pos - Min_Pos) / (unsigned short)(Hall_Max - Hall_Min)) + Min_Pos);
#else  // 0: INF -> Macro =  0x7FFF -> 0x8001
	return (Max_Pos - ((unsigned short)(position - Hall_Min) * (Max_Pos - Min_Pos) / (unsigned short)(Hall_Max - Hall_Min)));
#endif
}

unsigned short AF_convert(int position)
{
#if 0       // 1: INF -> Macro =  0x8001 -> 0x7FFF
    return (((position - Min_Pos) * (unsigned short)(Hall_Max - Hall_Min) / (Max_Pos - Min_Pos)) + Hall_Min) & 0xFFFF;
#else  // 0: INF -> Macro =  0x7FFF -> 0x8001
	return (((Max_Pos - position) * (unsigned short)(Hall_Max - Hall_Min) / (Max_Pos - Min_Pos)) + Hall_Min) & 0xFFFF;
#endif
}

inline static int moveLC898212XD(unsigned long a_u4Position)
{
	StmvTo( AF_convert(a_u4Position) ) ;	// Move to Target Position

	spin_lock(&g_LC898212XD_SpinLock);
	g_u4CurrPosition = (unsigned long)a_u4Position;
	spin_unlock(&g_LC898212XD_SpinLock);
	return 0;
}

inline static int setLC898212XDInf(unsigned long a_u4Position)
{
    spin_lock(&g_LC898212XD_SpinLock);
    g_u4LC898212XD_INF = a_u4Position;
    spin_unlock(&g_LC898212XD_SpinLock);	
    return 0;
}

inline static int setLC898212XDMacro(unsigned long a_u4Position)
{
    spin_lock(&g_LC898212XD_SpinLock);
    g_u4LC898212XD_MACRO = a_u4Position;
    spin_unlock(&g_LC898212XD_SpinLock);	
    return 0;	
}

static int lc898212xd_init(struct fimc_is_device_sensor *device)
{
	stSmvPar StSmvPar;
	u8 buf[4];
	int module_id = 0;

	unsigned int HallOff = 0x00;	 	// Please Read Offset from EEPROM or OTP
	unsigned int HallBias = 0x00;   // Please Read Bias from EEPROM or OTP
    
	LC898212XDDB("[LC898212XD] LC898212XD_init - Start\n");

	module_id = get_module_type(device);
	pr_info("%s(), module_id:%d\n", __func__, module_id);
	if (module_id < 0) {
		pr_err("%s(), err! invalid module_id:%d\n",
			__func__, module_id);
		return -EINVAL;
	}

	if (module_id == LITEON_MODULE) {
		E2PROMReadA(0x0238, &buf[0]);
		E2PROMReadA(0x0239, &buf[1]);
		E2PROMReadA(0x023a, &buf[2]);
		E2PROMReadA(0x023b, &buf[3]);
		Hall_Max = buf[0] | (buf[1] << 8);

		E2PROMReadA(0x023c, &buf[0]);
		E2PROMReadA(0x023d, &buf[1]);
		E2PROMReadA(0x023e, &buf[2]);
		E2PROMReadA(0x023f, &buf[3]);
		Hall_Min = buf[0] | (buf[1] << 8);

		E2PROMReadA(0x0240, &buf[0]);
		E2PROMReadA(0x0241, &buf[1]);
		E2PROMReadA(0x0242, &buf[2]);
		E2PROMReadA(0x0243, &buf[3]);
		HallBias= buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);

		E2PROMReadA(0x0244, &buf[0]);
		E2PROMReadA(0x0245, &buf[1]);
		E2PROMReadA(0x0246, &buf[2]);
		E2PROMReadA(0x0247, &buf[3]);
		HallOff= buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	} else if (module_id == SHARP_MODULE) {
		E2PROMReadA_sharp(0x0238, &buf[0]);
		E2PROMReadA_sharp(0x0239, &buf[1]);
		E2PROMReadA_sharp(0x023a, &buf[2]);
		E2PROMReadA_sharp(0x023b, &buf[3]);
		Hall_Max = buf[0] | (buf[1] << 8);

		E2PROMReadA_sharp(0x023c, &buf[0]);
		E2PROMReadA_sharp(0x023d, &buf[1]);
		E2PROMReadA_sharp(0x023e, &buf[2]);
		E2PROMReadA_sharp(0x023f, &buf[3]);
		Hall_Min = buf[0] | (buf[1] << 8);

		E2PROMReadA_sharp(0x0240, &buf[0]);
		E2PROMReadA_sharp(0x0241, &buf[1]);
		E2PROMReadA_sharp(0x0242, &buf[2]);
		E2PROMReadA_sharp(0x0243, &buf[3]);
		HallBias= buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);

		E2PROMReadA_sharp(0x0244, &buf[0]);
		E2PROMReadA_sharp(0x0245, &buf[1]);
		E2PROMReadA_sharp(0x0246, &buf[2]);
		E2PROMReadA_sharp(0x0247, &buf[3]);
		HallOff= buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);

	}

	pr_info("%s(), Hall_Max:0x%x, Hall_Min:0x%x, HallOff:0x%x, HallBias: 0x%x\n",
		__func__, Hall_Max, Hall_Min, HallOff, HallBias);

	AfInit( HallOff,  HallBias);	// Initialize driver IC

	// Step move parameter set
	StSmvPar.UsSmvSiz	= STMV_SIZE ;
	StSmvPar.UcSmvItv	= STMV_INTERVAL ;
	StSmvPar.UcSmvEnb	= STMCHTG_SET | STMSV_SET | STMLFF_SET ;
	StmvSet( StSmvPar ) ;
	
	ServoOn();	// Close loop ON

	LC898212XDDB("[LC898212XD] LC898212XD_init - End\n");

	return 0;
}

static ssize_t af_range_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	char *p = buf;
	int ret;
	int min_pos, max_pos;
	struct platform_device * pdev = to_platform_device(dev);
	struct fimc_is_device_sensor *device;

	device = (struct fimc_is_device_sensor *)platform_get_drvdata(pdev);

	ret = camera_module_active(device, true);
	if (ret) {
		pr_err("%s(), active camera module failed:%d\n",
			__func__, ret);
		return ret;
	}

	/* initialize camera af */
	ret = lc898212xd_init(device);
	if (ret) {
		pr_info("%s(), lc898212xd_init() failed:%d\n",
			__func__, ret);
		camera_module_active(device, false);
		return ret;
	}
	min_pos = AF_reverse_convert(MAX_INFI);
	max_pos = AF_reverse_convert(MAX_MACRO);
	p += sprintf(p, "Min AF position: %d\n", min_pos);
	p += sprintf(p, "Max AF position: %d\n", max_pos);

	camera_module_active(device, false);

	return (p - buf);
}

static ssize_t af_range_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t af_active_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	char *p = buf;

	if (!af_active_progress) {
		pr_err("%s(), focus not triggerred yet\n", __func__);
		return -EPERM;
	}

	p += scnprintf(p, sizeof(af_active_result), af_active_result);
	af_active_progress = false;
	return (p - buf);
}

static ssize_t af_active_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int val = -1;
	int ret;
	unsigned short pos;
	int real_pos_10;
	signed short	test_pos_16 = 0x0000; /* 16bit vcm moveto set, just for test */
	char *p = af_active_result;
	struct platform_device * pdev = to_platform_device(dev);
	struct fimc_is_device_sensor *device;

	device = (struct fimc_is_device_sensor *)platform_get_drvdata(pdev);

	if (af_active_progress) {
		pr_err("%s(), already triggered focus\n", __func__);
		return -EBUSY;
	}

	sscanf(buf, "%d", &val);
	if (val < 0 || val > 1023) {
		LC898212XDDB("[LC898212XD] invalid test moveto position\n");
		snprintf(af_active_result, sizeof(af_active_result), "Invalid AF move position!\n");
		/*
		* To comply with show method
		*/
		af_active_progress = true;
		return -EINVAL;
	}

	ret = camera_module_active(device, true);
	if (ret) {
		pr_err("%s(), active camera module failed:%d\n",
		__func__, ret);
		return ret;
	}

	/* initialize camera af */
	ret = lc898212xd_init(device);
	if (ret) {
		pr_info("%s(), lc898212xd_init() failed:%d\n",
			__func__, ret);
		camera_module_active(device, false);
		return ret;
	}

	moveLC898212XD(val);
	/* enough settle time for vcm moveto */
	msleep(1000);

	test_pos_16 = (signed short)AF_convert(val);	/* 10bit ADC exchange to 16bit ADC value */

	RamReadA(0x3C,	&pos);	/* Get Position */
	real_pos_10 = AF_reverse_convert(pos);

	camera_module_active(device, false);
	p += sprintf(p, "current AF position: %d\n", real_pos_10);

	pr_info("%s(), convert user's input to reg: %d -> %d, reverse convert %d -> %d\n",
		__func__, val, test_pos_16, test_pos_16, AF_reverse_convert(test_pos_16));
	pr_info("read from reg:%d, convers actual reg's value to user side: %d -> %d,",
		(signed short)pos, (signed short)pos, real_pos_10);

	/* check whether VCM moveto is blocked */
	pr_info("%s(), compare as signed, pos:%d, test_pos_16:%d\n",
		__func__, (signed short)pos, test_pos_16);

	if (((signed short)pos >= (test_pos_16 - 0x200))
		&& ((signed short)pos <= (test_pos_16 + 0x200))) {
		p += sprintf(p, "PASS:AF move is OK.\n");
	} else {
		p += sprintf(p, "NO PASS:AF move is not OK, maybe blocked!\n");
	}

	af_active_progress = true;
	return count;
}

static struct device_attribute dev_attr_af_active = {
	.attr = {.name = "camera_af_active", .mode = 0644},
	.show = af_active_show,
	.store = af_active_store,
};

static struct device_attribute dev_attr_af_range = {
	.attr = {.name = "camera_af_range", .mode = 0444},
	.show = af_range_show,
	.store = af_range_store,
};

int LC898212XD_probe(struct fimc_is_device_sensor * device,
	struct i2c_client *client)
{
	struct fimc_is_core *core;
	struct platform_device *pdev;

	pdev = device->pdev;
	core = device->private_data;

	g_pstLC898212XD_I2Cclient = client;

	spin_lock_init(&g_LC898212XD_SpinLock);

	device_create_file(&pdev->dev, &dev_attr_af_active);
	device_create_file(&pdev->dev, &dev_attr_af_range);

	dev_info(&client->dev, "%s() success, i2c client name && addr: %s, 0x%x\n", __func__,
		client->name, client->addr);
    return 0;
}

