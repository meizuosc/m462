#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
//#include <linux/input/synaptics_dsx.h>
#include "synaptics_dsx_core.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#define GESTURE_ERROR      0x00 

/*double tap */
#define DOUBLE_TAP          0xA0 

/*swipe  */
#define SWIPE_X_LEFT        0xB0 
#define SWIPE_X_RIGHT       0xB1 
#define SWIPE_Y_UP          0xB2
#define SWIPE_Y_DOWN        0xB3 

/*Unicode */
#define UNICODE_E           0xC0
#define UNICODE_C           0xC1
#define UNICODE_W           0xC2
#define UNICODE_M           0xC3
#define UNICODE_O           0xC4
#define UNICODE_S           0xC5
#define UNICODE_V_UP        0xC6
#define UNICODE_V_DOWN      0xC7
#define UNICODE_V_L         0xC8
#define UNICODE_V_R         0xC9
#define UNICODE_Z           0xCA

/*synaptics Unicode value*/
#define SYN_UNICODE_E   0x65
#define SYN_UNICODE_C   0x63
#define SYN_UNICODE_W   0x77
#define SYN_UNICODE_M   0x6D
#define SYN_UNICODE_S   0x73
#define SYN_UNICODE_Z   0x7A



/*disable gesture value */
#define ALL_CTR     0x01
#define TAP_CTR     0x02
#define UNICODE_CTR 0x03
#define SWIPE_CTR   0x04

#define gesture_type_addr  0x08
#define swipe_direction_addr 0x0418
#define unicode_switch_addr 0x042C
#define unicode_v_addr 	0x001A
#define gesture_control_addr (0x20)
#define gesture_only_addr (0x1B)
#define gestrue_bit  (1<<1)

#define SWIPE_INDEX 0 
#define TAP_INDEX 1
#define UNICODE_INDEX 2 
#define ALL_INDEX 3

#define DOUBLE_TAP_BIT    (0x03)
#define SWIPE_BIT         (0x07)
#define UNICODE_O_BIT     (0x08)
#define UNICODE_V_BIT     (0x0A)
#define UNICODE_BIT       (0x0B)
 
#define SWIPE_UP_BIT          (0x08)
#define SWIPE_DOWN_BIT        (0x04)
#define SWIPE_L_BIT           (0x02)
#define SWIPE_R_BIT           (0x01)
 
#define V_UP                  (0x01)/*up vee */
#define V_DOWN                (0x02)/*down vee */
#define V_LEFT                (0x04)/*left vee*/
#define V_RIGHT               (0x08)/*right vee */


/* F12_CTRL_REG*/
#define F12_DOUBLE_BIT    (1)
#define F12_SWIPE_BIT     (1<<1)
#define F12_O_BIT         (1<<3)
#define F12_V_BIT         (1<<5)
#define F12_UNICODE_BIT   (1<<6)
#define F12_SWIPE_OFFSET 6 
#define F12_CTR_SIZE  8
#define F12_SWIPE_PX  (1)   /*positive x Axis */ 
#define F12_SWIPE_NX  (1<<1)/*negative x Axis */
#define F12_SWIPE_PY  (1<<3)/*positve y Axis */
#define F12_SWIPE_NY   (1<<4)/*negative y Axis */

/*F51_CTR_REG:0x041E*/
#define F51_C_BIT (1)
#define F51_E_BIT (1<<1)
#define F51_W_BIT (1<<2)
#define F51_M_BIT (1<<3)
#define F51_S_BIT (1<<4)
#define F51_Z_BIT (1<<5)

static bool gesture_inited_mutex = false;
static struct mutex gesture_mutex;

//#define __GESTURE_WITH_MOTOR__

#ifdef __GESTURE_WITH_MOTOR__
extern void motor_enable(void );
extern void motor_disable(void);
extern int motor_set_vibration(int value);
static int work_init = 0 ;
static struct delayed_work motor_work;
static struct workqueue_struct *workqueue;

static void synaptics_motor_fn_work(struct work_struct *work)
{
	motor_disable();
}
static void  synaptics_enable_motor(void)
{
	if(!work_init){
    workqueue = create_singlethread_workqueue("motor_workqueue");
	INIT_DELAYED_WORK(&motor_work, synaptics_motor_fn_work);
	work_init = 1 ;
	}
	motor_set_vibration(0x20);
	motor_enable();
	
	queue_delayed_work(workqueue,
			&motor_work,HZ/5);
}

#endif

 static int synaptics_rmi4_control_gesture(struct synaptics_rmi4_data*rmi4_data);



 
 static void handler_Unicode_Gesture(struct synaptics_rmi4_data *rmi4_data,char *data)
 {
	short unicode ;

	if(!rmi4_data || !data)
		return ;

	unicode = data[1]<<8 | data[0] ;
	switch(unicode){
	 case SYN_UNICODE_C :
	 	rmi4_data->gesture_value = UNICODE_C ;
		break ;
	 case SYN_UNICODE_E :
	 	rmi4_data->gesture_value = UNICODE_E ;
		break ;
	 case SYN_UNICODE_M :
	 	rmi4_data->gesture_value = UNICODE_M ;
	    break ;
	 case SYN_UNICODE_W  :
	 	rmi4_data->gesture_value = UNICODE_W ;
		break ;
	case SYN_UNICODE_S   :
		rmi4_data->gesture_value = UNICODE_S ;
		break ;
	case SYN_UNICODE_Z  :
		rmi4_data->gesture_value = UNICODE_Z ;
		break ;
	default :
		rmi4_data->gesture_value = GESTURE_ERROR ;
		printk("ERROR :detect unicode gesture [%x]\n",unicode);
	}

	return ;
 }

static void handler_V_gesture(struct synaptics_rmi4_data * rmi4_data,char data)
{
	switch(data){
		case V_UP :
			rmi4_data->gesture_value  = UNICODE_V_UP ;
			break ;
		case V_DOWN : 
			rmi4_data->gesture_value = UNICODE_V_DOWN ;
		    break ;
		case V_LEFT :
			rmi4_data->gesture_value = UNICODE_V_L ;
			break ;
	    case V_RIGHT :
			rmi4_data->gesture_value = UNICODE_V_R ;
			break ;
		default :
			rmi4_data->gesture_value = GESTURE_ERROR ;
			printk("ERROR :detect V gesture[%x] \n",data);
	}
	
	return ;
}
static int handler_swipe_gesture(struct synaptics_rmi4_data * rmi4_data)
{
	int retval = -1 ;
	char swipe[1] ;

	if(!rmi4_data)
		return 0;
	
	retval = synaptics_rmi4_reg_read(rmi4_data,swipe_direction_addr,swipe,sizeof(swipe));
	if(retval<0){
	  printk(" read swipe direction error \n");
	  rmi4_data->gesture_value = GESTURE_ERROR ;
	  return 0;
	}

	switch(swipe[0]){
	 case SWIPE_UP_BIT :
	 	 rmi4_data->gesture_value = SWIPE_Y_UP ;
		 break ;
	 case SWIPE_DOWN_BIT :
	 	rmi4_data->gesture_value = SWIPE_Y_DOWN ;
		break ;
	case SWIPE_L_BIT  :
		rmi4_data->gesture_value = SWIPE_X_LEFT ;
		break ;
	case SWIPE_R_BIT  :
		rmi4_data->gesture_value = SWIPE_X_RIGHT;
		break ;
	default :
		printk("ERROR :detect swipe gesture[%x] \n",swipe[0]);
		rmi4_data->gesture_value = GESTURE_ERROR ;
	}
	return 0 ;
}

static char * dump_swipe_gesture(struct synaptics_rmi4_data *data)
{
	int value = data->gesture_value ;
	switch(value){
	   case SWIPE_X_LEFT :
	   	  return "SWIPE X LEFT" ;
	   case SWIPE_X_RIGHT :
	   	  return "SWIPE X RIGHT" ;
	   case SWIPE_Y_DOWN :
	   	  return "SWIPE Y DOWN" ;
	   case SWIPE_Y_UP :
	   	 return "SWIPE Y UP" ;
	   default :
	   	 return "SWIPE X/Y UNKOWN";

	}
	return "SWIPE X/Y UNKOWN" ;
}

static char * dump_unicode_gesture(struct synaptics_rmi4_data *data)
{
	int value = data->gesture_value ;
	switch(value){
	case UNICODE_C :
		return "UNICODE C";
	case UNICODE_E :
		return "UNICODE E";
	case UNICODE_W :
		return "UNICODE W";
	case UNICODE_M :
		return "UNICODE M";
	case UNICODE_S :
		return "UNICODE S";
	case UNICODE_Z :
		return "UNICODE Z";
	default :
		return "UNICODE UNKOWN";
	}
	return "UNICODE UNKOWN";
}

static char * dump_V_gesture(struct synaptics_rmi4_data *data)
{
	int value = data->gesture_value ;
	switch(value){
	case UNICODE_V_DOWN :
		return "V DOWN";
	case UNICODE_V_UP :
		return  "V UP";
	case UNICODE_V_L :
		return "V LEFT";
	case UNICODE_V_R :
		return "V RIGHT";
	default :
		return "V UNKONW";
	}
	return "V UNKONW";
}


 int synaptics_rmi4_f12_handler_gesture(struct synaptics_rmi4_data *rmi4_data)
{
	char propertes[5] ;
	
	int retval = -1 ;
	
	if(!rmi4_data)
		return 0 ;
	
	retval = synaptics_rmi4_reg_read(rmi4_data,gesture_type_addr,&propertes[0],sizeof(propertes));
	if(retval<0){
		printk("read gesture data error \n");
		return 0 ;
	}
	switch(propertes[0]){
	case DOUBLE_TAP_BIT :
		printk("double tap gesture\n");
		rmi4_data->gesture_value = DOUBLE_TAP ;
		break ;
	case SWIPE_BIT   :
		handler_swipe_gesture(rmi4_data);		
		printk("%s gesture x(%d),y(%d)\n",dump_swipe_gesture(rmi4_data),propertes[1],propertes[2]);
		break ;
    case UNICODE_O_BIT :
		 rmi4_data->gesture_value = UNICODE_O ;		 
         	printk("O gesture minimum speed [%d]\n",propertes[1]);
		break ;
    case UNICODE_V_BIT :
		 handler_V_gesture(rmi4_data,propertes[2]);
		printk("%s gesture minimum speed[%d]\n",dump_V_gesture(rmi4_data),propertes[1]);
		break ;
	case UNICODE_BIT  :
		handler_Unicode_Gesture(rmi4_data,&propertes[2]);		
		printk("%s gesture minimum speed[%d]\n",dump_unicode_gesture(rmi4_data),propertes[1]);
		break ;
	default :
	  printk(" Unsupport gesture detect[%x] \n",propertes[0]);
	  //rmi4_data->gesture_value = GESTURE_ERROR ;
	  return 0;
		
	}
	if(rmi4_data->gesture_value!=GESTURE_ERROR){
	#ifdef __GESTURE_WITH_MOTOR__
	synaptics_enable_motor();
	#endif
	input_report_key(rmi4_data->input_dev,KEY_GESTURE,1);
	input_report_key(rmi4_data->input_dev,KEY_GESTURE,0);
	input_sync(rmi4_data->input_dev);
	}
	
	return 0 ;
}


static void synaptics_rmi4_gesture_mode(struct synaptics_rmi4_data *rmi4_data,bool enable)
{
	char data[2] ;
	int retval = -1 ;
	retval = synaptics_rmi4_reg_read(rmi4_data,gesture_only_addr,&data[0],sizeof(data));
	if(enable)
	    data[0] |= gestrue_bit ;
	else 
		data[0] &= ~gestrue_bit ;
	
	retval |= synaptics_rmi4_reg_write(rmi4_data,gesture_only_addr,&data[0],sizeof(data));
	if(retval<0)
		printk("write gesture mode error \n");
   
	
	return ;
}

 void synaptics_rmi4_gesture_disable(struct synaptics_rmi4_data *rmi4_data)
{

	if(!gesture_inited_mutex)
	{		
		mutex_init(&gesture_mutex);		
		gesture_inited_mutex = true;
	}	
	mutex_lock(&gesture_mutex);
	 
	rmi4_data->current_page = 0xFF; // reset the page

	// Check the touch sleep state,if sleep then wakeup it
	
	synaptics_rmi4_gesture_mode(rmi4_data,false);
	rmi4_data->gesture_enable = false ;
	synaptics_rmi4_sensor_wake(rmi4_data);
	printk("disable gesture \n");
	mutex_unlock(&gesture_mutex);
}

 void synaptics_rmi4_gesture_enable(struct synaptics_rmi4_data *rmi4_data)
{

	if(!gesture_inited_mutex)
	{		
		mutex_init(&gesture_mutex);		
		gesture_inited_mutex = true;
	}	
	mutex_lock(&gesture_mutex);
	
	rmi4_data->gesture_value  = GESTURE_ERROR ;
	//synaptics_rmi4_sensor_wake(rmi4_data);
	synaptics_rmi4_gesture_mode(rmi4_data,true);
	synaptics_rmi4_control_gesture(rmi4_data);
	rmi4_data->gesture_enable = true ;
	printk("gesture enable\n");
	mutex_unlock(&gesture_mutex);

}

static int unicode_gesture_control(struct synaptics_rmi4_data *rmi4_data,char *data,char *reg)
{
	
	char v_unicode[26];
	if(!rmi4_data || !data||!reg)
		return -EIO ;
	
	/*handler V */
	 if(data[0]&0x01){
	 	reg[0] |= F12_V_BIT ;
		synaptics_rmi4_reg_read(rmi4_data,unicode_v_addr,&v_unicode[0],sizeof(v_unicode));
		v_unicode[22] |= (1 << 1);
		synaptics_rmi4_reg_write(rmi4_data,unicode_v_addr,&v_unicode[0],sizeof(v_unicode));
	 }else 
	 	reg[0] &=~(F12_V_BIT);

	/*handler O */
	if(data[0]&0x80)
		reg[0] |=F12_O_BIT ;
	else
		reg[0] &=~(F12_O_BIT);

	/*handler Unicode */
	if(data[0]&0x7E)
		reg[0] |=F12_UNICODE_BIT ;
	else
		reg[0] &=~(F12_UNICODE_BIT);

   return 0;
}

static int tap_gesture_control(struct synaptics_rmi4_data *rmi4_data,char *data,char *reg)
{
	if(!rmi4_data || !data||!reg)
		return -EIO ;

	if(data[0])
		reg[0] |= F12_DOUBLE_BIT ; /*enable double tap gesture */
	else 
		reg[0] &=~(F12_DOUBLE_BIT);/*disable double tap gesture */
	//return synaptics_rmi4_reg_write(rmi4_data,gesture_control_addr,reg,F12_CTR_SIZE);			
	return 0;
}

static int swipe_gesture_control(struct synaptics_rmi4_data *rmi4_data,char *data,char *reg)
{
	
	if(!rmi4_data || !data||!reg)
		return -EIO ;

	if(!(data[0]&0xF)){
	 /*disable all swipe gesture */
	 reg[0] &=~(F12_SWIPE_BIT) ;
	 reg[F12_SWIPE_OFFSET] &=~(0xF) ;
	}else {
		reg[0] |= (F12_SWIPE_BIT);	/*enable swipe gesture */	
		reg[F12_SWIPE_OFFSET] =data[0]&0xF ;/*disable/enable swipe x/y gesture */
	}

	//return synaptics_rmi4_reg_write(rmi4_data,gesture_control_addr,reg,F12_CTR_SIZE);
	return 0;
}

#if 0
static int disable_all_gesture(struct synaptics_rmi4_data *rmi4_data,char *data,char *reg)
{
	

	if(!rmi4_data||!data||!reg)
		return -EIO ;
	
	if(data[0]==0){
	   rmi4_data->disable_all = true ;
	}

	return 0;
}
#endif

 static int synaptics_rmi4_control_gesture(struct synaptics_rmi4_data*rmi4_data)
{
	char v_unicode[26];
	char gesture_reg[8] ;
	int retval = -1 ;
	char unicode =0xFF;
	int *p = (int *)rmi4_data->gesture_mask ;
	int all = (rmi4_data->gesture_mask[ALL_INDEX]&0xC0)>>6;
	retval = synaptics_rmi4_reg_read(rmi4_data,gesture_control_addr,&gesture_reg[0],sizeof(gesture_reg));
	if(retval<0){
		printk("read gesture control register error\n");
		return -ENODEV ;
	}
	if(all==2)/* enable some gesture */{
	swipe_gesture_control(rmi4_data,&rmi4_data->gesture_mask[SWIPE_INDEX],gesture_reg);
	unicode_gesture_control(rmi4_data,&rmi4_data->gesture_mask[UNICODE_INDEX],gesture_reg);
	tap_gesture_control(rmi4_data,&rmi4_data->gesture_mask[TAP_INDEX],gesture_reg);
	}else if(all==1){/*enable all gesture */
		gesture_reg[0] = 0xFF ;
		gesture_reg[F12_SWIPE_OFFSET] = 0x0F ;/*swipe */
		synaptics_rmi4_reg_read(rmi4_data,unicode_v_addr,&v_unicode[0],sizeof(v_unicode));
		v_unicode[22] |= (1 << 1);
		synaptics_rmi4_reg_write(rmi4_data,unicode_v_addr,&v_unicode[0],sizeof(v_unicode));
	}
	
	retval = synaptics_rmi4_reg_write(rmi4_data,gesture_control_addr,gesture_reg,F12_CTR_SIZE); 		  
	if(all==2)/*enable same unicode gesture */
	  unicode = (rmi4_data->gesture_mask[UNICODE_INDEX]&0x7E)>>1 ;
	else if(all==1) /*enable all unicode gesture */
		unicode = 0xFF ;
	
	retval |= synaptics_rmi4_reg_write(rmi4_data,unicode_switch_addr,&unicode,sizeof(unicode));			
	if(retval<0)
		printk("!!!!write gesture control ERROR!!!\n");
	printk("write gesture all[%d]mask(%x) \n",all,*p);
	return retval ;
}


/***********************************************
byte0 : 0=disable ,1=enable 
byte1 : enable/disalbe gesture (unicode e/w,swipe x/y)
byte2 : enable/disable gesture type(unicode,tap,swipe)
byte3 : PFU
***********************************************/
  ssize_t synaptics_rmi4_gesture_control_write(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
 {
	 
	 int tmp = 0 ;
	 struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	 
#if 1
	 const char * data = buf ;
	 printk("turn on/off [%x][%x][%x][%x] gesture control\n",
		  *(data+3),*(data+2),*(data+1),*(data));
	 	 
#else
	int data[4];
	sscanf(buf, "%d %d %d %d", data+3, data+2, data+1, data);
	 printk("%s: turn on/off [%x][%x][%x][%x] gesture control\n",
		  __func__, *(data+3),*(data+2),*(data+1),*(data));
	
#endif

	if(data[2]==ALL_CTR){
		rmi4_data->disable_all = !data[0] ;
	}else if(data[2]==SWIPE_CTR){
		rmi4_data->gesture_mask[SWIPE_INDEX] = 0x0F&data[0] ;
		//rmi4_data->gesture_mask[ALL_INDEX]   = ((0x0F&data[0])==0x0F?1:2)<<6 ;
	}else if(data[2]==UNICODE_CTR){
		rmi4_data->gesture_mask[UNICODE_INDEX] = 0xFF&data[0] ;
		//rmi4_data->gesture_mask[ALL_INDEX]     = ((0xFF&data[0])==0xFF?1:2)<<6 ;
	}else if(data[2]==TAP_CTR){
		rmi4_data->gesture_mask[TAP_INDEX] = 0x01&data[0] ;		
		//rmi4_data->gesture_mask[ALL_INDEX] = ((0x01&data[0])==0x01?1:2)<<6 ;
	}else {
		printk("parse gesture type error\n");		
		//rmi4_data->gesture_mask[ALL_INDEX] = 1
		return -EIO ;
	}

	tmp = ((rmi4_data->gesture_mask[SWIPE_INDEX]==0x0F)&&
		   (rmi4_data->gesture_mask[UNICODE_INDEX]==0xFF)&&
		   (rmi4_data->gesture_mask[TAP_INDEX]==0x01));
	rmi4_data->gesture_mask[ALL_INDEX] = (tmp?1:2)<<6 ;

	
	 return count;
}

ssize_t synaptics_rmi4_gesture_control_read(struct device *dev,
		 struct device_attribute *attr, char *buf)
 {
 
	 struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	 int *p = (int *)rmi4_data->gesture_mask ;
	 memcpy(buf,p,4);
	 printk("gesture mask %x %p \n",*p,buf);
	 return 4 ;
 }

ssize_t synaptics_rmi4_gesture_hex_write(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
 {
	 unsigned int value;
	 char * data = (char *)&value ;
	 int tmp = 0 ;
	 struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	 
	 if (sscanf(buf, "%x", &value) != 1){
		printk(KERN_ERR"%s:write gesture control data error\n",__func__);
		return -EINVAL;
	 }

	 printk("turn on/off [%x][%x][%x][%x] gesture control\n",
		  *(data+3),*(data+2),*(data+1),*(data));

	if(data[2]==ALL_CTR){
		rmi4_data->disable_all = !data[0] ;
	}else if(data[2]==SWIPE_CTR){
		rmi4_data->gesture_mask[SWIPE_INDEX] = 0x0F&data[0] ;
		//rmi4_data->gesture_mask[ALL_INDEX]   = ((0x0F&data[0])==0x0F?1:2)<<6 ;
	}else if(data[2]==UNICODE_CTR){
		rmi4_data->gesture_mask[UNICODE_INDEX] = 0xFF&data[0] ;
		//rmi4_data->gesture_mask[ALL_INDEX]     = ((0xFF&data[0])==0xFF?1:2)<<6 ;
	}else if(data[2]==TAP_CTR){
		rmi4_data->gesture_mask[TAP_INDEX] = 0x01&data[0] ;		
		//rmi4_data->gesture_mask[ALL_INDEX] = ((0x01&data[0])==0x01?1:2)<<6 ;
	}else {
		printk("parse gesture type error\n");		
		//rmi4_data->gesture_mask[ALL_INDEX] = 1
		return -EIO ;
	}

	tmp = ((rmi4_data->gesture_mask[SWIPE_INDEX]==0x0F)&&
		   (rmi4_data->gesture_mask[UNICODE_INDEX]==0xFF)&&
		   (rmi4_data->gesture_mask[TAP_INDEX]==0x01));
	rmi4_data->gesture_mask[ALL_INDEX] = (tmp?1:2)<<6 ;
	
	 return count;
}

ssize_t synaptics_rmi4_gesture_hex_read(struct device *dev,
		 struct device_attribute *attr, char *buf)
 {
	 struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	 int *p = (int *)rmi4_data->gesture_mask ;
	 int count = snprintf(buf, PAGE_SIZE, "%x\n",
			 *p);
	 printk("gesture %x detect \n",rmi4_data->gesture_value);
	
	 return count ;
 }

ssize_t synaptics_rmi4_gesture_value_read(struct device *dev,
		 struct device_attribute *attr, char *buf)
 {
	 struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
 	 int count = snprintf(buf, PAGE_SIZE, "%u\n",
			 rmi4_data->gesture_value);
	 printk("gesture %x detect \n",rmi4_data->gesture_value);
	 rmi4_data->gesture_value = GESTURE_ERROR ;
	 return count ;
 }

 ssize_t synaptics_rmi4_gesture_test(struct device *dev,
		  struct device_attribute *attr, char *buf)
  {
	  struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	  unsigned char value[8] ;
      int *p = (int *)rmi4_data->gesture_mask ;
	  int count = snprintf(buf, PAGE_SIZE, "gesture_value=%u\n",
			  rmi4_data->gesture_value);

	  count += snprintf(buf+count, PAGE_SIZE, "gesture_mask=%x\n",
			  *p);
	  
	  count += snprintf(buf+count, PAGE_SIZE, "disable_all=%x\n",
			  rmi4_data->disable_all);
	  
     synaptics_rmi4_reg_read(rmi4_data,gesture_only_addr,&value[0],3);
	 count += snprintf(buf+count, PAGE_SIZE, "gesture_only=%x\n",
			  value[2]);
     synaptics_rmi4_reg_read(rmi4_data,gesture_control_addr,&value[0],sizeof(value));

	count += snprintf(buf+count, PAGE_SIZE, "gesture_control=%x ",
			  value[0]);
	count += snprintf(buf+count ,PAGE_SIZE,"tap(%d),swipe(%d),O(%d),V(%d),unicode(%d)\n",
		!!(value[0]&F12_DOUBLE_BIT),!!(value[0]&F12_SWIPE_BIT),!!(value[0]&F12_O_BIT),
		      !!(value[0]&F12_V_BIT),!!(value[0]&F12_UNICODE_BIT));
	
	count += snprintf(buf+count, PAGE_SIZE, "swipe_control=%x :",
			  value[F12_SWIPE_OFFSET]);

	count += snprintf(buf+count, PAGE_SIZE, "UP(%d),DOWN(%d),L(%d),R(%d)\n",
			  !!(value[F12_SWIPE_OFFSET]&SWIPE_UP_BIT),!!(value[F12_SWIPE_OFFSET]&SWIPE_DOWN_BIT),
			  !!(value[F12_SWIPE_OFFSET]&SWIPE_L_BIT),!!(value[F12_SWIPE_OFFSET]&SWIPE_R_BIT));

	synaptics_rmi4_reg_read(rmi4_data,unicode_switch_addr,&value[0],sizeof(value[0]));		  

	
	count += snprintf(buf+count, PAGE_SIZE, "unicode_control=%x: ",
				  value[0]);
	count += snprintf(buf+count, PAGE_SIZE, "C(%d),E(%d),W(%d),M(%d),S(%d),Z(%d)\n",
				  !!(value[0]&F51_C_BIT),!!(value[0]&F51_E_BIT),!!(value[0]&F51_W_BIT),
				  !!(value[0]&F51_M_BIT),!!(value[0]&F51_S_BIT),!!(value[0]&F51_Z_BIT));
		
	  return count ;
  }

