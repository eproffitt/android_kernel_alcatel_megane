/* drivers/hwmon/mt6516/amit/TMD2772.c - TMD2772 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif

#ifdef MT6575
#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
#ifdef MT6577
#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6577
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "tmd2772.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define TMD2772_DEV_NAME     "TMD2772"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)

#define TMD2772_DEBUG      
#define PS_GET_NUM  20

/******************************************************************************
 * extern functions
*******************************************************************************/
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
#ifdef MT6575
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
	extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
	extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
	extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
										 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
										 kal_bool auto_umask);
	
#endif
#ifdef MT6516
extern void MT6516_EINTIRQUnmask(unsigned int line);
extern void MT6516_EINTIRQMask(unsigned int line);
extern void MT6516_EINT_Set_Polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void MT6516_EINT_Set_HW_Debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 MT6516_EINT_Set_Sensitivity(kal_uint8 eintno, kal_bool sens);
extern void MT6516_EINT_Registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_client *TMD2772_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id TMD2772_i2c_id[] = {{TMD2772_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_TMD2772={ I2C_BOARD_INFO("TMD2772", (0X72>>1))};

/*the adapter id & i2c address will be available in customization*/
//static unsigned short TMD2772_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const TMD2772_forces[] = { TMD2772_force, NULL };
//static struct i2c_client_address_data TMD2772_addr_data = { .forces = TMD2772_forces,};
/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int TMD2772_i2c_remove(struct i2c_client *client);
static int TMD2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int TMD2772_i2c_resume(struct i2c_client *client);
static void TMD2772_ps_calibrate(struct i2c_client *client);
static int TMD2772_init_client_for_cali(struct i2c_client *client);
static int TMD2772_init_client_for_cali_restore(struct i2c_client *client);
static int TMD2772_set_threshold_value(struct i2c_client *client);

static struct TMD2772_priv *g_TMD2772_ptr = NULL;

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct TMD2772_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct TMD2772_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct TMD2772_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver TMD2772_i2c_driver = {	
	.probe      = TMD2772_i2c_probe,
	.remove     = TMD2772_i2c_remove,
	.detect     = TMD2772_i2c_detect,
	.suspend    = TMD2772_i2c_suspend,
	.resume     = TMD2772_i2c_resume,
	.id_table   = TMD2772_i2c_id,
//	.address_data = &TMD2772_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = TMD2772_DEV_NAME,
	},
};

static struct TMD2772_priv *TMD2772_obj = NULL;
static struct platform_driver TMD2772_alsps_driver;
static struct mutex sensor_mutex;       /*added by pcyang 2013-01-08 for protect the p-sensor operate*/

/*----------------------------------------------------------------------------*/
/*when read or write the p-sensor,use it to lock the register   added by pcyang */
void __sched psensor_mutex_lock(struct mutex *lock)
{
    mutex_lock(lock);
    #ifdef TMD2772_DEBUG
    APS_DBG("psensor_mutex_lock is called\n");
    #endif
    
}
/*when read or write the p-sensor,use it to unlock the register   added by pcyang */
void __sched psensor_mutex_unlock(struct mutex *lock)
{
    mutex_unlock(lock);
    #ifdef  TMD2772_DEBUG
    APS_DBG("psensor_mutex_unlock is called\n");
    #endif
}


int TMD2772_get_addr(struct alsps_hw *hw, struct TMD2772_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void TMD2772_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2772")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2772")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
long TMD2772_read_ps(struct i2c_client *client, u16 *data)
{
//	struct TMD2772_priv *obj = i2c_get_clientdata(client);    
	//u16 ps_value;    
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	long res = 0;
        
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
    psensor_mutex_lock(&sensor_mutex);
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_read_ps is called\n");
    #endif
	buffer[0]=TMD2772_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
    // APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	buffer[0]=TMD2772_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{       
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
    // APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	if(res <= 0)
	{       
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
    psensor_mutex_unlock(&sensor_mutex);
	APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;    

EXIT_ERR:
	APS_ERR("TMD2772_read_ps fail\n");
    psensor_mutex_unlock(&sensor_mutex);
	return res;
}

/*----------------------------------------------------------------------------*/

static long TMD2772_enable_als(struct i2c_client *client, int enable)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    u8 databuf[2];	  
    long res = 0;
    uint32_t testbit_PS;

    APS_DBG("TMD2772_enable_als is called\n");
    
    if(client == NULL){
    	APS_DBG("CLIENT CANN'T EQUL NULL\n");
    	return -1;
    }

    /*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
    testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
    
    if(enable){
    	if(testbit_PS){	
        	databuf[0] = TMD2772_CMM_ENABLE;	
        	databuf[1] = 0x2F;
            
             psensor_mutex_lock(&sensor_mutex);
        	res = i2c_master_send(client, databuf, 0x2);
             psensor_mutex_unlock(&sensor_mutex);

            if(res <= 0){
    			goto EXIT_ERR;
    		}
            
        	/*debug code for reading register value*/
        	#if 0
        	res = i2c_master_recv(client, reg_value, 0x1);
            
        	if(res <= 0){
    			goto EXIT_ERR;
    		}
            
        	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
        	#endif
    	}
    	else{
        	databuf[0] = TMD2772_CMM_ENABLE;	
        	databuf[1] = 0x2B;
             psensor_mutex_lock(&sensor_mutex);
        	res = i2c_master_send(client, databuf, 0x2);
             psensor_mutex_unlock(&sensor_mutex);
                        
        	if(res <= 0){
    			goto EXIT_ERR;
    		}

        	/*debug code for reading register value*/
        	#if 0
        	res = i2c_master_recv(client, reg_value, 0x1);
            
        	if(res <= 0){
    			goto EXIT_ERR;
    		}
            
        	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
        	#endif

    	}
        
    	atomic_set(&obj->als_deb_on, 1);
    	atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
        
    	APS_DBG("TMD2772_enable_als: ALS power on!!!\n");
    }
    else
    {	
    	if(testbit_PS){
        	databuf[0] = TMD2772_CMM_ENABLE;	
        	databuf[1] = 0x2D;
            
             psensor_mutex_lock(&sensor_mutex);
        	res = i2c_master_send(client, databuf, 0x2);
             psensor_mutex_unlock(&sensor_mutex);
                        
        	if(res <= 0){
    			goto EXIT_ERR;
    		}
    	}
    	else{
        	databuf[0] = TMD2772_CMM_ENABLE;	
        	databuf[1] = 0x00;
            
             psensor_mutex_lock(&sensor_mutex);
        	res = i2c_master_send(client, databuf, 0x2);
             psensor_mutex_unlock(&sensor_mutex);
             
        	if(res <= 0){
    			goto EXIT_ERR;
    		}
    	}
        	/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
        	atomic_set(&obj->als_deb_on, 0);
        	APS_DBG("TMD2772_enable_als: ALS power off!!!\n");
    }
    
#if 0 /*yucong add for debug*/
    	buffer[0]=TMD2772_CMM_ENABLE;
    	res = i2c_master_send(client, buffer, 0x1);
    	if(res <= 0)
    	{
    		goto EXIT_ERR;
    	}
    	res = i2c_master_recv(client, reg_value, 0x1);
    	if(res <= 0)
    	{
    		goto EXIT_ERR;
    	}
    	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
#endif

    return 0;

    EXIT_ERR:
    APS_ERR("TMD2772_enable_als fail\n");
    return res;
    }

/*----------------------------------------------------------------------------*/
static long TMD2772_enable_ps(struct i2c_client *client, int enable)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    u8 databuf[2];    
    long res = 0;
    uint32_t testbit_ALS;
        
    int prox_sum = 0, prox_mean = 0, prox_max = 0;
    int prox_threshold_hi = 0, prox_threshold_lo = 0;
    int i;
    long ret = 0;
    u16 prox_data[20];
    
    if(client == NULL){
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    /*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
    testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
    
    if(enable){
        if(testbit_ALS){
            databuf[0] = TMD2772_CMM_ENABLE;    
            databuf[1] = 0x2F;           /*modified by pcyang to enable the proximity interrupt 2013-01-08*/
            
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);

            if(res <= 0)
            {
                goto EXIT_ERR;
            }
        }
        else{
            databuf[0] = TMD2772_CMM_ENABLE;    
            databuf[1] = 0x2D;          /*modified by pcyang to enable the proximity interrupt 2013-01-08*/

            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);

            if(res <= 0)
            {
                goto EXIT_ERR;
            }
        }

        atomic_set(&obj->ps_deb_on, 1);
        atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));

        if(ps_cali.valid == 0){                
            //calibraion during call.
            TMD2772_ps_calibrate(client);
        #ifdef TMD2772_DEBUG 
            APS_DBG("TMD2772_enable_ps:TMD2772_ps_calibrate, use caliberation value!!!\n");
        #endif
            TMD2772_set_threshold_value(client);
                    
            APS_DBG("threshold_low=%d  threshold_high=%d \n", atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high)); 
        }

       /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
       if(0 == obj->hw->polling_mode_ps)
       {
       #ifdef TMD2772_DEBUG 
          	APS_DBG("TMD2772_enable_ps: eint mode!!!\n");
       #endif
       
	   	databuf[0] = TMD2772_CMM_Persistence;
          	databuf[1] = 0x20;
            
          	psensor_mutex_lock(&sensor_mutex);
          	res = i2c_master_send(client, databuf, 0x2);
          	psensor_mutex_unlock(&sensor_mutex);
            
          	if(res <= 0){
              	goto EXIT_ERR;
          	}
            
          	if(testbit_ALS){
              	databuf[0] = TMD2772_CMM_ENABLE;    
              	databuf[1] = 0x2F;
              	psensor_mutex_lock(&sensor_mutex);
              	res = i2c_master_send(client, databuf, 0x2);
              	psensor_mutex_unlock(&sensor_mutex);
                
              	if(res <= 0){
              		goto EXIT_ERR;
              	}
          	}
          	else{      
              	databuf[0] = TMD2772_CMM_ENABLE;    
              	databuf[1] = 0x2D;
              	psensor_mutex_lock(&sensor_mutex);
              	res = i2c_master_send(client, databuf, 0x2);
              	psensor_mutex_unlock(&sensor_mutex);

              	if(res <= 0){
              		goto EXIT_ERR;
           		}
       		}	

          	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
       }
        	APS_DBG("TMD2772 power on\n");
    }
    else
    {
    /*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
        if(testbit_ALS){
            databuf[0] = TMD2772_CMM_ENABLE;    
            databuf[1] = 0x2B;
            
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
            
            if(res <= 0){
                goto EXIT_ERR;
            }
        }
        else{
            databuf[0] = TMD2772_CMM_ENABLE;    
            databuf[1] = 0x00;
            
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
            
            if(res <= 0){
                goto EXIT_ERR;
            }
        }
        
        atomic_set(&obj->ps_deb_on, 0);
        APS_DBG("TMD2772 power off\n");

        /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
        if(0 == obj->hw->polling_mode_ps){
            cancel_work_sync(&obj->eint_work);
            mt65xx_eint_mask(CUST_EINT_ALS_NUM);
        }
    }

    return 0;

EXIT_ERR:
    APS_ERR("TMD2772_enable_ps fail\n");
    return res;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int TMD2772_check_and_clear_intr(struct i2c_client *client) 
{
	//struct TMD2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong TMD2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("TMD2772_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int TMD2772_check_intr(struct i2c_client *client) 
{
//	struct TMD2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
    psensor_mutex_lock(&sensor_mutex);
	res = i2c_master_send(client, buffer, 0x1);
    psensor_mutex_unlock(&sensor_mutex);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
    psensor_mutex_lock(&sensor_mutex);
	res = i2c_master_recv(client, buffer, 0x1);
    psensor_mutex_unlock(&sensor_mutex);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("TMD2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("TMD2772_check_intr fail\n");
	return 1;
}

static int TMD2772_clear_intr(struct i2c_client *client) 
{
	//struct TMD2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

#if 0
	if((1 == intp) && (0 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
	}
	else if((0 == intp) && (1 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
	}
	else
#endif
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	}
    psensor_mutex_lock(&sensor_mutex);
    APS_DBG("TMD2772_clear_intr is called!\n");
	res = i2c_master_send(client, buffer, 0x1);
    psensor_mutex_unlock(&sensor_mutex);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("TMD2772_check_and_clear_intr fail\n");
	return 1;
}


/*-----------------------------------------------------------------------------*/
void TMD2772_eint_func(void)
{
	struct TMD2772_priv *obj = g_TMD2772_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int TMD2772_setup_eint(struct i2c_client *client)
{
	struct TMD2772_priv *obj = i2c_get_clientdata(client);        

	g_TMD2772_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, TMD2772_eint_func, 0);

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}

/*----------------------------------------------------------------------------*/
static int TMD2772_init_client_for_cali(struct i2c_client *client)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    u8 databuf[2];    
    int res = 0;

    databuf[0] = TMD2772_CMM_ENABLE;    
    databuf[1] = 0x01;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_ATIME;    
    databuf[1] = 0xEE;
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_PTIME;    
    databuf[1] = 0xFF;
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_WTIME;    
    databuf[1] = 0xFF;
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_CONFIG;    
    databuf[1] = 0x00;  
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_OFFSET;
    databuf[1] = TMD2772_CMM_OFFSET_VALUE;//0x01
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_PPCOUNT;    
    databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;//0x02
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_CONTROL;    
    databuf[1] = TMD2772_CMM_CONTROL_VALUE;//0x22
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }
    
    databuf[0] = TMD2772_CMM_ENABLE;	
    databuf[1] = 0x0F;
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
    	goto EXIT_ERR;
    }

    return TMD2772_SUCCESS;

EXIT_ERR:
    APS_ERR("TMD2772_init_client_for_cali error:res = %d\n", res);
    return res;
}

static int TMD2772_init_client(struct i2c_client *client)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    u8 databuf[2];    
    int res = 0;

    databuf[0] = TMD2772_CMM_ENABLE;    
    databuf[1] = 0x00;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_ATIME;    
    databuf[1] = 0xC9;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_PTIME;    
    databuf[1] = 0xFF;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_WTIME;    
    databuf[1] = 0xEE;
    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_CONFIG;    
    databuf[1] = 0x00;          //modify the data for less drive  added by pcyang 2013.01.14
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    /*JRD pcyang add 2013-01-08,add the p-sensor offset*/
    databuf[0] = TMD2772_CMM_OFFSET;
    databuf[1] = TMD2772_CMM_OFFSET_VALUE;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
    databuf[0] = TMD2772_CMM_PPCOUNT;    
    databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
    databuf[0] = TMD2772_CMM_CONTROL;    
    databuf[1] = TMD2772_CMM_CONTROL_VALUE;
    res = i2c_master_send(client, databuf, 0x2);
    
    if(res <= 0){
        goto EXIT_ERR;
    }

    if(0 == obj->hw->polling_mode_ps){
        databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
        databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        
        if(res <= 0){
            goto EXIT_ERR;
        }
        
        databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
        res = i2c_master_send(client, databuf, 0x2);
        
        if(res <= 0){
            goto EXIT_ERR;
        }
        
        databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
        databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        
        if(res <= 0){
            goto EXIT_ERR;
        }
        
        databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;	
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);;
        res = i2c_master_send(client, databuf, 0x2);
        
        if(res <= 0){
            goto EXIT_ERR;
        }
    }
    
    /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
    if((res = TMD2772_setup_eint(client))!=0){
        APS_ERR("setup eint: %d\n", res);
        return res;
    }
    
    if((res = TMD2772_check_and_clear_intr(client))){
        APS_ERR("check/clear intr: %d\n", res);
    }

    return TMD2772_SUCCESS;

    EXIT_ERR:
        APS_ERR("init dev: %d\n", res);
        return res;
    }

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int TMD2772_read_als(struct i2c_client *client, u16 *data)
{
	struct TMD2772_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
//	u16 als_value;
	int res = 0;
//	u8 reg_value[1];
       
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	/*debug tag for yucong*/
	#if 0
	buffer[0]=TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
	#endif
    
    psensor_mutex_lock(&sensor_mutex);
    
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_read_als is called\n");
    #endif
        
//get adc channel 0 value
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_L = 0x%x\n", als_value_low[0]);

	buffer[0]=TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_H = 0x%x\n", als_value_high[0]);
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

//get adc channel 1 value
	buffer[0]=TMD2772_CMM_C1DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_L = 0x%x\n", als_value_low[0]);	

	buffer[0]=TMD2772_CMM_C1DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_H = 0x%x\n", als_value_high[0]);	

	c1_value = als_value_low[0] | (als_value_high[0]<<8);
	c1_nf = obj->als_modulus*c1_value;	
	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;

	//APS_DBG("atio = %d\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
        {
                *data = 0;
        }
        else
	{
		APS_DBG("als_value is invalid!!\n");
        psensor_mutex_unlock(&sensor_mutex);
		return -1;
	}
    
	APS_DBG("als_value_lux = %d\n", *data);
	//printk("yucong: als_value_lux = %d\n", *data);
	psensor_mutex_unlock(&sensor_mutex);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("TMD2772_read_ps fail\n");
    psensor_mutex_unlock(&sensor_mutex);
	return res;
}
int TMD2772_read_als_ch0(struct i2c_client *client, u16 *data)
{
//	struct TMD2772_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value;	 
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
      
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
    psensor_mutex_lock(&sensor_mutex);
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_read_als_ch0 is called\n");
    #endif
//get adc channel 0 value
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{      
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{       	
		goto EXIT_ERR;
	}
	
	buffer[0]=TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{    
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{  
		goto EXIT_ERR;
	}
	
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	*data = c0_value;
    psensor_mutex_unlock(&sensor_mutex);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("TMD2772_read_ps fail\n");
    psensor_mutex_unlock(&sensor_mutex);
	return res;
}
/*----------------------------------------------------------------------------*/

static int TMD2772_get_als_value(struct TMD2772_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/

static int TMD2772_get_ps_value(struct TMD2772_priv *obj, u16 ps)
{
    int val;
    int invalid = 0;
    static int val_temp=1;
    
    APS_LOG("TMD2772_get_ps_value:  ps_thd_val_high = %d  ps_thd_val_low = %d  ps = %d\n" ,
                    atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low), ps);

    if((ps >atomic_read(&obj->ps_thd_val_high))){
        val = 0;  /*close*/
        val_temp = 0;
        intr_flag_value = 1;
    }
    else if((ps < atomic_read(&obj->ps_thd_val_low))){
        val = 1;  /*far away*/
        val_temp = 1;
        intr_flag_value = 0;
    }
    else{
        val = val_temp;
    }

    if(atomic_read(&obj->ps_suspend)){
        invalid = 1;
    }
    else if(1 == atomic_read(&obj->ps_deb_on)){
        unsigned long endt = atomic_read(&obj->ps_deb_end);
        if(time_after(jiffies, endt)){
        	atomic_set(&obj->ps_deb_on, 0);
        }

        if (1 == atomic_read(&obj->ps_deb_on)){
        	invalid = 1;
        }
    }
    else if (obj->als > 45000){
        APS_DBG("ligh too high will result to failt proximiy\n");
        return 1;  /*far away*/
    }

    if(!invalid){
        return val;
    }	
    else{
        return -1;
    }	
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void TMD2772_eint_work(struct work_struct *work)
{
    struct TMD2772_priv *obj = (struct TMD2772_priv *)container_of(work, struct TMD2772_priv, eint_work);
    int err;
    hwm_sensor_data sensor_data;
    u8 databuf[2];
    int res = 0;

    if((err = TMD2772_check_intr(obj->client)))
    {
        APS_ERR("TMD2772_eint_work check intrs: %d\n", err);
    }
    else
    {
        //get raw data
        TMD2772_read_ps(obj->client, &obj->ps);
        //mdelay(160);
        TMD2772_read_als_ch0(obj->client, &obj->als);
    #ifdef TMD2772_DEBUG
        APS_DBG("TMD2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
    #endif
        sensor_data.values[0] = TMD2772_get_ps_value(obj, obj->ps);
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
                
        /*singal interrupt function add*/
        if(intr_flag_value){
        #ifdef TMD2772_DEBUG     
            APS_DBG("TMD2772_eint_work is called in yes!\n");
        #endif
            
            databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
            databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);

            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
                
            if(res <= 0){
                return;
            }

            databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
            databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);

            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(obj->client, databuf, 0x2);

            psensor_mutex_unlock(&sensor_mutex);

            if(res <= 0){
                return;
            }

            databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
            databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);

            psensor_mutex_lock(&sensor_mutex); 
			res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);

            if(res <= 0){
                return;
            }
                
            databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
            databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);
                
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
                
            if(res <= 0){
                return;
            }
        }
        else{	
            databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
            databuf[1] = (u8)(0 & 0x00FF);
            psensor_mutex_lock(&sensor_mutex);
            APS_DBG("TMD2772_eint_work is called in no!\n");
            res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
            
            if(res <= 0)
            {
                return;
            }
            
            databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
            databuf[1] = (u8)((0 & 0xFF00) >> 8);
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
            
            if(res <= 0)
            {
                return;
            }
            
            databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
            databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
            
            if(res <= 0)
            {
            	    return;
            }
            
            databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
            databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
            psensor_mutex_lock(&sensor_mutex);
            res = i2c_master_send(obj->client, databuf, 0x2);
            psensor_mutex_unlock(&sensor_mutex);
            
            if(res <= 0)
            {
                return;
            }
        }

        //let up layer to know
        if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
        {
          APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
        }
    }
    
    TMD2772_clear_intr(obj->client);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);     
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int TMD2772_open(struct inode *inode, struct file *file)
{
	file->private_data = TMD2772_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int TMD2772_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

#if 0
static void TMD2772_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{

	   APS_LOG("TMD2772_WriteCalibration  1 %d," ,data_cali->close);
		   APS_LOG("TMD2772_WriteCalibration  2 %d," ,data_cali->far_away);
		   APS_LOG("TMD2772_WriteCalibration  3 %d,", data_cali->valid);
		   
	  if(data_cali->valid == 1)
	  {
	      if(data_cali->close < 100)
	      	{
		  	ps_cali.close = 200;
			ps_cali.far_away= 150;
			ps_cali.valid = 1;
	      	}
		  else if(data_cali->close > 900)
		  {
		  	ps_cali.close = 900;
			ps_cali.far_away= 750;
			ps_cali.valid = 1;
	      	}
		  else
		  {
			  ps_cali.close = data_cali->close;
			ps_cali.far_away= data_cali->far_away;
			ps_cali.valid = 1;
		  }
	  }
	  

}
#endif

#if 0
static int TMD2772_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0;
	 u16 data[21],sum,data_cali;

	 for(i = 0;i<20;i++)
	 	{
	 		mdelay(5);//50
			if(err = TMD2772_read_ps(client,&data[i]))
			{
				APS_ERR("TMD2772_read_data_for_cali fail: %d\n", i); 
				break;
			}
			else
				{
					sum += data[i];
			}
			mdelay(55);//160
	 	}
	 
	 for(j = 0;j<20;j++)
	 	APS_LOG("%d\t",data[j]);
	 
	 if(i == 20)
	 	{
			data_cali = sum/20;
			APS_LOG("TMD2772_read_data_for_cali data = %d",data_cali);
			if(data_cali>600)
			return -1;
			if(data_cali<=100)
			{
				ps_data_cali->close =data_cali*22/10;
				ps_data_cali->far_away = data_cali*19/10;
				ps_data_cali->valid =1;
			}
			else if(100<data_cali&&data_cali<300)
			{
				ps_data_cali->close = data_cali*2;
				ps_data_cali->far_away =data_cali*17/10;
				ps_data_cali->valid = 1;
			}
			else
			{
				ps_data_cali->close = data_cali*18/10;
				ps_data_cali->far_away =data_cali*15/10;
				ps_data_cali->valid = 1;
			}
		        if(ps_data_cali->close > 900)
		       {
		  	ps_data_cali->close = 900;
			ps_data_cali->far_away = 750;
			err= 0;
	         	}
			else  if(ps_data_cali->close < 100)
			{
			   ps_data_cali->close = 200;
			   ps_data_cali->far_away = 150;
			   err= 0;
			}

			ps_cali.close = ps_data_cali->close;
			ps_cali.far_away= ps_data_cali->far_away;
			ps_cali.valid = 1;
			APS_LOG("TMD2772_read_data_for_cali close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
	
	 	}
	 else
	 	{
	 	ps_data_cali->valid = 0;
	 	err=  -1;
	 	}
	 return err;
	 	

}
#endif

/*----------------------------------------------------------------------------*/
static long TMD2772_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct TMD2772_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	//struct PS_CALI_DATA_STRUCT ps_cali_temp;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = TMD2772_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = TMD2772_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = TMD2772_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = TMD2772_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = TMD2772_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = TMD2772_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = TMD2772_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = TMD2772_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = TMD2772_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = TMD2772_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

/*		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			TMD2772_WriteCalibration(&ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			break;
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			TMD2772_init_client_for_cali(obj->client);
			err = TMD2772_read_data_for_cali(obj->client,&ps_cali_temp);
			if(err)
			{
			   goto err_out;
			}
			TMD2772_init_client(obj->client);
			// TMD2772_enable_ps(obj->client, 1);
			TMD2772_enable(obj->client, 0);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*the function is used to read the p-sensor pulse count register to get the pulse count*/
int TMD2772_read_pulse_count(struct i2c_client *client, u8 *data)

{   
        int res=0;
        u8 buffer[1];
        u8 pulse_count[1];

       
    	if(client == NULL)

	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;

	}   
    //get pulse count
        psensor_mutex_lock(&sensor_mutex);
        APS_DBG("TMD2772_read_pulse_count is called\n");
        buffer[0]=TMD2772_CMM_PPCOUNT;
        res = i2c_master_send(client, buffer, 0x1);
        APS_DBG("send is over!res=%d\n,",buffer[0]);
        
        if(res <= 0)
	{
	    goto EXIT_ERR;
	}
        res = i2c_master_recv(client, pulse_count, 0x1);
        //APS_DBG("TMD2772_read_pulse_count is over!res=%d,pulse_count[0]=%d",res,pulse_count[0]);
        if(res <= 0)
	{
	    goto EXIT_ERR;
	}
        *data=pulse_count[0];
        psensor_mutex_unlock(&sensor_mutex);
        #ifdef TMD2772_DEBUG
        APS_DBG("TMD2772_read_pulse_count is over!res=%d,data=%d",res,*data);
        #endif
        return 0;

        EXIT_ERR:

	APS_ERR("TMD2772_read_pulse_count fail\n");
        psensor_mutex_unlock(&sensor_mutex);

	return res;
}

/*the function is used to write the p-sensor pulse count register*/
int TMD2772_write_pulse_count(struct i2c_client *client, u8 *data)

{
        int res=0;
        u8 databuf[2];
        int debug_res=0;
        u8 debug_pulse[1];
        u8 buf[1];
    	if(client == NULL)

	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;

	}  
        psensor_mutex_lock(&sensor_mutex);
        databuf[0]=TMD2772_CMM_PPCOUNT;
        databuf[1]=*data;
        res = i2c_master_send(client, databuf, 0x2);
        //APS_DBG("TMD2772_write_pulse_count is over!res=%d,databuf[1]=%d",res,databuf[1]);
        if(res <= 0)
	{
	    goto EXIT_ERR;
	}
        psensor_mutex_unlock(&sensor_mutex);
        return 0;
        
EXIT_ERR:

	APS_ERR("TMD2772_write_pulse_count fail\n");
        psensor_mutex_unlock(&sensor_mutex);

	return res;
  
    }

static ssize_t TMD2772_pulse_count_prx_show(struct device_driver *ddri, char *buf)

{
    	int res=0;
        U8 prx_pulse_count=0;
        
	if(!g_TMD2772_ptr)

	{
		APS_ERR("g_TMD2772_ptr is null!!\n");
		return 0;
	}
        /*read the p-sensor pulse count*/
        if((res=TMD2772_read_pulse_count(g_TMD2772_ptr->client,&prx_pulse_count)))
        {
            return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
           
        }
        else
        {  
            #ifdef TMD2772_DEBUG
            APS_DBG("TMD2772_pulse_count_prx_show2 is called!res=%d,prx_pulse_count=%d",res,prx_pulse_count);
            #endif
            return snprintf(buf, PAGE_SIZE, "%d\n", prx_pulse_count);
        }
   // return snprintf(buf, PAGE_SIZE, "%d\n", TMD2772_CMM_PPCOUNT_VALUE);
    
    }

static ssize_t TMD2772_pulse_count_prx_store(struct device_driver *ddri, const char *buf,ssize_t size)

{
        int res=0;
        int rc=0;
        u8 pulse_count_prx=0;

    	if(!g_TMD2772_ptr)

	{
		APS_ERR("g_TMD2772_ptr is null!!\n");
		return 0;
	}
        
        rc=kstrtou8(buf,10,&pulse_count_prx);
        //APS_DBG("TMD2772_pulse_count_prx_store is called!rc=%d,pulse_count_prx=%d",rc,pulse_count_prx);
        if(rc)
        {
            APS_ERR("TMD2772_pulse_count_prx_store fail!!\n");
            return -1;
        }
        //set pulse count
         if((res=TMD2772_write_pulse_count(g_TMD2772_ptr->client,&pulse_count_prx)))
        {
            return snprintf(pulse_count_prx, PAGE_SIZE, "ERROR: %d\n", res);
        }
        else
        {
            return size;
        }
        
    }

/*read Proximity Offset Register to get the offset*/
int  TMD2772_prx_offset_get(struct i2c_client *client, u8 *data)
{
    int res=0;
    u8 prx_offset[1];
    u8 buffer[1];
    
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
	return -1;
    }  
    psensor_mutex_lock(&sensor_mutex);
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_prx_offset_get is called\n");
    #endif
    buffer[0]=TMD2772_CMM_OFFSET;
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    res = i2c_master_recv(client, prx_offset, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    *data=prx_offset[0];
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_prx_offset_get is over res=%d,data=%d\n",res,*data);
    #endif
    psensor_mutex_unlock(&sensor_mutex);
    return 0;

EXIT_ERR:

    APS_ERR("TMD2772_prx_offset_get fail\n");
    psensor_mutex_unlock(&sensor_mutex);

    return res;
    
}

/*write data to Proximity Offset Register */
int  TMD2772_prx_offset_set(struct i2c_client *client, u8 *data)
{
    int res=0;
    u8 databuf[2];
    
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
	return -1;
    }  
    psensor_mutex_lock(&sensor_mutex);
    databuf[0]=TMD2772_CMM_OFFSET;
    databuf[1]=*data;
    res = i2c_master_send(client, databuf, 0x2);
    //APS_DBG("TMD2772_prx_offset_set is over!res=%d,databuf[1]=%d",res,databuf[1]);
    if(res<=0)
    {
        goto EXIT_ERR;
    }
    psensor_mutex_unlock(&sensor_mutex);
    return 0;
    
EXIT_ERR:

    APS_ERR("TMD2772_prx_offset_set fail\n");
    psensor_mutex_unlock(&sensor_mutex);
    return res;
}


static ssize_t TMD2772_offset_prx_show(struct device_driver *ddri, char *buf)

{
    int res=0;
    u8 prx_offset=0;
    
    if(!g_TMD2772_ptr)
    {
    	APS_ERR("g_TMD2772_ptr is null!!\n");
    	return 0;
    }
    //set Proximity offset
    if((res=TMD2772_prx_offset_get(g_TMD2772_ptr->client,&prx_offset)))
    {
          return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        return snprintf(buf, PAGE_SIZE, "%d\n", prx_offset);
    }
}

static ssize_t TMD2772_offset_prx_store(struct device_driver *ddri, const char *buf,ssize_t size)
{
    int res;
    u8 offset=0;
    int rc;
    
    if(!g_TMD2772_ptr)

    {
	APS_ERR("g_TMD2772_ptr is null!!\n");
	return 0;
    }

    rc=kstrtou8(buf,10,&offset);
    //APS_DBG("TMD2772_offset_prx_store is called!rc=%d,offset=%d",rc,offset);
   
    if(rc)
    {
        APS_ERR("TMD2772_offset_prx_store fail!!\n");
        return -1;
    }
    
    if((res=TMD2772_prx_offset_set(g_TMD2772_ptr->client,&offset)))
    {
        return snprintf(offset, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        return size;
    }
}
//show the raw data
static ssize_t TMD2772_show_ps(struct device_driver *ddri, char *buf)

{
	ssize_t res;

	if(!g_TMD2772_ptr)

	{
	    APS_ERR("g_TMD2772_ptr is null!!\n");
	    return 0;
	}
        //get the raw data
	if((res = TMD2772_read_ps(g_TMD2772_ptr->client, &g_TMD2772_ptr->ps)))

	{
	    return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
	    return snprintf(buf, PAGE_SIZE, "%d\n", g_TMD2772_ptr->ps);     
	}

}

/*Read the Control register to get the data*/
int  TMD2772_control_prx_get(struct i2c_client *client, u8 *data)
{
    int res=0;
    u8 control_prx[1];
    u8 buffer[1];

    
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
	return -1;
    }
    psensor_mutex_lock(&sensor_mutex);
    APS_DBG("TMD2772_control_prx_get is called\n");
    buffer[0]=TMD2772_CMM_CONTROL;
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    res = i2c_master_recv(client, control_prx, 0x1);
    
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    *data=control_prx[0];
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_control_prx_get is over!res=%d,data=%d",res,*data);
    #endif
    psensor_mutex_unlock(&sensor_mutex);
    return 0;
    
EXIT_ERR:
    
        APS_ERR("TMD2772_control_prx_get fail\n");
        psensor_mutex_unlock(&sensor_mutex);
        return res;
    
}

/*set the Control register*/
int  TMD2772_control_prx_set(struct i2c_client *client, u8 *data)
{
    int res=0;
    u8 databuf[2];
    
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
	return -1;
    }  
    psensor_mutex_lock(&sensor_mutex);
    databuf[0]=TMD2772_CMM_CONTROL;
    databuf[1]=*data;
    res = i2c_master_send(client, databuf, 0x2);
    //APS_DBG("TMD2772_control_prx_set is over!res=%d,databuf[1]=%d",res,databuf[1]);
    if(res<=0)
    {
        goto EXIT_ERR;
    }
    psensor_mutex_unlock(&sensor_mutex);
    return 0;
    
EXIT_ERR:

    APS_ERR("TMD2772_control_prx_set fail\n");
    psensor_mutex_unlock(&sensor_mutex);
    return res;
}


static ssize_t TMD2772_control_prx_show(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    u8 control_prx;
    
    if(!g_TMD2772_ptr)

    {
        APS_ERR("g_TMD2772_ptr is null!!\n");
        return 0;
    }
    
    if((res = TMD2772_control_prx_get(g_TMD2772_ptr->client, &control_prx)))

    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        return snprintf(buf, PAGE_SIZE, "%d\n", control_prx);     
    }
    
}

static ssize_t TMD2772_control_prx_store(struct device_driver *ddri, const char *buf,ssize_t size)
{
    ssize_t res;
    u8 control_prx;
    int rc;
    
    if(!g_TMD2772_ptr)

    {
        APS_ERR("g_TMD2772_ptr is null!!\n");
        return 0;
    }

    rc=kstrtou8(buf,10,&control_prx);
    APS_DBG("TMD2772_control_prx_store is called!rc=%d,control_prx=%d",rc,control_prx);
    if(rc)
    {
        APS_ERR("TMD2772_control_prx_store fail!!\n");
        return -1;
    }
    
    if((res=TMD2772_control_prx_set(g_TMD2772_ptr->client,&control_prx)))
    {
        return snprintf(control_prx, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        return size;
    }
}

long TMD2772_read_threshold(struct i2c_client *client, u16 *low_threshold,u16*high_threshold)
{
    u8 low_threshold_low[1], low_threshold_high[1],high_threshold_low[1],high_threshold_high[1];
    u8 buffer[1];
    long res = 0;
        
    if(client == NULL){
    	APS_DBG("CLIENT CANN'T EQUL NULL\n");
    	return -1;
    }
    
    psensor_mutex_lock(&sensor_mutex);
    buffer[0]=TMD2772_CMM_INT_LOW_THD_LOW;
    res = i2c_master_send(client, buffer, 0x1);
    
    if(res <= 0){
    	goto EXIT_ERR;
    }
    
    res = i2c_master_recv(client, low_threshold_low, 0x1);
    
    if(res <= 0){
    	goto EXIT_ERR;
    }
    
    buffer[0]=TMD2772_CMM_INT_LOW_THD_HIGH;
    res = i2c_master_send(client, buffer, 0x1);
    
    if(res <= 0){       
    	goto EXIT_ERR;
    }
    
    res = i2c_master_recv(client, low_threshold_high, 0x01);
    
    if(res <= 0){       
    	goto EXIT_ERR;
    }

    *low_threshold = low_threshold_low[0] | (low_threshold_high[0]<<8);

    buffer[0]=TMD2772_CMM_INT_HIGH_THD_LOW;
    
    res = i2c_master_send(client, buffer, 0x1);
    
    if(res <= 0){
    	goto EXIT_ERR;
    }
    
    res = i2c_master_recv(client, high_threshold_low, 0x1);
    
    if(res <= 0){
    	goto EXIT_ERR;
    }
    
    buffer[0]=TMD2772_CMM_INT_HIGH_THD_HIGH;
    res = i2c_master_send(client, buffer, 0x1);
    
    if(res <= 0){       
    	goto EXIT_ERR;
    }
    
    res = i2c_master_recv(client, high_threshold_high, 0x01);
    
    if(res <= 0){       
    	goto EXIT_ERR;
    }

    *high_threshold = high_threshold_low[0] | (high_threshold_high[0]<<8);
    
    #ifdef TMD2772_DEBUG
    APS_DBG("high_threshold_low[0]=%d,high_threshold_high[0]=%d",high_threshold_low[0] ,high_threshold_low[0]);
    #endif      
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_read_threshold is called!low_threshold=%d,high_threshold=%d",*low_threshold ,*high_threshold);
    #endif        
    psensor_mutex_unlock(&sensor_mutex);
    return 0;    

    EXIT_ERR:
        APS_ERR("TMD2772_read_threshold fail\n");
        psensor_mutex_unlock(&sensor_mutex);
    return res;
}

#if 0
int TMD2772_threshold_prx_set(struct i2c_client *client, u16 *low_threshold,u16*high_threshold)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    u16 prox_threshold_hi = 0, prox_threshold_lo = 0;

    prox_threshold_hi=*high_threshold;
    prox_threshold_lo=*low_threshold;

    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_threshold_prx_set is called!low_threshold=%d,high_threshold=%d",prox_threshold_lo ,prox_threshold_hi);
    #endif

     atomic_set(&obj->ps_thd_val_high, prox_threshold_hi);
     atomic_set(&obj->ps_thd_val_low,  prox_threshold_lo);

     return 0;
    
}
#endif

static ssize_t TMD2772_threshold_prx_show(struct device_driver *ddri, char *buf)
{
    //int res=0;
    u16 low_threshold=0;
    u16 high_threshold=0;

    if(!g_TMD2772_ptr){
        APS_ERR("g_TMD2772_ptr is null!!\n");
        return 0;
    }

    return snprintf(buf, PAGE_SIZE, "low_threshold=%d,  high_threshold=%d  \n", 
                         atomic_read(&g_TMD2772_ptr->ps_thd_val_low), atomic_read(&g_TMD2772_ptr->ps_thd_val_high));

    #if 0
    /*read threshold*/
    if((res=TMD2772_read_threshold(g_TMD2772_ptr->client,&low_threshold,&high_threshold))){
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);       
    }
    else{  
        return snprintf(buf, PAGE_SIZE, "low_threshold=%d, high_threshold=%d\n", low_threshold,high_threshold);
    }
    #endif
}

#if 0
static ssize_t TMD2772_threshold_prx_store(struct device_driver *ddri, const char *buf1,const char *buf2,ssize_t size)
{
    ssize_t res=0;
    u16 threshold_high=0;
    u16 threshold_low=0;
    int rc=0;
    if(!g_TMD2772_ptr)

    {
        APS_ERR("g_TMD2772_ptr is null!!\n");
        return 0;
    }
    
    if(buf1!=null)
    {
    
        rc=kstrtou16(buf1,10,&threshold_low);
        #ifdef TMD2772_DEBUG
        APS_DBG("TMD2772_threshold_prx_store is called!rc=%d,threshold_high=%d",rc,threshold_high);
        #endif
        if(rc)
        {
            APS_ERR("TMD2772_threshold_prx_store fail!!\n");
            return -1;
        }
    }
    rc=kstrtou16(buf2,10,&threshold_high);
    
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_threshold_prx_store is called!rc=%d,threshold_high=%d",rc,threshold_high);
    #endif
    if(rc)
    {
        APS_ERR("TMD2772_threshold_prx_store fail!!\n");
        return -1;
    }
    /*store the threshold*/
     if((res=TMD2772_threshold_prx_set(g_TMD2772_ptr->client,&threshold_low,&threshold_high)))
    {
        return snprintf(threshold_low, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        return size;
    }
    
}
#endif
int  TMD2772_config_prx_set(struct i2c_client *client, u8 *data)
{
    int res=0;
    u8 databuf[2];
    
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
	return -1;
    }  
    psensor_mutex_lock(&sensor_mutex);
    databuf[0]=TMD2772_CMM_CONFIG;
    databuf[1]=*data;
    res = i2c_master_send(client, databuf, 0x2);
    //APS_DBG("TMD2772_control_prx_set is over!res=%d,databuf[1]=%d",res,databuf[1]);
    if(res<=0)
    {
        goto EXIT_ERR;
    }
    psensor_mutex_unlock(&sensor_mutex);
    return 0;
    
EXIT_ERR:

    APS_ERR("TMD2772_config_prx_set fail\n");
    psensor_mutex_unlock(&sensor_mutex);
    return res;
}

int TMD2772_read_config(struct i2c_client *client, u8 *config_data)
{
    u8 cdata[1];
    int res=0;
    u8 buffer[1];

    if(client == NULL)
    {
    	APS_DBG("CLIENT CANN'T EQUL NULL\n");
    	return -1;
    }
    psensor_mutex_lock(&sensor_mutex);
    buffer[0]=TMD2772_CMM_CONFIG;
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
    	goto EXIT_ERR;
    }
    res = i2c_master_recv(client, cdata, 0x1);
    if(res <= 0)
    {
    	goto EXIT_ERR;
    }
    
    *config_data=cdata[0];
    
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_read_config is called!config_data=%d\n,",*config_data);
    #endif        
    psensor_mutex_unlock(&sensor_mutex);
    return 0; 
EXIT_ERR:
    APS_ERR("TMD2772_read_config fail\n");
    psensor_mutex_unlock(&sensor_mutex);
    return res;
}

static ssize_t TMD2772_config_prx_store(struct device_driver *ddri, const char *buf,ssize_t size)
{
    ssize_t res;
    u8 config_prx;
    int rc;
    
    if(!g_TMD2772_ptr)

    {
        APS_ERR("g_TMD2772_ptr is null!!\n");
        return 0;
    }

    rc=kstrtou8(buf,10,&config_prx);
    #ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_config_prx_store is called!rc=%d,config_prx=%d",rc,config_prx);
    #endif
    if(rc)
    {
        APS_ERR("TMD2772_config_prx_store fail!!\n");
        return -1;
    }
    
    if((res=TMD2772_config_prx_set(g_TMD2772_ptr->client,&config_prx)))
    {
        return snprintf(config_prx, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        return size;
    }
}


/*add the config read and write interface to change the drive*/
static ssize_t TMD2772_config_prx_show(struct device_driver *ddri, char *buf)
{
    int res=0;
    u8 config_data=0;
    
    if(!g_TMD2772_ptr)

    {
    	APS_ERR("g_TMD2772_ptr is null!!\n");
    	return 0;
    }
    /*read the config data*/
    if((res=TMD2772_read_config(g_TMD2772_ptr->client,&config_data)))
    {    
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);       
    }
    else
    {  
        return snprintf(buf, PAGE_SIZE, "%d\n",config_data );
    }
    
}

static ssize_t TMD2772_show_als(struct device_driver *ddri, char *buf)
{
    int res=0;

    if(!g_TMD2772_ptr){
        APS_ERR("TMD2772_show_als:g_TMD2772_ptr is null!!\n");
        return 0;
    }
    //get the raw data
    if((res = TMD2772_read_als(g_TMD2772_ptr->client, &g_TMD2772_ptr->als))){
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else{
        return snprintf(buf, PAGE_SIZE, "%d\n", g_TMD2772_ptr->als);     
    }
}

static DRIVER_ATTR(pulse_count_prx,  S_IWUSR | S_IRUGO, TMD2772_pulse_count_prx_show,TMD2772_pulse_count_prx_store);
static DRIVER_ATTR(offset_prx, S_IWUSR | S_IRUGO, TMD2772_offset_prx_show, TMD2772_offset_prx_store);
static DRIVER_ATTR(ps_prx,      S_IWUSR | S_IRUGO, TMD2772_show_ps,    NULL);
static DRIVER_ATTR(control_prx,S_IWUSR | S_IRUGO,TMD2772_control_prx_show,TMD2772_control_prx_store);
static DRIVER_ATTR(threshold_prx,S_IWUSR | S_IRUGO,TMD2772_threshold_prx_show,NULL);
static DRIVER_ATTR(config_prx, S_IWUSR | S_IRUGO,TMD2772_config_prx_show,TMD2772_config_prx_store);
static DRIVER_ATTR(als, S_IWUSR | S_IRUGO,TMD2772_show_als,NULL);


static struct driver_attribute *TMD2772_attr_list[] = {

    &driver_attr_pulse_count_prx,
    &driver_attr_offset_prx,
    &driver_attr_ps_prx,
    &driver_attr_control_prx,
    &driver_attr_threshold_prx,
    &driver_attr_config_prx,
    &driver_attr_als
};

/*the function is used to create the driver attribute file,the file is 
  in  /sys/bus/platform/drivers/als_ps/ directory                        */
static int TMD2772_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(TMD2772_attr_list)/sizeof(TMD2772_attr_list[0]));

    if (driver == NULL){
        return -EINVAL;
    }
    
    for(idx = 0; idx < num; idx++){  
        if((err = driver_create_file(driver, TMD2772_attr_list[idx]))){            
    	    APS_ERR("driver_create_file (%s) = %d\n", TMD2772_attr_list[idx]->attr.name, err);
        }
    }    

    return err;
}

/*----------------------------------------------------------------------------*/

/*the function is used to delete the created driver attribute file*/
static int TMD2772_delete_attr(struct device_driver *driver)

{

//         JOHN_LOG();

	int idx ,err = 0;
	int num = (int)(sizeof(TMD2772_attr_list)/sizeof(TMD2772_attr_list[0]));

	if (!driver)
        
	return -EINVAL;
    
	for (idx = 0; idx < num; idx++) 

	{
	    driver_remove_file(driver, TMD2772_attr_list[idx]);
	}

	return err;

}


/*----------------------------------------------------------------------------*/
static struct file_operations TMD2772_fops = {
	.owner = THIS_MODULE,
	.open = TMD2772_open,
	.release = TMD2772_release,
	.unlocked_ioctl = TMD2772_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice TMD2772_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &TMD2772_fops,
};
/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//struct TMD2772_priv *obj = i2c_get_clientdata(client);    
	//int err;
	APS_FUN();    
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = TMD2772_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = TMD2772_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		TMD2772_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_resume(struct i2c_client *client)
{
	//struct TMD2772_priv *obj = i2c_get_clientdata(client);        
	//int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	TMD2772_power(obj->hw, 1);
	if(err = TMD2772_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = TMD2772_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = TMD2772_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void TMD2772_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/

	struct TMD2772_priv *obj = container_of(h, struct TMD2772_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = TMD2772_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
  
}
/*----------------------------------------------------------------------------*/
static void TMD2772_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	
	struct TMD2772_priv *obj = container_of(h, struct TMD2772_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = TMD2772_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
	
}

int TMD2772_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct TMD2772_priv *obj = (struct TMD2772_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if((err = TMD2772_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 0	
					if(err = TMD2772_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
				else
				{
					if((err = TMD2772_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 0
					if(err = TMD2772_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				TMD2772_read_ps(obj->client, &obj->ps);
				
                                //mdelay(160);
				TMD2772_read_als_ch0(obj->client, &obj->als);
				APS_ERR("TMD2772_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = TMD2772_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

static int temp_als = 0;
int TMD2772_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct TMD2772_priv *obj = (struct TMD2772_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{      
				//mutex_unlock(&sensor_mutex);
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = TMD2772_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err);
                        // mutex_unlock(&sensor_mutex);
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
                    // mutex_unlock(&sensor_mutex);
				}
				else
				{
					if((err = TMD2772_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing know issue*/
				#if 1
				TMD2772_read_als(obj->client, &obj->als);
				if(obj->als == 0)
				{
					sensor_data->values[0] = -1;				
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
					TMD2772_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = TMD2772_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, TMD2772_DEV_NAME);
	return 0;
}

static int TMD2772_set_threshold_value(struct i2c_client *client)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    u8 databuf[2];    
    int res = 0;

#ifdef TMD2772_DEBUG
    APS_DBG("TMD2772_set_threshold_value is called!");
    APS_DBG("TMD2772_ps_calibrate: ps_thd_val_low = %d\n",  atomic_read(&obj->ps_thd_val_low)); 
    APS_DBG("TMD2772_ps_calibrate: ps_thd_val_high = %d\n", atomic_read(&obj->ps_thd_val_high)); 
#endif
    databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
    databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);

    psensor_mutex_lock(&sensor_mutex);
    res = i2c_master_send(client, databuf, 0x2);
    psensor_mutex_unlock(&sensor_mutex);
        
    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
    databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);

    psensor_mutex_lock(&sensor_mutex);
    res = i2c_master_send(client, databuf, 0x2);
    psensor_mutex_unlock(&sensor_mutex);

    if(res <= 0){
        goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
    databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);

    psensor_mutex_lock(&sensor_mutex);
    res = i2c_master_send(client, databuf, 0x2);
    psensor_mutex_unlock(&sensor_mutex);

    if(res <= 0){
        goto EXIT_ERR;
    }
        
    databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
    databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);
        
    psensor_mutex_lock(&sensor_mutex);
    res = i2c_master_send(client, databuf, 0x2);
    psensor_mutex_unlock(&sensor_mutex);
        
    if(res <= 0){
        goto EXIT_ERR;
    }	

    return TMD2772_SUCCESS;

EXIT_ERR:
    APS_ERR(": TMD2772_set_threshold_value is error : res = %d\n", res);
    return res; 
}

//Added by jrd.lipeng for boot-up calibration
/*----------------------------------------------------------------------------*/
/* when do boot-up calibration,  TMD2772_init_client_for_cali() changed some regs values which 
  * were set by TMD2772_init_client() before. So when calibration completes, call this func to restore
   * those regs values.*/
static int TMD2772_init_client_for_cali_restore(struct i2c_client *client)
{
    u8 databuf[2];    
    int res = 0;

    databuf[0] = TMD2772_CMM_ATIME;    
    databuf[1] = 0xC9;
    res = i2c_master_send(client, databuf, 0x2);
    if(res <= 0)
    {
    	goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_WTIME;    
    databuf[1] = 0xEE;
    res = i2c_master_send(client, databuf, 0x2);
    if(res <= 0)
    {
    	goto EXIT_ERR;
    }

    databuf[0] = TMD2772_CMM_ENABLE;    
    databuf[1] = 0x00;
    res = i2c_master_send(client, databuf, 0x2);
    if(res <= 0)
    {
    	goto EXIT_ERR;
    }

    return TMD2772_SUCCESS;

EXIT_ERR:
    APS_ERR("TMD2772_init_client_for_cali_restore error: res = %d\n", res);
    return res;
}

static void TMD2772_ps_calibrate(struct i2c_client *client)
{
    struct TMD2772_priv *obj = i2c_get_clientdata(client);
    int prox_sum = 0, prox_mean = 0, prox_max = 0, prox_min =1023;
    int prox_threshold_hi = 0, prox_threshold_lo = 0;
    int i, ret = 0, invalid = 0;
    u16 prox_data[PS_GET_NUM];
    
    APS_DBG("TMD2772_ps_calibrate is called!");

#if 0 //def TMD2772_DEBUG
    struct timeval starttime,endtime;
    int64_t usedtime;
    do_gettimeofday(&starttime);
#endif

    msleep(50);//add delay time,avoid get the sensor value is 0.
    
    for(i = 0; i < PS_GET_NUM; i++){
        if(ret = TMD2772_read_ps(client, &prox_data[i])){
        	APS_ERR("TMD2772_ps_calibrate: TMD2772_read_ps fail: %d\n", i);
        	return ret;
        }

        if(0 == prox_data[i]){
            invalid++;
            APS_DBG("TMD2772_ps_calibration:invalid = %d, i = %d\n", invalid, i);
        }
        else{                        
            if (prox_data[i] > prox_max)
                prox_max = prox_data[i]; //get the max value
            
            if (prox_data[i] < prox_min)
                prox_min = prox_data[i]; //get the min value      
                
            prox_sum += prox_data[i];          
 	      APS_DBG("the prox_data[%d]=%d\n",i,prox_data[i]);
        }
	  msleep(10);
    }
    
    if(PS_GET_NUM != invalid){
        prox_mean = prox_sum /(PS_GET_NUM - invalid);
    }
    else
    {
        prox_mean = 1023;
    }

    ps_cali.valid = 1;
    
    APS_DBG("TMD2772_ps_calibration:prox_min = %d, prox_max = %d, the prox_mean = %d, invalid = %d\n", 
                    prox_min, prox_max, prox_mean, invalid);

    /*when the prox_mean is so small,set the threshold is a fix value.by pcyang 2013.01.23*/
    if(prox_mean<5){
        prox_threshold_hi = 120;
        prox_threshold_lo = 40;
    }
    else if(prox_mean<40){
        prox_threshold_hi = prox_mean*8;
        prox_threshold_lo = prox_mean*2;
    }
    else if(prox_mean<45){
        prox_threshold_hi = prox_mean*75/10;
        prox_threshold_lo = prox_mean*19/10;
    }
    else if(prox_mean<50){
        prox_threshold_hi = prox_mean*7;
        prox_threshold_lo = prox_mean*19/10;
    }
    else if(prox_mean<60){
        prox_threshold_hi = prox_mean*6;
        prox_threshold_lo = prox_mean*18/10;
    }
    else if(prox_mean<70){
        prox_threshold_hi = prox_mean*55/10;
        prox_threshold_lo = prox_mean*17/10;
    }
    else if(prox_mean<80){
        prox_threshold_hi = prox_mean*5;
        prox_threshold_lo = prox_mean*17/10;
    }
    else if(prox_mean<100){
        prox_threshold_hi = prox_mean*25/10;
        prox_threshold_lo = prox_mean*18/10;
    }
    /*if the prox_mean is more than 100,I modified the threshold calculation methold.
    It depends on your handset test data.----modified by pcyang 2013-01-23*/
    else if(prox_mean<150){
        prox_threshold_hi = prox_mean*2;
        prox_threshold_lo = prox_mean*15/10;
    }
    else if(prox_mean<200){
        prox_threshold_hi = prox_mean*16/10;
        prox_threshold_lo = prox_mean*13/10;
    }
    else if(prox_mean<300){
        prox_threshold_hi = prox_mean*15/10;
        prox_threshold_lo = prox_mean*12/10;
    }
    else if(prox_mean<600){
        prox_threshold_hi = prox_mean*145/100;
        prox_threshold_lo = prox_mean*12/10;
    }
    else{
        ps_cali.valid = 0;
 		prox_threshold_hi = obj->hw->ps_threshold_high;
        prox_threshold_lo = obj->hw->ps_threshold_low;
    }

    atomic_set(&obj->ps_thd_val_high, prox_threshold_hi);
    atomic_set(&obj->ps_thd_val_low,  prox_threshold_lo);
    
    APS_DBG("TMD2772_ps_calibrate: ps_thd_val_low = %d\n",  atomic_read(&obj->ps_thd_val_low)); 
    APS_DBG("TMD2772_ps_calibrate: ps_thd_val_high = %d\n", atomic_read(&obj->ps_thd_val_high)); 

#if 0 //def TMD2772_DEBUG
    do_gettimeofday(&endtime);
    usedtime=1000*(endtime.tv_sec-starttime.tv_sec)+(endtime.tv_usec-starttime.tv_usec)/1000;
    APS_DBG("calibration time is: %d\n",usedtime);
    APS_DBG("ps polling mode: %d\n",obj->hw->polling_mode_ps);
#endif
}

/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct TMD2772_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
    int err = 0;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))){
        err = -ENOMEM;
        goto exit;
    }
    
    memset(obj, 0, sizeof(*obj));
    TMD2772_obj = obj;

    obj->hw = get_cust_alsps_hw();
    TMD2772_get_addr(obj->hw, &obj->addr);

    /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
    INIT_WORK(&obj->eint_work, TMD2772_eint_work);
    obj->client = client;
    i2c_set_clientdata(client, obj);	
    atomic_set(&obj->als_debounce, 50);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 200);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->als_cmd_val, 0xDF);
    atomic_set(&obj->ps_cmd_val,  0xC1);
    atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
    atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
    /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
    obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
    								//(400)/16*2.72 here is amplify *100 //16
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
    atomic_set(&obj->i2c_retry, 3);
    set_bit(CMC_BIT_ALS, &obj->enable);
    set_bit(CMC_BIT_PS, &obj->enable);
    
    //init the sensor mutex   added by pcyang
    mutex_init(&sensor_mutex);

    TMD2772_i2c_client = client;
    
    if((err = TMD2772_init_client(client))){
        goto exit_init_failed;
    }
    
    //bootup calibration ps,first enable the ps/als, then calibration, after disable the ps/als.	
    TMD2772_init_client_for_cali(client);
    TMD2772_ps_calibrate(client);
    TMD2772_set_threshold_value(client);
    TMD2772_init_client_for_cali_restore(client);  
	
    if((err = misc_register(&TMD2772_device))){
        APS_ERR("TMD2772_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if(err = TMD2772_create_attr(&TMD2772_alsps_driver.driver)){
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_ps.self = TMD2772_obj;
    
    /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
    if(1 == obj->hw->polling_mode_ps){
        obj_ps.polling = 1;
    }
    else{
        obj_ps.polling = 0;
    }

    obj_ps.sensor_operate = TMD2772_ps_operate;
    
    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps))){
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }
	
    obj_als.self = TMD2772_obj;
    obj_als.polling = 1;
    obj_als.sensor_operate = TMD2772_als_operate;
    
    if((err = hwmsen_attach(ID_LIGHT, &obj_als))){
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1,
    obj->early_drv.suspend  = TMD2772_early_suspend,
    obj->early_drv.resume   = TMD2772_late_resume,    
    register_early_suspend(&obj->early_drv);
#endif

    APS_LOG("%s: OK\n", __func__);
    return 0;

    exit_create_attr_failed:
        misc_deregister(&TMD2772_device);
    exit_misc_device_register_failed:
    exit_init_failed:

        kfree(obj);
    exit:
        TMD2772_i2c_client = NULL;           
        APS_ERR("%s: err = %d\n", __func__, err);
    return err;
}

/*----------------------------------------------------------------------------*/
static int TMD2772_i2c_remove(struct i2c_client *client)
{
	int err;	
/*	
	if(err = TMD2772_delete_attr(&TMD2772_i2c_driver.driver))
	{
		APS_ERR("TMD2772_delete_attr fail: %d\n", err);
	} 
	
*/
        if(err = TMD2772_delete_attr(&TMD2772_i2c_driver.driver))
    
        {
            APS_ERR("TMD2772_delete_attr fail: %d\n", err);
        } 

	if((err = misc_deregister(&TMD2772_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	TMD2772_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int TMD2772_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	TMD2772_power(hw, 1);    
	//TMD2772_force[0] = hw->i2c_num;
	//TMD2772_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",TMD2772_force[0],TMD2772_force[1]);
	if(i2c_add_driver(&TMD2772_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int TMD2772_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	TMD2772_power(hw, 0);    
	i2c_del_driver(&TMD2772_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver TMD2772_alsps_driver = {
	//.probe      = TMD2772_probe,
	//.remove     = TMD2772_remove,    
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init TMD2772_init(void)
{
	APS_FUN();
	i2c_register_board_info(0, &i2c_TMD2772, 1);
	if(platform_driver_register(&TMD2772_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit TMD2772_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&TMD2772_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(TMD2772_init);
module_exit(TMD2772_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("TMD2772 driver");
MODULE_LICENSE("GPL");



