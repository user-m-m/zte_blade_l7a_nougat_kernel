/*
 *  rpr0521.c - Linux kernel modules for Rohm rpr0521 proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2016 Aaron Liu / Rohm <aaron-liu@rohm.com.cn>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *
 */


#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio.h>
//#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
//#endif
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>

//#include "rpr0521.h"


/******************************************************************/


#define DRIVER_VERSION          ("1.0.1 20160322")
#define RPR0521_I2C_DRIVER_NAME ("rpr0521_i2c")
#define RPR0521_I2C_NAME        ("rohm,rpr0521_i2c")
#define RPR0521_DEV_NAME        ("rpr0521_alsps")
#define RPR0521_INPUTDEV_NAME   ("rpr0521_inputdev")

#define RPR0521_ALS_NAME ("lightsensor-level")
#define RPR0521_PS_NAME  ("proximity")


 
/* RPR0521 REGSTER */
/* Register Addresses define */
#define RPR0521_REG_ID_ADDR        (0x40)
#define RPR0521_REG_ID_VALUE     (0x0A)       
#define RPR0521_REG_ENABLE_ADDR    (0x41)
#define RPR0521_REG_ALS_ADDR        (0x42)
#define RPR0521_REG_PRX_ADDR        (0x43)
#define RPR0521_REG_PDATAL_ADDR    (0x44)
#define RPR0521_REG_PDATAH_ADDR    (0x45)
#define RPR0521_REG_ADATA0L_ADDR    (0x46)
#define RPR0521_REG_ADATA0H_ADDR    (0x47)
#define RPR0521_REG_ADATA1L_ADDR    (0x48)
#define RPR0521_REG_ADATA1H_ADDR    (0x49)
#define RPR0521_REG_INTERRUPT_ADDR    (0x4A)
#define RPR0521_REG_PIHTL_ADDR    (0x4B)
#define RPR0521_REG_PIHTH_ADDR    (0x4C)
#define RPR0521_REG_PILTL_ADDR    (0x4D)
#define RPR0521_REG_PILTH_ADDR    (0x4E)
#define RPR0521_REG_AIHTL_ADDR    (0x4F)
#define RPR0521_REG_AIHTH_ADDR    (0x50)
#define RPR0521_REG_AILTL_ADDR    (0x51)
#define RPR0521_REG_AILTH_ADDR    (0x52)
 
#define RPR0521_REG_PINT_STATUS         (0x80)   
#define RPR0521_REG_AINT_STATUS         (0x40)   


#define RPR0521_PIHT_NEAR             (0x0FFF)
#define RPR0521_PILT_FAR              (0)

 
#define RPR0521_INTERRUPT_GPIO        (166)

#define ALS_GAIN_AUTO_CHANGE  1

#if ALS_GAIN_AUTO_CHANGE
bool    gain_64x_flag = false;
#define ALS_AUTO_LUX_HIGH   150
#define ALS_AUTO_LUX_LOW    50
#define ALS_AUTO_GAIN_DIFF  32
unsigned int last_lux = 0;
#endif

typedef enum
{
    ALS_GAIN_1X    = 0,     
    ALS_GAIN_2X    = 1,     
    ALS_GAIN_64X   = 2,    
    ALS_GAIN_128X  = 3   
} als_gain_e;

typedef enum
{
    PS_GAIN_1X   = 0,
    PS_GAIN_2X   = 1,
    PS_GAIN_4X   = 2,
    PS_GAIN_INVALID  =3,
} ps_gain_e;



 
#define ALS_EN          (0x1 << 7)
#define ALS_OFF         (0x0 << 7)
#define PS_EN           (0x1 << 6)
#define PS_OFF          (0x0 << 6)
#define BOTH_STANDBY    (0)
#define PS10MS          (0x1)
#define PS40MS          (0x2)
#define PS100MS         (0x3)
#define ALS100MS_PS50MS (0x5)
#define BOTH100MS       (0x6)
#define BOTH400MS       (0xB)

#define CALC_MEASURE_100MS   (200)
#define CALC_MEASURE_400MS   (400)


#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)

#define PS_THH_ONLY         (0 << 4)
#define PS_THH_BOTH_HYS     (1 << 4)
#define PS_THH_BOTH_OUTSIDE (2 << 4)
#define POLA_ACTIVEL        (0 << 3)
#define POLA_INACTIVEL      (1 << 3)
#define OUTPUT_ANYTIME      (0 << 2)
#define OUTPUT_LATCH        (1 << 2)
#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)
#define PS_PERSISTENCE_SETTING      (1)    

 
typedef struct {
    unsigned short pdata;          
    unsigned short adata0;         
    unsigned short adata1;         
} READ_DATA_ARG;

/* *************************************************/
#define CHECK_RESULT(result)                        \
    if ( result < 0 )                               \
    {                                               \
        printk("Error occur !!!\n");                  \
        return result;                              \
    }
/* *************************************************/



 
typedef enum
{
    PRX_FAR_AWAY,
    PRX_NEAR_BY,
    PRX_NEAR_BY_UNKNOWN
} prx_nearby_type;


#define RPR521_PS_CALI_LOOP     5    
#define RPR521_PS_NOISE_DEFAULT 30   

#define RPR0521_PRX_NEAR_THRESHOLD       (20)  
#define RPR0521_PRX_FAR_THRESHOLD        (14) 

 
#define POWER_ON    (1)  
#define POWER_OFF   (0)   

#define RPR0521_AMBIENT_IR_FLAG              (6)  /*    ambient ir flag:
                                                   *     00: infrared low
                                                   *     01: infrared high
                                                   *     02: infrared too hight */

#define RPR_DEBUG_PRINTF
#define SPREADTRUM_PLATFORM

 
#define RPR0521_PS_CALIBRATIOIN_ON_CALL    (NULL)

 
#define RPR0521_PS_CALIBRATIOIN_ON_START   (NULL)



 
#define MIN_ALS_POLL_DELAY_NS    60000000

/* *************************************************/
#define COEFFICIENT               (4)
const unsigned long data0_coefficient[COEFFICIENT] = {24380, 49881, 2837, 2447};
const unsigned long data1_coefficient[COEFFICIENT] = {10149, 30809, 551, 954};
const unsigned long judge_coefficient[COEFFICIENT] = {1000, 1200, 1600, 2563};

 
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    char data0;
    char data1;
} gain_table[GAIN_FACTOR] = {
    {  1,   1},    
    {  1,   2},    
    {  1,  64},    
    {  1, 128},   
    {  2,   1},    
    {  2,   2},   
    {  2,  64},   
    {  2, 128},    
    { 64,   1},    
    { 64,   2},    
    { 64,  64},    
    { 64, 128},    
    {128,   1},   
    {128,   2},   
    {128,  64},   
    {128, 128}     
};
/* *************************************************/


typedef struct {
    unsigned long long  data;
    unsigned long long  data0;
    unsigned long long  data1;
    unsigned char       gain_data0;
    unsigned char       gain_data1;
    unsigned long       dev_unit;
    unsigned short      als_time;  
    unsigned short      als_data0;
    unsigned short      als_data1;
} calc_data_type;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} calc_ans_type;


struct rpr0521_data {

    struct i2c_client *client;

    unsigned short  als_measure_time;        
    unsigned short  als_gain_index;          

    unsigned short  thresh_near;             
    unsigned short  thresh_far;               

    prx_nearby_type prx_detection_state;     
    prx_nearby_type last_nearby;            

    unsigned short  ps_raw_data;             

    int als_lux_last;

   	int polling_mode_ps;        
	int polling_mode_als;         

    int irq;                      
    int int_pin;                 

    bool als_enabled;
    bool ps_enabled;
    bool als_suspend;
    bool ps_suspend;
    bool first_boot;

    struct work_struct rpr0521work;
    struct workqueue_struct *rpr0521wq;

#if 0
//#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend rpr0521_early_suspend;
#endif

    struct mutex io_lock;
    struct input_dev *als_input_dev;
    struct input_dev *ps_input_dev;
    struct wake_lock ps_wakelock;

    struct work_struct rpr0521_als_work;
    struct workqueue_struct *rpr0521_als_wq;
    struct hrtimer als_timer;


    ktime_t ps_poll_delay;
    ktime_t als_poll_delay;

};

/* *************************************************/
static struct rpr0521_data *rpr0521_i2c_data = NULL;
/* *************************************************/
static int  rpr0521_enable_ps(struct rpr0521_data *alsps_data, unsigned char enable);
static int  rpr0521_enable_als(struct rpr0521_data *alsps_data, unsigned char enable);
static void rpr0521_set_prx_thresh(struct rpr0521_data * alsps_data,  unsigned short pilt, unsigned short piht );
static int  rpr0521_read_data(struct rpr0521_data * alsps_data, READ_DATA_ARG *data);
unsigned int    rpr0521_als_convert_to_mlux( unsigned short  data0, unsigned short  data1, unsigned short  gain_index, unsigned short  time );

#if defined(RPR0521_PS_CALIBRATIOIN_ON_CALL) || defined(RPR0521_PS_CALIBRATIOIN_ON_START)
static int  rpr0521_ps_calibration(struct rpr0521_data * alsps_data);
#endif
/* *************************************************/

static int rpr0521_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
    unsigned char retry;
    int err;
    struct i2c_msg msgs[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &command,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = values,
        },
    };

    for (retry = 0; retry < 2; retry++)
    {
        err = i2c_transfer(client->adapter, msgs, 2);
        if (err == 2)
            break;
        else
            mdelay(5);
    }

    if (retry >= 2)
    {
        printk(KERN_ERR "%s: i2c read fail, err=%d\n", __func__, err);
        return -EIO;
    }
    return 0;
}

static int rpr0521_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
    int retry;
    int err;
    unsigned char data[11];
    struct i2c_msg msg;
    int index;

    if (!client)
        return -EINVAL;
    else if (length >= 10)
    {
        printk(KERN_ERR "%s:length %d exceeds 10\n", __func__, length);
        return -EINVAL;
    }

    data[0] = command;
    for (index=1;index<=length;index++)
        data[index] = values[index-1];

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = length+1;
    msg.buf = data;

    for (retry = 0; retry < 3; retry++)
    {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
            break;
        else
            mdelay(5);
    }

    if (retry >= 3)
    {
        printk(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);
        return -EIO;
    }
    return 0;
}

static int rpr0521_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
    unsigned char value;
    int err;
    err = rpr0521_i2c_read_data(client, command, 1, &value);
    if(err < 0)
        return err;

    err = value;
    err &= 0xff;

    return err;
}

static int rpr0521_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
    int err;
    err = rpr0521_i2c_write_data(client, command, 1, &value);
    return err;
}

/*****************************************************************************
 * @Brief: rpr0521_ps_calibration is used to get ps noise raw data.
 *         we just read 5 times ps raw data and get average, and set this average as ps noise raw data.
 *         Be note: als is disabled during this process, this process will cost 5*15ms at least
 *
 * @Param: alsps_data point to global RPR0521_DATA
 *
 * @Returns: 0 for success, other for failed.
 *
 *****************************************************************************/
#if (defined(RPR0521_PS_CALIBRATIOIN_ON_CALL) || defined(RPR0521_PS_CALIBRATIOIN_ON_START))
static int rpr0521_ps_calibration(struct rpr0521_data * alsps_data)
{
    int ret = 0;
    int i = 0, result = 0, average = 0;
    unsigned char   i2c_read_data[2] ={0}, i2c_data = 0, enable_old_setting = 0, interrupt_old_setting = 0;
    unsigned char   infrared_data = 0;
    unsigned short  pdata = 0;

    printk("rpr0521_calibration IN \n");

    if(NULL == alsps_data || NULL == alsps_data->client)
    {
        printk(" Parameter error \n");
        return -1;
    }

     
    interrupt_old_setting = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, RPR0521_REG_INTERRUPT_ADDR);
    if (interrupt_old_setting < 0) {
        printk( " I2C read error !!!  \n" );
        return -1;
    }

    i2c_data = interrupt_old_setting & 0XFE;

    result = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_INTERRUPT_ADDR, i2c_data);
    if (result < 0) {
        printk( " I2C read error !!!  \n" );
        return -1;
    }
     

    
    enable_old_setting = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, RPR0521_REG_ENABLE_ADDR);
    if (enable_old_setting < 0) {
        printk( " I2C read error !!!  \n" );
        ret = -1;
        goto err_interrupt_status;
    }
    i2c_data = (enable_old_setting & 0x30)  | PS_EN| PS10MS;    //just keep ps_pulse and ps operating mode
    result = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ENABLE_ADDR, i2c_data);
    if(result < 0)
    {
        printk( " I2C read error !!!  \n" );
        ret = -1;
        goto err_interrupt_status;
    }
     

 
    i2c_data = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, RPR0521_REG_PRX_ADDR);
    if(i2c_data < 0)
    {
        printk( " I2C read error !!!  \n" );
        ret = -1;
        goto err_exit;
    }
    infrared_data = i2c_data;
    if(infrared_data >> 6)   
    {
        ret = -1;
        goto err_exit;
    }

     
    for(i = 0; i < RPR521_PS_CALI_LOOP; i ++)
    {
        msleep(15);  
        result = rpr0521_i2c_read_data(alsps_data->client, RPR0521_REG_PDATAL_ADDR, sizeof(i2c_read_data), i2c_read_data);
        if(result < 0)
        {
            printk( " I2C read error !!!  \n" );
            ret = -1;
            goto err_exit;
        }
        pdata = (unsigned short)(((unsigned short)i2c_read_data[1] << 8) | i2c_read_data[0]);
        average += pdata & 0xFFF;
        printk(" pdata=%d i =%d\n", pdata, i);
    }

    average /= RPR521_PS_CALI_LOOP;  //average

    printk("rpr0521_ps_calib average=%d \n", average);

    alsps_data->thresh_far  = average + RPR0521_PRX_FAR_THRESHOLD;
    alsps_data->thresh_near = average + RPR0521_PRX_NEAR_THRESHOLD;


err_exit:
    rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ENABLE_ADDR, enable_old_setting);
err_interrupt_status:
    rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_INTERRUPT_ADDR, interrupt_old_setting);

    printk("rpr521 PS calibration end\r\n");

    return ret;
}
#endif

/*===========================================================================

  FUNCTION      rpr0521_set_prx_thresh

  DESCRIPTION   set ps interrupt threshold

  DEPENDENCIES  None

  RETURN VALUE  None

  SIDE EFFECT   None

  ===========================================================================*/
static void rpr0521_set_prx_thresh(struct rpr0521_data * alsps_data,  unsigned short  pilt, unsigned short piht )
{
    int         result;
    unsigned char          thresh[4];

#ifdef RPR0521_DEBUG
    unsigned char          read_thresh[4] = {0};
#endif

    printk("pilt low: 0x%x, piht hig: 0x%x\n", pilt, piht);
    thresh[2] = (pilt & 0xFF);  
    thresh[3] = (pilt >> 8);    
    thresh[0] = (piht & 0xFF);  
    thresh[1] = (piht >> 8);    

    printk(" befor write reg(0x4B) = 0x%x, reg(0x4C) = 0x%x, reg(0x4D) = 0x%x, reg(0x4E) = 0x%x\n",
                    thresh[0], thresh[1], thresh[2], thresh[3] );

     
    result  = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PIHTL_ADDR, thresh[0]);
    result |= rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PIHTH_ADDR, thresh[1]);
    result |= rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PILTL_ADDR, thresh[2]);
    result |= rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PILTH_ADDR, thresh[3]);
    if ( result < 0 )
    {
        printk("write data from IC error.\n");
        return ;
    }

#ifdef RPR0521_DEBUG
     
    result = rpr0521_i2c_read_data(alsps_data->client, RPR0521_REG_PIHTL_ADDR, sizeof(read_thresh), read_thresh);
    if ( result < 0 )
    {
        printk("Read data from IC error.\n");
        return ;
    }

    printk(" after write reg(0x4B) = 0x%x, reg(0x4C) = 0x%x, reg(0x4D) = 0x%x, reg(0x4E) = 0x%x\n",
                    read_thresh[0], read_thresh[1], read_thresh[2], read_thresh[3] );
#endif

}

static int rpr0521_init_all_reg(struct rpr0521_data *alsps_data)
{
    int ret;

    unsigned char w_mode[4];
#if ALS_GAIN_AUTO_CHANGE
    w_mode[0] = (LEDCURRENT_100MA | (ALS_GAIN_64X << 4) | (ALS_GAIN_64X << 2));   
#else
    w_mode[0] = (LEDCURRENT_100MA | (ALS_GAIN_2X << 4) | (ALS_GAIN_2X << 2));   
#endif
    w_mode[1] = (PS_GAIN_2X << 4) | PS_PERSISTENCE_SETTING;
    w_mode[2] = (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | MODE_PROXIMITY);
    w_mode[3] = (BOTH100MS);



    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ALS_ADDR, w_mode[0]);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
#if ALS_GAIN_AUTO_CHANGE
    gain_64x_flag = true;
#endif
    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PRX_ADDR, w_mode[1]);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_INTERRUPT_ADDR, w_mode[2]);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ENABLE_ADDR, w_mode[3]);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

     
    if(BOTH100MS == w_mode[3])
        alsps_data->als_measure_time = CALC_MEASURE_100MS;
    else if(BOTH400MS == w_mode[3])
        alsps_data->als_measure_time = CALC_MEASURE_400MS;

    
#if ALS_GAIN_AUTO_CHANGE
    alsps_data->als_gain_index = (ALS_GAIN_64X << 2) | (ALS_GAIN_64X);   
#else
    alsps_data->als_gain_index = (ALS_GAIN_2X << 2) | (ALS_GAIN_2X);   
#endif
    return 0;
}


static int rpr0521_check_id(struct rpr0521_data *alsps_data)
{
    unsigned char value;
    int err;

    err = rpr0521_i2c_read_data(alsps_data->client, RPR0521_REG_ID_ADDR, 1, &value);
    if(err < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
        return err;
    }

    printk(KERN_INFO "%s: ID=0x%x\n", __func__, value);

    if((value&0x3F) != RPR0521_REG_ID_VALUE)
    {
        printk(KERN_INFO "%s: ID=0x%x\n", __func__, value);
        err = -1;
    }

    return err;
}


static int rpr0521_software_reset(struct rpr0521_data *alsps_data)
{
    int ret = 0;

    printk("rpr0521_software_reset\n");
    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ID_ADDR, 0xC0);  //reset
    if (ret < 0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
    }
    return ret;
}

static unsigned int rpr0521_get_ps_raw_data(struct rpr0521_data *alsps_data)
{
    unsigned char value[2];
    int err;
    err = rpr0521_i2c_read_data(alsps_data->client, RPR0521_REG_PDATAL_ADDR, 2, &value[0]);
    if(err < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
        return err;
    }
    return ((value[1]<<8) | value[0]);
}


static int rpr0521_set_power_state(struct rpr0521_data *alsps_data, unsigned char state)
{
    int ret;
    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client,RPR0521_REG_ENABLE_ADDR, state);
    if(ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
    return ret;
}

static int rpr0521_get_power_state(struct rpr0521_data *alsps_data)
{
    int ret;
    ret = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, RPR0521_REG_ENABLE_ADDR);
    if(ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
    return ret;
}

static int rpr0521_enable_ps(struct rpr0521_data *alsps_data, unsigned char enable)
{
    int ret;
    unsigned char w_state_reg;
    unsigned char curr_ps_enable;
    unsigned int reading;

    curr_ps_enable = alsps_data->ps_enabled ?POWER_ON: POWER_OFF;

    if(curr_ps_enable == enable)
        return 0;


    if(alsps_data->first_boot == true)
    {
        alsps_data->first_boot = false;
    }

     
    ret = rpr0521_get_power_state(alsps_data);
    if(ret < 0)
        return ret;
    w_state_reg = (unsigned char) ret &( ~PS_EN);
    if(enable)
    {
        w_state_reg |= PS_EN;
    }
    else
    {
        w_state_reg |= PS_OFF;
    }
    ret = rpr0521_set_power_state(alsps_data, w_state_reg);
    if(ret < 0)
        return ret;

    if(enable)
    {

#ifdef RPR0521_PS_CALIBRATIOIN_ON_ENABLE
        
        ret = rpr0521_ps_calibration(alsps_data);
        if(ret == 0)
        {
            printk("ps calibration success! \n");
        }
        else
        {
            printk("ps calibration failed! \n");
        }

        if(0 == alsps_data->polling_mode_ps)   // set ps interrupt threshold
        {
            rpr0521_set_prx_thresh(alsps_data, alsps_data->thresh_far, alsps_data->thresh_near);
        }
#endif

        printk(KERN_INFO "%s: thresh_near=%d,thresh_far=%d\n", __func__, alsps_data->thresh_near,  alsps_data->thresh_far);

        enable_irq(alsps_data->irq);

        alsps_data->ps_enabled = true;

        input_report_abs(alsps_data->ps_input_dev, ABS_DISTANCE, 1);
        input_sync(alsps_data->ps_input_dev);
        wake_lock_timeout(&alsps_data->ps_wakelock, 3*HZ);
        reading = rpr0521_get_ps_raw_data(alsps_data);
        printk(KERN_INFO "%s: ps input  ps raw_data = %d\n", __func__, reading);
    }
    else
    {
        disable_irq(alsps_data->irq);
        alsps_data->ps_enabled = false;
    }

    return 0;
}

static int rpr0521_enable_als(struct rpr0521_data *alsps_data, unsigned char enable)
{
    int ret;
    unsigned char w_state_reg;
    unsigned char curr_als_enable = (alsps_data->als_enabled)?POWER_ON:POWER_OFF;

    printk(KERN_INFO "%s: enter, curr_als_enable=%d,enable=%d.\n", __func__, curr_als_enable, enable);
    if(curr_als_enable == enable)
	{
        return 0;
	}
	
    ret = rpr0521_get_power_state(alsps_data);
    if(ret < 0)
	{
        return ret;
	}

    w_state_reg = (unsigned char)(ret & (~ALS_EN));

    if(enable)
        w_state_reg |= ALS_EN;
    else
        w_state_reg |= ALS_OFF;

    ret = rpr0521_set_power_state(alsps_data, w_state_reg);
    if(ret < 0)
	{
        return ret;
	}
    if (enable)
    {
        alsps_data->als_enabled = true;
        hrtimer_start(&alsps_data->als_timer, alsps_data->als_poll_delay, HRTIMER_MODE_REL);
    }
    else
    {
        alsps_data->als_enabled = false;
        hrtimer_cancel(&alsps_data->als_timer);
        cancel_work_sync(&alsps_data->rpr0521_als_work);
    }

    return 0;
}

 
#if ALS_GAIN_AUTO_CHANGE
static int rpr0521_enable_als_hardware(struct rpr0521_data *alsps_data, unsigned char enable)
{
    int ret;
    unsigned char w_state_reg;
   	
    ret = rpr0521_get_power_state(alsps_data);
    if(ret < 0)
	{
        return ret;
	}

    w_state_reg = (unsigned char)(ret & (~ALS_EN));

    if(enable)
        w_state_reg |= ALS_EN;
    else
        w_state_reg |= ALS_OFF;

    ret = rpr0521_set_power_state(alsps_data, w_state_reg);
    if(ret < 0)
	{
        return ret;
	}
    printk(KERN_INFO "%s: %d OK.\n", __func__, enable);
    return 0;
}

bool rpr0521_change_gain_dynamic(struct rpr0521_data *alsps_data)
{
    int ret = 0;
    unsigned char i2c_data = 0x0;
    bool gain_changed = false;
    if (gain_64x_flag)
    {
         
        rpr0521_enable_als_hardware(alsps_data, POWER_OFF);
        
        i2c_data = (LEDCURRENT_100MA | (ALS_GAIN_2X << 4) | (ALS_GAIN_2X << 2));  // led current 100 mA
        ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ALS_ADDR, i2c_data);
        if (ret >= 0)
        {
            gain_64x_flag = false;
            alsps_data->als_gain_index = (ALS_GAIN_2X << 2) | (ALS_GAIN_2X);
            printk(KERN_INFO "%s: change gain to 2X successfully.\n", __func__);
        }
        else
        {
            printk(KERN_ERR "%s: change gain to 2X fail[i2c error]\n", __func__);
        }
        
        rpr0521_enable_als_hardware(alsps_data, POWER_ON);
        gain_changed = true;
    }
    else
    {
         
        rpr0521_enable_als_hardware(alsps_data, POWER_OFF);
        
        i2c_data = (LEDCURRENT_100MA | (ALS_GAIN_64X << 4) | (ALS_GAIN_64X << 2));  // led current 100 mA
        ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_ALS_ADDR, i2c_data);
        if (ret >= 0)
        {
            gain_64x_flag = true;
            alsps_data->als_gain_index = (ALS_GAIN_64X << 2) | (ALS_GAIN_64X);
            printk(KERN_INFO "%s: change gain to 64X successfully.\n", __func__);
        }
        else
        {
            printk(KERN_ERR "%s: change gain to 64X fail[i2c error]\n", __func__);
        }
         
        rpr0521_enable_als_hardware(alsps_data, POWER_ON);
        gain_changed = true;
    }
    printk(KERN_INFO "%s: return.\n", __func__);
    return gain_changed;
}
#endif
 

static int rpr0521_get_als_value(struct rpr0521_data *alsps_data)
{
    READ_DATA_ARG       data;
    int result;
    unsigned int reading_lux;
#if ALS_GAIN_AUTO_CHANGE
    bool gain_changed = false;
#endif
    result = rpr0521_read_data(alsps_data, &data);
    CHECK_RESULT(result);

    

    reading_lux = rpr0521_als_convert_to_mlux(data.adata0, data.adata1,
                                            alsps_data->als_gain_index, alsps_data->als_measure_time);

	//alsps_data->als_lux_last = reading_lux;	 
#if ALS_GAIN_AUTO_CHANGE
    if (((reading_lux >= ALS_AUTO_LUX_HIGH) && (gain_64x_flag))
        || ((reading_lux < ALS_AUTO_LUX_LOW) && (!gain_64x_flag)))
    {
        gain_changed = rpr0521_change_gain_dynamic(alsps_data);

        if (gain_changed && last_lux!= 0 && reading_lux != 0)
        {
            if ((last_lux / reading_lux == ALS_AUTO_GAIN_DIFF)
                || (reading_lux / last_lux) == ALS_AUTO_GAIN_DIFF)
            {
                reading_lux = last_lux;
            }
        }
    }
    last_lux = reading_lux;
#endif
    return reading_lux;
}

static ssize_t rpr0521_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    int ret;

    ret = rpr0521_get_power_state(alsps_data);
    if(ret < 0)
        return ret;
    ret = (ret & ALS_EN)? 1: 0;

    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t rpr0521_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct rpr0521_data *alsps_data = dev_get_drvdata(dev);
    unsigned char en;
	
    if (sysfs_streq(buf, "1"))
        en = 1;
    else if (sysfs_streq(buf, "0"))
        en = 0;
    else
    {
        printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }
    printk("%s: Enable ALS : %d\n", __func__, en);

    mutex_lock(&alsps_data->io_lock);
    printk(KERN_INFO "%s: disable als, %d!!\n", __func__, en);
    rpr0521_enable_als(alsps_data, en);
    mutex_unlock(&alsps_data->io_lock);
    return size;
}

static ssize_t rpr0521_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rpr0521_data *alsps_data = dev_get_drvdata(dev);
    unsigned int als_lux;
    als_lux = rpr0521_get_als_value(alsps_data);
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t rpr0521_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;
    ret = kstrtoul(buf, 16, &value);
    if(ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
    alsps_data->als_lux_last = value;
    input_report_abs(alsps_data->als_input_dev, ABS_MISC, value);
    input_sync(alsps_data->als_input_dev);
    printk(KERN_INFO "%s: als input event %ld lux\n",__func__, value);

    return size;
}

static ssize_t rpr0521_als_raw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rpr0521_data *alsps_data = dev_get_drvdata(dev);
    READ_DATA_ARG       data;

    rpr0521_read_data(alsps_data, &data);

    return scnprintf(buf, PAGE_SIZE, "[0]:%d [1]:%d\n", data.adata0, data.adata1);
}

static ssize_t rpr0521_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    int64_t delay;
    mutex_lock(&alsps_data->io_lock);
    delay = ktime_to_ns(alsps_data->als_poll_delay);
    mutex_unlock(&alsps_data->io_lock);
    return scnprintf(buf, PAGE_SIZE, "%lld\n", delay);
}

static ssize_t rpr0521_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;
    int ret;
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    ret = kstrtoull(buf, 10, &value);
    if(ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoull failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
#ifdef RPR_DEBUG_PRINTF
    printk(KERN_INFO "%s: set als poll delay=%lld\n", __func__, value);
#endif
    if(value < MIN_ALS_POLL_DELAY_NS)
    {
        printk(KERN_ERR "%s: delay is too small\n", __func__);
        value = MIN_ALS_POLL_DELAY_NS;
    }
    mutex_lock(&alsps_data->io_lock);
    if(value != ktime_to_ns(alsps_data->als_poll_delay))
        alsps_data->als_poll_delay = ns_to_ktime(value);
    mutex_unlock(&alsps_data->io_lock);
    return size;
}

static ssize_t rpr0521_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);

    ret = rpr0521_get_power_state(alsps_data);
    if(ret < 0)
        return ret;
    ret = (ret & PS_EN)?1:0;

    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t rpr0521_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    unsigned char en;
    if (sysfs_streq(buf, "1"))
        en = 1;
    else if (sysfs_streq(buf, "0"))
        en = 0;
    else
    {
        printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }
    printk(KERN_INFO "%s: Enable PS : %d\n", __func__, en);
    mutex_lock(&alsps_data->io_lock);
    rpr0521_enable_ps(alsps_data, en);
    mutex_unlock(&alsps_data->io_lock);
    return size;
}

static ssize_t rpr0521_ps_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rpr0521_data *alsps_data = dev_get_drvdata(dev);
    unsigned int ps_raw;
    ps_raw = rpr0521_get_ps_raw_data(alsps_data);
    return scnprintf(buf, PAGE_SIZE, "%d ps_raw\n", ps_raw);
}

static ssize_t rpr0521_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ps_thd_l1_reg, ps_thd_l2_reg;
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    ps_thd_l1_reg = rpr0521_i2c_smbus_read_byte_data(alsps_data->client,RPR0521_REG_PILTL_ADDR);
    if(ps_thd_l1_reg < 0)
    {
        printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);
        return -EINVAL;
    }
    ps_thd_l2_reg = rpr0521_i2c_smbus_read_byte_data(alsps_data->client,RPR0521_REG_PILTH_ADDR);
    if(ps_thd_l2_reg < 0)
    {
        printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l2_reg);
        return -EINVAL;
    }
    ps_thd_l1_reg = ps_thd_l1_reg  | ps_thd_l2_reg << 8;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}

static ssize_t rpr0521_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;
    unsigned char ps_thd_l1_reg, ps_thd_l2_reg;

    ret = kstrtoul(buf, 10, &value);
    if(ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    ps_thd_l1_reg = value & 0xFF;
    ps_thd_l2_reg = (value >> 8) & 0xFF;

    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PILTL_ADDR, ps_thd_l1_reg);
    if(ret < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
        return -EINVAL;
    }

    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PILTH_ADDR, ps_thd_l2_reg);
    if(ret < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
        return -EINVAL;
    }

    return size;
}

static ssize_t rpr0521_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned short ps_thd_h1_reg, ps_thd_h2_reg;

    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);

    ps_thd_h1_reg = rpr0521_i2c_smbus_read_byte_data(alsps_data->client,RPR0521_REG_PIHTL_ADDR);
    if(ps_thd_h1_reg < 0)
    {
        printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h1_reg);
        return -EINVAL;
    }

    ps_thd_h2_reg = rpr0521_i2c_smbus_read_byte_data(alsps_data->client,RPR0521_REG_PIHTH_ADDR);
    if(ps_thd_h2_reg < 0)
    {
        printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h2_reg);
        return -EINVAL;
    }
    ps_thd_h1_reg = ps_thd_h1_reg | ps_thd_h2_reg << 8 ;

    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
}

static ssize_t rpr0521_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;
    unsigned char ps_thd_h1_reg, ps_thd_h2_reg;

    ret = kstrtoul(buf, 10, &value);
    if(ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    ps_thd_h1_reg = value & 0xFF;
    ps_thd_h2_reg = (value >> 8) & 0xFF;

    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PIHTL_ADDR, ps_thd_h1_reg);
    if(ret < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
        return -EINVAL;
    }

    ret = rpr0521_i2c_smbus_write_byte_data(alsps_data->client, RPR0521_REG_PIHTH_ADDR, ps_thd_h2_reg);
    if(ret < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
        return -EINVAL;
    }

    return size;
}

static ssize_t rpr0521_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ps_reg[19];
    unsigned char cnt;
    int len = 0;
    struct rpr0521_data *alsps_data =  dev_get_drvdata(dev);

    for(cnt=0; cnt< 19;cnt++)
    {
        ps_reg[cnt] = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, (RPR0521_REG_ID_ADDR + cnt));
        if(ps_reg[cnt] < 0)
        {
            printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
            return -EINVAL;
        }
        else
        {
            printk(KERN_INFO "reg[0x%2X]=0x%2X\n", RPR0521_REG_ID_ADDR + cnt, ps_reg[cnt]);
            len += scnprintf(buf + len, PAGE_SIZE - len, "[0x%02X]=0x%02X\n", RPR0521_REG_ID_ADDR +  cnt, ps_reg[cnt]);
        }
    }

    return len;
}

static struct device_attribute als_enable_attribute     = __ATTR(enable, 0666, rpr0521_als_enable_show, rpr0521_als_enable_store);
static struct device_attribute als_lux_attribute        = __ATTR(lux,    0664, rpr0521_als_lux_show,    rpr0521_als_lux_store);
static struct device_attribute als_poll_delay_attribute = __ATTR(delay,  0664, rpr0521_als_delay_show,  rpr0521_als_delay_store);
static struct device_attribute als_raw_data             = __ATTR(als_raw,0664, rpr0521_als_raw_show,    NULL);


static struct device_attribute ps_enable_attribute      = __ATTR(enable,  0666, rpr0521_ps_enable_show, rpr0521_ps_enable_store);
static struct device_attribute ps_raw_attribute         = __ATTR(ps_raw,  0664, rpr0521_ps_lux_show,    NULL);
static struct device_attribute ps_code_thd_l_attribute  = __ATTR(codethdl,0664, rpr0521_ps_code_thd_l_show, rpr0521_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute  = __ATTR(codethdh,0664, rpr0521_ps_code_thd_h_show, rpr0521_ps_code_thd_h_store);
static struct device_attribute all_reg_attribute        = __ATTR(allreg,  0444, rpr0521_all_reg_show,    NULL);

static struct attribute *rpr0521_als_attrs [] =
{
    &als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_poll_delay_attribute.attr,
    &als_raw_data.attr,
    NULL
};

static struct attribute_group rpr0521_als_attribute_group = {
    .name = "driver",
    .attrs = rpr0521_als_attrs,
};

static struct attribute *rpr0521_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    &ps_raw_attribute.attr,
    &ps_code_thd_l_attribute.attr,
    &ps_code_thd_h_attribute.attr,
    &all_reg_attribute.attr,
    NULL
};

static struct attribute_group rpr0521_ps_attribute_group = {
    .name = "driver",
    .attrs = rpr0521_ps_attrs,
};

 
static int rpr521_open(struct inode *inode, struct file *file)
{
	file->private_data = rpr0521_i2c_data;

	if (!file->private_data)
	{
		printk(KERN_ERR "null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

static int rpr521_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static struct file_operations rpr521_fops = {
	.owner = THIS_MODULE,
	.open = rpr521_open,
	.release = rpr521_release,
};

static struct miscdevice rpr521_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &rpr521_fops,
};

static ssize_t show_chipinfo_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = rpr0521_i2c_data->client;

	if(NULL == client)
	{
		printk(KERN_ERR "i2c client is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "rpr0521\n");
}
static DEVICE_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);

/*-------------------------------------------------------------------------------*/
static enum hrtimer_restart rpr0521_als_timer_func(struct hrtimer *timer)
{
    struct rpr0521_data *alsps_data = container_of(timer, struct rpr0521_data, als_timer);
    queue_work(alsps_data->rpr0521_als_wq, &alsps_data->rpr0521_als_work);
    hrtimer_forward_now(&alsps_data->als_timer, alsps_data->als_poll_delay);
    return HRTIMER_RESTART;

}

/******************************************************************************
 * NAME       : rpr0521_driver_read_data
 * FUNCTION   : read the value of RGB data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_read_data(struct rpr0521_data * alsps_data, READ_DATA_ARG *data)
{
    int result;
    unsigned char  read_data[6] = {0};

    if (NULL == alsps_data || NULL == data)
    {
        printk(" Parameter error \n");
        return EINVAL;
    }

    /* block read */
    result = rpr0521_i2c_read_data(alsps_data->client, RPR0521_REG_PDATAL_ADDR, sizeof(read_data), read_data);
    if (result < 0) {
        printk( "ps_rpr0521_driver_general_read : transfer error \n");
    } else {
        data->pdata  =  (unsigned short )(((unsigned short )read_data[1] << 8) | read_data[0]);
        data->adata0 = (unsigned short )(((unsigned short )read_data[3] << 8) | read_data[2]);
        data->adata1 = (unsigned short )(((unsigned short )read_data[5] << 8) | read_data[4]);
        result       = 0;
    }

    alsps_data->ps_raw_data = data->pdata;   

    return (result);
}

/*===========================================================================

  FUNCTION      rpr0521_als_convert_to_mlux

  DESCRIPTION   Convert a raw data to a real milli lux

  DEPENDENCIES  None

  RETURN VALUE  milli lux value or 0 if there was a error

  SIDE EFFECT   None

  ===========================================================================*/
unsigned int rpr0521_als_convert_to_mlux( unsigned short  data0, unsigned short  data1, unsigned short  gain_index, unsigned short  time )
{

#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE     (65535)  //grace modify in 2014.4.9
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)
#define MLUX_UNIT        (1)    // Lux to mLux
#define CALC_ERROR       (0xFFFFFFFF)

    unsigned int       final_data;
    calc_data_type     calc_data;
    calc_ans_type      calc_ans;
    unsigned long      calc_judge;
    unsigned char      set_case;
    unsigned long      max_range;
    unsigned char      gain_factor;

    
 
    calc_data.als_data0  = data0;
    calc_data.als_data1  = data1;
    gain_factor          = gain_index & 0x0F;
    calc_data.gain_data0 = gain_table[gain_factor].data0;

    max_range = (unsigned long)MAX_OUTRANGE;

    
    if (calc_data.als_data0 == MAXRANGE_NMODE)
    {
        calc_ans.positive = max_range;
    }
    else
    {
        
        calc_data.als_time = time ;
        if (calc_data.als_time == 0)
        {
             
            printk("calc_data.als_time  == 0\n");  
            return (CALC_ERROR);
        }

        calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
        if (calc_judge < (calc_data.als_data0 * judge_coefficient[0]))
        {
            set_case = 0;
        }
        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
        {
            set_case = 1;
        }
        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2]))
        {
            set_case = 2;
        }
        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3]))
        {
            set_case = 3;
        }
        else
        {
            set_case = MAXSET_CASE;
        }

        
        if (set_case >= MAXSET_CASE)
        {
            calc_ans.positive = 0;  
        }
        else
        {
            calc_data.gain_data1 = gain_table[gain_factor].data1;
             
            calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
            calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
            if(calc_data.data0 < calc_data.data1)    
            {
                printk("rpr0521 calc_data.data0 < calc_data.data1\n");
                return (CALC_ERROR);
            }
             

            calc_data.data     = calc_data.data0 - calc_data.data1;
            calc_data.dev_unit = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;    
            if (calc_data.dev_unit == 0)
            {
                
                printk("rpr0521 calc_data.dev_unit == 0\n");
                return (CALC_ERROR);
            }

           
            calc_ans.positive = (unsigned long)((unsigned long)(calc_data.data) / calc_data.dev_unit);
            if (calc_ans.positive > max_range)
            {
                calc_ans.positive = max_range;
            } else {
                
            }
        }
    }

    final_data = calc_ans.positive;  

    
    return (final_data);

#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
#undef MLUX_UNIT
}

static void rpr0521_als_poll_work_func(struct work_struct *work)
{
/*
    struct rpr0521_data *alsps_data = container_of(work, struct rpr0521_data, rpr0521_als_work);
    unsigned int reading_lux;

    reading_lux = rpr0521_get_als_value(alsps_data);
    alsps_data->als_lux_last = reading_lux;

	input_report_abs(alsps_data->als_input_dev, ABS_MISC, reading_lux);
	input_sync(alsps_data->als_input_dev);

    //printk("als input event %d lux\n", reading_lux);
	return;
*/

#if 1
    struct rpr0521_data *alsps_data = container_of(work, struct rpr0521_data, rpr0521_als_work);
    unsigned int reading_lux;

    reading_lux = rpr0521_get_als_value(alsps_data);

	if(-1 != alsps_data->als_lux_last)
	{
		if((reading_lux <= alsps_data->als_lux_last * 7/10) || (reading_lux >= alsps_data->als_lux_last * 12/10))
		{
			alsps_data->als_lux_last = reading_lux;
#if 0
            if (reading_lux < 300)
            {
                reading_lux *= 3;
            }
            else if (reading_lux < 1000)
            {
                reading_lux = reading_lux*8/5 + 420;
            }
            else
            {
                reading_lux *= 2;
            }
#endif
			input_report_abs(alsps_data->als_input_dev, ABS_MISC, reading_lux);
			input_sync(alsps_data->als_input_dev);
		}
	}
	return;
#endif
}

/*===========================================================================

  FUNCTION      rpr0521_ps_get_real_value

  DESCRIPTION   This function is called to get real proximity status which will be saved to obj->prx_detection_state

  DEPENDENCIES  None

  RETURN VALUE  None

  SIDE EFFECT   None

  ===========================================================================*/
static void rpr0521_ps_get_real_value(struct rpr0521_data * alsps_data)
{
    int       result;
    unsigned short       pdata ;

    pdata = alsps_data->ps_raw_data;

    printk("obj->thresh_near(0x%x) obj->thresh_far(0x%x)\n", alsps_data->thresh_near, alsps_data->thresh_far);
	printk("qmm %s pdata = %d\n", __func__, pdata);
	
    if ( pdata >= alsps_data->thresh_near )
    {
        
        result = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, RPR0521_REG_PRX_ADDR);
        if ( result < 0 )
        {
            printk("Read data from IC error.\n");
            return;
        }
        printk("RPR0521_PRX_ADDR(0x%x) value = 0x%x\n", RPR0521_REG_PRX_ADDR, result);

        if ( 0 == (result >>  RPR0521_AMBIENT_IR_FLAG) )  
        {
             
            if ( alsps_data->last_nearby != PRX_NEAR_BY )
            {
                alsps_data->prx_detection_state = PRX_NEAR_BY;   
            }
            alsps_data->last_nearby = PRX_NEAR_BY;

            if (alsps_data->polling_mode_ps == 0)  
            {
                rpr0521_set_prx_thresh(alsps_data, alsps_data->thresh_far, RPR0521_PIHT_NEAR );  
            }
        }
        else  
        {
             
            if ( alsps_data->last_nearby != PRX_FAR_AWAY )
            {
                alsps_data->prx_detection_state = PRX_FAR_AWAY;  
                alsps_data->last_nearby = PRX_FAR_AWAY;
                rpr0521_set_prx_thresh(alsps_data, RPR0521_PILT_FAR, alsps_data->thresh_near);   
            }
        }
    }
    else if ( pdata < alsps_data->thresh_far )
    {
         
        if ( alsps_data->last_nearby != PRX_FAR_AWAY )
        {
            alsps_data->prx_detection_state = PRX_FAR_AWAY;  
        }
        alsps_data->last_nearby = PRX_FAR_AWAY;

        if (alsps_data->polling_mode_ps == 0)  
        {
            rpr0521_set_prx_thresh(alsps_data, RPR0521_PILT_FAR, alsps_data->thresh_near);   
        }
    }
}


 
static void rpr0521_work_func(struct work_struct *work)
{
	int result;
	int zte_state = 0;

    struct rpr0521_data *alsps_data = container_of(work, struct rpr0521_data, rpr0521work);

	printk("********************* qmm %s\n", __func__);
 
    result = rpr0521_i2c_smbus_read_byte_data(alsps_data->client, RPR0521_REG_INTERRUPT_ADDR);
    if ( result < 0 )
    {
        printk("Read data from IC error.\n");
  	    enable_irq(alsps_data->irq);
        return;
    }


 
    if ((result & RPR0521_REG_PINT_STATUS ))
    {
        alsps_data->ps_raw_data = rpr0521_get_ps_raw_data(alsps_data);
        if ( alsps_data->ps_raw_data < 0 )
        {
            printk("Read data from IC error.\n");
      	    enable_irq(alsps_data->irq);
            return;
        }

        rpr0521_ps_get_real_value(alsps_data);  
		 
		
		if(0 == alsps_data->prx_detection_state || 1 == alsps_data->prx_detection_state)
			zte_state = 1 - alsps_data->prx_detection_state;
		 
        input_report_abs(alsps_data->ps_input_dev, ABS_DISTANCE, zte_state);
        input_sync(alsps_data->ps_input_dev);
        wake_lock_timeout(&alsps_data->ps_wakelock, 3*HZ);
	}

    enable_irq(alsps_data->irq);

    return;
}

static irqreturn_t rpr0521_irq_handler(int irq, void *data)
{
    struct rpr0521_data *pData = data;
    printk(KERN_INFO "%s: proximity interrupt.\n", __func__);
    if(pData == NULL)
    {
        printk("ERROR NULL pointer!!!");
        return IRQ_HANDLED;
    }

    disable_irq_nosync(irq);
    queue_work(pData->rpr0521wq, &pData->rpr0521work);
    return IRQ_HANDLED;
}

static int rpr0521_init_all_setting(struct i2c_client *client)
{
    int ret;
    struct rpr0521_data *alsps_data = i2c_get_clientdata(client);

    ret = rpr0521_software_reset(alsps_data);
    if(ret < 0)
        return ret;


    alsps_data->last_nearby = PRX_NEAR_BY_UNKNOWN;
    alsps_data->thresh_near = RPR521_PS_NOISE_DEFAULT + RPR0521_PRX_NEAR_THRESHOLD;  // Todo
    alsps_data->thresh_far  = RPR521_PS_NOISE_DEFAULT + RPR0521_PRX_FAR_THRESHOLD;
    alsps_data->int_pin     = RPR0521_INTERRUPT_GPIO; //gpio


    ret = rpr0521_init_all_reg(alsps_data);
    if(ret < 0)
        return ret;

    alsps_data->als_enabled = false;
    alsps_data->ps_enabled  = false;
    alsps_data->als_suspend = false;
    alsps_data->ps_suspend  = false;
    alsps_data->first_boot  = true;

    alsps_data->polling_mode_ps   =  0;  
    alsps_data->polling_mode_als  =  1;  

#ifdef RPR0521_PS_CALIBRATIOIN_ON_START
    
    ret = rpr0521_ps_calibration(alsps_data);
    if(ret == 0)
    {
        printk("ps calibration success! \n");
    }
    else
    {
        printk("ps calibration failed! \n");
    }

#endif

    
    if(0 == alsps_data->polling_mode_ps)
        rpr0521_set_prx_thresh(alsps_data, alsps_data->thresh_far, alsps_data->thresh_near);

    return 0;
}

 
static int rpr0521_setup_irq(struct i2c_client *client)
{
    int irq, err = -EIO;
    struct rpr0521_data *alsps_data = i2c_get_clientdata(client);

    err = gpio_request(alsps_data->int_pin,"rpr-int");
    if(err < 0)
    {
        printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
        return err;
    }
    err = gpio_direction_input(alsps_data->int_pin);
    if(err < 0)
    {
        printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
        gpio_free(alsps_data->int_pin);
        return err;
    }

    irq = gpio_to_irq(alsps_data->int_pin);

#ifdef RPR_DEBUG_PRINTF
    printk(KERN_INFO "%s: int pin #=%d, irq=%d\n",__func__, alsps_data->int_pin, irq);
#endif
    if (irq <= 0)
    {
        printk(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, alsps_data->int_pin);
        return irq;
    }

    alsps_data->irq = irq;

	err = request_any_context_irq(irq, rpr0521_irq_handler, IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND|IRQF_ONESHOT, RPR0521_DEV_NAME, alsps_data);
    if (err < 0)
    {
        printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);
        gpio_free(alsps_data->int_pin);
        return err;
    }
    irq_set_irq_wake(irq, 1);
    disable_irq(irq);

    return 0;
}

static int rpr0521_suspend(struct device *dev)
{
    struct rpr0521_data *alsps_data = dev_get_drvdata(dev);

    printk(KERN_INFO "%s\n", __func__);

    if(alsps_data->als_enabled)
    {
        printk(KERN_INFO "%s: Enable ALS : 0\n", __func__);
        rpr0521_enable_als(alsps_data, POWER_OFF);
        alsps_data->als_suspend = true;
    }

	if(alsps_data->ps_enabled)
	{
		//rpr0521_enable_ps(alsps_data, POWER_OFF);
		alsps_data->ps_suspend = true;
    }

    return 0;
}

static int rpr0521_resume(struct device *dev)
{
    struct rpr0521_data *alsps_data = dev_get_drvdata(dev);

    printk(KERN_INFO "%s\n", __func__);

    if(alsps_data->als_suspend)
    {
        printk(KERN_INFO "%s: Enable ALS : 1\n", __func__);
        rpr0521_enable_als(alsps_data, POWER_ON);
        alsps_data->als_suspend = false;
    }

    if(alsps_data->ps_suspend)
	{
		//rpr0521_enable_ps(alsps_data, POWER_ON);
        alsps_data->ps_suspend = false;

    }

    return 0;
}

static const struct dev_pm_ops rpr0521_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rpr0521_suspend, rpr0521_resume)
};

#if 0
//#ifdef CONFIG_HAS_EARLYSUSPEND
static void rpr0521_early_suspend(struct early_suspend *handler)
{
    rpr0521_suspend(&rpr0521_i2c_data->client->dev);
}

static void rpr0521_late_resume(struct early_suspend *handler)
{
    rpr0521_resume(&rpr0521_i2c_data->client->dev);
}
#endif    //#ifdef CONFIG_HAS_EARLYSUSPEND

static int rpr0521_set_wq(struct rpr0521_data *alsps_data)
{
    unsigned short als_it = alsps_data->als_measure_time;  

     
    alsps_data->rpr0521_als_wq = create_singlethread_workqueue("rpr0521_als_wq");
    INIT_WORK(&alsps_data->rpr0521_als_work, rpr0521_als_poll_work_func);
    hrtimer_init(&alsps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    if(als_it == CALC_MEASURE_100MS)
        alsps_data->als_poll_delay = ns_to_ktime(CALC_MEASURE_100MS * NSEC_PER_MSEC);
    else
    {
        alsps_data->als_poll_delay = ns_to_ktime(CALC_MEASURE_400MS * NSEC_PER_MSEC);
        printk(KERN_INFO "%s: unknown ALS_IT=%d, set als_poll_delay=110ms\n", __func__, als_it);
    }
    alsps_data->als_timer.function = rpr0521_als_timer_func;


     
    alsps_data->rpr0521wq = create_singlethread_workqueue("rpr0521wq");
    INIT_WORK(&alsps_data->rpr0521work, rpr0521_work_func);

    return 0;
}


static int rpr0521_set_input_devices(struct rpr0521_data *alsps_data)
{
    int err;

    alsps_data->als_input_dev = input_allocate_device();
    if (alsps_data->als_input_dev == NULL)
    {
        printk(KERN_ERR "%s: could not allocate als device\n", __func__);
        err = -ENOMEM;
        return err;
    }
    alsps_data->ps_input_dev = input_allocate_device();
    if (alsps_data->ps_input_dev == NULL)
    {
        printk(KERN_ERR "%s: could not allocate ps device\n", __func__);
        err = -ENOMEM;
        return err;
    }
    alsps_data->als_input_dev->name = RPR0521_ALS_NAME;   
    alsps_data->ps_input_dev->name  = RPR0521_PS_NAME;    
    set_bit(EV_ABS, alsps_data->als_input_dev->evbit);
    set_bit(EV_ABS, alsps_data->ps_input_dev->evbit);
    input_set_abs_params(alsps_data->als_input_dev, ABS_MISC, 0, 43000, 0, 0);   
    input_set_abs_params(alsps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
    err = input_register_device(alsps_data->als_input_dev);
    if (err<0)
    {
        printk(KERN_ERR "%s: can not register als input device\n", __func__);
        return err;
    }
    err = input_register_device(alsps_data->ps_input_dev);
    if (err<0)
    {
        printk(KERN_ERR "%s: can not register ps input device\n", __func__);
        return err;
    }

    err = sysfs_create_group(&alsps_data->als_input_dev->dev.kobj, &rpr0521_als_attribute_group);
    if (err < 0)
    {
        printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
        return err;
    }
    err = sysfs_create_group(&alsps_data->ps_input_dev->dev.kobj, &rpr0521_ps_attribute_group);
    if (err < 0)
    {
        printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
        return err;
    }
    input_set_drvdata(alsps_data->als_input_dev, alsps_data);
    input_set_drvdata(alsps_data->ps_input_dev, alsps_data);


    return 0;
}

#ifdef RPR0521_PS_CALIBRATIOIN_ON_CALL
static struct proc_dir_entry *calibration_inCall = NULL;
static ssize_t calibration_inCall_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
    int ret = 0;
    if(*offset != 0)
    {
        printk(KERN_ERR "%s,return 0\n", __FUNCTION__);
        return 0;
    }
    
    ret = rpr0521_ps_calibration(rpr0521_i2c_data);
    if(ret == 0)
    {
        printk("%s: ps calibration success! \n", __func__);
    }
    else
    {
        printk("ps calibration failed! \n");
    }
    
     
    rpr0521_set_prx_thresh(rpr0521_i2c_data, rpr0521_i2c_data->thresh_far, rpr0521_i2c_data->thresh_near);

    offset += sprintf(buffer, "%d", ret);
    return ret;
}
static const struct file_operations calibration_inCall_fops = {
	.owner		= THIS_MODULE,
	.read       = calibration_inCall_read,
};
static void create_calibrate_proc_file(void)
{
	calibration_inCall = proc_create("driver/calibration_inCall", 0444, NULL, &calibration_inCall_fops);
    if(NULL == calibration_inCall)
	{
	    printk(KERN_ERR "create_proc_file calibration_inCall fail!\n");
	}
}
#endif

static int rpr0521_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err = -ENODEV;
    struct rpr0521_data *alsps_data;

    printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);
	
 
	err = rpr0521_i2c_smbus_read_byte_data(client, RPR0521_REG_ID_ADDR);
	printk("****** QMM ****** %s compatible read 0x%x\n", __func__, err);
	if(RPR0521_REG_ID_VALUE != (err & 0x3F))
	{
		printk("****** QMM ****** %s Not ROHM chip, return!\n", __func__);
		return -ENODEV;
	}
	printk("****** QMM ****** %s Use ROHM chip!\n", __func__);
 

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_I2C\n", __func__);
        return -ENODEV;
    }

    alsps_data = kzalloc(sizeof(struct rpr0521_data),GFP_KERNEL);
    if(!alsps_data)
    {
        printk(KERN_ERR "%s: failed to allocate rpr0521_data\n", __func__);
        return -ENOMEM;
    }
    memset(alsps_data, 0, sizeof(struct rpr0521_data));

    alsps_data->client = client;
    i2c_set_clientdata(client,alsps_data);

    err = rpr0521_check_id(alsps_data);
    if(err < 0)
    {
        printk(KERN_ERR "%s: check_id fail (%d)", __func__, err);
        kfree(alsps_data);
        return err;
    }

    rpr0521_i2c_data = alsps_data;

    mutex_init(&alsps_data->io_lock);
    wake_lock_init(&alsps_data->ps_wakelock, WAKE_LOCK_SUSPEND, "rpr052_1input_wakelock");


    err = rpr0521_init_all_setting(client);
    if(err < 0)
        goto err_init_all_setting;

     
    rpr0521_set_wq(alsps_data);



    err = rpr0521_set_input_devices(alsps_data);
    if(err < 0)
        goto err_setup_input_device;

    
    if((err = misc_register(&rpr521_device)))
	{
		printk(KERN_ERR "rpr521_device register failed\n");
		goto exit_misc_device_register_failed;
	}
    if((device_create_file(rpr521_device.this_device, &dev_attr_chipinfo)) < 0)
	{
		printk(KERN_ERR "create attribute err.\n");
		goto exit_create_attr_failed;
	}

     
    err = rpr0521_setup_irq(client);
    if(err < 0)
        goto err_rpr0521_setup_irq;

    device_init_wakeup(&client->dev, true);
#ifdef RPR0521_PS_CALIBRATIOIN_ON_CALL
    create_calibrate_proc_file();
#endif
#if 0
//#ifdef CONFIG_HAS_EARLYSUSPEND
    alsps_data->rpr0521_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    alsps_data->rpr0521_early_suspend.suspend = rpr0521_early_suspend;
    alsps_data->rpr0521_early_suspend.resume  = rpr0521_late_resume;
    register_early_suspend(&alsps_data->rpr0521_early_suspend);
#endif

    printk(KERN_INFO "%s: probe successfully\n", __func__);
    return 0;

err_rpr0521_setup_irq:
    free_irq(alsps_data->irq, alsps_data);
    gpio_free(alsps_data->int_pin);
    
    device_remove_file(&rpr521_device.this_device, &dev_attr_chipinfo);
exit_create_attr_failed:
    misc_deregister(&rpr521_device);
exit_misc_device_register_failed:
    
err_setup_input_device:
    sysfs_remove_group(&alsps_data->ps_input_dev->dev.kobj, &rpr0521_ps_attribute_group);
    sysfs_remove_group(&alsps_data->als_input_dev->dev.kobj, &rpr0521_als_attribute_group);
    input_unregister_device(alsps_data->ps_input_dev);
    input_unregister_device(alsps_data->als_input_dev);
    input_free_device(alsps_data->ps_input_dev);
    input_free_device(alsps_data->als_input_dev);
err_init_all_setting:
    hrtimer_try_to_cancel(&alsps_data->als_timer);
    destroy_workqueue(alsps_data->rpr0521_als_wq);

    destroy_workqueue(alsps_data->rpr0521wq);

    wake_lock_destroy(&alsps_data->ps_wakelock);
    mutex_destroy(&alsps_data->io_lock);
    kfree(alsps_data);
    return err;
}

static int rpr0521_remove(struct i2c_client *client)
{
    struct rpr0521_data *alsps_data = i2c_get_clientdata(client);

    device_init_wakeup(&client->dev, false);

    free_irq(alsps_data->irq, alsps_data);
    gpio_free(alsps_data->int_pin);



    sysfs_remove_group(&alsps_data->ps_input_dev->dev.kobj, &rpr0521_ps_attribute_group);
    sysfs_remove_group(&alsps_data->als_input_dev->dev.kobj, &rpr0521_als_attribute_group);
    input_unregister_device(alsps_data->ps_input_dev);
    input_unregister_device(alsps_data->als_input_dev);
    input_free_device(alsps_data->ps_input_dev);
    input_free_device(alsps_data->als_input_dev);

    hrtimer_try_to_cancel(&alsps_data->als_timer);
    destroy_workqueue(alsps_data->rpr0521_als_wq);


    wake_lock_destroy(&alsps_data->ps_wakelock);
    mutex_destroy(&alsps_data->io_lock);
    kfree(alsps_data);
    rpr0521_i2c_data = NULL;

    return 0;
}

static const struct i2c_device_id rpr0521_id[] =
{
    { RPR0521_I2C_DRIVER_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, rpr0521_id);

static struct of_device_id rpr0521_match_table[] = {
    { .compatible = "rohm,rpr0521_i2c", },
    { },
};

static struct i2c_driver rpr0521_alsps_driver =
{
    .driver = {
        .name  = RPR0521_I2C_DRIVER_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = rpr0521_match_table,
#endif
        .pm = &rpr0521_pm_ops,
    },
    .probe  = rpr0521_probe,
    .remove = rpr0521_remove,
    .id_table = rpr0521_id,
};

static int __init rpr0521_init(void)
{
    int ret;
    printk(KERN_INFO "%s: rpr0521 driver initial.\n", __func__);
    ret = i2c_add_driver(&rpr0521_alsps_driver);
    if (ret)
    {
        i2c_del_driver(&rpr0521_alsps_driver);
        return ret;
    }

    return 0;
}

static void __exit rpr0521_exit(void)
{
    i2c_del_driver(&rpr0521_alsps_driver);
}

module_init(rpr0521_init);
module_exit(rpr0521_exit);
MODULE_AUTHOR("Aaron Liu<aaron-liu@rohm.com.cn>");
MODULE_DESCRIPTION("Rohm ALSPS Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
