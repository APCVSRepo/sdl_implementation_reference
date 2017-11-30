/* drivers/input/touchscreen/gt9xx.c
 * 
 * 2010 - 2013 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 */

#include <linux/irq.h>
#include "gt9xx.h"

#if GT9XX_ICS_SLOT_REPORT
    #include <linux/input/mt.h>
#endif

#define GTP_POLL_TIME 10
static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * gt9xx_i2c_connect_client = NULL; 
u8 gt9xx_config[GT9XX_CONFIG_MAX_LENGTH + GT9XX_ADDR_LENGTH]
                = {GT9XX_REG_CONFIG_DATA >> 8, GT9XX_REG_CONFIG_DATA & 0xff};

#if GT9XX_HAVE_TOUCH_KEY
    static const u16 touch_key_array[] = GT9XX_KEY_TAB;
    #define GT9XX_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
    
#if GT9XX_DEBUG_ON
    static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH};
    static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif
    
#endif

static s8 gt9xx_i2c_test(struct i2c_client *client);
void gt9xx_gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gt9xx_gtp_send_cfg(struct i2c_client *client);
static void gt9xx_int_sync(s32 ms, struct goodix_gt9xx_ts_data *ts);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};

#if GT9XX_CREATE_WR_NODE
extern s32 gt9xx_init_wr_node(struct i2c_client*);
extern void gt9xx_uninit_wr_node(void);
#endif

#if GT9XX_AUTO_UPDATE
extern u8 gt9xx_gup_init_update_proc(struct goodix_gt9xx_ts_data *);
#endif

#if GT9XX_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gt9xx_gtp_esd_switch(struct i2c_client *, s32);
#endif

//*********** For GT9XXF Start **********//
#if GT9XX_COMPATIBLE_MODE
extern s32 gt9xx_i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gt9xx_i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gt9xx_gup_clk_calibration(void);
extern s32 gt9xx_gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gt9xx_gup_check_fs_mounted(char *path_name);

void gt9xx_gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gt9xx_gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_gt9xx_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_gt9xx_ts_data *ts, u8 mode);

#endif
//********** For GT9XXF End **********//

typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
#if GT9XX_GESTURE_WAKEUP
static s8 gt9xx_enter_doze(struct goodix_gt9xx_ts_data *ts);
#endif

static u8 grp_cfg_version = 0;

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GT9XX_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer: 
      2: succeed, otherwise: failed
*********************************************************/
static s32 gt9xx_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GT9XX_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GT9XX_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 400*1000;
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GT9XX_ADDR_LENGTH;
    msgs[1].buf   = &buf[GT9XX_ADDR_LENGTH];
    //msgs[1].scl_rate = 400 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GT9XX_COMPATIBLE_MODE
        struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(client);
    #endif
    
    #if GT9XX_GESTURE_WAKEUP
        // reset chip would quit doze mode
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GT9XX_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GT9XX_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        { 
            gt9xx_gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gt9xx_gtp_reset_guitar(client, 10);  
        }
    }
    return ret;
}



/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GT9XX_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer: 
        1: succeed, otherwise: failed
*********************************************************/
static s32 gt9xx_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GT9XX_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
   // msg.scl_rate = 400 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GT9XX_COMPATIBLE_MODE
        struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(client);
    #endif
    
    #if GT9XX_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GT9XX_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GT9XX_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        { 
            gt9xx_gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gt9xx_gtp_reset_guitar(client, 10);  
        }
    }
    return ret;
}


/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gt9xx_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;
    
    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gt9xx_i2c_read(client, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gt9xx_i2c_read(client, confirm_buf, len + 2);
        
        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    GT9XX_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
s32 gt9xx_gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;

#if GT9XX_DRIVER_SEND_CFG
    s32 retry = 0;
    struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(client);

    if (ts->fixed_cfg)
    {
        GT9XX_INFO("Ic fixed config, no config sent!");
        return 0;
    }
    else if (ts->pnl_init_error)
    {
        GT9XX_INFO("Error occured in init_panel, no config sent");
        return 0;
    }
    
    GT9XX_INFO("Driver send config.");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gt9xx_i2c_write(client, gt9xx_config , GT9XX_CONFIG_MAX_LENGTH + GT9XX_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif
    return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gt9xx_gtp_irq_disable(struct goodix_gt9xx_ts_data *ts)
{
    unsigned long irqflags;

    GT9XX_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gt9xx_gtp_irq_enable(struct goodix_gt9xx_ts_data *ts)
{
    unsigned long irqflags = 0;

    GT9XX_DEBUG_FUNC();
    
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event 
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gt9xx_touch_down(struct goodix_gt9xx_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GT9XX_CHANGE_X2Y
    GT9XX_SWAP(x, y);
#endif

#if GT9XX_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 1); 
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->input_dev);
#endif

    GT9XX_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gt9xx_touch_up(struct goodix_gt9xx_ts_data* ts, s32 id)
{
#if GT9XX_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
    GT9XX_DEBUG("Touch id[%2d] release!", id);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 0); 
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
    input_mt_sync(ts->input_dev);
#endif
}

#if GT9XX_WITH_PEN

static void gt9xx_pen_init(struct goodix_gt9xx_ts_data *ts)
{
    s32 ret = 0;
    
    GT9XX_INFO("Request input device for pen/stylus.");
    
    ts->pen_dev = input_allocate_device();
    if (ts->pen_dev == NULL)
    {
        GT9XX_ERROR("Failed to allocate input device for pen/stylus.");
        return;
    }
    
    ts->pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    
#if GT9XX_ICS_SLOT_REPORT
    input_mt_init_slots(ts->pen_dev, 16);               // 
#else
    ts->pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

    set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, ts->pen_dev->propbit);
    //set_bit(INPUT_PROP_POINTER, ts->pen_dev->propbit);
    
#if GT9XX_PEN_HAVE_BUTTON
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS);
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS2);
#endif

    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
    
    ts->pen_dev->name = "goodix-pen";
    ts->pen_dev->id.bustype = BUS_I2C;
    
    ret = input_register_device(ts->pen_dev);
    if (ret)
    {
        GT9XX_ERROR("Register %s input device failed", ts->pen_dev->name);
        return;
    }
}

static void gt9xx_pen_down(s32 x, s32 y, s32 w, s32 id)
{
    struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(gt9xx_i2c_connect_client);

#if GT9XX_CHANGE_X2Y
    GT9XX_SWAP(x, y);
#endif
    
    input_report_key(ts->pen_dev, BTN_TOOL_PEN, 1);
#if GT9XX_ICS_SLOT_REPORT
    input_mt_slot(ts->pen_dev, id);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
    input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
#else
    input_report_key(ts->pen_dev, BTN_TOUCH, 1);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
    input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->pen_dev);
#endif
    GT9XX_DEBUG("(%d)(%d, %d)[%d]", id, x, y, w);
}

static void gt9xx_pen_up(s32 id)
{
    struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(gt9xx_i2c_connect_client);
    
    input_report_key(ts->pen_dev, BTN_TOOL_PEN, 0);
    
#if GT9XX_ICS_SLOT_REPORT
    input_mt_slot(ts->pen_dev, id);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, -1);
#else
    
    input_report_key(ts->pen_dev, BTN_TOUCH, 0);
#endif

}
#endif

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void gt9xx_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GT9XX_READ_COOR_ADDR >> 8, GT9XX_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GT9XX_MAX_TOUCH + 1]={GT9XX_READ_COOR_ADDR >> 8, GT9XX_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
#if GT9XX_WITH_PEN
    u8 pen_active = 0;
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_gt9xx_ts_data *ts = NULL;

#if GT9XX_COMPATIBLE_MODE
    u8 rqst_buf[3] = {0x80, 0x43};  // for GT9XXF
#endif

#if GT9XX_GESTURE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#endif

    GT9XX_DEBUG_FUNC();
    ts = container_of(work, struct goodix_gt9xx_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
#if GT9XX_GESTURE_WAKEUP
    if (DOZE_ENABLED == doze_status)
    {               
        ret = gt9xx_i2c_read(gt9xx_i2c_connect_client, doze_buf, 3);
        GT9XX_DEBUG("0x814B = 0x%02X", doze_buf[2]);
        if (ret > 0)
        {     
            if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
                (doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') || 
                (doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
                (doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') || 
                (doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
                (doze_buf[2] == 0x5E) /* ^ */
                )
            {
                if (doze_buf[2] != 0x5E)
                {
                    GT9XX_INFO("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
                }
                else
                {
                    GT9XX_INFO("Wakeup by gesture(^), light up the screen!");
                }
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gt9xx_i2c_write(gt9xx_i2c_connect_client, doze_buf, 3);
			}
			else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
				(doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
            {
                char *direction[4] = {"Right", "Down", "Up", "Left"};
                u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;
                
                GT9XX_INFO("%s slide to light up the screen!", direction[type]);
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gt9xx_i2c_write(gt9xx_i2c_connect_client, doze_buf, 3);
            }
            else if (0xCC == doze_buf[2])
            {
                GT9XX_INFO("Double click to light up the screen!");
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gt9xx_i2c_write(gt9xx_i2c_connect_client, doze_buf, 3);
            }
            else
            {
                // clear 0x814B
                doze_buf[2] = 0x00;
                gt9xx_i2c_write(gt9xx_i2c_connect_client, doze_buf, 3);
                gt9xx_enter_doze(ts);
            }
        }
        if (ts->use_irq)
        {
            gt9xx_gtp_irq_enable(ts);
        }
        return;
    }
#endif

    ret = gt9xx_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GT9XX_ERROR("I2C transfer error. errno:%d\n ", ret);
        if (ts->use_irq)
        {
            gt9xx_gtp_irq_enable(ts);
        }
        return;
    }
    
    finger = point_data[GT9XX_ADDR_LENGTH];
	
#if GT9XX_COMPATIBLE_MODE
    // GT9XXF
    if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type))     // request arrived
    {
        ret = gt9xx_i2c_read(ts->client, rqst_buf, 3);
        if (ret < 0)
        {
           GT9XX_ERROR("Read request status error!");
           goto exit_work_func;
        } 
        
        switch (rqst_buf[2])
        {
        case GT9XX_RQST_CONFIG:
            GT9XX_INFO("Request for config.");
            ret = gt9xx_gtp_send_cfg(ts->client);
            if (ret < 0)
            {
                GT9XX_ERROR("Request for config unresponded!");
            }
            else
            {
                rqst_buf[2] = GT9XX_RQST_RESPONDED;
                gt9xx_i2c_write(ts->client, rqst_buf, 3);
                GT9XX_INFO("Request for config responded!");
            }
            break;
            
        case GT9XX_RQST_BAK_REF:
            GT9XX_INFO("Request for backup reference.");
            ts->rqst_processing = 1;
            ret = gtp_bak_ref_proc(ts, GT9XX_BAK_REF_SEND);
            if (SUCCESS == ret)
            {
                rqst_buf[2] = GT9XX_RQST_RESPONDED;
                gt9xx_i2c_write(ts->client, rqst_buf, 3);
                ts->rqst_processing = 0;
                GT9XX_INFO("Request for backup reference responded!");
            }
            else
            {
                GT9XX_ERROR("Requeset for backup reference unresponed!");
            }
            break;
            
        case GT9XX_RQST_RESET:
            GT9XX_INFO("Request for reset.");
            gt9xx_gtp_recovery_reset(ts->client);
            break;
            
        case GT9XX_RQST_MAIN_CLOCK:
            GT9XX_INFO("Request for main clock.");
            ts->rqst_processing = 1;
            ret = gtp_main_clk_proc(ts);
            if (FAIL == ret)
            {
                GT9XX_ERROR("Request for main clock unresponded!");
            }
            else
            {
                GT9XX_INFO("Request for main clock responded!");
                rqst_buf[2] = GT9XX_RQST_RESPONDED;
                gt9xx_i2c_write(ts->client, rqst_buf, 3);
                ts->rqst_processing = 0;
                ts->clk_chk_fs_times = 0;
            }
            break;
            
        default:
            GT9XX_INFO("Undefined request: 0x%02X", rqst_buf[2]);
            rqst_buf[2] = GT9XX_RQST_RESPONDED;  
            gt9xx_i2c_write(ts->client, rqst_buf, 3);
            break;
        }
    }
#endif
    if (finger == 0x00)
    {
        if (ts->use_irq)
        {
            gt9xx_gtp_irq_enable(ts);
        }
        return;
    }

    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GT9XX_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GT9XX_MAX_TOUCH] = {(GT9XX_READ_COOR_ADDR + 10) >> 8, (GT9XX_READ_COOR_ADDR + 10) & 0xff};

        ret = gt9xx_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

    if(touch_num && DOZE_ENABLED == doze_status)  
    {
	 doze_status = DOZE_WAKEUP;
        input_report_key(ts->input_dev, KEY_POWER, 1);
        input_sync(ts->input_dev);
        input_report_key(ts->input_dev, KEY_POWER, 0);
        input_sync(ts->input_dev);

	return;
    } 

#if (GT9XX_HAVE_TOUCH_KEY || GT9XX_PEN_HAVE_BUTTON)
    key_value = point_data[3 + 8 * touch_num];
    
    if(key_value || pre_key)
    {
    #if GT9XX_PEN_HAVE_BUTTON
        if (key_value == 0x40)
        {
            GT9XX_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else if (key_value == 0x10)
        {
            GT9XX_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            pen_active = 1;
        }
        else if (key_value == 0x20)
        {
            GT9XX_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else
        {
            GT9XX_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            if ( (pre_key == 0x40) || (pre_key == 0x20) ||
                 (pre_key == 0x10) 
               )
            {
                pen_active = 1;
            }
        }
        if (pen_active)
        {
            touch_num = 0;      // shield pen point
            //pre_touch = 0;    // clear last pen status
        }
    #endif
    
    #if GT9XX_HAVE_TOUCH_KEY
        if (!pre_touch)
        {
            for (i = 0; i < GT9XX_MAX_KEY_NUM; i++)
            {
            #if GT9XX_DEBUG_ON
                for (ret = 0; ret < 4; ++ret)
                {
                    if (key_codes[ret] == touch_key_array[i])
                    {
                        GT9XX_DEBUG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
                        break;
                    }
                }
            #endif
                input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));   
            }
            touch_num = 0;  // shield fingers
        }
    #endif
    }
#endif
    pre_key = key_value;

    GT9XX_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GT9XX_ICS_SLOT_REPORT

#if GT9XX_WITH_PEN
    if (pre_pen && (touch_num == 0))
    {
        GT9XX_DEBUG("Pen touch UP(Slot)!");
        gt9xx_pen_up(0);
        pen_active = 1;
        pre_pen = 0;
    }
#endif
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;
        u8 report_num = 0;
        coor_data = &point_data[3];
        
        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;
        
        #if GT9XX_WITH_PEN
            id = coor_data[pos];
            if ((id & 0x80))  
            {
                GT9XX_DEBUG("Pen touch DOWN(Slot)!");
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);
                
                gt9xx_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pre_touch = 0;
                pen_active = 1;
            }    
        #endif
        
            touch_index |= (0x01<<id);
        }
        
        GT9XX_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",id, touch_index,pre_touch);
        for (i = 0; i < GT9XX_MAX_TOUCH; i++)
        {
        #if GT9XX_WITH_PEN
            if (pre_pen == 1)
            {
                break;
            }
        #endif
        
            if ((touch_index & (0x01<<i)))
            {
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                gt9xx_touch_down(ts, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;
                
                report_num++;
                if (report_num < touch_num)
                {
                    pos += 8;
                    id = coor_data[pos] & 0x0F;
                    touch_index |= (0x01<<id);
                }
            }
            else
            {
                gt9xx_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }
#else
    input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value)); 
    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | (coor_data[2] << 8);
            input_y  = coor_data[3] | (coor_data[4] << 8);
            input_w  = coor_data[5] | (coor_data[6] << 8);
        
        #if GT9XX_WITH_PEN
            id = coor_data[0];
            if (id & 0x80)
            {
                GT9XX_DEBUG("Pen touch DOWN!");
                gt9xx_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pen_active = 1;
                break;
            }
            else
        #endif
            {
                gt9xx_touch_down(ts, id, input_x, input_y, input_w);
            }
        }
    }
    else if (pre_touch)
    {
    #if GT9XX_WITH_PEN
        if (pre_pen == 1)
        {
            GT9XX_DEBUG("Pen touch UP!");
            gt9xx_pen_up(0);
            pre_pen = 0;
            pen_active = 1;
        }
        else
    #endif
        {
            GT9XX_DEBUG("Touch Release!");
            gt9xx_touch_up(ts, 0);
        }
    }

    pre_touch = touch_num;
#endif

#if GT9XX_WITH_PEN
    if (pen_active)
    {
        pen_active = 0;
        input_sync(ts->pen_dev);
    }
    else
#endif
    {
        input_sync(ts->input_dev);
    }

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gt9xx_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            GT9XX_INFO("I2C write end_cmd error!");
        }
    }
    if (ts->use_irq)
    {
        gt9xx_gtp_irq_enable(ts);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode. 
        HRTIMER_NORESTART: no restart mode
*********************************************************/
/*
static enum hrtimer_restart gt9xx_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_gt9xx_ts_data *ts = container_of(timer, struct goodix_gt9xx_ts_data, timer);

    GT9XX_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GT9XX_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}
*/

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
/*
static irqreturn_t gt9xx_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_gt9xx_ts_data *ts = dev_id;

    GT9XX_DEBUG_FUNC();
 
    gt9xx_gtp_irq_disable(ts);

    queue_work(goodix_wq, &ts->work);
    
    return IRQ_HANDLED;
}
*/

/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
static void gt9xx_int_sync(s32 ms, struct goodix_gt9xx_ts_data *ts)
{
    //GT9XX_GPIO_OUTPUT(gpio_to_irq(ts->irq_pin), 0);
	GT9XX_GPIO_OUTPUT(ts->irq_pin, 0);
    msleep(ms);
    GT9XX_GPIO_AS_INT(ts->irq_pin);
}


/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gt9xx_gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(client); 

    GT9XX_DEBUG_FUNC();
    GT9XX_INFO("Guitar reset");
    GT9XX_GPIO_OUTPUT(ts->rst_pin, 0);   // begin select I2C slave addr
    msleep(ms);                         // T2: > 10ms
    // HIGH: 0x28/0x29, LOW: 0xBA/0xBB
    GT9XX_GPIO_OUTPUT(ts->irq_pin, client->addr == 0x14);

    msleep(2);                          // T3: > 100us
    GT9XX_GPIO_OUTPUT(ts->rst_pin, 1);
    
    msleep(6);                          // T4: > 5ms

#if (GT9XX_CHIP == CHIP_GT911)
    GT9XX_GPIO_AS_INPUT(ts->rst_pin);    // end select I2C slave addr
#endif

#if GT9XX_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        return;
    }
#endif

    gt9xx_int_sync(50, ts);  
#if GT9XX_ESD_PROTECT
    gtp_init_ext_watchdog(client);
#endif
}

#if GT9XX_GESTURE_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gt9xx_enter_doze(struct goodix_gt9xx_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GT9XX_REG_SLEEP >> 8), (u8)GT9XX_REG_SLEEP, 8};

    GT9XX_DEBUG_FUNC();

    GT9XX_DEBUG("Entering gesture mode.");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gt9xx_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GT9XX_DEBUG("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gt9xx_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            GT9XX_INFO("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    GT9XX_ERROR("GTP send gesture cmd failed.");
    return ret;
}
#else 
/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
#if 0
static s8 gt9xx_enter_sleep(struct goodix_gt9xx_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GT9XX_REG_SLEEP >> 8), (u8)GT9XX_REG_SLEEP, 5};

#if GT9XX_COMPATIBLE_MODE
    u8 status_buf[3] = {0x80, 0x44};
#endif
    
    GT9XX_DEBUG_FUNC();
    
#if GT9XX_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // GT9XXF: host interact with ic
        ret = gt9xx_i2c_read(ts->client, status_buf, 3);
        if (ret < 0)
        {
            GT9XX_ERROR("failed to get backup-reference status");
        }
        
        if (status_buf[2] & 0x80)
        {
            ret = gtp_bak_ref_proc(ts, GT9XX_BAK_REF_STORE);
            if (FAIL == ret)
            {
                GT9XX_ERROR("failed to store bak_ref");
            }
        }
    }
#endif

    GT9XX_GPIO_OUTPUT(ts->irq_pin, 0);
    msleep(5);
    
    while(retry++ < 5)
    {
        ret = gt9xx_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GT9XX_INFO("GTP enter sleep!");
            
            return ret;
        }
        msleep(10);
    }
    GT9XX_ERROR("GTP send sleep cmd failed.");
    return ret;
}
#endif
#endif 
/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
#if 0
static s8 gt9xx_wakeup_sleep(struct goodix_gt9xx_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;
    
    GT9XX_DEBUG_FUNC();

#if GT9XX_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 opr_buf[3] = {0x41, 0x80};
        
        GT9XX_GPIO_OUTPUT(ts->irq_pin, 1);
        msleep(5);
    
        for (retry = 0; retry < 10; ++retry)
        {
            // hold ss51 & dsp
            opr_buf[2] = 0x0C;
            ret = gt9xx_i2c_write(ts->client, opr_buf, 3);
            if (FAIL == ret)
            {
                GT9XX_ERROR("failed to hold ss51 & dsp!");
                continue;
            }
            opr_buf[2] = 0x00;
            ret = gt9xx_i2c_read(ts->client, opr_buf, 3);
            if (FAIL == ret)
            {
                GT9XX_ERROR("failed to get ss51 & dsp status!");
                continue;
            }
            if (0x0C != opr_buf[2])
            {
                GT9XX_DEBUG("ss51 & dsp not been hold, %d", retry+1);
                continue;
            }
            GT9XX_DEBUG("ss51 & dsp confirmed hold");
            
            ret = gt9xx_gtp_fw_startup(ts->client);
            if (FAIL == ret)
            {
                GT9XX_ERROR("failed to startup GT9XXF, process recovery");
                gtp_esd_recovery(ts->client);
            }
            break;
        }
        if (retry >= 10)
        {
            GT9XX_ERROR("failed to wakeup, processing esd recovery");
            gtp_esd_recovery(ts->client);
        }
        else
        {
            GT9XX_INFO("GT9XXF gtp wakeup success");
        }
        return ret;
    }
#endif

#if GT9XX_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gt9xx_gtp_reset_guitar(ts->client, 20);
        
        GT9XX_INFO("GTP wakeup sleep.");
        return 1;
    }
#else
    while(retry++ < 10)
    {
    #if GT9XX_GESTURE_WAKEUP
        if (DOZE_WAKEUP != doze_status)  
        {
            GT9XX_INFO("Powerkey wakeup.");
        }
        else   
        {
            GT9XX_INFO("Gesture wakeup.");
        }
        doze_status = DOZE_DISABLED;
        gt9xx_gtp_irq_disable(ts);
        gt9xx_gtp_reset_guitar(ts->client, 10);
        gt9xx_gtp_irq_enable(ts);
        
    #else
        GT9XX_GPIO_OUTPUT(ts->irq_pin, 1);
        msleep(5);
    #endif
    
        ret = gt9xx_i2c_test(ts->client);
        if (ret > 0)
        {
            GT9XX_INFO("GTP wakeup sleep.");
            
        #if (!GT9XX_GESTURE_WAKEUP)
            {
                gt9xx_int_sync(25, ts);
            #if GT9XX_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
            #endif
            }
        #endif
            
            return ret;
        }
        gt9xx_gtp_reset_guitar(ts->client, 20);
    }
#endif

    GT9XX_ERROR("GTP wakeup sleep failed.");
    return ret;
}
#endif

#if GT9XX_DRIVER_SEND_CFG
static s32 gt9xx_get_info(struct goodix_gt9xx_ts_data *ts)
{
    u8 opr_buf[6] = {0};
    s32 ret = 0;
    
    ts->abs_x_max = GT9XX_MAX_WIDTH;
    ts->abs_y_max = GT9XX_MAX_HEIGHT;
    ts->int_trigger_type = GT9XX_INT_TRIGGER;
        
    opr_buf[0] = (u8)((GT9XX_REG_CONFIG_DATA+1) >> 8);
    opr_buf[1] = (u8)((GT9XX_REG_CONFIG_DATA+1) & 0xFF);
    
    ret = gt9xx_i2c_read(ts->client, opr_buf, 6);
    if (ret < 0)
    {
        return FAIL;
    }
    
    ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];
    
    opr_buf[0] = (u8)((GT9XX_REG_CONFIG_DATA+6) >> 8);
    opr_buf[1] = (u8)((GT9XX_REG_CONFIG_DATA+6) & 0xFF);
    
    ret = gt9xx_i2c_read(ts->client, opr_buf, 3);
    if (ret < 0)
    {
        return FAIL;
    }
    ts->int_trigger_type = opr_buf[2] & 0x03;
    
    GT9XX_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    
    return SUCCESS;    
}
#endif 

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
static s32 gt9xx_init_panel(struct goodix_gt9xx_ts_data *ts)
{
    s32 ret = -1;

#if GT9XX_DRIVER_SEND_CFG
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    u8 sensor_id = 0; 
    u8 retry = 0;
    
    
    u8 cfg_info_group0[] = GT9XX_CTP_CFG_GROUP0;
    u8 cfg_info_group1[] = GT9XX_CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = GT9XX_CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = GT9XX_CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = GT9XX_CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = GT9XX_CTP_CFG_GROUP5;
    u8 *send_cfg_buf[] = {cfg_info_group0, cfg_info_group1, cfg_info_group2,
                        cfg_info_group3, cfg_info_group4, cfg_info_group5};
    u8 cfg_info_len[] = { GT9XX_CFG_GROUP_LEN(cfg_info_group0),
                          GT9XX_CFG_GROUP_LEN(cfg_info_group1),
                          GT9XX_CFG_GROUP_LEN(cfg_info_group2),
                          GT9XX_CFG_GROUP_LEN(cfg_info_group3),
                          GT9XX_CFG_GROUP_LEN(cfg_info_group4),
                          GT9XX_CFG_GROUP_LEN(cfg_info_group5)};

    GT9XX_DEBUG_FUNC();
    GT9XX_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);

    
#if GT9XX_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ts->fw_error = 0;
    }
    else
#endif
    {
        ret = gt9xx_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
        if (SUCCESS == ret) 
        {
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                GT9XX_ERROR("Firmware error, no config sent!");
                return -1;
            }
        }
    }

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
        (!cfg_info_len[3]) && (!cfg_info_len[4]) && 
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
    #if GT9XX_COMPATIBLE_MODE
        msleep(50);
    #endif
        ret = gt9xx_i2c_read_dbl_check(ts->client, GT9XX_REG_SENSOR_ID, &sensor_id, 1);
        if (SUCCESS == ret)
        {
        	
        	while((sensor_id == 0xff)&&(retry++ < 3))
    			{
    				msleep(100);
        		ret = gt9xx_i2c_read_dbl_check(ts->client, GT9XX_REG_SENSOR_ID, &sensor_id, 1);
       			GT9XX_ERROR("GTP sensor_ID read failed time %d.",retry);
        		
   				}
   				/****************/
            if (sensor_id >= 0x06)
            {
                GT9XX_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                ts->pnl_init_error = 1;
                return -1;
            }
        }
        else
        {
            GT9XX_ERROR("Failed to get sensor_id, No config sent!");
            ts->pnl_init_error = 1;
            return -1;
        }
        GT9XX_INFO("Sensor_ID: %d", sensor_id);
    }
    ts->gtp_cfg_len = cfg_info_len[sensor_id];
    GT9XX_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1, ts->gtp_cfg_len);
    
    if (ts->gtp_cfg_len < GT9XX_CONFIG_MIN_LENGTH)
    {
        GT9XX_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }

#if GT9XX_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ts->fixed_cfg = 0;
    }
    else
#endif
    {
        ret = gt9xx_i2c_read_dbl_check(ts->client, GT9XX_REG_CONFIG_DATA, &opr_buf[0], 1);
        
        if (ret == SUCCESS)
        {
            GT9XX_DEBUG("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id+1, 
                        send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
            
            if (opr_buf[0] < 90)    
            {
                grp_cfg_version = send_cfg_buf[sensor_id][0];       // backup group config version
                send_cfg_buf[sensor_id][0] = 0x00;
                ts->fixed_cfg = 0;
            }
            else        // treated as fixed config, not send config
            {
                GT9XX_INFO("Ic fixed config with config version(%d, 0x%02X)", opr_buf[0], opr_buf[0]);
                ts->fixed_cfg = 1;
                gt9xx_get_info(ts);
                return 0;
            }
        }
        else
        {
            GT9XX_ERROR("Failed to get ic config version!No config sent!");
            return -1;
        }
    }
    
    memset(&gt9xx_config[GT9XX_ADDR_LENGTH], 0, GT9XX_CONFIG_MAX_LENGTH);
    memcpy(&gt9xx_config[GT9XX_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);

#if GT9XX_CUSTOM_CFG
    gt9xx_config[RESOLUTION_LOC]     = (u8)GT9XX_MAX_WIDTH;
    gt9xx_config[RESOLUTION_LOC + 1] = (u8)(GT9XX_MAX_WIDTH>>8);
    gt9xx_config[RESOLUTION_LOC + 2] = (u8)GT9XX_MAX_HEIGHT;
    gt9xx_config[RESOLUTION_LOC + 3] = (u8)(GT9XX_MAX_HEIGHT>>8);
    
    if (GT9XX_INT_TRIGGER == 0)  //RISING
    {
        gt9xx_config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GT9XX_INT_TRIGGER == 1)  //FALLING
    {
        gt9xx_config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG
    
    check_sum = 0;
    for (i = GT9XX_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += gt9xx_config[i];
    }
    gt9xx_config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

    ts->gtp_cfg_len = GT9XX_CONFIG_MAX_LENGTH;
    ret = gt9xx_i2c_read(ts->client, gt9xx_config, ts->gtp_cfg_len + GT9XX_ADDR_LENGTH);
    if (ret < 0)
    {
        GT9XX_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GT9XX_MAX_WIDTH;
        ts->abs_y_max = GT9XX_MAX_HEIGHT;
        ts->int_trigger_type = GT9XX_INT_TRIGGER;
    }
    
#endif // GTP_DRIVER_SEND_CFG

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (gt9xx_config[RESOLUTION_LOC + 1] << 8) + gt9xx_config[RESOLUTION_LOC];
        ts->abs_y_max = (gt9xx_config[RESOLUTION_LOC + 3] << 8) + gt9xx_config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (gt9xx_config[TRIGGER_LOC]) & 0x03; 
    }

#if GT9XX_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 sensor_num = 0;
        u8 driver_num = 0;
        u8 have_key = 0;
        
        have_key = (gt9xx_config[GT9XX_REG_HAVE_KEY - GT9XX_REG_CONFIG_DATA + 2] & 0x01);
        
        if (1 == ts->is_950)
        {
            driver_num = gt9xx_config[GT9XX_REG_MATRIX_DRVNUM - GT9XX_REG_CONFIG_DATA + 2];
            sensor_num = gt9xx_config[GT9XX_REG_MATRIX_SENNUM - GT9XX_REG_CONFIG_DATA + 2];
            if (have_key)
            {
                driver_num--;
            }
            ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
        }
        else
        {
            driver_num = (gt9xx_config[CFG_LOC_DRVA_NUM] & 0x1F) + (gt9xx_config[CFG_LOC_DRVB_NUM]&0x1F);
            if (have_key)
            {
                driver_num--;
            }
            sensor_num = (gt9xx_config[CFG_LOC_SENS_NUM] & 0x0F) + ((gt9xx_config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
            ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
        }
    
        GT9XX_INFO("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
           driver_num, sensor_num, have_key, ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
        return 0;
    }
    else
#endif
    {
    #if GT9XX_DRIVER_SEND_CFG
        ret = gt9xx_gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            GT9XX_ERROR("Send config error.");
        }
        // set config version to CTP_CFG_GROUP, for resume to send config
        gt9xx_config[GT9XX_ADDR_LENGTH] = grp_cfg_version;
        check_sum = 0;
        for (i = GT9XX_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
        {
            check_sum += gt9xx_config[i];
        }
        gt9xx_config[ts->gtp_cfg_len] = (~check_sum) + 1;
    #endif
        GT9XX_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    }
    
    msleep(10);
    return 0;
}


static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    char temp_data[GT9XX_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
    int i;
    
    if (*ppos)
    {
        return 0;
    }
    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GT9XX_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", gt9xx_config[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    gt9xx_i2c_read(gt9xx_i2c_connect_client, temp_data, GT9XX_CONFIG_MAX_LENGTH + 2);
    for (i = 0 ; i < GT9XX_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i+2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;

    GT9XX_DEBUG("write count %d\n", count);

    if (count > GT9XX_CONFIG_MAX_LENGTH)
    {
        GT9XX_ERROR("size not match [%d:%d]\n", GT9XX_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&gt9xx_config[2], buffer, count))
    {
        GT9XX_ERROR("copy from user fail\n");
        return -EFAULT;
    }

    ret = gt9xx_gtp_send_cfg(gt9xx_i2c_connect_client);

    if (ret < 0)
    {
        GT9XX_ERROR("send config failed.");
    }

    return count;
}
/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
s32 gt9xx_gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GT9XX_REG_VERSION >> 8, GT9XX_REG_VERSION & 0xff};

    GT9XX_DEBUG_FUNC();

    ret = gt9xx_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GT9XX_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    if (buf[5] == 0x00)
    {
        GT9XX_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GT9XX_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gt9xx_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GT9XX_REG_CONFIG_DATA >> 8, GT9XX_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    GT9XX_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = gt9xx_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GT9XX_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 gt9xx_request_io_port(struct goodix_gt9xx_ts_data *ts)
{
    s32 ret = 0;

    GT9XX_DEBUG_FUNC();
#if (GT9XX_CHIP == CHIP_GT911)
    //ret = GT9XX_GPIO_REQUEST(gpio_to_irq(ts->irq_pin), "GT9XX_INT_PORT");
    ret = GT9XX_GPIO_REQUEST(ts->irq_pin, "GT9XX_INT_PORT");
    //ret = GT9XX_GPIO_REQUEST(gpio_to_irq(ts->irq_pin), NULL);
    if (ret < 0) 
    {
        GT9XX_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)ts->irq_pin, ret);
        ret = -ENODEV;
    }
    else 
#endif
    {
        GT9XX_GPIO_AS_INT(ts->irq_pin);  
        ts->client->irq = gpio_to_irq(ts->irq_pin);
    }

#if (GT9XX_CHIP == CHIP_GT911)
    //ret = GT9XX_GPIO_REQUEST(gpio_to_irq(ts->rst_pin), "GT9XX_RST_PORT");
      ret = GT9XX_GPIO_REQUEST(ts->rst_pin, "GT9XX_RST_PORT");
    if (ret < 0) 
    {
        GT9XX_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)ts->rst_pin,ret);
        ret = -ENODEV;
    }
	else
#endif
	{
    	GT9XX_GPIO_AS_INPUT(ts->rst_pin);
    	gt9xx_gtp_reset_guitar(ts->client, 20);
	}
	
#if (GT9XX_CHIP == CHIP_GT911)    
    if(ret < 0)
    {
        GT9XX_GPIO_FREE(ts->rst_pin);
        GT9XX_GPIO_FREE(ts->irq_pin);
    }
#endif
    return ret;
}

static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_gt9xx_ts_data *ts = dev_id;

    GT9XX_DEBUG_FUNC();
 
    gt9xx_gtp_irq_disable(ts);

    queue_work(goodix_wq, &ts->work);
    
    return IRQ_HANDLED;
}
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_gt9xx_ts_data *ts = container_of(timer, struct goodix_gt9xx_ts_data, timer);

    GT9XX_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}
/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gt9xx_request_irq(struct goodix_gt9xx_ts_data *ts)
{

#if 0
    s32 ret = -1;
   // const u8 irq_table[] = GTP_IRQ_TAB;

    GT9XX_DEBUG_FUNC();
    GT9XX_DEBUG("INT trigger type:%x", ts->int_trigger_type);

    ret  = request_irq(ts->client->irq, 
                       gt9xx_ts_irq_handler,
                       0,
                       ts->client->name,
                       ts);
    if (ret)
    {
        GT9XX_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        GT9XX_GPIO_AS_INPUT(ts->irq_pin);
//        GTP_GPIO_FREE(GT9XX_INT_PORT);
        return -1;
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = gt9xx_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else 
    {
        gt9xx_gtp_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
#else
    s32 ret = -1;

    GT9XX_DEBUG_FUNC();
    GT9XX_DEBUG("INT trigger type:%x", ts->int_trigger_type);
    
    ts->irq=gpio_to_irq(ts->irq_pin);       //If not defined in client
    if (ts->irq)
    {
        ts->client->irq = ts->irq;
        ret = devm_request_threaded_irq(&(ts->client->dev), ts->irq, NULL, 
            goodix_ts_irq_handler, 0 /*ts->irq_flags | IRQF_ONESHOT*/ /*irq_table[ts->int_trigger_type]*/, 
            ts->client->name, ts);
        if (ret != 0) {
            GT9XX_ERROR("Cannot allocate ts INT!ERRNO:%d\n", ret);
            goto test_pit;
        }
        //gtp_irq_disable(ts->irq);
        GT9XX_INFO("  <%s>_%d     ts->irq=%d   ret = %d\n", __func__, __LINE__, ts->irq, ret);
    }else{
        GT9XX_ERROR("   ts->irq  error \n");
        ret = 1;
        goto test_pit;
    }
/*
    ret  = request_irq(ts->client->irq, 
                       goodix_ts_irq_handler,
                       irq_table[ts->int_trigger_type],
                       ts->client->name,
                       ts);
*/                       
test_pit:
    if (ret)
    {
        GT9XX_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        //GTP_GPIO_AS_INPUT(GTP_INT_PORT);
        gpio_direction_input(ts->irq_pin);
        //s3c_gpio_setpull(pin, S3C_GPIO_PULL_NONE);
        
        GT9XX_GPIO_FREE(ts->irq_pin);

        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else 
    {
        GT9XX_INFO("  <%s>_%d     ts->irq=%d   ret = %d\n", __func__, __LINE__, ts->irq, ret);
        gt9xx_gtp_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
#endif
}

/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int gt9xx_ts_early_suspend(struct tp_device *tp_d)
{
    struct goodix_gt9xx_ts_data *ts;
    //s8 ret = -1;    
    ts = container_of(tp_d, struct goodix_gt9xx_ts_data, tp);
    
    GT9XX_DEBUG_FUNC();
    
    GT9XX_INFO("System suspend.");

#if 1 
	doze_status = DOZE_ENABLED;
#else
    ts->gtp_is_suspend = 1;
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GT9XX_GESTURE_WAKEUP
    ret = gt9xx_enter_doze(ts);
#else
    if (ts->use_irq)
    {
        gt9xx_gtp_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gt9xx_enter_sleep(ts);
#endif 
    if (ret < 0)
    {
        GT9XX_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability    
    msleep(58);
#endif 
	return 0;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int gt9xx_ts_late_resume(struct tp_device *tp_d)
{
    struct goodix_gt9xx_ts_data *ts;
    //s8 ret = -1;
    ts = container_of(tp_d, struct goodix_gt9xx_ts_data, tp);
    
    GT9XX_DEBUG_FUNC();
    
    GT9XX_INFO("System resume.");

#if 1
	doze_status = DOZE_DISABLED;
    gt9xx_gtp_irq_disable(ts);
    gt9xx_gtp_reset_guitar(ts->client, 10);
    gt9xx_gtp_irq_enable(ts);
	gt9xx_gtp_send_cfg(ts->client);
#else    
    ret = gt9xx_wakeup_sleep(ts);

#if GT9XX_GESTURE_WAKEUP
    doze_status = DOZE_DISABLED;
#endif

    if (ret < 0)
    {
        GT9XX_ERROR("GTP later resume failed.");
    }
#if (GT9XX_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // do nothing
    }
    else
#endif
    {
        gt9xx_gtp_send_cfg(ts->client);
    }

    if (ts->use_irq)
    {
        gt9xx_gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(ts->client, SWITCH_ON);
#endif
#endif
	return 0;
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gt9xx_request_input_dev(struct goodix_gt9xx_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GT9XX_HAVE_TOUCH_KEY
    u8 index = 0;
#endif
  
    GT9XX_DEBUG_FUNC();
  
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GT9XX_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GT9XX_ICS_SLOT_REPORT
#if (GT9XX_CHIP == CHIP_GT928)
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit); 
#endif
    input_mt_init_slots(ts->input_dev, 16,0);     // in case of "out of memory"
#else
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#if GT9XX_HAVE_TOUCH_KEY
    for (index = 0; index < GT9XX_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);  
    }
#endif

#if 1 
    input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif 

#if GT9XX_CHANGE_X2Y
    GT9XX_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    sprintf(phys, "input/ts");
    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;
    
    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GT9XX_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }
    
#if 1
    ts->tp.tp_suspend = gt9xx_ts_early_suspend;
    ts->tp.tp_resume = gt9xx_ts_late_resume;
    tp_register_fb(&ts->tp);
#endif

#if GT9XX_WITH_PEN
    gt9xx_pen_init(ts);
#endif

    return 0;
}

//************** For GT9XXF Start *************//
#if GT9XX_COMPATIBLE_MODE

s32 gt9xx_gtp_fw_startup(struct i2c_client *client)
{
    u8 opr_buf[4];
    s32 ret = 0;
    struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(client); 
	
    //init sw WDT
	opr_buf[0] = 0xAA;
	ret = gt9xx_i2c_write_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    
    //release SS51 & DSP
    opr_buf[0] = 0x00;
    ret = gt9xx_i2c_write_bytes(client, 0x4180, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    //int sync
    gt9xx_int_sync(25, ts);  
    
    //check fw run status
    ret = gt9xx_i2c_read_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    if(0xAA == opr_buf[0])
    {
        GT9XX_ERROR("IC works abnormally,startup failed.");
        return FAIL;
    }
    else
    {
        GT9XX_INFO("IC works normally, Startup success.");
        opr_buf[0] = 0xAA;
        gt9xx_i2c_write_bytes(client, 0x8041, opr_buf, 1);
        return SUCCESS;
    }
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
    s32 retry = 0;
    s32 ret = 0;
    struct goodix_gt9xx_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    
    gt9xx_gtp_irq_disable(ts);
    
    GT9XX_INFO("GT9XXF esd recovery mode");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gt9xx_gup_fw_download_proc(NULL, GT9XX_FL_ESD_RECOVERY); 
        if (FAIL == ret)
        {
            GT9XX_ERROR("esd recovery failed %d", retry+1);
            continue;
        }
        ret = gt9xx_gtp_fw_startup(ts->client);
        if (FAIL == ret)
        {
            GT9XX_ERROR("GT9XXF start up failed %d", retry+1);
            continue;
        }
        break;
    }
    gt9xx_gtp_irq_enable(ts);
    
    if (retry >= 5)
    {
        GT9XX_ERROR("failed to esd recovery");
        return FAIL;
    }
    
    GT9XX_INFO("Esd recovery successful");
    return SUCCESS;
}

void gt9xx_gtp_recovery_reset(struct i2c_client *client)
{
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(client, SWITCH_OFF);
#endif
    GT9XX_DEBUG_FUNC();
    
    gtp_esd_recovery(client); 
    
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_gt9xx_ts_data *ts, u8 mode)
{
    s32 ret = 0;
    s32 i = 0;
    s32 j = 0;
    u16 ref_sum = 0;
    u16 learn_cnt = 0;
    u16 chksum = 0;
    s32 ref_seg_len = 0;
    s32 ref_grps = 0;
    struct file *ref_filp = NULL;
    u8 *p_bak_ref;
    
    ret = gt9xx_gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->ref_chk_fs_times++;
        GT9XX_DEBUG("Ref check /data times/MAX_TIMES: %d / %d", ts->ref_chk_fs_times, GT9XX_CHK_FS_MNT_MAX);
        if (ts->ref_chk_fs_times < GT9XX_CHK_FS_MNT_MAX)
        {
            msleep(50);
            GT9XX_INFO("/data not mounted.");
            return FAIL;
        }
        GT9XX_INFO("check /data mount timeout...");
    }
    else
    {
        GT9XX_INFO("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times, GT9XX_CHK_FS_MNT_MAX);
    }
    
    p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);
    
    if (NULL == p_bak_ref)
    {
        GT9XX_ERROR("Allocate memory for p_bak_ref failed!");
        return FAIL;
    }
    
    if (ts->is_950)
    {
        ref_seg_len = ts->bak_ref_len / 6;
        ref_grps = 6;
    }
    else
    {
        ref_seg_len = ts->bak_ref_len;
        ref_grps = 1;
    }
    ref_filp = filp_open(GT9XX_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(ref_filp))
    { 
        GT9XX_ERROR("Failed to open/create %s.", GT9XX_BAK_REF_PATH);
        if (GT9XX_BAK_REF_SEND == mode)
        {
            goto bak_ref_default;
        }
        else
        {
            goto bak_ref_exit;
        }
    }
    
    switch (mode)
    {
    case GT9XX_BAK_REF_SEND:
        GT9XX_INFO("Send backup-reference");
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ret = ref_filp->f_op->read(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        if (ret < 0)
        {
            GT9XX_ERROR("failed to read bak_ref info from file, sending defualt bak_ref");
            goto bak_ref_default;
        }
        for (j = 0; j < ref_grps; ++j)
        {
            ref_sum = 0;
            for (i = 0; i < (ref_seg_len); i += 2)
            {
                ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i+1 + j * ref_seg_len];
            }
            learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len -4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -3]);
            chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len -2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -1]);
            GT9XX_DEBUG("learn count = %d", learn_cnt);
            GT9XX_DEBUG("chksum = %d", chksum);
            GT9XX_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);
            // Sum(1~ref_seg_len) == 1
            if (1 != ref_sum)
            {
                GT9XX_INFO("wrong chksum for bak_ref, reset to 0x00 bak_ref");
                memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
            }
            else
            {
                if (j == (ref_grps - 1))
                {
                    GT9XX_INFO("backup-reference data in %s used", GT9XX_BAK_REF_PATH);
                }
            }
        }
        ret = gt9xx_i2c_write_bytes(ts->client, GT9XX_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (FAIL == ret)
        {
            GT9XX_ERROR("failed to send bak_ref because of iic comm error");
            goto bak_ref_exit;
        }
        break;
        
    case GT9XX_BAK_REF_STORE:
        GT9XX_INFO("Store backup-reference");
        ret = gt9xx_i2c_read_bytes(ts->client, GT9XX_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (ret < 0)
        {
            GT9XX_ERROR("failed to read bak_ref info, sending default back-reference");
            goto bak_ref_default;
        }
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        break;
        
    default:
        GT9XX_ERROR("invalid backup-reference request");
        break;
    }
    ret = SUCCESS;
    goto bak_ref_exit;

bak_ref_default:
    
    for (j = 0; j < ref_grps; ++j)
    {
        memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
        p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1     
    }
    ret = gt9xx_i2c_write_bytes(ts->client, GT9XX_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
    if (!IS_ERR(ref_filp))
    {
        GT9XX_INFO("write backup-reference data into %s", GT9XX_BAK_REF_PATH);
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
    }
    if (ret == FAIL)
    {
        GT9XX_ERROR("failed to load the default backup reference");
    }
    
bak_ref_exit:
    
    if (p_bak_ref)
    {
        kfree(p_bak_ref);
    }
    if (ref_filp && !IS_ERR(ref_filp))
    {
        filp_close(ref_filp, NULL);
    }
    return ret;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
    u8 chksum = 0;
    u8 main_clock = p_main_clk[0];
    s32 i = 0;
    
    if (main_clock < 50 || main_clock > 120)    
    {
        return FAIL;
    }
    
    for (i = 0; i < 5; ++i)
    {
        if (main_clock != p_main_clk[i])
        {
            return FAIL;
        }
        chksum += p_main_clk[i];
    }
    chksum += p_main_clk[5];
    if ( (chksum) == 0)
    {
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}

static s32 gtp_main_clk_proc(struct goodix_gt9xx_ts_data *ts)
{
    s32 ret = 0;
    s32 i = 0;
    s32 clk_chksum = 0;
    struct file *clk_filp = NULL;
    u8 p_main_clk[6] = {0};

    ret = gt9xx_gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->clk_chk_fs_times++;
        GT9XX_DEBUG("Clock check /data times/MAX_TIMES: %d / %d", ts->clk_chk_fs_times, GT9XX_CHK_FS_MNT_MAX);
        if (ts->clk_chk_fs_times < GT9XX_CHK_FS_MNT_MAX)
        {
            msleep(50);
            GT9XX_INFO("/data not mounted.");
            return FAIL;
        }
        GT9XX_INFO("Check /data mount timeout!");
    }
    else
    {
        GT9XX_INFO("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times, GT9XX_CHK_FS_MNT_MAX);
    }
    
    clk_filp = filp_open(GT9XX_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(clk_filp))
    {
        GT9XX_ERROR("%s is unavailable, calculate main clock", GT9XX_MAIN_CLK_PATH);
    }
    else
    {
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
       
        ret = gtp_verify_main_clk(p_main_clk);
        if (FAIL == ret)
        {
            // recalculate main clock & rewrite main clock data to file
            GT9XX_ERROR("main clock data in %s is wrong, recalculate main clock", GT9XX_MAIN_CLK_PATH);
        }
        else
        { 
            GT9XX_INFO("main clock data in %s used, main clock freq: %d", GT9XX_MAIN_CLK_PATH, p_main_clk[0]);
            filp_close(clk_filp, NULL);
            goto update_main_clk;
        }
    }
    
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
    ret = gt9xx_gup_clk_calibration();
    gtp_esd_recovery(ts->client);
    
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(ts->client, SWITCH_ON);
#endif

    GT9XX_INFO("calibrate main clock: %d", ret);
    if (ret < 50 || ret > 120)
    {
        GT9XX_ERROR("wrong main clock: %d", ret);
        goto exit_main_clk;
    }
    
    // Sum{0x8020~0x8025} = 0
    for (i = 0; i < 5; ++i)
    {
        p_main_clk[i] = ret;
        clk_chksum += p_main_clk[i];
    }
    p_main_clk[5] = 0 - clk_chksum;
    
    if (!IS_ERR(clk_filp))
    {
        GT9XX_DEBUG("write main clock data into %s", GT9XX_MAIN_CLK_PATH);
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
        filp_close(clk_filp, NULL);
    }
    
update_main_clk:
    ret = gt9xx_i2c_write_bytes(ts->client, GT9XX_REG_MAIN_CLK, p_main_clk, 6);
    if (FAIL == ret)
    {
        GT9XX_ERROR("update main clock failed!");
        return FAIL;
    }
    return SUCCESS;
    
exit_main_clk:
    if (!IS_ERR(clk_filp))
    {
        filp_close(clk_filp, NULL);
    }
    return FAIL;
}


s32 gtp_gt9xxf_init(struct i2c_client *client)
{
    s32 ret = 0;
    
    ret = gt9xx_gup_fw_download_proc(NULL, GT9XX_FL_FW_BURN); 
    if (FAIL == ret)
    {
        return FAIL;
    }
    
    ret = gt9xx_gtp_fw_startup(client);
    if (FAIL == ret)
    {
        return FAIL;
    }
    return SUCCESS;
}

void gtp_get_chip_type(struct goodix_gt9xx_ts_data *ts)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;
    
    msleep(10);
    
    ret = gt9xx_i2c_read_dbl_check(ts->client, GT9XX_REG_CHIP_TYPE, opr_buf, 10);
    
    if (FAIL == ret)
    {
        GT9XX_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
        ts->chip_type = CHIP_TYPE_GT9;
        return;
    }
    
    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        ts->chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        ts->chip_type = CHIP_TYPE_GT9F;
    }
    GT9XX_INFO("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

#endif
//************* For GT9XXF End ************//

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes. 
        0: succeed.
*******************************************************/
static int gt9xx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct goodix_gt9xx_ts_data *ts;
    u16 version_info;
    
    struct device_node *np = client->dev.of_node;
    enum of_gpio_flags rst_flags; //, pwr_flags;
    //u32 val;
	printk("___%s() start____ \n", __func__);
    GT9XX_DEBUG_FUNC();
    
    //do NOT remove these logs
    GT9XX_INFO("GT9XX Driver Version: %s", GT9XX_DRIVER_VERSION);
    GT9XX_INFO("GT9XX Driver Built@%s, %s", __TIME__, __DATE__);
    GT9XX_INFO("GT9XX I2C Address: 0x%02x", client->addr);

    gt9xx_i2c_connect_client = client;
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        GT9XX_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GT9XX_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }
    
    memset(ts, 0, sizeof(*ts));
    ts->irq_pin = of_get_named_gpio_flags(np, "touch-gpio", 0, (enum of_gpio_flags *)(&ts->irq_flags));
    ts->rst_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);
    INIT_WORK(&ts->work, gt9xx_ts_work_func);
    ts->client = client;
    spin_lock_init(&ts->irq_lock);          // 2.6.39 later
    // ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
#if GT9XX_ESD_PROTECT
    ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
    GT9XX_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);  
    spin_lock_init(&ts->esd_lock);
    // ts->esd_lock = SPIN_LOCK_UNLOCKED;
#endif
    i2c_set_clientdata(client, ts);
    
    ts->gtp_rawdiff_mode = 0;

    ret = gt9xx_request_io_port(ts);
//    if (ret < 0)
//    {
//        GT9XX_ERROR("GTP request IO port failed.");
//        kfree(ts);
//        return ret;
//    }
    
#if GT9XX_COMPATIBLE_MODE
    gtp_get_chip_type(ts);
    
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ret = gtp_gt9xxf_init(ts->client);
        if (FAIL == ret)
        {
            GT9XX_INFO("Failed to init GT9XXF.");
        }
    }
#endif

    ret = gt9xx_i2c_test(client);
    if (ret < 0)
    {
        GT9XX_ERROR("I2C communication ERROR!");
        ret =  -ENODEV;
        goto err_goodix_is_not_exist;
    }

    ret = gt9xx_gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        GT9XX_ERROR("Read version failed.");
        ret =  -ENODEV;
        goto err_goodix_is_not_exist;
    }
    
    ret = gt9xx_init_panel(ts);
    if (ret < 0)
    {
        GT9XX_ERROR("GTP init panel failed.");
        ts->abs_x_max = GT9XX_MAX_WIDTH;
        ts->abs_y_max = GT9XX_MAX_HEIGHT;
        ts->int_trigger_type = GT9XX_INT_TRIGGER;
    }
    
    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        GT9XX_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        GT9XX_INFO("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }
    
#if GT9XX_ESD_PROTECT
    gt9xx_gtp_esd_switch(client, SWITCH_ON);
#endif
#if GT9XX_AUTO_UPDATE
    ret = gt9xx_gup_init_update_proc(ts);
    if (ret < 0)
    {
        GT9XX_ERROR("Create update thread error.");
    }
#endif

    ret = gt9xx_request_input_dev(ts);
    if (ret < 0)
    {
        GT9XX_ERROR("GTP request input dev failed");
    }
    
    ret = gt9xx_request_irq(ts); 
    if (ret < 0)
    {
        GT9XX_INFO("GTP works in polling mode.");
    }
    else
    {
        GT9XX_INFO("GTP works in interrupt mode.");
    }

    if (ts->use_irq)
    {
        gt9xx_gtp_irq_enable(ts);
    }
    
#if GT9XX_CREATE_WR_NODE
    gt9xx_init_wr_node(client);
#endif
    

    return 0;
err_goodix_is_not_exist:
#if (GT9XX_CHIP == CHIP_GT911) 
    GT9XX_GPIO_FREE(ts->rst_pin);
    GT9XX_GPIO_FREE(ts->irq_pin);
#endif
    i2c_set_clientdata(client, NULL);
    kfree(ts);
	
  return ret;
}


/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int gt9xx_ts_remove(struct i2c_client *client)
{
    struct goodix_gt9xx_ts_data *ts = i2c_get_clientdata(client);
    
    GT9XX_DEBUG_FUNC();
    
#if GT9XX_CREATE_WR_NODE
    gt9xx_uninit_wr_node();
#endif

#if GT9XX_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    if (ts) 
    {
        if (ts->use_irq)
        {
            GT9XX_GPIO_AS_INPUT(ts->irq_pin);
            GT9XX_GPIO_FREE(ts->irq_pin);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }   
    
    GT9XX_INFO("GTP driver removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}

#if GT9XX_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GT9XX_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GT9XX_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GT9XX_ADDR_LENGTH;
    msgs[1].buf   = &buf[GT9XX_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {    
        GT9XX_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GT9XX_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        GT9XX_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gt9xx_gtp_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_gt9xx_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    spin_lock(&ts->esd_lock);
    
    if (SWITCH_ON == on)     // switch on esd 
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
            spin_unlock(&ts->esd_lock);
            GT9XX_INFO("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            GT9XX_INFO("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
    GT9XX_DEBUG("[Esd]Init external watchdog");
    return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i;
    s32 ret = -1;
    struct goodix_gt9xx_ts_data *ts = NULL;
    u8 esd_buf[5] = {0x80, 0x40};
    
    GT9XX_DEBUG_FUNC();
   
    ts = i2c_get_clientdata(gt9xx_i2c_connect_client);
		
		
		if (ts->gtp_is_suspend || ts->enter_update)
    {
        GT9XX_INFO("Esd suspended or IC update firmware!");
        return;
    }
   	
		 
    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);
        
        GT9XX_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        { 
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};
                
                gtp_i2c_read_no_rst(ts->client, chk_buf, 4);
                
                GT9XX_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);
                
                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = 3;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else 
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA; 
                gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
    #if GT9XX_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {        
            if (ts->rqst_processing)
            {
                GT9XX_INFO("Request processing, no esd recovery");
            }
            else
            {
                GT9XX_ERROR("IC working abnormally! Process esd recovery.");
                esd_buf[0] = 0x42;
                esd_buf[1] = 0x26;
                esd_buf[2] = 0x01;
                esd_buf[3] = 0x01;
                esd_buf[4] = 0x01;
                gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
                msleep(50);
                gtp_esd_recovery(ts->client);
            }
        }
        else
    #endif
        {
            GT9XX_ERROR("IC working abnormally! Process reset guitar.");
            esd_buf[0] = 0x42;
            esd_buf[1] = 0x26;
            esd_buf[2] = 0x01;
            esd_buf[3] = 0x01;
            esd_buf[4] = 0x01;
            gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
            msleep(50);
            gt9xx_gtp_reset_guitar(ts->client, 50);
            msleep(50);
            gt9xx_gtp_send_cfg(ts->client);
        }
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        GT9XX_INFO("Esd suspended!");
    }
    return;
}
#endif

static const struct i2c_device_id gt9xx_ts_id[] = {
    { GT9XX_I2C_NAME, 0 },
    { }
};

static struct of_device_id goodix_ts_dt_ids[] = {
    { .compatible = "goodix,gt9xx" },
    { }
};
static struct i2c_driver gt9xx_ts_driver = {
    .probe      = gt9xx_ts_probe,
    .remove     = gt9xx_ts_remove,
    .id_table   = gt9xx_ts_id,
    .driver = {
        .name     = GT9XX_I2C_NAME,
        .owner    = THIS_MODULE,
	.of_match_table = of_match_ptr(goodix_ts_dt_ids),
    },
};

/*******************************************************    
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int gt9xx_ts_init(void)
{
    s32 ret;

    GT9XX_DEBUG_FUNC();   
    GT9XX_INFO("GTP driver installing...");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GT9XX_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
#if GT9XX_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
    ret = i2c_add_driver(&gt9xx_ts_driver);
    return ret; 
}

/*******************************************************    
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void  gt9xx_ts_exit(void)
{
    GT9XX_DEBUG_FUNC();
    GT9XX_INFO("GTP driver exited.");
    i2c_del_driver(&gt9xx_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

late_initcall(gt9xx_ts_init);
module_exit(gt9xx_ts_exit);

MODULE_DESCRIPTION("GT9XX Series Driver");
MODULE_LICENSE("GPL");
