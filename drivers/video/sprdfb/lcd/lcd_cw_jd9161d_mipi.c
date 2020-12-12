/* drivers/video/sc8825/lcd_cw_jd9161d_mipi.c
 *
 * Support for cw_jd9161d mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/delay.h>

#include <soc/sprd/gpio.h>

#include "../sprdfb_panel.h"

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

#define MAX_DATA   48
#define LCD_ID		236

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

typedef struct LCM_force_cmd_code_tag{
	unsigned int datatype;
	LCM_Init_Code real_cmd_code;
}LCM_Force_Cmd_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

static LCM_Init_Code init_data[] = {
	{LCM_SEND(6), {4,0,0xBF,0x91,0x61,0xF2}},
	{LCM_SEND(5), {3,0,0xB3,0x00,0x84}}, 
	{LCM_SEND(5), {3,0,0xB4,0x00,0x96}}, 
	{LCM_SEND(9), {7,0,0xB8,0x00,0xBF,0x01,0x00,0xBF,0x01}}, 
	{LCM_SEND(6), {4,0,0xBA,0x3E,0x23,0x00}}, 
	{LCM_SEND(2), {0xC3,0x04}}, 
	{LCM_SEND(5), {3,0,0xC4,0x30,0x6A}}, 
	{LCM_SEND(12), {10,0,0xC7,0x00,0x21,0x42,0x05,0x25,0x2B,0x12,0xA5,0xA5}}, 
	 
	{LCM_SEND(41), {39,0,0xC8,0x79,0x75,0x63,0x4F,0x3C,0x27,0x26,0x10,0x2B,0x2E,0x32,0x55,0x4A,0x5B,0x55,0x5C,0x57,
			0x4F,0x41,0x79,0x75,0x63,0x4F,0x3C,0x27,0x26,0x10,0x2B,0x2E,0x32,0x55,0x4A,0x5B,0x55,0x5C,0x57,0x4F,0x41}},
	{LCM_SEND(19), {17,0,0xD4,0x1E,0x1F,0x17,0x18,0x06,0x04,0x0A,0x08,0x00,0x02,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},				 
	{LCM_SEND(19), {17,0,0xD5,0x1E,0x1F,0x17,0x18,0x07,0x05,0x0B,0x09,0x01,0x03,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},				 
	{LCM_SEND(19), {17,0,0xD6,0x1F,0x1E,0x17,0x18,0x07,0x09,0x0B,0x05,0x03,0x01,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},				 
	{LCM_SEND(19), {17,0,0xD7,0x1F,0x1E,0x17,0x18,0x06,0x08,0x0A,0x04,0x02,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},				 
	{LCM_SEND(23), {21,0,0xD8,0x20,0x00,0x00,0x30,0x06,0x30,0x01,0x02,0x00,0x01,0x02,0x70,0x6C,0x00,0x00,0x73,0x0A,0x70,0x6C,0x08}},		 
	{LCM_SEND(22), {20,0,0xD9,0x00,0x0F,0x00,0x80,0x00,0x00,0x06,0x7B,0x00,0xBC,0x00,0x3B,0x6D,0x18,0x00,0x00,0x00,0x03,0x7B}},			 
	 
    	{LCM_SEND(2), {0x35,0x00}},
	{LCM_SEND(2), {0xBE,0x01}},
	 
	{LCM_SEND(2), {0xC1,0x10}},

	 
	{LCM_SEND(16), {14,0,0xEB,0xF7,0x05,0xFA,0x05,0x05,0xFB,0x00,0x05,0x08,0x05,0x46,0xFF,0xFF}},
	{LCM_SEND(6), {4,0,0xE0,0x01,0x04,0xFF}},
	 
	
	 
	//{LCM_SEND(13), {11, 0, 0xCC,0x34,0x20,0x38,0x60,0x11,0x91,0x00,0x40,0x00,0x00}},
	 
	//{LCM_SEND(2), {0xBE,0x00}},
	
	//{LCM_SEND(5), {3,0,0xC5,0x04,0x00}},

     
	{LCM_SEND(13), {11, 0, 0xCC,0x34,0x20,0x38,0x60,0x11,0x91,0x00,0x40,0x00,0x00}},
	{LCM_SEND(2), {0xBE,0x00}},
     {LCM_SEND(9), {6,0,0xD0,0x2E,0x00,0x11,0x04,0x80}},

	 
	{LCM_SEND(5), {3,0,0xE7,0x01,0x12}},
	{LCM_SEND(5), {3,0,0xDE,0x00,0x03}},
	 
	
	 
	//{LCM_SEND(2), {0x51,0xFF}},
	{LCM_SEND(2), {0x53,0x24}},
    {LCM_SEND(2), {0x55,0x01}},
	
	{LCM_SEND(2), {0x11,0x00}}, 
	{LCM_SLEEP(120)},
	{LCM_SEND(2), {0x29,0x00}}, 
	{LCM_SLEEP(10)},

 };

static LCM_Init_Code disp_on =  {LCM_SEND(1), {0x29}};

static LCM_Init_Code sleep_in[] =  {
    {LCM_SEND(1), {0x28}},
    {LCM_SLEEP(120)},
    {LCM_SEND(1), {0x10}},
    {LCM_SLEEP(10)},
};

static LCM_Init_Code sleep_out[] =  {
    {LCM_SEND(1), {0x11}},
    {LCM_SLEEP(120)},
    {LCM_SEND(1), {0x29}},
    {LCM_SLEEP(20)},
};

static LCM_Force_Cmd_Code rd_prep_code[]={
	{0x39, {LCM_SEND(6), {4,0,0xBF,0x91,0x61,0xF2}}},
	{0x37, {LCM_SEND(2), {0x3, 0}}},
};

static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};
static int32_t cw_jd9161d_mipi_init(struct panel_spec *self)
{
	int32_t i;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk( "kernel cw_jd9161d_mipi_init\n");

	msleep(2);
	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(20);
		}else if(tag & LCM_TAG_SLEEP){
			//udelay((init->tag & LCM_TAG_MASK) * 1000);
				msleep(init->tag & LCM_TAG_MASK);
		}
		init++;
	}
	mipi_eotp_set(1,1);

	return 0;
}

static uint32_t cw_jd9161d_readid(struct panel_spec *self)
{
	 
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code;
	uint8_t read_data[3] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;
	int return_val = 0;
	
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("kernel lcd_cw_jd9161d_mipi read id!\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);

	for(j = 0; j < 4; j++){
		rd_prepare = rd_prep_code;
		for(i = 0; i < ARRAY_SIZE(rd_prep_code); i++){
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if(tag & LCM_TAG_SEND){
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
				udelay(20);
			}else if(tag & LCM_TAG_SLEEP){
				msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}
			rd_prepare++;
		}
		read_rtn = mipi_force_read(0x04, 3,(uint8_t *)read_data);
		printk("lcd_cw_jd9161d_mipi read id 0x04 value is 0x%x, 0x%x, 0x%x!\n", read_data[0], read_data[1], read_data[2]);

		if((0x91 == read_data[0] && 0x61 == read_data[1] && 0x00 == read_data[2])){
			printk("lcd_cw_jd9161d_mipi read id success!\n");

			gpio_request(LCD_ID, "LCD_ID");
			gpio_direction_input(LCD_ID);
			return_val = gpio_get_value(LCD_ID);
			printk("lcd_cw_jd9161d_mipi gpio(lcd_id)=%d\n", return_val);
			
			if (!return_val) {
				printk("lcd_cw_jd9161d_mipi matched\n");
				mipi_eotp_set(1,1);
				return 0x9161d;				
			}

		}
	}
	mipi_eotp_set(1,1);
	return 0;
}

static int32_t cw_jd9161d_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("cw_jd9161d_enter_sleep, is_sleep = %d\n", is_sleep);

	if(is_sleep){
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	}else{
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}
	mipi_eotp_set(1,0);

	for(i = 0; i <size ; i++){
		tag = (sleep_in_out->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
			udelay(20);
		}else if(tag & LCM_TAG_SLEEP){
			//udelay((sleep_in_out->tag & LCM_TAG_MASK) * 1000);
			msleep((sleep_in_out->tag & LCM_TAG_MASK));
		}
		sleep_in_out++;
	}
	return 0;
}

static uint32_t cw_jd9161d_readpowermode(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
	uint8_t read_data[1] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;
	uint32_t reg_val_1 = 0;
	uint32_t reg_val_2 = 0;

	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	pr_debug("lcd_cw_jd9161d_mipi read power mode!\n");
	mipi_eotp_set(0,1);

	for(j = 0; j < 4; j++){
		rd_prepare = rd_prep_code_1;
		for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++){
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if(tag & LCM_TAG_SEND){
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
				udelay(20);
			}else if(tag & LCM_TAG_SLEEP){
				msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}
			rd_prepare++;
		}

		read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
		 

		if((0x9c == read_data[0])  && (0 == read_rtn)){
			pr_debug("lcd_cw_jd9161d_mipi read power mode success!\n");
			mipi_eotp_set(1,1);
			return 0x9c;
		}
	}

	mipi_eotp_set(1,1);
	return 0x0;
}


static uint32_t cw_jd9161d_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	pr_debug("cw_jd9161d_check_esd!\n");
	mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;

	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;

	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_lp_mode();
	}else{
		mipi_set_data_lp_mode();
	}
	power_mode = cw_jd9161d_readpowermode(self);
	//power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("cw_jd9161d_check_esd OK!\n");
		return 1;
	}else{
		printk("cw_jd9161d_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}


static struct panel_operations lcd_cw_jd9161d_mipi_operations = {
	.panel_init = cw_jd9161d_mipi_init,
	.panel_readid = cw_jd9161d_readid,
	.panel_enter_sleep = cw_jd9161d_enter_sleep,
	.panel_esd_check = cw_jd9161d_check_esd,
};

static struct timing_rgb lcd_cw_jd9161d_mipi_timing = {
	.hfp = 50,  
	.hbp = 50, 
	.hsync = 10, 
	.vfp = 10, 
	.vbp = 15,
	.vsync = 2,  
};

static struct info_mipi lcd_cw_jd9161d_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24,  
	.lan_number = 2,
	.phy_feq = 496*1000,  
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_cw_jd9161d_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_cw_jd9161d_mipi_spec = {
	//.cap = PANEL_CAP_NOT_TEAR_SYNC,
	.width = 480,
	.height = 854,
	.fps = 60,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.reset_timing = {5, 15, 120},
	.info = {
		.mipi = &lcd_cw_jd9161d_mipi_info
	},
	.ops = &lcd_cw_jd9161d_mipi_operations,
};

struct panel_cfg lcd_cw_jd9161d_mipi = {
	 
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x9161d,
	.lcd_name = "lcd_cw_jd9161d_mipi",
	.panel = &lcd_cw_jd9161d_mipi_spec,
};

static int __init lcd_cw_jd9161d_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_cw_jd9161d_mipi);
}

subsys_initcall(lcd_cw_jd9161d_mipi_init);
