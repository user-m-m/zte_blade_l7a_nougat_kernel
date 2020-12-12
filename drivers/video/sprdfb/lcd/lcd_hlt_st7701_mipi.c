/* drivers/video/sc8825/lcd_hlt_st7701_mipi.c
 *
 * Support for hlt_st7701 mipi LCD device
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
    {LCM_SEND(2),{0x11,0x00}}, 
	{LCM_SLEEP(120)},	
	{LCM_SEND(8),{6,0,0xFF,0X77,0x01,0x00,0x00,0x10}},
	{LCM_SEND(5),{3,0,0xC0,0xE9,0x03}},
	{LCM_SEND(5),{3,0,0xC1,0x11,0x02}},
	{LCM_SEND(5),{3,0,0xC2,0x01,0x05}},
	{LCM_SEND(2),{0xCC,0x10}},
	{LCM_SEND(19),{17,0,0xB0,0x00,0x06,0x11,0x12,0x18,0x0A,0x0A,0x09,0x09,0x1D,0x09,0x14,0x10,0x0E,0x11,0x19}},
	{LCM_SEND(19),{17,0,0xB1,0x00,0x06,0x11,0x11,0x15,0x09,0x0B,0x09,0x09,0x23,0x09,0x17,0x14,0x18,0x1E,0x19}},
	{LCM_SEND(8),{6,0,0xFF,0x77,0x01,0x00,0x00,0x11}},
	{LCM_SEND(2),{0xB0,0x5D}},
	{LCM_SEND(2),{0xB1,0x38}},
	{LCM_SEND(2),{0xB2,0x87}},
	{LCM_SEND(2),{0xB3,0x80}},
	{LCM_SEND(2),{0xB5,0x47}},
	{LCM_SEND(2),{0xB7,0x85}},
	{LCM_SEND(2),{0xB8,0x20}},
	{LCM_SEND(2),{0xB9,0x10}},
	{LCM_SEND(2),{0xC1,0x78}},
	{LCM_SEND(2),{0xC2,0x78}},
	{LCM_SEND(2),{0xD0,0x88}},
	{LCM_SEND(6),{4,0,0xE0,0x00,0x00,0x02}},
	{LCM_SEND(14),{12,0,0xE1,0x08,0x00,0x0A,0x00,0x07,0x00,0x09,0x00,0x00,0x33,0x33}},
	{LCM_SEND(16),{14,0,0xE2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{LCM_SEND(7),{5,0,0xE3,0x00,0x00,0x33,0x33}},
	{LCM_SEND(5),{3,0,0xE4,0x44,0x44}},
	{LCM_SEND(19),{17,0,0xE5,0x0E,0x60,0xA0,0xA0,0x10,0x60,0xA0,0xA0,0x0A,0x60,0xA0,0xA0,0x0C,0x60,0xA0,0xA0}},
	{LCM_SEND(7),{5,0,0xE6,0x00,0x00,0x33,0x33}},
	{LCM_SEND(5),{3,0,0xE7,0x44,0x44}},
	{LCM_SEND(19),{17,0,0xE8,0x0D,0x60,0xA0,0xA0,0x0F,0x60,0xA0,0xA0,0x09,0x60,0xA0,0xA0,0x0B,0x60,0xA0,0xA0}},
	{LCM_SEND(10),{8,0,0xEB,0x02,0x01,0xE4,0xE4,0x44,0x00,0x40}},
	{LCM_SEND(5),{3,0,0xEC,0x02,0x01}},
	{LCM_SEND(19),{17,0,0xED,0xAB,0x89,0x76,0x54,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x10,0x45,0x67,0x98,0xBA}},
	{LCM_SEND(8),{6,0,0xEF,0x10,0x0D,0x04,0x08,0x0F}},
	{LCM_SEND(8),{6,0,0xFF,0x77,0x01,0x00,0x00,0x00}},
		
	 /*{LCM_SEND( 8), { 6, 0, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x12} },
	{LCM_SEND(17), {15, 0,0xd1,0x81,0x20,0x03,0x56,0x08,0x01,0xc0,0x01,0xe0,0xc0,0x01,0xe0,0x03,0x56} },
	{LCM_SEND( 2), {0xD2, 0x05} }, 
	

    {LCM_SEND(8), {6,0,0xFF,0x77,0x01,0x00,0x00,0x10}},
	  */
	{LCM_SEND(2), {0xBB,0x02}},
	{LCM_SEND(2), {0xBC,0x02}},
	{LCM_SEND(2), {0xCA,0x10}},
	 
	
	{LCM_SEND(8), {6,0,0xFF,0x77,0x01,0x00,0x00,0x00}},
	 
	{LCM_SEND(2), {0x51,0x00}},
	{LCM_SEND(2), {0x53,0x2c}},
    {LCM_SEND(2), {0x55,0x01}},
	//{LCM_SEND(2), {0x21,0x00}},
	
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
	{0x39, {LCM_SEND(8), {6,0,0xFF,0x77,0x01,0x00,0x00,0x11}}},
	{0x37, {LCM_SEND(2), {0xd1,0x11}}},
};


static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};

static int32_t hlt_st7701_mipi_init(struct panel_spec *self)
{
	int32_t i;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk( "kernel hlt_st7701_mipi_init\n");

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

static uint32_t hlt_st7701_readid(struct panel_spec *self)
{
	 
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code;
	uint8_t read_data[2] = {0};
	uint8_t read_da_data = 0;
	int32_t read_rtn = 0;
	unsigned int tag = 0;
	int return_val = 0;
	
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("kernel lcd_hlt_st7701_mipi read id!\n");

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
		read_rtn = mipi_force_read(0xA1, 2,(uint8_t *)read_data);
		printk("lcd_hlt_st7701_mipi read id 0x04 value is 0x%x, 0x%x!\n", read_data[0], read_data[1]);
		mipi_force_read(0xDA, 1,&read_da_data);
		printk("lcd_hlt_st7701_mipi read 0xDA value is 0x%x!\n", read_da_data);

		if(0x88 == read_data[0] && 0x02 == read_data[1]){
			printk("lcd_hlt_st7701_mipi read id success!\n");
			if(0x10 == read_da_data){
			printk("lcd_hlt_st7701_mipi matched\n");
			return 0x8802;	
			}			
		}
	}
	mipi_eotp_set(1,1);
	return 0;
}

static int32_t hlt_st7701_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("hlt_st7701_enter_sleep, is_sleep = %d\n", is_sleep);

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

static uint32_t hlt_st7701_readpowermode(struct panel_spec *self)
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

	pr_debug("lcd_hlt_st7701_mipi read power mode!\n");
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
			pr_debug("lcd_hlt_st7701_mipi read power mode success!\n");
			mipi_eotp_set(1,1);
			return 0x9c;
		}
	}

	mipi_eotp_set(1,1);
	return 0x0;
}


static uint32_t hlt_st7701_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	pr_debug("hlt_st7701_check_esd!\n");
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
	power_mode = hlt_st7701_readpowermode(self);
	//power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("hlt_st7701_check_esd OK!\n");
		return 1;
	}else{
		printk("hlt_st7701_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}


static struct panel_operations lcd_hlt_st7701_mipi_operations = {
	.panel_init = hlt_st7701_mipi_init,
	.panel_readid = hlt_st7701_readid,
	.panel_enter_sleep = hlt_st7701_enter_sleep,
	.panel_esd_check = hlt_st7701_check_esd,
};

static struct timing_rgb lcd_hlt_st7701_mipi_timing = {
	.hfp = 70,   
	.hbp = 70, 
	.hsync = 20, 
	.vfp = 20,  
	.vbp = 20,
	.vsync = 4,  
};

static struct info_mipi lcd_hlt_st7701_mipi_info = {
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
	.timing = &lcd_hlt_st7701_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_hlt_st7701_mipi_spec = {
	//.cap = PANEL_CAP_NOT_TEAR_SYNC,
	.width = 480,
	.height = 854,
	.fps = 60,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.reset_timing = {5, 15, 120},
	.info = {
		.mipi = &lcd_hlt_st7701_mipi_info
	},
	.ops = &lcd_hlt_st7701_mipi_operations,
};

struct panel_cfg lcd_hlt_st7701_mipi = {
	 
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x8802,
	.lcd_name = "lcd_hlt_st7701_mipi",
	.panel = &lcd_hlt_st7701_mipi_spec,
};

static int __init lcd_hlt_st7701_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_hlt_st7701_mipi);
}

subsys_initcall(lcd_hlt_st7701_mipi_init);
