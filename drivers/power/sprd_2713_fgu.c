/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <soc/sprd/sci.h>
#include <soc/sprd/sci_glb_regs.h>
#include <soc/sprd/adi.h>
#include <soc/sprd/adc.h>

#include "sprd_2713_fgu.h"
#include "sprd_battery.h"

#ifdef AUTO_CNOM
#include <linux/types.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif
#define REGS_FGU_BASE ANA_FGU_BASE

 
#define REG_FGU_START                   SCI_ADDR(REGS_FGU_BASE, 0x0000)
#define REG_FGU_CONFIG                  SCI_ADDR(REGS_FGU_BASE, 0x0004)
#define REG_FGU_ADC_CONFIG              SCI_ADDR(REGS_FGU_BASE, 0x0008)
#define REG_FGU_STATUS                  SCI_ADDR(REGS_FGU_BASE, 0x000c)
#define REG_FGU_INT_EN                  SCI_ADDR(REGS_FGU_BASE, 0x0010)
#define REG_FGU_INT_CLR                 SCI_ADDR(REGS_FGU_BASE, 0x0014)
#define REG_FGU_INT_RAW                 SCI_ADDR(REGS_FGU_BASE, 0x0018)
#define REG_FGU_INT_STS                 SCI_ADDR(REGS_FGU_BASE, 0x001c)
#define REG_FGU_VOLT_VAL                SCI_ADDR(REGS_FGU_BASE, 0x0020)
#define REG_FGU_OCV_VAL                 SCI_ADDR(REGS_FGU_BASE, 0x0024)
#define REG_FGU_POCV_VAL                SCI_ADDR(REGS_FGU_BASE, 0x0028)
#define REG_FGU_CURT_VAL                SCI_ADDR(REGS_FGU_BASE, 0x002c)
#define REG_FGU_HIGH_OVER               SCI_ADDR(REGS_FGU_BASE, 0x0030)
#define REG_FGU_LOW_OVER                SCI_ADDR(REGS_FGU_BASE, 0x0034)
#define REG_FGU_VTHRE_HH                SCI_ADDR(REGS_FGU_BASE, 0x0038)
#define REG_FGU_VTHRE_HL                SCI_ADDR(REGS_FGU_BASE, 0x003c)
#define REG_FGU_VTHRE_LH                SCI_ADDR(REGS_FGU_BASE, 0x0040)
#define REG_FGU_VTHRE_LL                SCI_ADDR(REGS_FGU_BASE, 0x0044)
#define REG_FGU_OCV_LOCKLO              SCI_ADDR(REGS_FGU_BASE, 0x0048)
#define REG_FGU_OCV_LOCKHI              SCI_ADDR(REGS_FGU_BASE, 0x004c)
#define REG_FGU_CLBCNT_SETH             SCI_ADDR(REGS_FGU_BASE, 0x0050)
#define REG_FGU_CLBCNT_SETL             SCI_ADDR(REGS_FGU_BASE, 0x0054)
#define REG_FGU_CLBCNT_DELTH          SCI_ADDR(REGS_FGU_BASE, 0x0058)
#define REG_FGU_CLBCNT_DELTL           SCI_ADDR(REGS_FGU_BASE, 0x005c)
#define REG_FGU_CLBCNT_LASTOCVH         SCI_ADDR(REGS_FGU_BASE, 0x0060)
#define REG_FGU_CLBCNT_LASTOCVL         SCI_ADDR(REGS_FGU_BASE, 0x0064)
#define REG_FGU_CLBCNT_VALH             SCI_ADDR(REGS_FGU_BASE, 0x0068)
#define REG_FGU_CLBCNT_VALL             SCI_ADDR(REGS_FGU_BASE, 0x006c)
#define REG_FGU_CLBCNT_QMAXH            SCI_ADDR(REGS_FGU_BASE, 0x0070)
#define REG_FGU_CLBCNT_QMAXL            SCI_ADDR(REGS_FGU_BASE, 0x0074)
#define REG_FGU_QMAX_TOSET        SCI_ADDR(REGS_FGU_BASE, 0x0078)
#define REG_FGU_QMAX_TIMER          SCI_ADDR(REGS_FGU_BASE, 0x007c)
#define REG_FGU_RELAX_CURT_THRE         SCI_ADDR(REGS_FGU_BASE, 0x0080)
#define REG_FGU_RELAX_CNT_THRE          SCI_ADDR(REGS_FGU_BASE, 0x0084)
#define REG_FGU_RELAX_CNT               SCI_ADDR(REGS_FGU_BASE, 0x0088)
#define REG_FGU_OCV_LAST_CNT            SCI_ADDR(REGS_FGU_BASE, 0x008c)
#define REG_FGU_CURT_OFFSET             SCI_ADDR(REGS_FGU_BASE, 0x0090)

#define REG_FGU_USER_AREA_SET             SCI_ADDR(REGS_FGU_BASE, 0x00A0)
#define REG_FGU_USER_AREA_CLEAR             SCI_ADDR(REGS_FGU_BASE, 0x00A4)
#define REG_FGU_USER_AREA_STATUS             SCI_ADDR(REGS_FGU_BASE, 0x00A8)

#define REG_FGU_VOLT_VALUE_BUF0		SCI_ADDR(REGS_FGU_BASE, 0x00D0)
#define REG_FGU_CURT_VALUE_BUF0		SCI_ADDR(REGS_FGU_BASE, 0x00F0)

#define BITS_POWERON_TYPE(_x_)           ( (_x_) << 12 & (0xF000))
#define BITS_RTC_AREA(_x_)           ( (_x_) << 0 & (0xFFF) )

 
#define BIT_QMAX_UPDATE_EN              ( BIT(2) )
#define BIT_FGU_RESET                   ( BIT(1) )
#define BIT_WRITE_SELCLB_EN             ( BIT(0) )

 
#define BIT_VOLT_H_VALID                ( BIT(12) )
#define BIT_FGU_DISABLE_EN              ( BIT(11) )
#define BIT_CLBCNT_DELTA_MODE           ( BIT(10) )
#define BITS_ONEADC_DUTY(_x_)           ( (_x_) << 8 & (BIT(8)|BIT(9)) )
#define BIT_CURT_DUTY                   ( BIT(7) )
#define BITS_VOLT_DUTY(_x_)             ( (_x_) << 5 & (BIT(5)|BIT(6)) )
#define BIT_AD1_ENABLE                  ( BIT(4) )
#define BIT_SW_DIS_CURT                 ( BIT(3) )
#define BIT_FORCE_LOCK_EN               ( BIT(2) )
#define BIT_LOW_POWER_MODE              ( BIT(1) )
#define BIT_AUTO_LOW_POWER              ( BIT(0) )

 
#define BIT_FORCE_AD1_VIN_EN            ( BIT(7) )
#define BIT_FORCE_AD0_VIN_EN            ( BIT(6) )
#define BIT_FORCE_AD0_IIN_EN            ( BIT(5) )
#define BIT_FORCE_AD_EN                 ( BIT(4) )
#define BIT_AD1_VOLT_REF                ( BIT(3) )
#define BIT_AD0_VOLT_REF                ( BIT(2) )
#define BIT_AD01_RESET                  ( BIT(1) )
#define BIT_AD01_PD                     ( BIT(0) )

 
#define BIT_POWER_LOW                   ( BIT(5) )
#define BIT_CURT_LOW                    ( BIT(4) )
#define BITS_OCV_LOCK_STS(_x_)          ( (_x_) << 2 & (BIT(2)|BIT(3)) )
#define BIT_QMAX_UPDATE_STS             ( BIT(1) )
#define BIT_WRITE_ACTIVE_STS            ( BIT(0) )

 
#define BIT_CURT_RDEN_INT               ( BIT(7) )
#define BIT_VOLT_RDEN_INT               ( BIT(6) )
#define BIT_QMAX_UPD_TOUT               ( BIT(5) )
#define BIT_QMAX_UPD_DONE               ( BIT(4) )
#define BIT_RELX_CNT_INT                ( BIT(3) )
#define BIT_CLBCNT_DELTA_INT            ( BIT(2) )
#define BIT_VOLT_HIGH_INT               ( BIT(1) )
#define BIT_VOLT_LOW_INT                ( BIT(0) )

#define BITS_VOLT_VALUE(_x_)            ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

 
#define BITS_CURT_VALUE(_x_)            ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

 
#define BITS_CLBCNT_SETH(_x_)           ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

 
#define BITS_CLBCNT_SETL(_x_)           ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )

 
#define BITS_CLBCNT_DELTHH(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

 
#define BITS_CLBCNT_DELTHL(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )

 
#define BITS_RELAX_CUR_THRE(_x_)        ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

 
#define BITS_RELAX_CNT_THRE(_x_)        ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

 
#define BITS_RELAX_CNT_VAL(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

 
#define BITS_OCV_LAST_CNT(_x_)          ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

 
#define BITS_CURT_OFFSET_VAL(_x_)       ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

#define SPRDFGU__DEBUG
#ifdef SPRDFGU__DEBUG
#define FGU_DEBUG(format, arg...) do{\
    pr_info("sprdfgu: " format, ## arg);\
    }while(0)
#else
#define FGU_DEBUG(format, arg...)
#endif

#define SPRDFGU_OCV_VALID_TIME    20

#define CUR_0ma_IDEA_ADC    8192

#define FGU_CUR_SAMPLE_HZ   2

#define FIRST_POWERTON  0xF
#define NORMAIL_POWERTON  0x5
#define WDG_POWERTON  0xA
struct sprdfgu_drivier_data {
	struct sprd_battery_platform_data *pdata;
	int adp_status;
	int warning_cap;
	int shutdown_vol;
	int bat_full_vol;
	int cur_rint;
	int poweron_rint;
	int init_cap;
	int init_clbcnt;
	unsigned int int_status;
	struct delayed_work fgu_irq_work;
	struct power_supply sprdfgu;
	struct mutex lock;
	struct wake_lock low_power_lock;
};

struct sprdfgu_drivier_data sprdfgu_data;
struct sprdfgu_cal {
	int cur_1000ma_adc;
	int vol_1000mv_adc;
	int cur_offset;
	int vol_offset;
	int cal_type;
};
static struct sprdfgu_cal fgu_cal = { 2872, 678, 0, 0, SPRDBAT_FGUADC_CAL_NO };

 
struct delayed_work sprdfgu_debug_work;
uint32_t sprdfgu_debug_log_time = 120;
static int poweron_clbcnt;
static u32 start_time;
 

static int fgu_nv_4200mv = 2752;
static int fgu_nv_3600mv = 2374;
static int fgu_0_cur_adc = 8338;

static int cmd_vol_raw, cmd_cur_raw;

static BLOCKING_NOTIFIER_HEAD(fgu_chain_head);
extern int in_calibration(void);

#define REG_SYST_VALUE                  ((void __iomem *)(SPRD_SYSCNT_BASE + 0x0004))
static u32 sci_syst_read(void)
{
#if !(defined(CONFIG_ARCH_SCX35L64)||defined(CONFIG_ARCH_SCX35LT8))    //mingwei TODO
	u32 t = __raw_readl(REG_SYST_VALUE);
	while (t != __raw_readl(REG_SYST_VALUE))
		t = __raw_readl(REG_SYST_VALUE);
	return t;
#else
	return 0;
#endif
}

static int sprdfgu_vol2capacity(uint32_t voltage)
{
	int percentum =
	    sprdbat_interpolate(voltage, sprdfgu_data.pdata->ocv_tab_size,
				sprdfgu_data.pdata->ocv_tab);

	return percentum;
}

static int __init fgu_cal_start(char *str)
{
	unsigned int fgu_data[3] = { 0 };
	char *cali_data = &str[1];
	if (str) {
		printk("sprdfgu fgu_cal%s!\n", str);
		sscanf(cali_data, "%d,%d,%d", &fgu_data[0], &fgu_data[1],
		       &fgu_data[2]);
		printk("sprdfgu fgu_data: 0x%x 0x%x,0x%x!\n", fgu_data[0],
		       fgu_data[1], fgu_data[2]);
		fgu_nv_4200mv = (fgu_data[0] >> 16) & 0xffff;
		fgu_nv_3600mv = (fgu_data[1] >> 16) & 0xffff;
		fgu_0_cur_adc = fgu_data[2];
		fgu_cal.cal_type = SPRDBAT_FGUADC_CAL_NV;
	}
	return 1;
}

__setup("fgu_cal", fgu_cal_start);
static int __init fgu_cmd(char *str)
{
	int fgu_data[2] = { 0 };
	char *cali_data = &str[1];
	if (str) {
		pr_info("fgu cmd%s!\n", str);
		sscanf(cali_data, "%d,%d", &fgu_data[0], &fgu_data[1]);
		pr_info("fgu_cmd adc_data: 0x%x 0x%x!\n", fgu_data[0],
			fgu_data[1]);
		cmd_vol_raw = fgu_data[0];
		cmd_cur_raw = fgu_data[1];
	}
	return 1;
}

__setup("fgu_init", fgu_cmd);

static int sprdfgu_cal_init(void)
{
	BUG_ON(fgu_nv_4200mv <= fgu_nv_3600mv);
	if (0 == fgu_nv_3600mv) {
		fgu_cal.vol_1000mv_adc =
		    DIV_ROUND_CLOSEST((fgu_nv_4200mv) * 10, 42);
		fgu_cal.vol_offset = 0;
		fgu_cal.cur_offset = 0;
		fgu_cal.cur_1000ma_adc =
		    DIV_ROUND_CLOSEST(fgu_cal.vol_1000mv_adc * 4 *
				      sprdfgu_data.pdata->rsense_real,
				      sprdfgu_data.pdata->rsense_spec);
	} else {
		fgu_cal.vol_1000mv_adc =
		    DIV_ROUND_CLOSEST((fgu_nv_4200mv - fgu_nv_3600mv) * 10, 6);
		fgu_cal.vol_offset =
		    0 - (fgu_nv_4200mv * 10 - fgu_cal.vol_1000mv_adc * 42) / 10;
		fgu_cal.cur_offset = CUR_0ma_IDEA_ADC - fgu_0_cur_adc;
		fgu_cal.cur_1000ma_adc =
		    DIV_ROUND_CLOSEST(fgu_cal.vol_1000mv_adc * 4 *
				      sprdfgu_data.pdata->rsense_real,
				      sprdfgu_data.pdata->rsense_spec);
	}
	if (SPRDBAT_FGUADC_CAL_CHIP == fgu_cal.cal_type) {
		fgu_cal.vol_offset += sprdfgu_data.pdata->fgu_cal_ajust;
		printk("sprdfgu: sprdfgu_data.pdata->fgu_cal_ajust = %d\n",
		       sprdfgu_data.pdata->fgu_cal_ajust);
	}

	printk
	    ("sprdfgu: sprdfgu_cal_init fgu_nv_4200mv = %d,fgu_nv_3600mv = %d,fgu_0_cur_adc = %d\n",
	     fgu_nv_4200mv, fgu_nv_3600mv, fgu_0_cur_adc);
	printk
	    ("sprdfgu: sprdfgu_cal_init fgu_cal.cur_1000ma_adc = %d,fgu_cal.vol_1000mv_adc = %d,fgu_cal.vol_offset = %d,fgu_cal.cur_offset = %d\n",
	     fgu_cal.cur_1000ma_adc, fgu_cal.vol_1000mv_adc, fgu_cal.vol_offset,
	     fgu_cal.cur_offset);
	return 0;
}


#ifdef CONFIG_OTP_SPRD
int sci_efuse_fgu_cal_get(unsigned int *p_cal_data);
#endif
static int sprdfgu_cal_from_chip(void)
{
	unsigned int fgu_data[4] = { 0 };

#ifdef CONFIG_OTP_SPRD
	if (!sci_efuse_fgu_cal_get(fgu_data)) {
		printk("sprdfgu: sprdfgu_cal_from_chip efuse no cal data\n");
		return 1;
	}
#else
	printk("sprdfgu: sprdfgu_cal_from_chip no sprd efuse\n");
	return 1;
#endif
	printk("sprdfgu fgu_data: 0x%x 0x%x,0x%x,0x%x!\n", fgu_data[0],
	       fgu_data[1], fgu_data[2], fgu_data[3]);
	printk("sprdfgu: sprdfgu_cal_from_chip\n");

	fgu_nv_4200mv = fgu_data[0];
#if defined(CONFIG_ADIE_SC2723S) || defined(CONFIG_ADIE_SC2723)//|| defined(CONFIG_ADIE_SC2731)	//2723 use one point to cal fgu adc
	fgu_nv_3600mv = 0;
	fgu_0_cur_adc = 0;
	printk("sprdfgu: one point\n");
#else
	fgu_nv_3600mv = fgu_data[1];
	fgu_0_cur_adc = fgu_data[2];
	printk("sprdfgu: three point\n");
#endif
	fgu_cal.cal_type = SPRDBAT_FGUADC_CAL_CHIP;

	return 0;
}

static u32 sprdfgu_adc2vol_mv(u32 adc)
{
	return ((adc + fgu_cal.vol_offset) * 1000) / fgu_cal.vol_1000mv_adc;
}

static u32 sprdfgu_vol2adc_mv(u32 vol)
{
	return (vol * fgu_cal.vol_1000mv_adc) / 1000 - fgu_cal.vol_offset;
}

static int sprdfgu_adc2cur_ma(int adc)
{
	return (adc * 1000) / fgu_cal.cur_1000ma_adc;
}

static u32 sprdfgu_cur2adc_ma(u32 cur)
{
	return (cur * fgu_cal.cur_1000ma_adc) / 1000;
}

#if defined(CONFIG_ADIE_SC2723S) || defined(CONFIG_ADIE_SC2723) || defined(CONFIG_ADIE_SC2731)
static void sprdfgu_rtc_reg_write(uint32_t val)
{
	sci_adi_write(REG_FGU_USER_AREA_CLEAR, BITS_RTC_AREA(~val),
		      BITS_RTC_AREA(~0));
	sci_adi_write(REG_FGU_USER_AREA_SET, BITS_RTC_AREA(val),
		      BITS_RTC_AREA(~0));
}

static uint32_t sprdfgu_rtc_reg_read(void)
{
	int shft = __ffs(BITS_RTC_AREA(~0));
	return (sci_adi_read(REG_FGU_USER_AREA_STATUS) & BITS_RTC_AREA(~0)) >>
	    shft;
}

static void sprdfgu_poweron_type_write(uint32_t val)
{
	sci_adi_write(REG_FGU_USER_AREA_CLEAR, BITS_POWERON_TYPE(~val),
		      BITS_POWERON_TYPE(~0));
	sci_adi_write(REG_FGU_USER_AREA_SET, BITS_POWERON_TYPE(val),
		      BITS_POWERON_TYPE(~0));
}

static uint32_t sprdfgu_poweron_type_read(void)
{
	int shft = __ffs(BITS_POWERON_TYPE(~0));
	return (sci_adi_read(REG_FGU_USER_AREA_STATUS) & BITS_POWERON_TYPE(~0))
	    >> shft;
}
#endif
static inline int sprdfgu_clbcnt_get(void)
{
	volatile int cc1 = 0, cc2 = 1;

	do {
		cc1 = (sci_adi_read(REG_FGU_CLBCNT_VALL)) & 0xFFFF;
		cc1 |= (((sci_adi_read(REG_FGU_CLBCNT_VALH)) & 0xFFFF) << 16);

		cc2 = (sci_adi_read(REG_FGU_CLBCNT_VALL)) & 0xFFFF;
		cc2 |= (((sci_adi_read(REG_FGU_CLBCNT_VALH)) & 0xFFFF) << 16);
	} while (cc1 != cc2);

	//FGU_DEBUG("sprdfgu_clbcnt_get cc: %d\n", cc1);
	return cc1;
}

static inline int sprdfgu_clbcnt_set(int clbcc)
{
	sci_adi_write(REG_FGU_CLBCNT_SETL, clbcc & 0xFFFF, ~0);
	sci_adi_write(REG_FGU_CLBCNT_SETH, (clbcc >> 16) & 0xFFFF, ~0);
	sci_adi_set(REG_FGU_START, BIT_WRITE_SELCLB_EN);
	udelay(130);
	return 0;
}

static inline int sprdfgu_reg_get(unsigned long reg)
{
	volatile int vaule;

	do {
		vaule = sci_adi_read(reg);
	} while (vaule != sci_adi_read(reg));

	return vaule;
}

static inline int sprdfgu_clbcnt_init(u32 capacity)
{
	int init_cap =
	    DIV_ROUND_CLOSEST(sprdfgu_data.pdata->cnom * capacity, 100);
	int clbcnt =
	    DIV_ROUND_CLOSEST(init_cap * fgu_cal.cur_1000ma_adc * 36 *
			      FGU_CUR_SAMPLE_HZ, 10);
	return clbcnt;
}

static inline void sprdfgu_soc_adjust(int capacity)
{
	sprdfgu_data.init_cap = capacity;
	sprdfgu_data.init_clbcnt = sprdfgu_clbcnt_get();
	FGU_DEBUG("sprdfgu_soc_adjust sprdfgu_data.init_cap= %d,%d\n",
		  sprdfgu_data.init_cap, sprdfgu_data.init_clbcnt);
}

uint32_t sprdfgu_read_vbat_vol(void)
{
	u32 cur_vol_raw;
	uint32_t temp;
	cur_vol_raw = sprdfgu_reg_get(REG_FGU_VOLT_VAL);
	//FGU_DEBUG("cur_vol_raw = %d\n", cur_vol_raw);
	temp = sprdfgu_adc2vol_mv(cur_vol_raw);
	//FGU_DEBUG("sprdfgu_read_vbat_vol : %d\n", temp);
	return temp;
}

static inline u32 sprdfgu_ocv_vol_get(void)
{
	u32 ocv_vol_raw;
	ocv_vol_raw = sprdfgu_reg_get(REG_FGU_OCV_VAL);
	//FGU_DEBUG("ocv_vol_raw = %x\n", ocv_vol_raw);
	return sprdfgu_adc2vol_mv(ocv_vol_raw);
}

static inline int sprdfgu_cur_current_get(void)
{
	int current_raw;

	current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);
	//FGU_DEBUG("current_raw: %d\n", current_raw);

	return sprdfgu_adc2cur_ma(current_raw - CUR_0ma_IDEA_ADC);
}

int sprdfgu_read_batcurrent(void)
{
#ifndef CONFIG_SPRD_NOFGUCURRENT_CHG
	int temp = sprdfgu_cur_current_get();
	//FGU_DEBUG("sprdfgu_read_batcurrent : %d\n", temp);
	return temp;
#else
	return 0;
#endif
}

static int sprdfgu_read_vbat_ocv_pure(uint32_t * vol)
{
	if (sprdfgu_reg_get(REG_FGU_OCV_LAST_CNT) > SPRDFGU_OCV_VALID_TIME
	    || sprdfgu_reg_get(REG_FGU_OCV_VAL) == 0) {
		*vol = 0;
		return 0;
	}
	*vol = sprdfgu_ocv_vol_get();
	return 1;
}

uint32_t sprdfgu_read_vbat_ocv(void)
{
#ifndef CONFIG_SPRD_NOFGUCURRENT_CHG
	uint32_t vol;
	int rint = sprdfgu_data.cur_rint;

#ifdef SPRDFGU_TEMP_COMP_SOC
        {
            int temp;

            temp = sprdbat_read_temp();
            rint = sprdbat_interpolate(temp/10, sprdfgu_data.pdata->rint_temp_tab_size,
				sprdfgu_data.pdata->rint_temp_tab);
            FGU_DEBUG("OHH, new rint:%d,temp:%d\n", rint, temp);
        }
#endif

 
/*
	if (sprdfgu_read_vbat_ocv_pure(&vol)) {
		FGU_DEBUG("hwocv...\n");
		return vol;
	} else {
		return sprdfgu_read_vbat_vol() -
		    (sprdfgu_read_batcurrent() * rint) / 1000;
	}
	*/
	return sprdfgu_read_vbat_vol() -
		  (sprdfgu_read_batcurrent() * rint) / 1000;
#else
	return sprdfgu_read_vbat_vol();
#endif
}

static int sprdfgu_temp_comp_soc(int soc, int temp)
{
	int cnom_temp;
	int comp_soc, delta_soc;

	cnom_temp = 0;

	cnom_temp = sprdbat_interpolate(temp, sprdfgu_data.pdata->cnom_temp_tab_size,
				sprdfgu_data.pdata->cnom_temp_tab);
	delta_soc =
	    (sprdfgu_data.pdata->cnom - cnom_temp) * 100 / sprdfgu_data.pdata->cnom;

	comp_soc = (long)(soc - delta_soc) * 100 / (100 - delta_soc);

	if (comp_soc < 0)
		comp_soc = 0;

	if (comp_soc > 100)
		comp_soc = 100;

	FGU_DEBUG("OHH, cnom_temp %d, delta_soc %d,comp_soc %d,soc %d,temp %d\n",
		  cnom_temp, delta_soc, comp_soc, soc, temp);

	return (comp_soc);
}

#ifdef AUTO_CNOM
static int cnom_start_record;
static int cnom_start_counter;
static int8_t record_level = CNOM_CAP_LEVEL;
static int sprdfgu_cnominfo_opera(uint u_opera, uint u_cnom)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	char buf[30] = {0};
	uint u_temp_cnom = u_cnom;
	uint u_opera_type = u_opera;
	static uint u_last_cnom;

	 
	if (sprdfgu_data.pdata->fgu_mode) {
		return 0;
	}

	fp = filp_open(REAL_CNOM_SAVE_PATH, O_RDWR, 0644);
	if (IS_ERR(fp)) {
		FGU_DEBUG("cnominfo_opera create\n");

		fp = filp_open(REAL_CNOM_SAVE_PATH, O_CREAT | O_RDWR, 0644);
		if (IS_ERR(fp)) {
			FGU_DEBUG("cnominfo_opera create file err\n");
			return 0;
		}

		if (CNOM_OPERA_GET == u_opera_type) {
			u_opera_type = CNOM_OPERA_SET;
			FGU_DEBUG("first create , set default value firstly\n");
		}
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	switch (u_opera_type) {
	case CNOM_OPERA_GET:
		vfs_read(fp, buf, 5, &pos);
		sscanf(buf, "%05d", &u_temp_cnom);
		FGU_DEBUG("cnominfo_opera read:%s, %d\n",
				buf, u_temp_cnom);
		if (u_temp_cnom > MAX_CNOM)	{
			u_temp_cnom = MAX_CNOM;
		}
		u_last_cnom = u_temp_cnom;
		break;

	case CNOM_OPERA_SET:
		if (u_temp_cnom > MAX_CNOM)	{
			u_temp_cnom = MAX_CNOM;
		}
		if (u_temp_cnom != u_last_cnom)	{
			u_last_cnom = u_temp_cnom;
			sprintf(buf, "%05d", u_temp_cnom);
			vfs_write(fp, buf, 5, &pos);
		}
		FGU_DEBUG("cnominfo_opera write: %d\n", u_temp_cnom);
		break;

	 
	case CNOM_OPERA_REC_HISTORY:
		{
			struct timex  txc;
			struct rtc_time tm;

			do_gettimeofday(&(txc.time));
			rtc_time_to_tm(txc.time.tv_sec,&tm);
			tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8):(tm.tm_hour - 16);
			tm.tm_mon += 1;
			tm.tm_mday += 1;
			tm.tm_year += 1900;
			FGU_DEBUG("cnominfo_opera:%d-%d-%d %d:%d:%d\n", tm.tm_year,\
				tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
			pos = 10+record_level*30; /*30 = sizeof(buf)+reserved*/
			sprintf(buf, "%05d-%04d/%02d/%02d/%02d:%02d-%02d", u_cnom, tm.tm_year,\
				tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, record_level);
			vfs_write(fp, buf, sizeof(buf), &pos);
		}
		break;

	default:
		FGU_DEBUG("cnominfo_opera null\n");
		break;
	}

	filp_close(fp, NULL);
	set_fs(fs);

	return u_temp_cnom;
}

/************************************************************

START_CNOM_RECORD
END_CNOM_RECORD

************************************************************/

int sprdfgu_cnom_state_change(int state)
{
	static int cnom_begin, cnom_end, cnom_debug, debug_flag;
	int cnom_delta = 0;
	int temp = sprdfgu_data.pdata->cnom, temp_cnom = 0;
	 
	if (sprdfgu_data.pdata->fgu_mode || sprdbat_read_temp() < CNOM_TEMPER_LIMIT) {
		return temp;
	}

	switch (state) {
	case END_CNOM_RECORD:
		if (cnom_start_record != 0) {
			cnom_end =  sprdfgu_clbcnt_get();
			cnom_delta = cnom_begin - cnom_end;
			temp = DIV_ROUND_CLOSEST(cnom_delta,
						(3600 * FGU_CUR_SAMPLE_HZ));
			temp = sprdfgu_adc2cur_ma(temp);
			sprdfgu_cnominfo_opera(CNOM_OPERA_SET, temp);
			cnom_start_record = 0;
			FGU_DEBUG("cnom_state_change  cnom_end %d cnom %d\n",
					cnom_end, temp);
		}
		break;

	case START_CNOM_RECORD:
		cnom_begin = sprdfgu_clbcnt_get();
		cnom_start_counter = cnom_begin;
		cnom_start_record = 1;

 
		temp_cnom = sprdfgu_cnominfo_opera(CNOM_OPERA_GET, 0);
		if (temp_cnom != 0) {
			sprdfgu_data.pdata->cnom = temp_cnom;
		}

		FGU_DEBUG("cnom_state_change cnom_begin %d\n", cnom_begin);
		break;

	case ADJUST_CNOM_RECORD:
		if (cnom_start_record != 0)	{
			sprdfgu_soc_adjust(1);
			FGU_DEBUG("cnom_state_change sprdfgu_soc_adjust\n");
		}
		break;

	case DEBUG_CNOM_RECORD:  
		if (debug_flag == 0)
			cnom_debug = sprdfgu_clbcnt_get();
		debug_flag++;
		cnom_delta = cnom_debug - sprdfgu_clbcnt_get();
		cnom_delta = DIV_ROUND_CLOSEST(cnom_delta,
						(3600 * FGU_CUR_SAMPLE_HZ));
		cnom_delta = sprdfgu_adc2cur_ma(cnom_delta);
		FGU_DEBUG("cnom_state_change  debug begin = 0x%x ,\
			end = 0x%x, delta = %d \n",
				cnom_begin, cnom_end, cnom_delta);
		break;

	default:
		break;
	}

	return temp;
}

/**********************************************************************
	Battery capacity adaptation:

	Function start at batter full state . At the end of discharging

	cnom will record and update next poweron;

**********************************************************************/
uint sprdfgu_cnom_record(uint capacity, uint vbat_ocv)
{
	 
	if (sprdfgu_data.pdata->fgu_mode) {
		return capacity;
	}
	FGU_DEBUG("cnom_record capacity = %d ,vbat_ocv= %d\n",
			capacity, vbat_ocv);

	if (1 == cnom_start_record) {
		if (vbat_ocv > CNOM_RECORD_LOW_VOLTAGE_ADJUST) {
			sprdfgu_cnom_state_change(ADJUST_CNOM_RECORD);
			return CNOM_RECORD_END_CAP;
		} else{
			sprdfgu_data.pdata->cnom =
				sprdfgu_cnom_state_change(END_CNOM_RECORD);
		}
	}
	sprdfgu_cnom_state_change(DEBUG_CNOM_RECORD);

	return capacity;
}

#ifdef AUTO_CNOM_AHEAD
 
int sprdbat_interpolate_percent(int x, int n, struct sprdbat_table_data *tab)
{
	int index;
	int y;

	if (x >= tab[0].x)
		y = tab[0].y*100;
	else if (x <= tab[n - 1].x)
		y = tab[n - 1].y*100;
	else {
		 
		for (index = 1; index < n; index++)
			if (x > tab[index].x)
				break;
		 
		y = (tab[index - 1].y - tab[index].y) * (x - tab[index].x)
		    * 100 / (tab[index - 1].x - tab[index].x);
		y += tab[index].y*100;
	}
	return y;
}

/*Study cnom before discharge completely,about 0%-5% cap. The range of
error is from 0% to 2%, and getting smaller and smaller from 5% cap to 0%*/
unsigned char sprdfgu_verify_ocv(int ocv_flag, int vol_hw)
{
	uint32_t verify_capacity = 0;  
	uint32_t verify_ocv = 0;   
	uint32_t verify_cnom = 0;  
	uint32_t verify_cnom_full = 0;   
	uint32_t verify_vbat = 0; 
	int32_t cnom_end, cnom_delta, temp = 0;
	int32_t verify_current = 0;   
	int8_t  cap_level = 0;  
	static uint32_t cnom_rec[CNOM_CAP_LEVEL+1] = {
		MAX_CNOM, MAX_CNOM, MAX_CNOM, MAX_CNOM, MAX_CNOM, MAX_CNOM};
	static int32_t pointer[CNOM_CAP_LEVEL+1] = {0, 0, 0, 0, 0, 0};
	static int16_t init_flag;
	static int8_t old_cap_level;

	if ((cnom_start_record == 0) || (sprdfgu_data.pdata->fgu_mode == 1) ||
		 (sprdbat_read_temp() < CNOM_TEMPER_LIMIT)) {
		return 0;
	}

	 
	verify_current = sprdfgu_read_batcurrent();
	verify_vbat = sprdfgu_read_vbat_vol();
	cnom_end =  sprdfgu_clbcnt_get();
	if ((verify_current > 0) || (verify_current <= (-50))) {
		return 0;
	}

	 
	if (ocv_flag == 1) {
		verify_ocv = vol_hw;
	} else{
		verify_ocv = verify_vbat - (verify_current*sprdfgu_data.cur_rint)/1000;
	}

	 
	/*verify_capacity = sprdfgu_vol2capacity(verify_ocv);*/
	verify_capacity = sprdbat_interpolate_percent(verify_ocv, sprdfgu_data.pdata->ocv_tab_size,
				sprdfgu_data.pdata->ocv_tab);
	if (verify_capacity > CNOM_CAP_LEVEL*100+49) {
		FGU_DEBUG("verify_cap is  %d\n", verify_capacity);
		return 0;
	}

	 
	cnom_delta = cnom_start_counter - cnom_end;
	temp = DIV_ROUND_CLOSEST(cnom_delta, (3600 * FGU_CUR_SAMPLE_HZ));
	verify_cnom = sprdfgu_adc2cur_ma(temp);

	 
	verify_cnom_full = (verify_cnom*10000)/(10000 - verify_capacity);
	/*FGU_DEBUG("cur %05d,verify_cap %04d, sfocv %05d, cnom %06d,\
	cnom_f %06d\n", verify_current, verify_capacity, verify_ocv,
		      verify_cnom,verify_cnom_full);*/

	 
	if ((verify_capacity <= CNOM_CAP_LEVEL*100)
		&& verify_cnom_full < sprdfgu_data.pdata->cnom*CNOM_BOUNDARY/100){
		/*FGU_DEBUG("cap %04d, sfocv %05d, cnom %05d,cnom_f %05d\n",
			verify_capacity, verify_ocv, verify_cnom, verify_cnom_full);*/
		cap_level = (verify_capacity*2/100+1)/2;
		pointer[cap_level]++;

        
		if(cnom_rec[cap_level] > verify_cnom_full){
			cnom_rec[cap_level] = verify_cnom_full;
		}

		/*record the cnom :
		  or change to next level
		  or study more than 3 times in current time
		  and can record from 5% to 0%,can't 0%->5%*/
		if (((cap_level < old_cap_level) ||
			(pointer[cap_level] >= CNOM_BUFF_CNT)) &&
			(cap_level <= record_level)) {
			sprdfgu_cnominfo_opera(CNOM_OPERA_SET, cnom_rec[old_cap_level]);
			record_level = old_cap_level;
			sprdfgu_cnominfo_opera(CNOM_OPERA_REC_HISTORY, cnom_rec[old_cap_level]);
			pointer[old_cap_level] = 0;
			FGU_DEBUG("verify_cu %05d,cnom_rec is %05d level is %d\n",\
				verify_current, cnom_rec[old_cap_level], old_cap_level);
		}
		old_cap_level = cap_level;
	}
	return 0;
}
#endif
#endif

int sprdfgu_read_soc(void)
{
	int cur_cc, cc_delta, capcity_delta, temp;
//#ifndef AUTO_CNOM
	uint32_t cur_ocv;
//#endif
#ifdef AUTO_CNOM
	uint32_t temp_cnom = 0;
	static uint32_t uinitflag;
	 
	if (0 == uinitflag) {
		temp_cnom = sprdfgu_cnominfo_opera(CNOM_OPERA_GET,
									sprdfgu_data.pdata->cnom);
		if (temp_cnom != 0) {
			sprdfgu_data.pdata->cnom = temp_cnom;
			uinitflag = 1;
		}
	}
#endif

	mutex_lock(&sprdfgu_data.lock);

	cur_cc = sprdfgu_clbcnt_get();
	cc_delta = cur_cc - sprdfgu_data.init_clbcnt;
	temp = DIV_ROUND_CLOSEST(cc_delta, (3600 * FGU_CUR_SAMPLE_HZ));
	temp = sprdfgu_adc2cur_ma(temp);
	FGU_DEBUG("sprdfgu_read_soc delta %dmAh,sprdfgu_data.init_clbcnt:%d\n",
		  temp, sprdfgu_data.init_clbcnt);
	capcity_delta = DIV_ROUND_CLOSEST(temp * 100, sprdfgu_data.pdata->cnom);
	FGU_DEBUG("sprdfgu_read_soc delta capacity %d,full capacity %d\n",
		  capcity_delta, sprdfgu_data.pdata->cnom);

	capcity_delta += sprdfgu_data.init_cap;

	FGU_DEBUG("sprdfgu_read_soc soc %d,sprdfgu_data.init_cap %d\n",
		  capcity_delta, sprdfgu_data.init_cap);
	
	cur_ocv = sprdfgu_read_vbat_vol();
	FGU_DEBUG("sprdfgu_read_soc cur_ocv %d and rsense_real %d\n", cur_ocv, sprdfgu_data.pdata->rsense_real);
	if(sprdfgu_cur_current_get() < 0){ 
		if (cur_ocv <= 3450 && capcity_delta > 1) {
			sprdfgu_soc_adjust(1);
			capcity_delta = 1;
			}
		if (cur_ocv <= 3390) {
			FGU_DEBUG("sprdfgu_read_soc vol 0...\n");
			capcity_delta = 0;
			sprdfgu_soc_adjust(capcity_delta);
			}
		}
#ifndef AUTO_CNOM
	cur_ocv = sprdfgu_read_vbat_ocv();
	if (cur_ocv >= sprdfgu_data.bat_full_vol) {
		FGU_DEBUG("sprdfgu_read_soc cur_ocv %d\n", cur_ocv);
		if (capcity_delta < 100 || capcity_delta > 102) {
			capcity_delta = 100;
			sprdfgu_soc_adjust(100);
		}
	}

	if (capcity_delta > 100) {
		capcity_delta = 100;
		sprdfgu_soc_adjust(100);
	}
	
	if(sprdfgu_cur_current_get() < 0){ 
	if (capcity_delta <= sprdfgu_data.warning_cap
	    && cur_ocv > sprdfgu_data.pdata->alm_vol) {
		FGU_DEBUG("sprdfgu_read_soc soc low...\n");
		capcity_delta = sprdfgu_data.warning_cap + 1;
		sprdfgu_soc_adjust(capcity_delta);
	} else if (capcity_delta <= 0 && cur_ocv > sprdfgu_data.shutdown_vol) {
		FGU_DEBUG("sprdfgu_read_soc soc 0...\n");
		capcity_delta = 1;
		sprdfgu_soc_adjust(capcity_delta);
	} else if (cur_ocv < sprdfgu_data.shutdown_vol) {
		FGU_DEBUG("sprdfgu_read_soc vol 0...\n");
		capcity_delta = 0;
		sprdfgu_soc_adjust(capcity_delta);
	} else if (capcity_delta > sprdfgu_data.warning_cap
		   && cur_ocv < sprdfgu_data.pdata->alm_vol) {
		FGU_DEBUG("sprdfgu_read_soc high...\n");
		sprdfgu_soc_adjust(sprdfgu_vol2capacity(cur_ocv));
		capcity_delta = sprdfgu_vol2capacity(cur_ocv);
	}
}
#endif
#ifdef SPRDFGU_TEMP_COMP_SOC
            {
                int temp;

                temp = sprdbat_read_temp();
				FGU_DEBUG("OHH, sprdfgu_read_soc cnom adjust by temp, old cap=%d, temp=%d\n", capcity_delta, temp);
                capcity_delta =
                    sprdfgu_temp_comp_soc(capcity_delta, temp / 10);
				FGU_DEBUG("OHH, sprdfgu_read_soc cnom adjust by temp, new cap=%d\n", capcity_delta);
            }
#endif

#ifdef SPRDFGU_TEMP_COMP_SOC
        if (sprdfgu_read_vbat_vol() < sprdfgu_data.pdata->soft_vbat_uvlo) {
            FGU_DEBUG("TEMP_COMP_SOC vol 0...\n");
            capcity_delta = 0;
            sprdfgu_soc_adjust(capcity_delta);
        }
#endif

	mutex_unlock(&sprdfgu_data.lock);
	return capcity_delta;
}

 
int sprdfgu_avg_current_query(void)
{
	int cur_cc, cc_delta, raw_avg, temp, curr_avg, capcity;
	u32 cur_time = sci_syst_read();
	u32 time_delta;

	cur_cc = sprdfgu_clbcnt_get();
	time_delta = cur_time - start_time;
	cc_delta = cur_cc - poweron_clbcnt;
	temp = time_delta / 500;
	raw_avg = cc_delta / temp;

	//FGU_DEBUG("start_time:%d,cur_time : %d,poweron_clbcnt: 0x%x,cur_cc:0x%x\n",
	//        start_time, cur_time, poweron_clbcnt, cur_cc);
	FGU_DEBUG("time_delta : %d,cc_delta: %d,raw_avg = %d\n", time_delta,
		  cc_delta, raw_avg);
	curr_avg = sprdfgu_adc2cur_ma(raw_avg);

	temp = time_delta / 3600;
	capcity = temp * curr_avg;
	FGU_DEBUG("capcity/1000 capcity = :  %dmah\n", capcity);
	return curr_avg;
}

static void sprdfgu_debug_works(struct work_struct *work)
{
	FGU_DEBUG("dump fgu msg s\n");
#if defined(CONFIG_ADIE_SC2731)
	return 0;
#endif
	if (!sprdfgu_data.pdata->fgu_mode) {
		FGU_DEBUG("avg current = %d\n", sprdfgu_avg_current_query());
	}
	FGU_DEBUG("vol:%d,softocv:%d,hardocv:%d,current%d,cal_type:%d\n",
		  sprdfgu_read_vbat_vol(), sprdfgu_read_vbat_ocv(),
		  sprdfgu_ocv_vol_get(), sprdfgu_cur_current_get(),
		  fgu_cal.cal_type);
	//FGU_DEBUG("pocv_raw = 0x%x,pocv_voltage = %d\n",
	//        sci_adi_read(REG_FGU_POCV_VAL),
	//        sprdfgu_adc2vol_mv(sci_adi_read(REG_FGU_POCV_VAL)));
	//FGU_DEBUG("REG_FGU_CURT_OFFSET--- = %d\n",
	//sci_adi_read(REG_FGU_CURT_OFFSET));
	//FGU_DEBUG("REG_FGU_RELAX_CURT_THRE--- = %d\n",
	//        sci_adi_read(REG_FGU_RELAX_CURT_THRE));
	//FGU_DEBUG("REG_FGU_RELAX_CNT--- = %d\n",
	//        sci_adi_read(REG_FGU_RELAX_CNT));
	//FGU_DEBUG("REG_FGU_OCV_LAST_CNT--- = %d\n",
	//        sci_adi_read(REG_FGU_OCV_LAST_CNT));
	//FGU_DEBUG("REG_FGU_LOW_OVER--- = %d\n", sci_adi_read(REG_FGU_LOW_OVER));
	//FGU_DEBUG("REG_FGU_CONFIG--- = 0x%x\n", sci_adi_read(REG_FGU_CONFIG));
	//printk("ANA_REG_GLB_MP_MISC_CTRL 0x%x ,0x%x \n", sci_adi_read(ANA_REG_GLB_MP_MISC_CTRL), sci_adi_read(ANA_REG_GLB_DCDC_CTRL2));
	if (!sprdfgu_data.pdata->fgu_mode) {
		FGU_DEBUG("soc():%d\n", sprdfgu_read_soc());
	}
#if defined(CONFIG_ADIE_SC2731)
		{
			int i = 0;
			int cur_reg;
			u32 vol_reg;
			for(i = 0; i < 8; i++)
			{
				vol_reg = sprdfgu_reg_get(REG_FGU_VOLT_VALUE_BUF0 + i*4);
				cur_reg = sprdfgu_reg_get(REG_FGU_CURT_VALUE_BUF0 + i*4);
				FGU_DEBUG("voltage buf_id=%d:reg=0x%x:vol=%d\n",
					i,vol_reg,sprdfgu_adc2vol_mv(vol_reg));
				FGU_DEBUG("current buf_id=%d:reg=0x%x:cur=%d\n",
					i,cur_reg,sprdfgu_adc2cur_ma(cur_reg - CUR_0ma_IDEA_ADC));
			}
		}
#endif

	FGU_DEBUG("dump fgu msg e\n");
	schedule_delayed_work(&sprdfgu_debug_work, sprdfgu_debug_log_time * HZ);
}

static void sprdfgu_cal_battery_impedance(void)
{
	int delta_vol_raw;
	int delta_current_raw;
	int temp;
	int impedance;

	delta_vol_raw = sprdfgu_reg_get(REG_FGU_VOLT_VAL);
	delta_current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);
#if 0				 
	cmd_vol_raw = sprdfgu_reg_get(REG_FGU_POCV_VAL);
	cmd_cur_raw = sprdfgu_reg_get(REG_FGU_CLBCNT_QMAXL) << 1;
#endif
	printk("sprdfgu: fgu delta_vol_raw: 0x%x delta_current_raw 0x%x!\n",
	       delta_vol_raw, delta_current_raw);
	printk("sprdfgu: fgu cmd_vol_raw: 0x%x cmd_cur_raw 0x%x!\n",
	       cmd_vol_raw, cmd_cur_raw);
	if (0 == cmd_vol_raw || 0 == cmd_cur_raw) {
		printk(KERN_ERR
		       "sprdfgu: sprdfgu_cal_battery_impedance warning.....!\n");
		return;
	}
	delta_vol_raw -= cmd_vol_raw;
	delta_current_raw -= cmd_cur_raw;
	delta_vol_raw = ((delta_vol_raw) * 1000) / fgu_cal.vol_1000mv_adc;
	delta_current_raw = sprdfgu_adc2cur_ma(delta_current_raw);
	printk("sprdfgu: delta vol delta_vol_raw: %d delta_current_raw %d!\n",
	       (delta_vol_raw), (delta_current_raw));
	temp = (delta_vol_raw * 1000) / delta_current_raw;

	impedance = abs(temp);
	if (impedance > 100) {
		sprdfgu_data.poweron_rint = impedance;
	} else {
		printk("sprdfgu: impedance warning: %d!\n", impedance);
	}
	printk("sprdfgu: fgu sprdfgu_data.poweron_rint: %d!\n",
	       sprdfgu_data.poweron_rint);
}

uint32_t sprdfgu_read_capacity(void)
{
	int32_t voltage;
	int cap;

	if (sprdfgu_data.pdata->fgu_mode) {
		voltage = sprdfgu_read_vbat_ocv();
		cap = sprdfgu_vol2capacity(voltage);
	} else {
		cap = sprdfgu_read_soc();
	}
	if (cap > 100)
		cap = 100;
	else if (cap < 0)
		cap = 0;
	return cap;
}

uint32_t sprdfgu_poweron_capacity(void)
{
	return sprdfgu_data.init_cap;
}

void sprdfgu_adp_status_set(int plugin)
{
	sprdfgu_data.adp_status = plugin;
	if (plugin) {
		uint32_t adc;
		adc = sprdfgu_vol2adc_mv(sprdfgu_data.pdata->alm_vol);
		sci_adi_set(REG_FGU_INT_CLR, BIT_VOLT_LOW_INT);
		sci_adi_write(REG_FGU_LOW_OVER, adc & 0xFFFF, ~0);
		sci_adi_clr(REG_FGU_INT_EN, BIT_CLBCNT_DELTA_INT);
	}
}

static void sprdfgu_irq_works(struct work_struct *work)
{
	uint32_t cur_ocv;
	uint32_t adc;
	int cur_soc;

	wake_lock_timeout(&(sprdfgu_data.low_power_lock), 2 * HZ);

	printk("sprdfgu: sprdfgu_irq_works......0x%x.cur vol = %d\n",
	       sprdfgu_data.int_status, sprdfgu_read_vbat_vol());
	if (sprdfgu_data.int_status & BIT_VOLT_HIGH_INT) {
		blocking_notifier_call_chain(&fgu_chain_head, 1, 0);
	}

	if (sprdfgu_data.int_status & BIT_VOLT_LOW_INT) {
		cur_soc = sprdfgu_read_soc();	 
		mutex_lock(&sprdfgu_data.lock);

		cur_ocv = sprdfgu_read_vbat_ocv();
		if (cur_ocv <= sprdfgu_data.shutdown_vol) {
			printk
			    ("sprdfgu: sprdfgu_irq_works...sprdfgu_data.shutdown_vol .\n");
			sprdfgu_soc_adjust(0);
		} else if (cur_ocv <= sprdfgu_data.pdata->alm_vol) {
			printk
			    ("sprdfgu: sprdfgu_irq_works...sprdfgu_data.pdata->alm_vol %d.\n",
			     cur_soc);
			if (cur_soc > sprdfgu_data.warning_cap) {
				sprdfgu_soc_adjust(sprdfgu_data.warning_cap);
			} else if (cur_soc <= 0) {
				sprdfgu_soc_adjust(sprdfgu_vol2capacity
						   (cur_ocv));
			}
			if (!sprdfgu_data.adp_status) {
				adc =
				    sprdfgu_vol2adc_mv
				    (sprdfgu_data.shutdown_vol);
				sci_adi_write(REG_FGU_LOW_OVER, adc & 0xFFFF,
					      ~0);
			}
		} else {
			 
		}
		mutex_unlock(&sprdfgu_data.lock);
	}
}

static irqreturn_t _sprdfgu_interrupt(int irq, void *dev_id)
{
	sprdfgu_data.int_status = sci_adi_read(REG_FGU_INT_STS);
	sci_adi_set(REG_FGU_INT_CLR, sprdfgu_data.int_status);
	printk
	    ("sprdfgu: _sprdfgu_interrupt.....raw..0x%x,sprdfgu_data.int_status0x%x\n",
	     sci_adi_read(REG_FGU_INT_RAW), sprdfgu_data.int_status);
	schedule_delayed_work(&sprdfgu_data.fgu_irq_work, 0);
	udelay(60);		 
	return IRQ_HANDLED;
}

static int sprdfgu_int_init(void)
{
	uint32_t adc;
	int delta_cc = sprdfgu_clbcnt_init(1);
	int ret = -ENODEV;

	INIT_DELAYED_WORK(&sprdfgu_data.fgu_irq_work, sprdfgu_irq_works);

	sci_adi_set(REG_FGU_INT_CLR, 0xFFFF);

	adc = sprdfgu_vol2adc_mv(sprdfgu_data.pdata->chg_bat_safety_vol);
	sci_adi_write(REG_FGU_HIGH_OVER, adc & 0xFFFF, ~0);
	adc = sprdfgu_vol2adc_mv(sprdfgu_data.pdata->alm_vol);
	sci_adi_write(REG_FGU_LOW_OVER, adc & 0xFFFF, ~0);

	sci_adi_write(REG_FGU_CLBCNT_DELTL, delta_cc & 0xFFFF, ~0);
	sci_adi_write(REG_FGU_CLBCNT_DELTH, (delta_cc >> 16) & 0xFFFF, ~0);

	ret = request_irq(sprdfgu_data.pdata->irq_fgu, _sprdfgu_interrupt,
			  IRQF_NO_SUSPEND, "sprdfgu", NULL);

	if (ret) {
		printk(KERN_ERR "sprdfgu: request sprdfgu irq %d failed\n",
		       sprdfgu_data.pdata->irq_fgu);
	}

	sci_adi_set(REG_FGU_INT_EN, BIT_VOLT_HIGH_INT);
	return 0;
}

void sprdfgu_pm_op(int is_suspend)
{
	if (is_suspend) {
		if (!sprdfgu_data.adp_status) {
			sci_adi_set(REG_FGU_INT_EN, BIT_VOLT_LOW_INT);
			if (sprdfgu_read_vbat_ocv() <
			    sprdfgu_data.pdata->alm_vol) {
				sci_adi_set(REG_FGU_INT_CLR,
					    BIT_CLBCNT_DELTA_INT);
				sci_adi_set(REG_FGU_INT_EN,
					    BIT_CLBCNT_DELTA_INT);
			}
		}
	} else {
		sci_adi_clr(REG_FGU_INT_EN,
			    BIT_VOLT_LOW_INT | BIT_CLBCNT_DELTA_INT);
	}
}
int __weak in_calibration(void){return 0;}
static void sprdfgu_hw_init(void)
{
	u32 cur_vol_raw, ocv_raw;
	int current_raw;
	u32 pocv_raw;
	FGU_DEBUG("FGU_Init\n");

#if !defined(CONFIG_ARCH_SCX15) && !defined(CONFIG_ADIE_SC2723S) && !defined(CONFIG_ADIE_SC2723)
	//sci_adi_set(ANA_REG_GLB_MP_MISC_CTRL, (BIT(1)));
	//sci_adi_write(ANA_REG_GLB_DCDC_CTRL2, (4 << 8), (7 << 8));
#endif

	sci_adi_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_FGU_EN);
	sci_adi_set(ANA_REG_GLB_RTC_CLK_EN, BIT_RTC_FGU_EN | BIT_RTC_FGUA_EN);

#if !defined(CONFIG_ARCH_SCX15) && !defined(CONFIG_ADIE_SC2723S) && !defined(CONFIG_ADIE_SC2723) && !defined(CONFIG_ADIE_SC2731)
	sci_adi_write(REG_FGU_CONFIG, BITS_VOLT_DUTY(3), BITS_VOLT_DUTY(3) | BIT_VOLT_H_VALID);	//mingwei
#endif
	sci_adi_write(REG_FGU_RELAX_CURT_THRE, BITS_RELAX_CUR_THRE(sprdfgu_cur2adc_ma(sprdfgu_data.pdata->relax_current)), BITS_RELAX_CUR_THRE(~0));	//mingwei
	udelay(130);
	sprdfgu_cal_battery_impedance();
	pocv_raw = sci_adi_read(REG_FGU_POCV_VAL);
	cur_vol_raw = sprdfgu_reg_get(REG_FGU_VOLT_VAL);
	ocv_raw = sprdfgu_reg_get(REG_FGU_OCV_VAL);
	current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);
#if defined(CONFIG_ADIE_SC2731)
#else
	start_time = sci_syst_read();
#endif
#if defined(CONFIG_ADIE_SC2723S) ||defined(CONFIG_ADIE_SC2723) ||defined(CONFIG_ADIE_SC2731)
	FGU_DEBUG("REG_FGU_USER_AREA_STATUS- = 0x%x\n",
		  sci_adi_read(REG_FGU_USER_AREA_STATUS));
	if ((FIRST_POWERTON == sprdfgu_poweron_type_read())
	    || (sprdfgu_rtc_reg_read() == 0xFFF) || (sprdfgu_rtc_reg_read() == 0)) {
		FGU_DEBUG("FIRST_POWERTON- = 0x%x\n",
			  sprdfgu_poweron_type_read());
#ifdef CONFIG_SPRD_EXT_IC_POWER
		if(gpio_get_value(sprdfgu_data.pdata->gpio_vchg_detect) 
			&& (!sprdfgu_data.pdata->ocv_type)){
			uint32_t soft_ocv = sprdfgu_read_vbat_vol() -
							(sprdfgu_adc2cur_ma
							(current_raw - CUR_0ma_IDEA_ADC +
							fgu_cal.cur_offset) * sprdfgu_data.poweron_rint) / 1000;

			sprdfgu_data.init_cap = sprdfgu_vol2capacity(soft_ocv);
			FGU_DEBUG
			    ("Charger poweron soft_ocv:%d,sprdfgu_data.init_cap:%d\n",
			     soft_ocv, sprdfgu_data.init_cap);
		} else
#endif
		{
			int poci_raw =
			    sprdfgu_reg_get(REG_FGU_CLBCNT_QMAXL) << 1;
			int poci_curr =
			    sprdfgu_adc2cur_ma(poci_raw - CUR_0ma_IDEA_ADC +
					       fgu_cal.cur_offset);
			uint32_t p_ocv =
			    sprdfgu_adc2vol_mv(pocv_raw) -
			    (poci_curr * sprdfgu_data.poweron_rint) / 1000;
			sprdfgu_data.init_cap = sprdfgu_vol2capacity(p_ocv);
			FGU_DEBUG
			    ("poci_raw_mv:%d,poci_current:%d,p_softocv:%d,sprdfgu_data.init_cap:%d\n",
			     sprdfgu_adc2vol_mv(pocv_raw), poci_curr, p_ocv, sprdfgu_data.init_cap);
		}
		sprdfgu_rtc_reg_write(sprdfgu_data.init_cap);
	} else {
		sprdfgu_data.init_cap = sprdfgu_rtc_reg_read();
		FGU_DEBUG("NORMAIL_POWERTON-- sprdfgu_data.init_cap= %d\n",
			  sprdfgu_data.init_cap);
	}
	sprdfgu_poweron_type_write(NORMAIL_POWERTON);

#else
	{
		int cnt = 50000;
		while((sprdfgu_read_vbat_vol() < 3000) && --cnt){
			FGU_DEBUG("voltage not ready = %d\n", sprdfgu_read_vbat_vol());
			udelay(100);
		}
		if(cnt <= 0) {
			printk(KERN_EMERG "sprdfgu: voltage error!!!!");
			BUG_ON(1);
		}
	}
	{
		uint32_t soft_ocv = sprdfgu_read_vbat_vol() -
		    (sprdfgu_adc2cur_ma
		     (current_raw - CUR_0ma_IDEA_ADC +
		      fgu_cal.cur_offset) * sprdfgu_data.cur_rint) / 1000;
		sprdfgu_data.init_cap = sprdfgu_vol2capacity(soft_ocv);
	}
#endif

	sprdfgu_data.init_clbcnt = poweron_clbcnt =
	    sprdfgu_clbcnt_init(sprdfgu_data.init_cap);
	sprdfgu_clbcnt_set(poweron_clbcnt);
#if  !defined(CONFIG_ADIE_SC2723S) && !defined(CONFIG_ADIE_SC2723)
	if (!in_calibration()) {
		sci_adi_write(REG_FGU_CURT_OFFSET, fgu_cal.cur_offset, ~0);
	}
#endif

	FGU_DEBUG("pocv_raw = 0x%x,pocv_voltage = %d\n", pocv_raw,
		  sprdfgu_adc2vol_mv(pocv_raw));
	FGU_DEBUG("current voltage raw_data =  0x%x,cur voltage = %d\n",
		  cur_vol_raw, sprdfgu_adc2vol_mv(cur_vol_raw));
	FGU_DEBUG("ocv_raw: 0x%x,ocv voltage = %d\n", ocv_raw,
		  sprdfgu_adc2vol_mv(ocv_raw));
	FGU_DEBUG("current_raw: 0x%x,current = %d\n", current_raw,
		  sprdfgu_adc2cur_ma(current_raw - CUR_0ma_IDEA_ADC));
	FGU_DEBUG("poweron_clbcnt: 0x%x,cur_cc0x%x\n", poweron_clbcnt,
		  sprdfgu_clbcnt_get());
	FGU_DEBUG("sprdfgu_data.poweron_rint = %d\n", sprdfgu_data.poweron_rint);
#if defined(CONFIG_ADIE_SC2731)
	{
		int i = 0;
		int cur_reg;
		u32 vol_reg;
		for(i = 0; i < 8; i++)
		{
			vol_reg = sprdfgu_reg_get(REG_FGU_VOLT_VALUE_BUF0 + i*4);
			cur_reg = sprdfgu_reg_get(REG_FGU_CURT_VALUE_BUF0 + i*4);
			FGU_DEBUG("voltage buf_id=%d:reg=0x%x:vol=%d\n",
				i,vol_reg,sprdfgu_adc2vol_mv(vol_reg));
			FGU_DEBUG("current buf_id=%d:reg=0x%x:cur=%d\n",
				i,cur_reg,sprdfgu_adc2cur_ma(cur_reg - CUR_0ma_IDEA_ADC));
		}
	}
#endif
	FGU_DEBUG("REG_FGU_STATUS = 0x%x\n", sci_adi_read(REG_FGU_STATUS));
#ifdef CONFIG_SPRD_EXT_IC_POWER
	gpio_free(sprdfgu_data.pdata->gpio_vchg_detect);
#endif

}

int sprdfgu_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&fgu_chain_head, nb);
}

int sprdfgu_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&fgu_chain_head, nb);
}

static ssize_t sprdfgu_store_attribute(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);
static ssize_t sprdfgu_show_attribute(struct device *dev,
				      struct device_attribute *attr, char *buf);

#define SPRDFGU_ATTR(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR | S_IWGRP, },  \
	.show = sprdfgu_show_attribute,                  \
	.store = sprdfgu_store_attribute,                              \
}
#define SPRDFGU_ATTR_RO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO, },  \
	.show = sprdfgu_show_attribute,                  \
}
#define SPRDFGU_ATTR_WO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IWUSR | S_IWGRP, },  \
	.store = sprdfgu_store_attribute,                              \
}

static struct device_attribute sprdfgu_attribute[] = {
	SPRDFGU_ATTR_RO(fgu_vol_adc),
	SPRDFGU_ATTR_RO(fgu_current_adc),
	SPRDFGU_ATTR_RO(fgu_vol),
	SPRDFGU_ATTR_RO(fgu_current),
	SPRDFGU_ATTR_WO(fgu_log_time),
	SPRDFGU_ATTR_RO(fgu_cal_from_type),
};

enum SPRDFGU_ATTRIBUTE {
	FGU_VOL_ADC = 0,
	FGU_CURRENT_ADC,
	FGU_VOL,
	FGU_CURRENT,
	FGU_LOG_TIME,
	FGU_CAL_FROM_TYPE,
};

static ssize_t sprdfgu_store_attribute(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	unsigned long set_value;
	const ptrdiff_t off = attr - sprdfgu_attribute;

	set_value = simple_strtoul(buf, NULL, 10);
	pr_info("sprdfgu_store_attribute %lu %lu\n", off, set_value);

	switch (off) {
	case FGU_LOG_TIME:
		sprdfgu_debug_log_time = set_value;
		break;
	default:
		count = -EINVAL;
		break;
	}
	return count;
}

static ssize_t sprdfgu_show_attribute(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - sprdfgu_attribute;

	switch (off) {
	case FGU_VOL_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sci_adi_read(REG_FGU_VOLT_VAL));
		break;
	case FGU_CURRENT_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sci_adi_read(REG_FGU_CURT_VAL));
		break;
	case FGU_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdfgu_read_vbat_vol());
		break;
	case FGU_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdfgu_read_batcurrent());
		break;
	case FGU_CAL_FROM_TYPE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       fgu_cal.cal_type);
		break;

	default:
		i = -EINVAL;
		break;
	}

	return i;
}

static int sprdfgu_creat_attr(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sprdfgu_attribute); i++) {
		rc = device_create_file(dev, &sprdfgu_attribute[i]);
		if (rc)
			goto sprd_attrs_failed;
	}
	goto sprd_attrs_succeed;

sprd_attrs_failed:
	while (i--)
		device_remove_file(dev, &sprdfgu_attribute[i]);

sprd_attrs_succeed:
	return rc;
}

static int sprdfgu_power_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	return -EINVAL;
}

int sprdfgu_init(struct sprd_battery_platform_data *pdata)
{
	int ret = 0;
	int temp;
	      

	sprdfgu_data.pdata = pdata;
	sprdfgu_data.warning_cap =
	    sprdfgu_vol2capacity(sprdfgu_data.pdata->alm_vol);
	temp = sprdfgu_data.pdata->ocv_tab_size;
	sprdfgu_data.shutdown_vol = sprdfgu_data.pdata->ocv_tab[temp - 1].x;

	sprdfgu_data.bat_full_vol = sprdfgu_data.pdata->ocv_tab[0].x;
	sprdfgu_data.cur_rint = sprdfgu_data.pdata->rint;
	sprdfgu_data.poweron_rint = sprdfgu_data.pdata->rint;

#ifdef CONFIG_SPRD_EXT_IC_POWER
	ret = gpio_request(sprdfgu_data.pdata->gpio_vchg_detect, "chg_vchg_detect");
	if (ret) {
		FGU_DEBUG("Already request vchg gpio:%d\n",sprdfgu_data.pdata->gpio_vchg_detect);
	}
	gpio_direction_input(sprdfgu_data.pdata->gpio_vchg_detect);
#endif

	if (fgu_cal.cal_type == SPRDBAT_FGUADC_CAL_NO) {
		sprdfgu_cal_from_chip();	                         
	}
	sprdfgu_cal_init();
	sprdfgu_hw_init();

	sprdfgu_data.sprdfgu.name = "sprdfgu";
	sprdfgu_data.sprdfgu.get_property = sprdfgu_power_get_property;
	ret = power_supply_register(NULL, &sprdfgu_data.sprdfgu);
	if (ret) {
		pr_err("register power supply error!\n");
		return -EFAULT;
	}
	sprdfgu_creat_attr(sprdfgu_data.sprdfgu.dev);

	INIT_DELAYED_WORK(&sprdfgu_debug_work, sprdfgu_debug_works);
	mutex_init(&sprdfgu_data.lock);
	wake_lock_init(&(sprdfgu_data.low_power_lock), WAKE_LOCK_SUSPEND,
		       "sprdfgu_powerlow_lock");
	sprdfgu_int_init();
	schedule_delayed_work(&sprdfgu_debug_work, sprdfgu_debug_log_time * HZ);
	FGU_DEBUG("sprdfgu_init end\n");
	return ret;
}

int sprdfgu_reset(void)
{
#if defined(CONFIG_ADIE_SC2731)
#else
	start_time = sci_syst_read();
#endif
	sprdfgu_data.init_cap = sprdfgu_vol2capacity(sprdfgu_read_vbat_ocv());
#if defined(CONFIG_ADIE_SC2723S) || defined(CONFIG_ADIE_SC2723) || defined(CONFIG_ADIE_SC2731)
	sprdfgu_rtc_reg_write(sprdfgu_data.init_cap);
#endif
	sprdfgu_data.init_clbcnt = poweron_clbcnt =
	    sprdfgu_clbcnt_init(sprdfgu_data.init_cap);
	sprdfgu_clbcnt_set(poweron_clbcnt);

	return 0;
}

void sprdfgu_record_cap(u32 cap)
{
#if defined(CONFIG_ADIE_SC2723S) || defined(CONFIG_ADIE_SC2723)
	sprdfgu_rtc_reg_write(cap);
#endif
}