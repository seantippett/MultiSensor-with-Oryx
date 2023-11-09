/*
 * app_video.c
 *
 *  Created on: Jul. 7, 2022
 *      Author: stippett
 */


#include <string.h>
#include <math.h>

#include "cr_section_macros.h"

#include "app_audio.h"


#include "app_shared.h"
//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"

/* lwIP */
#include "lwip/opt.h"
#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/timeouts.h"

/* Video */
#include "camera_support.h"
#include "fsl_video_common.h"
#include "fsl_camera.h"
#include "fsl_camera_device.h"
#include "fsl_ov5640.h"

#include "board.h"
#include "app_video.h"
#include "app_jack.h"

#include "JPEGENC.h"
#include "app_logging.h"
#include "fsl_cache.h"

#include "ov5640.h"

struct ov5640 cameraOV5640;


enum cam_running_mode runmode;


#define OV5640_FLASH_THRESHHOLD		32
#define OV5640_HFLIP_MASK		0x06
#define OV5640_VFLIP_MASK		0x06
#define OV5640_BINNING_MASK		0x01
#define OV5640_TIMING_REG20		0x3820
#define OV5640_TIMING_REG21		0x3821

const struct ov5640_timing_cfg timing_cfg_jpeg[OV5640_SIZE_LAST] = {
	[OV5640_SIZE_QVGA] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 320,
		.v_output_size	= 240,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_VGA] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 640,
		.v_output_size	= 480,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_720P] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 1280,
		.v_output_size	= 720,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_1280x960] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 1280,
		.v_output_size	= 960,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_UXGA] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 1600,
		.v_output_size	= 1200,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_QXGA] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 2048,
		.v_output_size	= 1536,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_5MP] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 2560,
		.v_output_size	= 1920,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x20,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x12,
	},
};

const struct ov5640_timing_cfg timing_cfg_yuv[OV5640_SIZE_LAST] = {
	[OV5640_SIZE_QVGA] = {
		/* Timing control  2624 x 1952 --> 2592 x 1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 320,
		.v_output_size	= 240,
		/* ISP Windowing size 1296 x 972 --> 1280 x 960 */
		.isp_h_offset	= 8,
		.isp_v_offset	= 6,
		/* Total size (+blanking) */
		.h_total_size	= 2200,
		.v_total_size	= 1280,
		/* Sensor Read Binning Enabled */
		.h_odd_ss_inc	= 3,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 3,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x01,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x11,
	},
	[OV5640_SIZE_VGA] = {
		/* Timing control  2624 x 1952 --> 2592 x 1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 640,
		.v_output_size	= 480,
		/* ISP Windowing size  1296 x 972 --> 1280 x 960 */
		.isp_h_offset	= 8,
		.isp_v_offset	= 6,
		/* Total size (+blanking) */
		.h_total_size	= 2200,
		.v_total_size	= 1280,
		/* Sensor Read Binning Enabled */
		.h_odd_ss_inc	= 3,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 3,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x01,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x11,
	},
	[OV5640_SIZE_720P] = {
		/* Timing control  2624 x 1952 --> 2592 x 1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 1280,
		.v_output_size	= 720,
		/* ISP Windowing size 1296 x 972 --> 1280 x 960 */
		.isp_h_offset	= 8,
		.isp_v_offset	= 6,
		/* Total size (+blanking) */
		.h_total_size	= 2200,
		.v_total_size	= 1280,
		/* Sensor Read Binning Enabled */
		.h_odd_ss_inc	= 3,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 3,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x01,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x11,
	},
	[OV5640_SIZE_1280x960] = {
		/* Timing control 2624x1952 --> 2592x1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 1280,
		.v_output_size	= 960,
		/* ISP Windowing size 1296x972 --> 1280x960 */
		.isp_h_offset	= 8,
		.isp_v_offset	= 6,
		/* Total size (+blanking) */
		.h_total_size	= 2200,
		.v_total_size	= 1280,
		/* Sensor Read Binning Enabled */
		.h_odd_ss_inc	= 3,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 3,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x01,
		.sclk_dividers	= 0x01,
		.sys_mipi_clk	= 0x11,
	},
	[OV5640_SIZE_UXGA] = {
		/* Timing control  2624 x 1952 --> 2592 x 1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 1600,
		.v_output_size	= 1200,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Disabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x00,
		.sclk_dividers	= 0x02,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_QXGA] = {
		/* Timing control  2624 x 1952 --> 2592 x 1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 2048,
		.v_output_size	= 1536,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Enabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x00,
		.sclk_dividers	= 0x02,
		.sys_mipi_clk	= 0x12,
	},
	[OV5640_SIZE_5MP] = {
		/* Timing control  2624 x 1952 --> 2592 x 1944 */
		.x_addr_start	= 16,
		.y_addr_start	= 4,
		.x_addr_end	= 2607,
		.y_addr_end	= 1947,
		/* Output image size */
		.h_output_size	= 2560,
		.v_output_size	= 1920,
		/* ISP Windowing size 2592x1944 --> 2560x1920 */
		.isp_h_offset	= 16,
		.isp_v_offset	= 12,
		/* Total size (+blanking) */
		.h_total_size	= 2844,
		.v_total_size	= 1968,
		/* Sensor Read Binning Enabled */
		.h_odd_ss_inc	= 1,
		.h_even_ss_inc	= 1,
		.v_odd_ss_inc	= 1,
		.v_even_ss_inc	= 1,
		.out_mode_sel	= 0x00,
		.sclk_dividers	= 0x02,
		.sys_mipi_clk	= 0x12,
	},
};


/* Find a data format by a pixel code in an array */
int ov5640_find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ov5640_fmts); i++)
		if (ov5640_fmts[i].code == code)
			break;
	/* If not found, select latest */
	if (i >= ARRAY_SIZE(ov5640_fmts))
		i = ARRAY_SIZE(ov5640_fmts) - 1;
	return i;
}
/* Find a frame size in an array */
int ov5640_find_framesize(uint32_t width, uint32_t height)
{
	int i;
	for (i = 0; i < OV5640_SIZE_LAST; i++) {
		if ((ov5640_frmsizes[i].width >= width) &&
		    (ov5640_frmsizes[i].height >= height))
			break;
	}
	/* If not found, select biggest */
	if (i >= OV5640_SIZE_LAST)
		i = OV5640_SIZE_LAST - 1;
	return i;
}
struct ov5640 {
//	struct v4l2_subdev subdev;
//	struct v4l2_subdev_sensor_interface_parms *plat_parms;
//	struct v4l2_ctrl_handler hdl;
	int i_size;
	int i_fmt;
	int brightness;
	int contrast;
	int sharpness;
	int saturation;
	int hflip;
	int vflip;
	int framerate;
	enum v4l2_colorfx			colorlevel;
	enum v4l2_power_line_frequency		antibanding;
	enum v4l2_auto_n_preset_white_balance	whitebalance;
	enum v4l2_auto_focus_range		focus_mode;
	enum v4l2_flash_led_mode		flashmode;
	/*
	 * focus_status = 1 focusing
	 * focus_status = 0 focus cancelled or not focusing
	 */
	uint32_t focus_status;		//atomic_t focus_status;
	/*
	 * touch_focus holds number of valid touch focus areas. 0 = none
	 */
	int touch_focus;
//	v4l2_touch_area touch_area[OV5640_MAX_FOCUS_AREAS];
	short fireflash;
};

int soc_camera_power_off(void){
	CAMERA_RESET_ASSERT;				// Camera RESETB - put in to RESET
	vTaskDelay(10);		// minimum 1mS reset.
	return 1;
}
int soc_camera_power_on(void){
	vTaskDelay(1000);						// The delay here is a shitty hack.  It's to make sure we're done setting up the PHY before we use the same I2C port to talk to the camera.
	CAMERA_RESET_UNASSERT;				// Camera RESETB - put out of RESET
	vTaskDelay(50);						// we need 20mS after we come out of RESET before we access the I2C bus.
	return 1;
}


int AE_high, AE_low;
int aeTarget = 52;
int initNeeded = 1;
int ov5640_reg_writes(const struct ov5640_reg reglist[])
{
	int err = 0, index;
	for (index = 0; ((reglist[index].reg != 0xFFFF) && (err == 0)); index++) {
		err |=	OV5640_WriteReg((&cameraDevice), reglist[index].reg,
				     reglist[index].val);

		/*  Check for Pause condition */
		if ((reglist[index + 1].reg == 0xFFFF)
		    && (reglist[index + 1].val != 0)) {
			vTaskDelay(reglist[index + 1].val);
			index += 1;
		}
	}
	return 0;
}

int getOV5640_AVG_READOUT(void){
	uint8_t retVal;
	// samples register 0x56A1 which in the datasheet is labeled as "AVG READOUT"
	// I've seen other drivers test this to see if a flash is required.
	// It's used in the camera's AEC algorithm.
	OV5640_ReadReg((&cameraDevice), 0x56A1, &retVal);
	return (int)retVal;
}
int ov5640_s_power(int on)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
//	struct ov5640 *ov5640 = to_ov5640(client);
//	int ret = 0;
	if (!on)
		return soc_camera_power_off();
	initNeeded = 1;
	return soc_camera_power_on();
}

int ov5640_set_AE_target(int target)
{
	int fast_high, fast_low;
	AE_low = target * 23 / 25; /* 0.92 */
	AE_high = target * 27 / 25; /* 1.08 */
	fast_high = AE_high << 1;
	if (fast_high > 255)
		fast_high = 255;
	fast_low = AE_low >> 1;

	OV5640_WriteReg((&cameraDevice), 0x3a0f, AE_high);
	OV5640_WriteReg((&cameraDevice), 0x3a10, AE_low);
	OV5640_WriteReg((&cameraDevice), 0x3a1b, AE_high);
	OV5640_WriteReg((&cameraDevice), 0x3a1e, AE_low);
	OV5640_WriteReg((&cameraDevice), 0x3a11, fast_high);
	OV5640_WriteReg((&cameraDevice), 0x3a1f, fast_low);
	return 0;
}

int ov5640_get_shutter(void)
{
	/* read shutter, in number of line period */
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int shutter;
	uint8_t val;
	OV5640_ReadReg((&cameraDevice), 0x3500, &val);  // ov5640_reg_read(client, 0x3500, &val);
	shutter = (val & 0x0f);
	OV5640_ReadReg((&cameraDevice), 0x3501, &val);  // 	ov5640_reg_read(client, 0x3501, &val);
	shutter = (shutter << 8) + val;
	OV5640_ReadReg((&cameraDevice), 0x3502, &val);  // 	ov5640_reg_read(client, 0x3502, &val);
	shutter = (shutter << 4) + (val >> 4);
	return shutter;
}
int ov5640_set_shutter(int shutter)
{
	/* write shutter, in number of line period */
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t val;
	shutter = shutter & 0xFFFF;
	val = (shutter & 0x0F) << 4;
	OV5640_WriteReg((&cameraDevice), 0x3502, val);  //ov5640_reg_write(client, 0x3502, val);
	val = (shutter & 0xFFF) >> 4;
	OV5640_WriteReg((&cameraDevice), 0x3501, val);  //	ov5640_reg_write(client, 0x3501, val);
	val = shutter >> 12;
	OV5640_WriteReg((&cameraDevice), 0x3500, val);  //	ov5640_reg_write(client, 0x3500, val);
	return 0;
}

void setAEC_AGC(int enable){
	if (enable) {
		/* turn on auto AE/AG */
		OV5640_ModifyReg((&cameraDevice), 0x3A00, 0b00000011, 0b00000000);
	} else {
		/* turn off AE/AG */
		OV5640_ModifyReg((&cameraDevice), 0x3A00, 0b00000011, 0b00000011);
	}
}

void setNightMode(int enable){
	if(enable){
		OV5640_ModifyReg((&cameraDevice), 0x3A00, 0b00000100, 0b00000100);
	}else{
		OV5640_ModifyReg((&cameraDevice), 0x3A00, 0b00000100, 0b00000000);
	}
}
int ov5640_get_gain16(void)
{
	/* read gain, 16 = 1x */
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint16_t gain16;
	uint8_t val;
	OV5640_ReadReg((&cameraDevice), 0x350A, &val); //	ov5640_reg_read(client, 0x350A, &val);
	gain16 = val & 0x03;
	OV5640_ReadReg((&cameraDevice), 0x350B, &val); //	ov5640_reg_read(client, 0x350B, &val);
	gain16 = (gain16 << 8) + val;
	return gain16;
}
int ov5640_set_gain16(int gain16)
{
	/* write gain, 16 = 1x */
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t val;
	gain16 = gain16 & 0x3FF;
	val = gain16 & 0xFF;
	OV5640_WriteReg((&cameraDevice), 0x350B, val); //	ov5640_reg_write(client, 0x350b, val);
	val = gain16 >> 8;
	OV5640_WriteReg((&cameraDevice), 0x350A, val); //	ov5640_reg_write(client, 0x350a, val);
	return 0;
}
int ov5640_get_banding(void)
{
	/* get banding filter value */
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t val;
	int banding;
	OV5640_ReadReg((&cameraDevice), 0x3C01, &val); //		ov5640_reg_read(client, 0x3c01, &val);
	if (val & 0x80) {
		/* manual */
		OV5640_ReadReg((&cameraDevice), 0x3C00, &val); //			ov5640_reg_read(client, 0x3c00, &val);
		if (val & 0x04) {
			/* 50Hz */
			banding = 50;
		} else {
			/* 60Hz */
			banding = 60;
		}
	} else {
		/* auto */
		OV5640_ReadReg((&cameraDevice), 0x3C0C, &val); //			ov5640_reg_read(client, 0x3c0c, &val);
		if (val & 0x01) {
			/* 50Hz */
			banding = 50;
		} else {
			/* 60Hz */
			banding = 60;
		}
	}
	return banding;
}
int ov5640_get_red_gain16(void)
{
	/* read gain, 16 = 1x */

	uint16_t gain16;
	uint8_t val;
	OV5640_ReadReg((&cameraDevice), 0x3400, &val); //		ov5640_reg_read(client, 0x3400, &val);
	gain16 = val & 0x0F;
	OV5640_ReadReg((&cameraDevice), 0x3401, &val); //			ov5640_reg_read(client, 0x3401, &val);
	gain16 = (gain16 << 8) + val;
	return gain16;
}
int ov5640_get_green_gain16(void)
{
	/* read gain, 16 = 1x */

	uint16_t gain16;
	uint8_t val;
	OV5640_ReadReg((&cameraDevice), 0x3402, &val); //			ov5640_reg_read(client, 0x3402, &val);
	gain16 = val & 0x0F;
	OV5640_ReadReg((&cameraDevice), 0x3403, &val); //			ov5640_reg_read(client, 0x3403, &val);
	gain16 = (gain16 << 8) + val;
	return gain16;
}
int ov5640_get_blue_gain16(void)
{
	/* read gain, 16 = 1x */

	uint16_t gain16;
	uint8_t val;
	OV5640_ReadReg((&cameraDevice), 0x3404, &val); //			ov5640_reg_read(client, 0x3404, &val);
	gain16 = val & 0x0F;
	OV5640_ReadReg((&cameraDevice), 0x3405, &val); //			ov5640_reg_read(client, 0x3405, &val);
	gain16 = (gain16 << 8) + val;
	return gain16;
}
/* For capture routines */
#define XVCLK 1300
int ae_target = 44;
int ae_low, ae_high;
int preview_sysclk, preview_hts;
void ov5640_set_banding(void)
{
	int preview_vts;
	int band_step60, max_band60, band_step50, max_band50;

	struct ov5640 *ov5640 = &cameraOV5640;;
	const struct ov5640_timing_cfg *timing_cfg;
	int i = ov5640->i_size;
	if (ov5640_fmts[ov5640->i_fmt].code == V4L2_MBUS_FMT_JPEG_1X8)
		timing_cfg = &timing_cfg_jpeg[i];
	else
		timing_cfg = &timing_cfg_yuv[i];
	/* read preview PCLK */
	preview_sysclk = ov5640_get_sysclk();
	/* read preview HTS */
	preview_hts = timing_cfg->h_total_size;
	/* read preview VTS */
	preview_vts = timing_cfg->v_total_size;


	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("preview_hts=0x%x, VTS: 0x%x, preview_sysclk=%ul\n", preview_hts, preview_vts, preview_sysclk);
	xSemaphoreGive( xPrintMutex );

	OV5640_WriteReg((&cameraDevice), 0x3a02, (preview_vts >> 8));
	OV5640_WriteReg((&cameraDevice), 0x3a03, (preview_vts & 0xff));
	OV5640_WriteReg((&cameraDevice), 0x3a14, (preview_vts >> 8));
	OV5640_WriteReg((&cameraDevice), 0x3a15, (preview_vts & 0xff));
	/* calculate banding filter */
	/* 60Hz */
	band_step60 = preview_sysclk * 100 / preview_hts * 100 / 120;
	OV5640_WriteReg((&cameraDevice), 0x3a0a, (band_step60 >> 8));
	OV5640_WriteReg((&cameraDevice), 0x3a0b, (band_step60 & 0xff));
	max_band60 = (int)((preview_vts - 4) / band_step60);
	OV5640_WriteReg((&cameraDevice), 0x3a0d, max_band60);
	/* 50Hz */
	band_step50 = preview_sysclk * 100 / preview_hts;
	OV5640_WriteReg((&cameraDevice), 0x3a08, (band_step50 >> 8));
	OV5640_WriteReg((&cameraDevice), 0x3a09, (band_step50 & 0xff));
	max_band50 = (int)((preview_vts - 4) / band_step50);
	OV5640_WriteReg((&cameraDevice), 0x3a0e, max_band50);

	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("band_step60:0x%x max_band60:0x%x\n", band_step60, max_band60);
		PRINTF("band_step50:0x%x max_band50:0x%x\n", band_step50, max_band50);
	xSemaphoreGive( xPrintMutex );

}
int ov5640_set_ae_target(int target)
{
	/* stable in high */

	int fast_high, fast_low;
	ae_low = target * 23 / 25;	/* 0.92 */
	ae_high = target * 27 / 25;	/* 1.08 */
	fast_high = ae_high << 1;
	if (fast_high > 255)
		fast_high = 255;
	fast_low = ae_low >> 1;
	OV5640_WriteReg((&cameraDevice), 0x3a0f, ae_high);
	OV5640_WriteReg((&cameraDevice), 0x3a10, ae_low);
	OV5640_WriteReg((&cameraDevice), 0x3a1b, ae_high);
	OV5640_WriteReg((&cameraDevice), 0x3a1e, ae_low);
	OV5640_WriteReg((&cameraDevice), 0x3a11, fast_high);
	OV5640_WriteReg((&cameraDevice), 0x3a1f, fast_low);
	return 0;
}
int ov5640_config_preview(void)
{
	int ret;

	OV5640_WriteReg((&cameraDevice), 0x3503, 0x00);
	ret = ov5640_config_timing();
	ov5640_set_banding();
	setNightMode(0);
	ov5640_set_ae_target(ae_target);
	return ret;
}
int ov5640_video_probe(void)
{
//	unsigned long flags;
	int ret = 0;
	uint8_t id_high, id_low, revision = 0;
//	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	ret = ov5640_s_power(1);
	if (ret < 0)
		return ret;
	ret = OV5640_ReadReg((&cameraDevice), OV5640_CHIP_ID_HIGH, &id_high);
	ret += OV5640_ReadReg((&cameraDevice), OV5640_CHIP_ID_LOW, &id_low);
	ret += OV5640_ReadReg((&cameraDevice), 0x302A, &revision);
	if (ret) {
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Failure to detect OV5640 chip\n");
		xSemaphoreGive( xPrintMutex );
		goto ei2c;
	}
	revision &= 0xF;
//	flags = SOCAM_DATAWIDTH_8;
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Detected a OV5640 chip, revision %x\n", revision);
	xSemaphoreGive( xPrintMutex );
	/* init the sensor here */
	ret = ov5640_init();
	if (ret){
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Failed to initialize sensor\n");
		xSemaphoreGive( xPrintMutex );

	}
ei2c:
// T.S. NOOO!!! - 	ov5640_s_power(0);
	return ret;
}

int stream_mode = -1;
int ov5640_s_stream(int enable)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("enable:%d runmode:%d  stream_mode:%d\n", enable, runmode, stream_mode);
	xSemaphoreGive( xPrintMutex );

	if (enable == stream_mode)
		return ret;
	if (enable) {
		int delayMs = 50;
		if (CAM_RUNNING_MODE_PREVIEW == runmode) {
			/* need more delay to get stable
			 * preview output, or burst capture may
			 * calc based on wrong exposure/gain
			 */
			delayMs = 200;
		}
		ov5640_reg_writes(ov5640_stream);	// T.S. added this as a test
		/* Power Up, Start Streaming */
		/* ret = ov5640_reg_writes(client, ov5640_stream); */
		/* use MIPI on/off as OVT suggested on AppNote */
		OV5640_WriteReg((&cameraDevice), 0x4202, 0x00);
#ifdef TEST_PATTERN
		OV5640_WriteReg((&cameraDevice), 0x503D, 0x80);
#endif
		//msleep(delayMs);
		vTaskDelay(delayMs);
	} else {
		/* Stop Streaming, Power Down */
		/* ret = ov5640_reg_writes(client, ov5640_power_down); */
		OV5640_WriteReg((&cameraDevice), 0x4202, 0x0f);
	}
	stream_mode = enable;
	return ret;
}




int ov5640_probe(void)
{
//	unsigned int i;
	struct ov5640 *ov5640;
	ov5640 = &cameraOV5640;
//	struct soc_camera_subdev_desc *ssd = client->dev.platform_data;
//	struct soc_camera_desc *sdesc = container_of(ssd, struct soc_camera_desc, subdev_desc);
//	struct soc_camera_device *icd;
	int ret;
//	if (!ssd) {
//		dev_err(&client->dev, "OV5640 driver needs platform data\n");
//		return -EINVAL;
//	}
//	if (!ssd->drv_priv) {
//		dev_err(&client->dev,
//			"OV5640 driver needs i/f platform data\n");
//		return -EINVAL;
//	}
//	ov5640 = kzalloc(sizeof(struct ov5640), GFP_KERNEL);
//	if (!ov5640)
//		return -ENOMEM;
//	v4l2_i2c_subdev_init(&ov5640->subdev, client, &ov5640_subdev_ops);
	ov5640->i_size	= OV5640_SIZE_1280x960;
	ov5640->i_fmt	= 0;	/* First format in the list */
//	ov5640->plat_parms = ssd->drv_priv;
//	v4l2_i2c_subdev_init(&ov5640->subdev, client, &ov5640_subdev_ops);
//	v4l2_ctrl_handler_init(&ov5640->hdl, ARRAY_SIZE(ov5640_controls) +
//						OV5640_NUM_OF_STD_CTRLS);
//	if (ov5640->hdl.error)
//		dev_dbg(&client->dev, "Error set during init itself! %d\n",
//			ov5640->hdl.error);
	/* register standard controls */
/*
	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_BRIGHTNESS, EV_MINUS_2, EV_PLUS_2, 1, EV_DEFAULT);
	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops, V4L2_CID_CONTRAST,
			CONTRAST_MINUS_1, CONTRAST_PLUS_1, 1, CONTRAST_DEFAULT);
	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops, V4L2_CID_SATURATION,
		OV5640_SATURATION_MIN, OV5640_SATURATION_MAX,
		OV5640_SATURATION_STEP, OV5640_SATURATION_DEF);
	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops, V4L2_CID_SHARPNESS,
		OV5640_SHARPNESS_MIN, OV5640_SHARPNESS_MAX,
		OV5640_SHARPNESS_STEP, OV5640_SHARPNESS_DEF);
	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_HFLIP, 0, 1, 1, 0);
*/
	ov5640->hflip = 0;
	/* Correct flipped mounting on hawaii_garnet_edn010 */
#ifdef CONFIG_MACH_HAWAII_GARNET
	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_VFLIP, 0, 1, 1, 1);
	ov5640->vflip = 1;
#else
//	v4l2_ctrl_new_std(&ov5640->hdl, &ov5640_ctrl_ops,
//		V4L2_CID_VFLIP, 0, 1, 1, 0);
	ov5640->vflip = 0;
#endif
//	if (ov5640->hdl.error) {
//		dev_err(&client->dev,
//			"Standard controls initialization error %d\n",
//			ov5640->hdl.error);
//		ret = ov5640->hdl.error;
//		goto ctrl_hdl_err;
//	}
	/* register standard menu controls */
/*	v4l2_ctrl_new_std_menu(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_COLORFX, V4L2_COLORFX_NEGATIVE, 0,
		V4L2_COLORFX_NONE);
	v4l2_ctrl_new_std_menu(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_POWER_LINE_FREQUENCY,
		V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
		V4L2_CID_POWER_LINE_FREQUENCY_50HZ);
	v4l2_ctrl_new_std_menu(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		V4L2_WHITE_BALANCE_CLOUDY, 0,
		V4L2_WHITE_BALANCE_AUTO);
	v4l2_ctrl_new_std_menu(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_AUTO_FOCUS_RANGE,
		V4L2_AUTO_FOCUS_RANGE_INFINITY, 0,
		V4L2_AUTO_FOCUS_RANGE_AUTO);
	v4l2_ctrl_new_std_menu(&ov5640->hdl, &ov5640_ctrl_ops,
		V4L2_CID_FLASH_LED_MODE,
		V4L2_FLASH_LED_MODE_TORCH, 0,
		V4L2_FLASH_LED_MODE_FLASH);
	if (ov5640->hdl.error) {
		dev_err(&client->dev,
			"Standard menu controls initialization error %d\n",
			ov5640->hdl.error);
		ret = ov5640->hdl.error;
		goto ctrl_hdl_err;
	}*/
	/* register custom controls */
/*	for (i = 0; i < ARRAY_SIZE(ov5640_controls); ++i)
		v4l2_ctrl_new_custom(&ov5640->hdl, &ov5640_controls[i], NULL);
	ov5640->subdev.ctrl_handler = &ov5640->hdl;
	if (ov5640->hdl.error) {
		dev_err(&client->dev,
			"Custom controls initialization error %d\n",
			ov5640->hdl.error);
		ret = ov5640->hdl.error;
		goto ctrl_hdl_err;
	} */
	ret = ov5640_video_probe();
	if (ret) {
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("ov5640_probe: failed to probe the sensor\n");
		xSemaphoreGive( xPrintMutex );
		goto vid_probe_fail;
	}
	return ret;
//ctrl_hdl_err:
vid_probe_fail:
//	v4l2_ctrl_handler_free(&ov5640->hdl);
//	kfree(ov5640);
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("ov5640_probe failed with ret = %d\n", ret);
	xSemaphoreGive( xPrintMutex );
//	pr_err("ov5640_probe failed with ret = %d\n", ret);
	return ret;
}

int ov5640_get_sysclk(void)
{
	/* calculate sysclk */
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = &cameraOV5640;
	uint8_t val;
	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv,
	    sysclk;
	int sclk_rdiv_map[] = { 1, 2, 4, 8 };
	int  i = ov5640->i_size;
	const struct ov5640_timing_cfg *timing_cfg;
	if (ov5640_fmts[ov5640->i_fmt].code == V4L2_MBUS_FMT_JPEG_1X8)
		timing_cfg = &timing_cfg_jpeg[i];
	else
		timing_cfg = &timing_cfg_yuv[i];
	OV5640_ReadReg((&cameraDevice), 0x3034, &val);
	val &= 0x0F;
	Bit_div2x = 1;
	if (val == 8 || val == 10)
		Bit_div2x = val / 2;
	OV5640_ReadReg((&cameraDevice), 0x3035, &val);
	SysDiv = val >> 4;
	if (SysDiv == 0)
		SysDiv = 16;
	OV5640_ReadReg((&cameraDevice), 0x3036, &val);
	if (val == 0)
		val = 0x68;
	Multiplier = val;
	OV5640_ReadReg((&cameraDevice), 0x3037, &val);
	if ((val & 0x0f) == 0)
		val = 0x12;
	PreDiv = val & 0x0f;
	Pll_rdiv = ((val >> 4) & 0x01) + 1;
	val = timing_cfg->sclk_dividers;
	val &= 0x03;
	sclk_rdiv = sclk_rdiv_map[val];
	VCO = XVCLK * Multiplier / PreDiv;
	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;
	return sysclk;
}
int ov5640_set_vts(int VTS)
{
	/* write VTS to registers */
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t val;
	val = VTS & 0xFF;
	OV5640_WriteReg((&cameraDevice), 0x380F, val);
	val = VTS >> 8;
	OV5640_WriteReg((&cameraDevice), 0x380E, val);
	return 0;
}

int ov5640_config_timing(void)
{
	struct ov5640 *ov5640 = &cameraOV5640;
	int ret, i = ov5640->i_size;
	const struct ov5640_timing_cfg *timing_cfg;
	uint8_t val;
//	dev_info(&client->dev, "%s: code[0x%x] i:%d\n", __func__,
//			       ov5640_fmts[ov5640->i_fmt].code, i);
	if (ov5640_fmts[ov5640->i_fmt].code == V4L2_MBUS_FMT_JPEG_1X8)
		timing_cfg = &timing_cfg_jpeg[i];
	else
		timing_cfg = &timing_cfg_yuv[i];
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3800,
			       ((timing_cfg->x_addr_start & 0xFF00) >> 8));
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3801, timing_cfg->x_addr_start & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3802,
			       (timing_cfg->y_addr_start & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3803, timing_cfg->y_addr_start & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3804, (timing_cfg->x_addr_end & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3805, timing_cfg->x_addr_end & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3806, (timing_cfg->y_addr_end & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3807, timing_cfg->y_addr_end & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3808,
			       (timing_cfg->h_output_size & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3809, timing_cfg->h_output_size & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x380A,
			       (timing_cfg->v_output_size & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x380B, timing_cfg->v_output_size & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x380C,
			       (timing_cfg->h_total_size & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x380D, timing_cfg->h_total_size & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x380E,
			       (timing_cfg->v_total_size & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x380F, timing_cfg->v_total_size & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3810,
			       (timing_cfg->isp_h_offset & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3811, timing_cfg->isp_h_offset & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3812,
			       (timing_cfg->isp_v_offset & 0xFF00) >> 8);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3813, timing_cfg->isp_v_offset & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3814,
			       ((timing_cfg->h_odd_ss_inc & 0xF) << 4) |
			       (timing_cfg->h_even_ss_inc & 0xF));
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3815,
			       ((timing_cfg->v_odd_ss_inc & 0xF) << 4) |
			       (timing_cfg->v_even_ss_inc & 0xF));
	if (ret)
		return ret;
	OV5640_ReadReg((&cameraDevice), OV5640_TIMING_REG21, &val);
	if (timing_cfg->out_mode_sel & OV5640_BINNING_MASK)
		val |= OV5640_BINNING_MASK;
	else
		val &= ~(OV5640_BINNING_MASK);
	if (ov5640->hflip)
		val |= OV5640_HFLIP_MASK;
	else
		val &= ~(OV5640_HFLIP_MASK);
	ret = OV5640_WriteReg((&cameraDevice), OV5640_TIMING_REG21, val);
	if (ret)
		return ret;
	OV5640_ReadReg((&cameraDevice), OV5640_TIMING_REG20, &val);
	if (ov5640->vflip)
		val |= OV5640_VFLIP_MASK;
	else
		val &= ~(OV5640_VFLIP_MASK);
	ret = OV5640_WriteReg((&cameraDevice), OV5640_TIMING_REG20, val);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice),
			       0x3108, timing_cfg->sclk_dividers & 0xFF);
	if (ret)
		return ret;
	ret = OV5640_WriteReg((&cameraDevice), 0x3035, timing_cfg->sys_mipi_clk & 0xFF);
	if (ret)
		return ret;
	/* msleep(50); */
	return ret;
}


#if(0)
static int ov5640_flash_control(struct i2c_client *client,
				enum v4l2_flash_led_mode control)
{
	int ret = 0;
	switch (control) {
	case V4L2_FLASH_LED_MODE_FLASH:
		#ifdef CONFIG_VIDEO_ADP1653
		adp1653_gpio_strobe(0);
		adp1653_gpio_toggle(1);
		usleep_range(30, 31);
		adp1653_set_timer(1, 0x5);
		adp1653_set_ind_led(1);
		/* Flash current indicator LED ON */
		adp1653_set_torch_flash(28);
		/* Strobing should hapen later */
		#endif
		#ifdef CONFIG_VIDEO_AS3643
		as3643_gpio_toggle(1);
		usleep_range(25, 30);
		as3643_set_ind_led(0x80, 900000);
		#endif
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		#ifdef CONFIG_VIDEO_ADP1653
		adp1653_gpio_toggle(1);
		adp1653_gpio_strobe(0);
		usleep_range(30, 31);
		adp1653_set_timer(1, 0);
		adp1653_set_ind_led(1);
		/* Torch current no indicator LED */
		adp1653_set_torch_flash(10);
		adp1653_sw_strobe(1);
		#endif
		#ifdef CONFIG_VIDEO_AS3643
		as3643_gpio_toggle(1);
		usleep_range(25, 30);
		as3643_set_torch_flash(0x80);
		#endif
		break;
	case V4L2_FLASH_LED_MODE_NONE:
	default:
		#ifdef CONFIG_VIDEO_ADP1653
		adp1653_clear_all();
		adp1653_gpio_toggle(0);
		#endif
		#ifdef CONFIG_VIDEO_AS3643
		as3643_clear_all();
		as3643_gpio_toggle(0);
		#endif
		break;
	}
	return ret;
}
static int ov5640_pre_flash(struct i2c_client *client)
{
	int ret = 0;
	struct ov5640 *ov5640 = &cameraOV5640;
	ov5640->fireflash = 0;
	if (V4L2_FLASH_LED_MODE_FLASH == ov5640->flashmode) {
		ret = ov5640_flash_control(client, ov5640->flashmode);
		ov5640->fireflash = 1;
	} else if (V4L2_FLASH_LED_MODE_FLASH_AUTO == ov5640->flashmode) {
		uint8_t average = 0;
		OV5640_ReadReg((&cameraDevice), 0x56a1, &average);
		if ((average & 0xFF) < OV5640_FLASH_THRESHHOLD) {
			ret = ov5640_flash_control(client,
						V4L2_FLASH_LED_MODE_FLASH);
			ov5640->fireflash = 1;
		}
	}
	if (1 == ov5640->fireflash)
		msleep(50);
	return ret;
}



#endif

#if(1)


//OV5640_WriteReg((&cameraDevice), 0x3a0f, AE_high);
//OV5640_ReadReg((&cameraDevice), 0x56A1, &retVal);


int ov5640_config_capture(void)
{
	int ret = 0;
	// struct i2c_client *client = v4l2_get_subdevdata(sd);
	int preview_shutter, preview_gain16;
	uint8_t average;
	int capture_shutter, capture_gain16;
//	int red_gain16, green_gain16, blue_gain16;
	int capture_sysclk, capture_hts, capture_vts;
	int banding, capture_bandingfilter, capture_max_band;
	long capture_gain16_shutter;
	struct ov5640 *ov5640 = &cameraOV5640;
	const struct ov5640_timing_cfg *timing_cfg;
	int i;
	/*disable aec/agc */
	OV5640_WriteReg((&cameraDevice), 0x3503, 0x03);  //	ov5640_reg_write(client, 0x3503, 0x03);

			/* read preview shutter */
	preview_shutter = ov5640_get_shutter();
	/* read preview gain */
	preview_gain16 = ov5640_get_gain16();
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("preview_shutter=0x%x, preview_gain16=0x%x\r\n", preview_shutter, preview_gain16);
	xSemaphoreGive( xPrintMutex );

	/*ov5640_reg_read(client, 0x558c, &preview_uv); */
//	red_gain16	= ov5640_get_red_gain16();  - TS I removed these because they were unused.
//	green_gain16 = ov5640_get_green_gain16();
//	blue_gain16	= ov5640_get_blue_gain16();
	/* get average */
	average = getOV5640_AVG_READOUT();//	ov5640_reg_read(client, 0x56a1, &average);

	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("preview avg=0x%x\r\n", average);
	xSemaphoreGive( xPrintMutex );

	/* turn off night mode for capture */
	setNightMode(0);//	ov5640_set_night_mode(sd, 0);
	/* Write capture setting */
	ov5640_reg_writes(hawaii_capture_init);
	ov5640_config_timing();


	/* preview_hts & preview_sysclk are static and set in config_preview */
	/* print preview_hts and preview_sysclk */
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("preview_hts=0x%x, preview_sysclk=%ul\n", preview_hts, preview_sysclk);
	xSemaphoreGive( xPrintMutex );


	/* read capture VTS */
	i = ov5640->i_size;
	if (ov5640_fmts[ov5640->i_fmt].code == V4L2_MBUS_FMT_JPEG_1X8)
		timing_cfg = &timing_cfg_jpeg[i];
	else
		timing_cfg = &timing_cfg_yuv[i];
	capture_vts	= timing_cfg->v_total_size;
	capture_hts	= timing_cfg->h_total_size;
	capture_sysclk	= ov5640_get_sysclk();

	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("capture_vts=0x%x, capture_hts=0x%x, capture_sysclk=%ul\n", capture_vts, capture_hts, capture_sysclk);
	xSemaphoreGive( xPrintMutex );

	/* calculate capture banding filter */
	banding = ov5640_get_banding();
	if (banding == 60) {
		/* 60Hz */
		capture_bandingfilter =
		    capture_sysclk * 100 / capture_hts * 100 / 120;
	} else {
		/* 50Hz */
		capture_bandingfilter = capture_sysclk * 100 / capture_hts;
	}
	capture_max_band = (int)((capture_vts - 4) / capture_bandingfilter);
	/*preview_shutter = preview_shutter * 5 / 4; */
	/* calculate capture shutter/gain16 */
	capture_gain16_shutter =
	    preview_gain16 * preview_shutter * capture_sysclk;
	if (average > ae_low && average < ae_high) {
		/* in stable range */
		capture_gain16_shutter =
		    capture_gain16_shutter / preview_sysclk * preview_hts /
		    capture_hts * ae_target / average;
	} else {
		capture_gain16_shutter =
		    capture_gain16_shutter / preview_sysclk * preview_hts /
		    capture_hts;
	}
	/* gain to shutter */
	if (capture_gain16_shutter < (capture_bandingfilter * 16)) {
		/* shutter < 1/100 */
		capture_shutter = capture_gain16_shutter / 16;
		if (capture_shutter < 1)
			capture_shutter = 1;
		capture_gain16 = capture_gain16_shutter / capture_shutter;
		if (capture_gain16 < 16) {
			capture_gain16 = 16;
		}
	} else {
		if (capture_gain16_shutter >
		    (capture_bandingfilter * capture_max_band * 16)) {
			/* exposure reach max */
			capture_shutter =
			    capture_bandingfilter * capture_max_band;
			capture_gain16 =
			    capture_gain16_shutter / capture_shutter;
		} else {
			/* 1/100 < capture_shutter =< max,
			 * capture_shutter = n/100 */
			capture_shutter =
				(int)(capture_gain16_shutter / 16 /
					capture_bandingfilter) *
					capture_bandingfilter;
			capture_gain16 =
				capture_gain16_shutter / capture_shutter;
		}
	}
	/* write capture gain */
	ov5640_set_gain16(capture_gain16);
	/* write capture shutter */
	/* capture_shutter = capture_shutter * 122 / 100; */
//	dev_info(&client->dev, "%s shutter=0x%x, capture_vts=0x%x\n", __func__,
//				capture_shutter, capture_vts);

	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("shutter=0x%x, capture_vts=0x%x\n", capture_shutter, capture_vts);
	xSemaphoreGive( xPrintMutex );

	if (capture_shutter > (capture_vts - 4)) {
		capture_vts = capture_shutter + 4;
		ov5640_set_vts(capture_vts);
	}
	ov5640_set_shutter(capture_shutter);
	return ret;
}
#endif
int ov5640_g_fmt(struct v4l2_mbus_framefmt *mf)
{

	struct ov5640 *ov5640 = &cameraOV5640;
	mf->width	= ov5640_frmsizes[ov5640->i_size].width;
	mf->height	= ov5640_frmsizes[ov5640->i_size].height;
	mf->code	= ov5640_fmts[ov5640->i_fmt].code;
	mf->colorspace	= ov5640_fmts[ov5640->i_fmt].colorspace;
	mf->field	= V4L2_FIELD_NONE;
	return 0;
}
int ov5640_try_fmt( struct v4l2_mbus_framefmt *mf)
{
	int i_fmt;
	int i_size;
	i_fmt = ov5640_find_datafmt(mf->code);
	mf->code	= ov5640_fmts[i_fmt].code;
	mf->colorspace	= ov5640_fmts[i_fmt].colorspace;
	mf->field	= V4L2_FIELD_NONE;
	i_size = ov5640_find_framesize(mf->width, mf->height);
	mf->width	= ov5640_frmsizes[i_size].width;
	mf->height	= ov5640_frmsizes[i_size].height;
	return 0;
}

int ov5640_s_fmt(struct v4l2_mbus_framefmt *mf)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = &cameraOV5640;
	int ret = 0;
	ret = ov5640_try_fmt(mf);
	if (ret < 0)
		return ret;
	ov5640->i_size	= ov5640_find_framesize(mf->width, mf->height);
	ov5640->i_fmt	= ov5640_find_datafmt(mf->code);
	/* To avoide reentry init sensor, remove from here */
	if (initNeeded > 0) {
		ret = ov5640_reg_writes(hawaii_common_init);
		initNeeded = 0;
	}
	if (ret) {
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Error configuring configscript_common1\n");
		xSemaphoreGive( xPrintMutex );
		return ret;
	}
	switch ((uint32_t) ov5640_fmts[ov5640->i_fmt].code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		ret = ov5640_reg_writes(hawaii_preview_init);
		if (ret)
			return ret;
		ret = OV5640_WriteReg((&cameraDevice), 0x4300, 0x32);
		if (ret)
			return ret;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		ret = ov5640_reg_writes(hawaii_preview_init);
		if (ret)
			return ret;
		ret = OV5640_WriteReg((&cameraDevice), 0x4300, 0x30);
		if (ret)
			return ret;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		ret = ov5640_reg_writes(jpeg_init_common);
		if (ret)
			return ret;
		break;
	default:
		/* This shouldn't happen */
		ret = -EINVAL;
		return ret;
	}
	if (CAM_RUNNING_MODE_PREVIEW == runmode)
		ov5640_config_preview();
	return ret;
}


int ov5640_init(void)
{
	struct ov5640 *ov5640 = &cameraOV5640;
	int ret = 0;
	ret = ov5640_reg_writes(configscript_common1);
	if (ret)
		goto out;
#if defined(CONFIG_RHEA_CLOVER_ICS)
	/*Code turn off flash led */
	if (ov5640_reg_write(client, 0x3000, 0x00))
		goto out;
	if (ov5640_reg_write(client, 0x3004, 0xFF))
		goto out;
	if (ov5640_reg_write(client, 0x3016, 0x02))
		goto out;
	if (ov5640_reg_write(client, 0x3b07, 0x0A))
		goto out;
	if (ov5640_reg_write(client, 0x3b00, 0x03))
		goto out;
#endif
	/* Power Up, Start Streaming for AF Init */
	ret = ov5640_reg_writes(ov5640_stream);
	if (ret)
		goto out;
	/* Delay for sensor streaming */
	//msleep(20);
	vTaskDelay(20);
	/* AF Init */
//	ret = ov5640_af_enable();
//	if (ret)
//		goto out;
	/* Stop Streaming, Power Down */
	ret = ov5640_reg_writes(ov5640_power_down);
	/* default brightness and contrast */
	ov5640->brightness	= 0;// t.s. not used.   EV_DEFAULT;
	ov5640->contrast	= 0;// t.s. not used.  CONTRAST_DEFAULT;
	ov5640->colorlevel	= V4L2_COLORFX_NONE;
	ov5640->antibanding	= V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
	ov5640->whitebalance	= V4L2_WHITE_BALANCE_AUTO;
	ov5640->framerate	= 0;// t.s. not used.  FRAME_RATE_AUTO;
	ov5640->focus_mode	= V4L2_AUTO_FOCUS_RANGE_AUTO;
	ov5640->touch_focus	= 0;
//	atomic_set(&ov5640->focus_status, OV5640_NOT_FOCUSING);
	ov5640->focus_status	=OV5640_NOT_FOCUSING;
	ov5640->flashmode	= V4L2_FLASH_LED_MODE_NONE;
	ov5640->fireflash	= 0;
//	dev_dbg(&client->dev, "Sensor initialized\n");
out:
	return ret;
}













