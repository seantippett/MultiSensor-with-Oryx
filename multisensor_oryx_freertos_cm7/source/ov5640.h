#ifndef _OV5640_h
#define _OV5640_h

#include <string.h>
#include <math.h>

#include "cr_section_macros.h"

#include "app_shared.h"


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

#define ENABLE_COLOR_PATTERN 0
#define OV5640_CHIP_ID_HIGH             0x300A
#define OV5640_CHIP_ID_LOW              0x300B
#define OV5640_CHIP_REVISION            0x302A
#define OV5640_SATURATION_MIN			0
#define OV5640_SATURATION_MAX			200
#define OV5640_SATURATION_STEP          100
#define OV5640_SATURATION_DEF			100
#define OV5640_SHARPNESS_MIN			0
#define OV5640_SHARPNESS_MAX			200
#define OV5640_SHARPNESS_STEP           100
#define OV5640_SHARPNESS_DEF			100
/* Auto Focus Command */
#define OV5640_CMD_MAIN					0x3022
#define OV5640_CMD_ACK					0x3023
#define OV5640_CMD_FW_STATUS				0x3029
/*  Auto Focus Status */
#define OV5640_AF_NORMALIZED_W				80
#define OV5640_AF_NORMALIZED_H				60
#define OV5640_AF_SUCCESS				0
#define OV5640_AF_PENDING				1
#define OV5640_AF_FAIL					2
#define OV5640_NOT_FOCUSING				0
#define OV5640_FOCUSING					1
#define OV5640_MAX_FOCUS_AREAS				5

#define	ENOMEM		12	/* Out of Memory */
#define	EINVAL		22	/* Invalid argument */
#define ENOSPC		28	/* No space left on device */

extern enum cam_running_mode runmode;
enum  	v4l2_mbus_pixelcode {
  V4L2_MBUS_FMT_FIXED = 0x0001, V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE = 0x1001, V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE = 0x1002, V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE = 0x1003,
  V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE = 0x1004, V4L2_MBUS_FMT_BGR565_2X8_BE = 0x1005, V4L2_MBUS_FMT_BGR565_2X8_LE = 0x1006, V4L2_MBUS_FMT_RGB565_2X8_BE = 0x1007,
  V4L2_MBUS_FMT_RGB565_2X8_LE = 0x1008, V4L2_MBUS_FMT_Y8_1X8 = 0x2001, V4L2_MBUS_FMT_UYVY8_1_5X8 = 0x2002, V4L2_MBUS_FMT_VYUY8_1_5X8 = 0x2003,
  V4L2_MBUS_FMT_YUYV8_1_5X8 = 0x2004, V4L2_MBUS_FMT_YVYU8_1_5X8 = 0x2005, V4L2_MBUS_FMT_UYVY8_2X8 = 0x2006, V4L2_MBUS_FMT_VYUY8_2X8 = 0x2007,
  V4L2_MBUS_FMT_YUYV8_2X8 = 0x2008, V4L2_MBUS_FMT_YVYU8_2X8 = 0x2009, V4L2_MBUS_FMT_Y10_1X10 = 0x200a, V4L2_MBUS_FMT_YUYV10_2X10 = 0x200b,
  V4L2_MBUS_FMT_YVYU10_2X10 = 0x200c, V4L2_MBUS_FMT_Y12_1X12 = 0x2013, V4L2_MBUS_FMT_UYVY8_1X16 = 0x200f, V4L2_MBUS_FMT_VYUY8_1X16 = 0x2010,
  V4L2_MBUS_FMT_YUYV8_1X16 = 0x2011, V4L2_MBUS_FMT_YVYU8_1X16 = 0x2012, V4L2_MBUS_FMT_YUYV10_1X20 = 0x200d, V4L2_MBUS_FMT_YVYU10_1X20 = 0x200e,
  V4L2_MBUS_FMT_SBGGR8_1X8 = 0x3001, V4L2_MBUS_FMT_SGBRG8_1X8 = 0x3013, V4L2_MBUS_FMT_SGRBG8_1X8 = 0x3002, V4L2_MBUS_FMT_SRGGB8_1X8 = 0x3014,
  V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8 = 0x300b, V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8 = 0x300c, V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8 = 0x3009, V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8 = 0x300d,
  V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE = 0x3003, V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE = 0x3004, V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE = 0x3005, V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE = 0x3006,
  V4L2_MBUS_FMT_SBGGR10_1X10 = 0x3007, V4L2_MBUS_FMT_SGBRG10_1X10 = 0x300e, V4L2_MBUS_FMT_SGRBG10_1X10 = 0x300a, V4L2_MBUS_FMT_SRGGB10_1X10 = 0x300f,
  V4L2_MBUS_FMT_SBGGR12_1X12 = 0x3008, V4L2_MBUS_FMT_SGBRG12_1X12 = 0x3010, V4L2_MBUS_FMT_SGRBG12_1X12 = 0x3011, V4L2_MBUS_FMT_SRGGB12_1X12 = 0x3012,
  V4L2_MBUS_FMT_JPEG_1X8 = 0x4001, V4L2_MBUS_FMT_S5C_UYVY_JPEG_1X8 = 0x5001
};

enum v4l2_colorspace {
	/*
	 * Default colorspace, i.e. let the driver figure it out.
	 * Can only be used with video capture.
	 */
	V4L2_COLORSPACE_DEFAULT       = 0,

	/* SMPTE 170M: used for broadcast NTSC/PAL SDTV */
	V4L2_COLORSPACE_SMPTE170M     = 1,

	/* Obsolete pre-1998 SMPTE 240M HDTV standard, superseded by Rec 709 */
	V4L2_COLORSPACE_SMPTE240M     = 2,

	/* Rec.709: used for HDTV */
	V4L2_COLORSPACE_REC709        = 3,

	/*
	 * Deprecated, do not use. No driver will ever return this. This was
	 * based on a misunderstanding of the bt878 datasheet.
	 */
	V4L2_COLORSPACE_BT878         = 4,

	/*
	 * NTSC 1953 colorspace. This only makes sense when dealing with
	 * really, really old NTSC recordings. Superseded by SMPTE 170M.
	 */
	V4L2_COLORSPACE_470_SYSTEM_M  = 5,

	/*
	 * EBU Tech 3213 PAL/SECAM colorspace. This only makes sense when
	 * dealing with really old PAL/SECAM recordings. Superseded by
	 * SMPTE 170M.
	 */
	V4L2_COLORSPACE_470_SYSTEM_BG = 6,

	/*
	 * Effectively shorthand for V4L2_COLORSPACE_SRGB, V4L2_YCBCR_ENC_601
	 * and V4L2_QUANTIZATION_FULL_RANGE. To be used for (Motion-)JPEG.
	 */
	V4L2_COLORSPACE_JPEG          = 7,

	/* For RGB colorspaces such as produces by most webcams. */
	V4L2_COLORSPACE_SRGB          = 8,

	/* AdobeRGB colorspace */
	V4L2_COLORSPACE_ADOBERGB      = 9,

	/* BT.2020 colorspace, used for UHDTV. */
	V4L2_COLORSPACE_BT2020        = 10,

	/* Raw colorspace: for RAW unprocessed images */
	V4L2_COLORSPACE_RAW           = 11,
};


int ov5640_init(void);

void APP_InitCamera(void);
#define OV5640_SCCB_ADDR            0x3CU

#define OV5640_WriteReg(handle, reg, val)                             \
    SCCB_WriteReg(OV5640_SCCB_ADDR, kSCCB_RegAddr16Bit, (reg), (val), \
                  ((ov5640_resource_t *)((handle)->resource))->i2cSendFunc)

#define OV5640_WriteMultiRegs(handle, reg, val, len)                             \
    SCCB_WriteMultiRegs(OV5640_SCCB_ADDR, kSCCB_RegAddr16Bit, (reg), (val), len, \
                        ((ov5640_resource_t *)((handle)->resource))->i2cSendFunc)

#define OV5640_ReadReg(handle, reg, val)                             \
    SCCB_ReadReg(OV5640_SCCB_ADDR, kSCCB_RegAddr16Bit, (reg), (val), \
                 ((ov5640_resource_t *)((handle)->resource))->i2cReceiveFunc)

#define OV5640_ModifyReg(handle, reg, clrMask, val)                               \
    SCCB_ModifyReg(OV5640_SCCB_ADDR, kSCCB_RegAddr16Bit, (reg), (clrMask), (val), \
                   ((ov5640_resource_t *)((handle)->resource))->i2cReceiveFunc,   \
                   ((ov5640_resource_t *)((handle)->resource))->i2cSendFunc)




extern int AE_high, AE_low;
extern int aeTarget;
int ov5640_probe(void);
int ov5640_video_probe(void);
int ov5640_get_sysclk(void);
int ov5640_set_AE_target(int target);
void setAEC_AGC(int enable);
void setNightMode(int enable);
int getOV5640_AVG_READOUT(void);
int ov5640_config_capture(void);
int ov5640_get_shutter(void);
int ov5640_set_shutter(int shutter);
int ov5640_get_gain16(void);
int ov5640_set_gain16(int gain16);
int ov5640_get_banding(void);
int ov5640_get_red_gain16(void);
int ov5640_get_green_gain16(void);
int ov5640_get_blue_gain16(void);
int ov5640_s_stream(int enable);
int ov5640_set_ae_target(int target);
int ov5640_config_preview(void);
void ov5640_set_banding(void);
int ov5640_config_timing(void);

struct ov5640_reg {
	uint16_t reg;
	uint8_t val;
};
struct ov5640_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};
struct ov5640_af_zone_scale {
	uint32_t x_scale;
	uint32_t y_scale;
};
struct ov5640_timing_cfg {
	uint16_t x_addr_start;
	uint16_t y_addr_start;
	uint16_t x_addr_end;
	uint16_t y_addr_end;
	uint16_t h_output_size;
	uint16_t v_output_size;
	uint16_t h_total_size;
	uint16_t v_total_size;
	uint16_t isp_h_offset;
	uint16_t isp_v_offset;
	uint8_t h_odd_ss_inc;
	uint8_t h_even_ss_inc;
	uint8_t v_odd_ss_inc;
	uint8_t v_even_ss_inc;
	uint8_t out_mode_sel;
	uint8_t sclk_dividers;
	uint8_t sys_mipi_clk;
};
static const struct ov5640_datafmt ov5640_fmts[] = {
	/*
	 * Order important: first natively supported,
	 * second supported with a GPIO extender
	 */
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
/*	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG}, */
};
enum ov5640_size {
	OV5640_SIZE_QVGA,	/* 320 x 240 */
	OV5640_SIZE_VGA,	/* 640 x 480 */
	OV5640_SIZE_720P,
	OV5640_SIZE_1280x960,	/* 1280 x 960  (1.2M) */
	OV5640_SIZE_UXGA,	/* 1600 x 1200 (2M)   */
	OV5640_SIZE_QXGA,	/* 2048 x 1536 (3M)   */
	OV5640_SIZE_5MP,
	OV5640_SIZE_LAST,
	OV5640_SIZE_MAX
};
enum cam_running_mode {
	CAM_RUNNING_MODE_NOTREADY,
	CAM_RUNNING_MODE_PREVIEW,
	CAM_RUNNING_MODE_CAPTURE,
	CAM_RUNNING_MODE_CAPTURE_DONE,
	CAM_RUNNING_MODE_RECORDING,
};

enum v4l2_colorfx {
 V4L2_COLORFX_NONE = 0,
 V4L2_COLORFX_BW = 1,
 V4L2_COLORFX_SEPIA = 2,
 V4L2_COLORFX_NEGATIVE = 3,
 V4L2_COLORFX_EMBOSS = 4,
 V4L2_COLORFX_SKETCH = 5,
 V4L2_COLORFX_SKY_BLUE = 6,
 V4L2_COLORFX_GRASS_GREEN = 7,
 V4L2_COLORFX_SKIN_WHITEN = 8,
 V4L2_COLORFX_VIVID = 9,
 V4L2_COLORFX_AQUA = 10,
 V4L2_COLORFX_ART_FREEZE = 11,
 V4L2_COLORFX_SILHOUETTE = 12,
 V4L2_COLORFX_SOLARIZATION = 13,
 V4L2_COLORFX_ANTIQUE = 14,
 V4L2_COLORFX_SET_CBCR = 15,
};
enum v4l2_power_line_frequency {
 V4L2_CID_POWER_LINE_FREQUENCY_DISABLED = 0,
 V4L2_CID_POWER_LINE_FREQUENCY_50HZ = 1,
 V4L2_CID_POWER_LINE_FREQUENCY_60HZ = 2,
 V4L2_CID_POWER_LINE_FREQUENCY_AUTO = 3,
};

enum v4l2_auto_n_preset_white_balance {
 V4L2_WHITE_BALANCE_MANUAL = 0,
 V4L2_WHITE_BALANCE_AUTO = 1,
 V4L2_WHITE_BALANCE_INCANDESCENT = 2,
 V4L2_WHITE_BALANCE_FLUORESCENT = 3,
 V4L2_WHITE_BALANCE_FLUORESCENT_H = 4,
 V4L2_WHITE_BALANCE_HORIZON = 5,
 V4L2_WHITE_BALANCE_DAYLIGHT = 6,
 V4L2_WHITE_BALANCE_FLASH = 7,
 V4L2_WHITE_BALANCE_CLOUDY = 8,
 V4L2_WHITE_BALANCE_SHADE = 9,
};

enum v4l2_auto_focus_range {
 V4L2_AUTO_FOCUS_RANGE_AUTO = 0,
 V4L2_AUTO_FOCUS_RANGE_NORMAL = 1,
 V4L2_AUTO_FOCUS_RANGE_MACRO = 2,
 V4L2_AUTO_FOCUS_RANGE_INFINITY = 3,
};

enum v4l2_flash_led_mode {
 V4L2_FLASH_LED_MODE_NONE,
 V4L2_FLASH_LED_MODE_FLASH,
 V4L2_FLASH_LED_MODE_TORCH,
};

struct v4l2_frmsize_discrete {
  uint32_t           width;      /* Frame width [pixel] */
  uint32_t           height;     /* Frame height [pixel] */
};

static const struct v4l2_frmsize_discrete ov5640_frmsizes[OV5640_SIZE_LAST] = {
	{320, 240},
	{640, 480},
	{1280, 720},
	{1280, 960},
	{1600, 1200},
	{2048, 1536},
	{2560, 1920},
};
/* Scalers to map image resolutions into AF 80x60 virtual viewfinder */
static const struct ov5640_af_zone_scale af_zone_scale[OV5640_SIZE_LAST] = {
	{4, 4},
	{8, 8},
	{16, 12},
	{16, 16},
	{20, 15},
	{26, 26},
	{32, 32},
};

extern struct ov5640 cameraOV5640;

static const struct ov5640_reg configscript_common1[] = {
	/* System Control */
	//{0x3103, 0x11},
	//{0x3008, 0x82},               /* Reset [7] PowerDn [6] */
	//{0xFFFF, 3},          /* Sleep 3ms */
	//{0x3008, 0x42},
	{0x3103, 0x03},		/* PLL Clock Select */
	/* IO Config */
	{0x3017, 0x00},		/* IO [3:0] D9-D6 (MIPI MD1-D9:D8 MC-D7:D6) */
	{0x3018, 0x00},		/* IO [7:2] D5-D0 (MIPI MD0-D5:D4)
				   [1]GPIO1 [0]GPIO0 (MIPI MD2/MC/MD1) */
	/* MIPI Control */
	{0x4800, 0x04},
	{0x3034, 0x18},
	{0x300e, 0x45},		/* MIPI Control  Dual Lane */
	/* CLKS = Src=13Mhz:  676Mbps 8-bit */
	{0x3037, 0x12},		/* PLL Pre-Div [0:3], /2=6.5Mhz
				   PLL Root Div [4] /1=676Mhz */
	{0x3036, 0x68},		/* PLL Mult 4~252 0:7  0x68=104=676Mhz */
	/* PLL ADCLK */
	{0x303d, 0x20},		/*PreDivSp [5:4] /2=6.5Mhz */
	{0x303b, 0x1E},		/*DivCntsb [4:0] *30=195Mhz */
	/*???? */
	{0x302e, 0x08},		/*undocumented */
	/*Format control */
	{0x501f, 0x00},		/*ISP Format */
	/*JPG Control */
	{0x4713, 0x02},		/*JPG Mode Select */
	/*JPG Quality? */
	{0x4407, 0x04},
	{0x440e, 0x00},
	/*VFIFO Control */
	{0x460b, 0x35},		/*???? */
	{0x460c, 0x22},		/*PCLK Divider Manual */
	/*???? */
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	/*???? */
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	/*VCM Control */
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x3604, 0x02},
	{0x3605, 0x8a},
	/*???? */
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	/*AEC Controls */
	{0x3a13, 0x43},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	/*???? */
	{0x3635, 0x1c},
	{0x3634, 0x40},
	{0x3622, 0x01},
	/*50/60Hz Detector */
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x08},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3814, 0x11},		/*X incr */
	{0x3815, 0x11},		/*Y incr */
	/* Sensor Timing control  2624 x 1952 --> 2560 x 1920 */
	{0x3800, 0x00},		/*X start */
	{0x3801, 0x20},
	{0x3802, 0x00},		/*Y start */
	{0x3803, 0x10},
	{0x3804, 0x0a},		/*X end */
	{0x3805, 0x1f},
	{0x3806, 0x07},		/*Y end */
	{0x3807, 0x8f},
	/* Output size */
	{0x3808, 0x0a},		/*output X  2592 */
	{0x3809, 0x20},
	{0x380a, 0x07},		/*output Y  1944 */
	{0x380b, 0x98},
	/* Total size (+blanking) */
	{0x380c, 0x08},		/*Total X  2844 */
	{0x380d, 0x98},
	{0x380e, 0x05},		/*Total Y  1968 */
	{0x380f, 0x00},
	/* ISP Windowing size  2560 x 1920 --> 2560 x 1920 */
	{0x3810, 0x00},		/*ISP X offset = 0 */
	{0x3811, 0x00},
	{0x3812, 0x00},		/*ISP Y offset = 0 */
	{0x3813, 0x00},
	/*???? */
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x62},
	{0x3709, 0x52},
	{0x370c, 0x03},
	/*AEC/AGC */
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	/*BLC Control */
	{0x4001, 0x02},
	{0x4004, 0x02},
	/*ISP Control */
	{0x5000, 0xa7},
	{0x5001, 0xa3},		/*isp scale down  Special Effects */
	/*AWB Control */
	{0x5180, 0xff},
	{0x5181, 0x56},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x10},
	{0x5187, 0x14},
	{0x5188, 0x10},
	{0x5189, 0x81},
	{0x518a, 0x5a},
	{0x518b, 0xb6},
	{0x518c, 0xa9},
	{0x518d, 0x4c},
	{0x518e, 0x34},
	{0x518f, 0x60},
	{0x5190, 0x48},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x9b},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x09},
	{0x519d, 0x1e},
	{0x519e, 0x38},
	/*CCM Control */
	{0x5381, 0x26},
	{0x5382, 0x4e},
	{0x5383, 0x0c},
	{0x5384, 0x12},
	{0x5385, 0x70},
	{0x5386, 0x82},
	{0x5387, 0x6e},
	{0x5388, 0x58},
	{0x5389, 0x14},
	{0x538b, 0x98},
	{0x538a, 0x01},
	/*CIP Control */
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	/*Gamma Control */
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	/*SDE Control */
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	/* LSC OV5640LENCsetting */
	{0x5800, 0x32},
	{0x5801, 0x1d},
	{0x5802, 0x19},
	{0x5803, 0x18},
	{0x5804, 0x1d},
	{0x5805, 0x38},
	{0x5806, 0x12},
	{0x5807, 0x0a},
	{0x5808, 0x07},
	{0x5809, 0x07},
	{0x580a, 0x0b},
	{0x580b, 0x0f},
	{0x580c, 0x0e},
	{0x580d, 0x05},
	{0x580e, 0x01},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x0a},
	{0x5812, 0x0c},
	{0x5813, 0x04},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x03},
	{0x5817, 0x0a},
	{0x5818, 0x12},
	{0x5819, 0x09},
	{0x581a, 0x06},
	{0x581b, 0x05},
	{0x581c, 0x09},
	{0x581d, 0x12},
	{0x581e, 0x32},
	{0x581f, 0x18},
	{0x5820, 0x14},
	{0x5821, 0x13},
	{0x5822, 0x17},
	{0x5823, 0x2d},
	{0x5824, 0x28},
	{0x5825, 0x2a},
	{0x5826, 0x28},
	{0x5827, 0x28},
	{0x5828, 0x2a},
	{0x5829, 0x28},
	{0x582a, 0x25},
	{0x582b, 0x24},
	{0x582c, 0x24},
	{0x582d, 0x08},
	{0x582e, 0x26},
	{0x582f, 0x42},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x06},
	{0x5833, 0x26},
	{0x5834, 0x26},
	{0x5835, 0x44},
	{0x5836, 0x24},
	{0x5837, 0x2a},
	{0x5838, 0x4a},
	{0x5839, 0x2a},
	{0x583a, 0x0c},
	{0x583b, 0x2c},
	{0x583c, 0x28},
	{0x583d, 0xce},
	{0x5025, 0x00},
	/*AEC Controls */
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x26},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},
#if ENABLE_COLOR_PATTERN
	{0x503d, 0x80},		/* Solid Colour Bars */
#if 0
	{0x503d, 0x80},		/* Solid Colour Bars */
	{0x503d, 0x81},		/* Gradual change @ vertical mode 1 */
	{0x503d, 0x82},		/* Gradual change horizontal */
	{0x503d, 0x83},		/* Gradual change @ vertical mode 2 */
#endif
#endif
	{0xFFFF, 0x00},
};
static const struct ov5640_reg yuv422_init_common[] = {
	/* System Clock Div */
	{0x3035, 0x11},		/*SystemClkDiv 7:4, /1=728Mhz
				   MIPI Sclk Div 3:0, /1=728Mhz */
	/*System/IO pad Control */
	{0x3000, 0x00},		/*Resets */
	{0x3002, 0x1c},
	{0x3004, 0xff},		/*Clocks */
	{0x3006, 0xc3},
	/*Format control */
	{0x4300, 0x32},		/*Output Format[7:4] Sequence[3:0] (UVYV) */
	/*MIPI Control */
	{0x4837, 0x0b},
	/*PCLK Divider */
	{0x3824, 0x01},		/*Scale Divider [4:0] */
	{0x3008, 0x42},		/*stop sensor streaming */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg jpeg_init_common[] = {
	/* System Clock Div */
	{0x3035, 0x12},		/*SystemClkDiv 7:4, /1=676Mhz
				   MIPI Sclk Div 3:0, /1=676Mhz */
	/*System/IO pad Control */
	{0x3000, 0x00},		/*Resets */
	{0x3002, 0x00},
	{0x3004, 0xff},		/*Clocks */
	{0x3006, 0xff},
	/*Format control */
	{0x4300, 0x32},		/*Output Format[7:4] Sequence[3:0] (UVYV) */
	/*MIPI Control */
	{0x4837, 0x16},
	/*PCLK Divider */
	{0x3824, 0x04},		/*Scale Divider [4:0] */
	{0x3008, 0x42},		/*stop sensor streaming */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg hawaii_common_init[] = {
	{0x3103, 0x11},		// SCCB system control
	{0x3008, 0x82},		// software reset
	{0xFFFF, 5},		// delay 5ms
	{0x3008, 0x42},		// software power down
	{0x3103, 0x03},		// SCCB system control
	{0x3017, 0x00},		// set Frex}, Vsync}, Href}, PCLK, D[9:6] input
	{0x3018, 0x00},		// set d[5:0], GPIO[1:0] input
	{0x4800, 0x04},
	{0x3034, 0x18},		// MIPI 8-bit mode
	{0x3037, 0x12},		// PLL
	{0x3036, 0x68},
	{0x303d, 0x20},
	{0x303b, 0x1e},
	{0x3108, 0x01},		// system divider
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	//system
	{0x3731, 0x12},
	{0x3600, 0x08},		// VCM debug mode
	{0x3601, 0x33},		// VCM debug mode
	{0x302d, 0x60},		// system control
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},		// AGC pre-gain40 = 1x
	{0x3a18, 0x01},		//0x00 gain ceiling  //set to 0x1 to fix The back camera preview in dark enviroment, can't see anything
	{0x3a19, 0x58},		//78 // gain ceiling--f8,20120620
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	// 50Hz/60Hz
	{0x3c00, 0x04},		// 50/60Hz---04 ---set to 50HZ
	{0x3c01, 0xb4},		// 50/60Hz---34 --- manual banding filter
	{0x3c04, 0x28},		// threshold for low sum
	{0x3c05, 0x98},		// threshold for high sum
	{0x3c06, 0x00},		// light meter 1 threshold high
	{0x3c08, 0x00},		// light meter 2 threshold high
	{0x3c09, 0x1c},		// light meter 2 threshold low
	{0x3c0a, 0x9c},		// sample number high
	{0x3c0b, 0x40},		// sample number low
	// timing
	{0x3800, 0x00},
	{0x3801, 0x20},
	{0x3802, 0x00},
	{0x3803, 0x10},
	{0x3804, 0x0a},
	{0x3805, 0x1f},
	{0x3806, 0x07},
	{0x3807, 0x8f},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3810, 0x00},
	{0x3811, 0x00},
	{0x3812, 0x00},
	{0x3813, 0x00},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3708, 0x64},
	{0x3a08, 0x01},		// B50
	{0x4001, 0x02},		// BLC start line
	{0x4005, 0x1a},		// BLC always update
	{0x3000, 0x00},		// system reset 0
	{0x3002, 0x1c},		// system reset 2
	{0x3004, 0xff},		// clock enable 00
	{0x3006, 0xc3},		// clock enable 2
	{0x300e, 0x45},		// MIPI control}, 2 lane}, MIPI enable
	{0x302e, 0x08},
	{0x4300, 0x32},		// YUV 422}, UYVY --------- UYVY swap ZHUOHENGFENG
	{0x501f, 0x00},		// ISP YUV 422
	{0x4407, 0x04},		// JPEG QS
	{0x440e, 0x00},
	{0x5000, 0xa7},		// ISP control}, Lenc on}, gamma on}, BPC on}, WPC on}, CIP on
	// AWB
	// Kimi@20120721 for Turnkey Truly sensor
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x16},
	{0x5187, 0x16},
	{0x5188, 0x16},
	{0x5189, 0x72},
	{0x518a, 0x68},
	{0x518b, 0xf0},
	{0x518c, 0xb2},
	{0x518d, 0x50},
	{0x518e, 0x30},
	{0x518f, 0x30},
	{0x5190, 0x50},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	// color matrix
	// Kimi@20120721 for Turnkey Truly sensor
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x12},
	{0x5384, 0x06},
	{0x5385, 0x7a},
	{0x5386, 0x80},
	{0x5387, 0x84},
	{0x5388, 0x69},
	{0x5389, 0x1b},
	{0x538b, 0x98},
	{0x538a, 0x01},
	// CIP
	{0x5308, 0x35},
	{0x5300, 0x08},		// sharpen-MT th1
	{0x5301, 0x30},		// sharpen-MT th2
	{0x5302, 0x10},		// sharpen-MT off1
	{0x5303, 0x00},		// sharpen-MT off2
	{0x5304, 0x08},		// De-noise th1
	{0x5305, 0x30},		// De-noise th2
	{0x5306, 0x08},		// De-noise off1
	{0x5307, 0x16},		// De-noise off2
	{0x5309, 0x08},		// sharpen-TH th1
	{0x530a, 0x30},		// sharpen-TH th2
	{0x530b, 0x04},		// sharpen-TH off1
	{0x530c, 0x06},		// sharpen-TH off2
	{0x5025, 0x00},
	{0x4740, 0x21},		//kimi suggestied should after init setting & before preview
	// gamma
	// Kimi@20120721 for Turnkey Truly sensor
	{0x5490, 0x15},
	{0x5481, 0x06},
	{0x5482, 0x12},
	{0x5483, 0x1e},
	{0x5484, 0x4a},
	{0x5485, 0x58},
	{0x5486, 0x65},
	{0x5487, 0x72},
	{0x5488, 0x7d},
	{0x5489, 0x88},
	{0x548a, 0x92},
	{0x548b, 0xa3},
	{0x548c, 0xb2},
	{0x548d, 0xc8},
	{0x548e, 0xdd},
	{0x548f, 0xf0},
	{0x4009, 0x08},
	// UV adjust
	{0x5580, 0x02},
	{0x5588, 0x01},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	//max gain
	{0x3019, 0x78},
	// lens correction
	// Kimi@20120721 for Turnkey Truly sensor
	{0x5800, 0x32},		//G
	{0x5801, 0x1d},
	{0x5802, 0x19},
	{0x5803, 0x18},
	{0x5804, 0x1d},
	{0x5805, 0x38},
	{0x5806, 0x12},
	{0x5807, 0x0a},
	{0x5808, 0x07},
	{0x5809, 0x07},
	{0x580a, 0x0b},
	{0x580b, 0x0f},
	{0x580c, 0x0e},
	{0x580d, 0x05},
	{0x580e, 0x01},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x0a},
	{0x5812, 0x0c},
	{0x5813, 0x04},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x03},
	{0x5817, 0x0a},
	{0x5818, 0x12},
	{0x5819, 0x09},
	{0x581a, 0x06},
	{0x581b, 0x05},
	{0x581c, 0x09},
	{0x581d, 0x12},
	{0x581e, 0x32},
	{0x581f, 0x18},
	{0x5820, 0x14},
	{0x5821, 0x13},
	{0x5822, 0x17},
	{0x5823, 0x2d},
	{0x5824, 0x28},		//BR
	{0x5825, 0x2a},
	{0x5826, 0x28},
	{0x5827, 0x28},
	{0x5828, 0x2a},
	{0x5829, 0x28},
	{0x582a, 0x25},
	{0x582b, 0x24},
	{0x582c, 0x24},
	{0x582d, 0x08},
	{0x582e, 0x26},
	{0x582f, 0x42},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x06},
	{0x5833, 0x26},
	{0x5834, 0x26},
	{0x5835, 0x44},
	{0x5836, 0x24},
	{0x5837, 0x2a},
	{0x5838, 0x4a},
	{0x5839, 0x2a},
	{0x583a, 0x0c},
	{0x583b, 0x2c},
	{0x583c, 0x28},
	{0x583d, 0xce},
	//AE window C60
	// Kimi@20120721 for Turnkey Truly sensor
	{0x5688, 0x41},
	{0x5689, 0x14},
	{0x568a, 0xE4},
	{0x568b, 0x4E},
	{0x568c, 0xE4},
	{0x568d, 0x4E},
	{0x568e, 0x41},
	{0x568f, 0x14},
	//AE Target
	{0x3a0f, 0x30},		// stable in high
	{0x3a10, 0x28},		// stable in low
	{0x3a1b, 0x30},		// stable out high
	{0x3a1e, 0x28},		// stable out low
	{0x3a11, 0x60},		// fast zone high
	{0x3a1f, 0x14},		// fast zone low
	//zhf add @20120805
	{0x3501, 0x20},		/* fix enter preview black screen issue. */
	{0x350b, 0x70},
	{0x3008, 0x42},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg hawaii_preview_init[] = {
	//SXGA prview
	{0x3a00, 0x7c},
	{0x3035, 0x11},
#ifdef CONFIG_MACH_HAWAII_GARNET_C_A18
	{0x3036, 0x46},     /* G5 wants 20fps */
#else
	{0x3036, 0x68},		//30fps
#endif
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x4004, 0x02},		// BLC line number
	{0x4713, 0x03},		// JPEG mode 3
	{0x460b, 0x35},		// debug
	{0x460c, 0x22},		// VFIFO, PCLK manual
	{0x4837, 0x0a},
	{0x3824, 0x01},
	{0x5001, 0xa3},		// SDE on, scale on, UV average off, CMX on, AWB on
	{0x3503, 0x00},		// AGC on, AEC on
	{0x3008, 0x02},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg hawaii_capture_init[] = {
	{0x3a00, 0x78},
	{0x3035, 0x12},
	{0x3036, 0x68},		//Kimi set to 20 fps @20120930
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x4004, 0x06},		// BLC line number
	{0x4713, 0x00},		// JPEG mode
	{0x460b, 0x35},
	{0x460c, 0x22},		// VFIFO PCLK manual
	{0x4837, 0x0a},
	{0x3824, 0x01},
	{0x5001, 0x83},		// SDE on scale off UV average off CMX on AWB on
	{0x3503, 0x03},		// AGC off AEC off
	{0x3008, 0x02},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_stream[] = {
	/* System Control */
	{0x300e, 0x45},		/* enable 2 lane MIPI */
	{0x3008, 0x02},		/* enable streaming */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_power_down[] = {
	/* System Control */
	{0x3008, 0x42},		/* disable streaming */
	{0x300e, 0x58},		/* power down MIPI */
	{0xFFFF, 1},		/* Sleep 1ms */
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/*EFFECT */
static const struct ov5640_reg ov5640_effect_normal_tbl[] = {
	{0x5580, 0x02},		/*00 58 */
	{0x5583, 0x40},
	{0x5584, 0x40},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_effect_bw_tbl[] = {
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x18},		/*18 58 */
	{0x5583, 0x80},
	{0x5584, 0x80},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_effect_sepia_tbl[] = {
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x1a},		/*18 58 */
	{0x5583, 0x40},
	{0x5584, 0xa0},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_effect_negative_tbl[] = {
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x42},		/*40 58 */
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/*AntiBanding */
static const struct ov5640_reg ov5640_antibanding_auto_tbl[] = {
	/*@@ Auto-XCLK24MHz */
	{0x3622, 0x01},		/*PD-sel */
	{0x3635, 0x1c},		/*VMREF 3635[2:0] */
	{0x3634, 0x40},		/*I_5060 3643[2:0] */
	{0x3c01, 0x34},
	{0x3c00, 0x00},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x08},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x300c, 0x22},		/*50/60div 300c[2:0] */
	{0x3c0a, 0x9c},		/* 4e */
	{0x3c0b, 0x40},		/* 1f */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_antibanding_50z_tbl[] = {
	/*Band 50Hz */
	{0x3c01, 0xb4},		/*80 */
	{0x3c00, 0x04},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_antibanding_60z_tbl[] = {
	/*Band 60Hz */
	{0x3c01, 0xb4},		/*80 */
	{0x3c00, 0x00},
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*Lens_shading*/
static const struct ov5640_reg ov5640_lens_shading_on_tbl[] = {
	/*@@ Lenc On(C) */
	{0x5000, 0xa7},		/*80 80 */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_lens_shading_off_tbl[] = {
	/*@@ Lenc Off */
	{0x5000, 0x27},		/*00 80 */
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/*Contrast */
static const struct ov5640_reg ov5640_contrast_default_lv3_tbl[] = {
	/*@@ Contrast default */
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x06},		/*04 04
				   ;Enable BIT2 for contrast/brightness control */
	{0x5586, 0x20},		/*Gain */
	{0x5585, 0x00},		/*Offset */
	{0x5588, 0x01},		/*00 04 ;Offset sign */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_contrast_lv0_tbl[] = {
	/*@@ Contrast +3 */
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x06},		/*04 04
				   ;Enable BIT2 for contrast/brightness control */
	{0x5586, 0x2c},		/*Gain */
	{0x5585, 0x1c},		/*Offset */
	{0x5588, 0x01},		/*00 04 ;Offset sign */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_contrast_lv1_tbl[] = {
	/*@@ Contrast +2 */
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x06},		/*04 04
				   ;Enable BIT2 for contrast/brightness control */
	{0x5586, 0x28},		/*Gain */
	{0x5585, 0x18},		/*Offset */
	{0x5588, 0x01},		/*00 04 ;Offset sign */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_contrast_lv2_tbl[] = {
	/*@@ Contrast +1 */
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x06},		/*04 04
				   ;Enable BIT2 for contrast/brightness control */
	{0x5586, 0x24},		/*Gain */
	{0x5585, 0x10},		/*Offset */
	{0x5588, 0x01},		/*00 04 ;Offset sign */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_contrast_lv4_tbl[] = {
	/*@@ Contrast -1  */
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x06},		/*04 04
				   ;Enable BIT2 for contrast/brightness control */
	{0x5586, 0x1c},		/*Gain */
	{0x5585, 0x1c},		/*Offset */
	{0x5588, 0x01},		/*00 04 ;Offset sign */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_contrast_lv5_tbl[] = {
	/*@@ Contrast -2  */
	{0x5001, 0xab},		/*80 80 */
	{0x5580, 0x06},		/*04 04
				   ;Enable BIT2 for contrast/brightness control */
	{0x5586, 0x18},		/*Gain */
	{0x5585, 0x18},		/*Offset */
	{0x5588, 0x01},		/*00 04 ;Offset sign */
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*sharpness */
static const struct ov5640_reg ov5640_sharpness_default_lv2_tbl[] = {
	/*@@ @@ Sharpness 2 */
	{0x5308, 0x65},		/*40 40 */
	{0x5502, 0x04},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_sharpness_lv0_tbl[] = {
	/*@@ @@ Sharpness OFF */
	{0x5308, 0x65},		/*40 40 */
	{0x5502, 0x00},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_sharpness_lv1_tbl[] = {
	/* @@ @@ Sharpness 1 */
	{0x5308, 0x65},		/* 40 40 */
	{0x5502, 0x02},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_sharpness_lv3_tbl[] = {
	/* @@ @@ Sharpness 3 */
	{0x5308, 0x65},		/* 40 40 */
	{0x5502, 0x08},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_sharpness_lv4_tbl[] = {
	/* @@ @@ Sharpness OFF */
	{0x5308, 0x65},		/* 40 40 */
	{0x5502, 0x10},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_sharpness_lv5_tbl[] = {
	/* @@ @@ Sharpness OFF */
	{0x5308, 0x65},		/* 40 40 */
	{0x5502, 0x00},
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* saturation */
static const struct ov5640_reg ov5640_saturation_default_lv3_tbl[] = {
	/* @@ x1 (Default) */
	{0x5001, 0xab},		/* 80 80  SDE_En */
	{0x5583, 0x40},
	{0x5584, 0x40},
	{0x5580, 0x00},		/* 00 02 */
	{0x5588, 0x41},		/* 40 40 */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_saturation_lv0_tbl[] = {
	/* @@ x0.25 */
	{0x5001, 0xab},		/* 80 80  SDE_En */
	{0x5583, 0x20},
	{0x5584, 0x20},
	{0x5580, 0x02},		/* 00 02 */
	{0x5588, 0x41},		/* 40 40 */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_saturation_lv1_tbl[] = {
	/* @@ x0.5 */
	{0x5001, 0xab},		/* 80 80  SDE_En */
	{0x5583, 0x30},
	{0x5584, 0x30},
	{0x5580, 0x02},		/* 00 02 */
	{0x5588, 0x41},		/* 40 40 */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_saturation_lv2_tbl[] = {
	/* @@ x0.75 */
	{0x5001, 0xab},		/* 80 80  SDE_En */
	{0x5583, 0x40},
	{0x5584, 0x40},
	{0x5580, 0x02},		/* 00 02 */
	{0x5588, 0x41},		/* 40 40 */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_saturation_lv4_tbl[] = {
	/* @@ x1.25 */
	{0x5001, 0xab},		/* 80 80  SDE_En */
	{0x5583, 0x50},
	{0x5584, 0x50},
	{0x5580, 0x02},		/* 00 02 */
	{0x5588, 0x41},		/* 40 40 */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_saturation_lv5_tbl[] = {
	/* @@ x1.5 */
	{0x5001, 0xab},		/* 80 80  SDE_En */
	{0x5583, 0x60},
	{0x5584, 0x60},
	{0x5580, 0x02},		/* 00 02 */
	{0x5588, 0x41},		/* 40 40 */
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* brightness */
static const struct ov5640_reg ov5640_brightness_lv0_tbl[] = {
	/* @@ +1.7EV */
	{0x3a0f, 0x60},
	{0x3a10, 0x58},
	{0x3a11, 0xa0},
	{0x3a1b, 0x60},
	{0x3a1e, 0x58},
	{0x3a1f, 0x20},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_brightness_lv1_tbl[] = {
	/* @@ +1.0EV */
	{0x3a0f, 0x50},
	{0x3a10, 0x48},
	{0x3a11, 0x90},
	{0x3a1b, 0x50},
	{0x3a1e, 0x48},
	{0x3a1f, 0x20},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_brightness_lv2_default_tbl[] = {
	/* @@ default */
	{0x3a0f, 0x38},
	{0x3a10, 0x30},
	{0x3a11, 0x61},
	{0x3a1b, 0x38},
	{0x3a1e, 0x30},
	{0x3a1f, 0x10},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_brightness_lv3_tbl[] = {
	/* @@ -1.0EV */
	{0x3a0f, 0x20},
	{0x3a10, 0x18},
	{0x3a11, 0x41},
	{0x3a1b, 0x20},
	{0x3a1e, 0x18},
	{0x3a1f, 0x10},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_brightness_lv4_tbl[] = {
	/* @@ -1.7EV */
	{0x3a0f, 0x10},
	{0x3a10, 0x08},
	{0x3a11, 0x10},
	{0x3a1b, 0x10},
	{0x3a1e, 0x08},
	{0x3a1f, 0x10},
	{0xFFFF, 0x00}
};
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Auto Expourse */
static const struct ov5640_reg ov5640_ae_average_tbl[] = {
	/* @@ Whole Image Average */
	{0x5688, 0x11},		/* Zone 1/Zone 0 weight */
	{0x5689, 0x11},		/* Zone 3/Zone 2 weight */
	{0x569a, 0x11},		/* Zone 5/Zone 4 weight */
	{0x569b, 0x11},		/* Zone 7/Zone 6 weight */
	{0x569c, 0x11},		/* Zone 9/Zone 8 weight */
	{0x569d, 0x11},		/* Zone b/Zone a weight */
	{0x569e, 0x11},		/* Zone d/Zone c weight */
	{0x569f, 0x11},		/* Zone f/Zone e weight */
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_ae_centerweight_tbl[] = {
	/*@@ Whole Image Center More weight */
	{0x5688, 0x62},
	{0x5689, 0x26},
	{0x568a, 0xe6},
	{0x568b, 0x6e},
	{0x568c, 0xea},
	{0x568d, 0xae},
	{0x568e, 0xa6},
	{0x568f, 0x6a},
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*default awb */
static const struct ov5640_reg ov5640_wb_def[] = {
	{0x3212, 0x03},		// start group 3 //zhuohengfeng remove the group setting for "Restore" option
	{0x3406, 0x00},
	{0x3400, 0x04},
	{0x3401, 0x00},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0x00},
	{0x3212, 0x13},		// end group 3
	{0x3212, 0xa3},
	{0xFFFF, 0x00}
};
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* White Balance */
static const struct ov5640_reg ov5640_wb_fluorescent[] = {
	{0x3400, 0x05},
	{0x3401, 0x44},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x07},
	{0x3405, 0x0c},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_wb_daylight[] = {
	{0x3400, 0x06},
	{0x3401, 0x1c},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0xf3},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_wb_cloudy[] = {
	{0x3400, 0x07},
	{0x3401, 0x1c},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0xf3},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_wb_tungsten[] = {
	{0x3400, 0x04},
	{0x3401, 0x28},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x08},
	{0x3405, 0x55},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_5[] = {
	{0x3036, 0x34},		/* PLL Mult 4~252 0:7  0x34=52=328Mhz */
	{0x380c, 0x19},		/*Total X  6600 */
	{0x380d, 0xc8},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_7[] = {
	{0x3036, 0x48},		/* PLL Mult 4~252 0:7  0x48=72=473Mhz */
	{0x380c, 0x19},		/*Total X  6600 */
	{0x380d, 0xc8},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_10[] = {
	{0x3036, 0x68},		/* PLL Mult 4~252 0:7  0x68=104=676Mhz */
	{0x380c, 0x19},		/*Total X  6600 */
	{0x380d, 0xc8},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_15[] = {
	{0x3036, 0x68},		/* PLL Mult 4~252 0:7  0x68=104=676Mhz */
	{0x380c, 0x11},		/*Total X  4400 */
	{0x380d, 0x30},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_20[] = {
	{0x3036, 0x68},		/* PLL Mult 4~252 0:7  0x68=104=676Mhz */
	{0x380c, 0x0c},		/*Total X  3300 */
	{0x380d, 0xe4},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_25[] = {
	{0x3036, 0x68},		/* PLL Mult 4~252 0:7  0x68=104=676Mhz */
	{0x380c, 0x0a},		/*Total X  2640 */
	{0x380d, 0x50},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_fps_30[] = {
	{0x3036, 0x68},		/* PLL Mult 4~252 0:7  0x68=104=676Mhz */
	{0x380c, 0x08},		/*Total X  2200 */
	{0x380d, 0x98},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_afpreinit_tbl[] = {
/* Sensor should be  Streaming */
	{0x3000, 0x20},
	{0xFFFF, 0x00}
};
static const struct ov5640_reg ov5640_afpostinit_tbl[] = {
	{0x3022, 0x00},
	{0x3023, 0x00},
	{0x3024, 0x00},
	{0x3025, 0x00},
	{0x3026, 0x00},
	{0x3027, 0x00},
	{0x3028, 0x00},
	{0x3029, 0x7f},
	{0x3000, 0x00},
	{0xFFFF, 0x00}
};
/*
 *	E N U M S
 */
enum v4l2_field {
	V4L2_FIELD_ANY           = 0, /* driver can choose from none,
					 top, bottom, interlaced
					 depending on whatever it thinks
					 is approximate ... */
	V4L2_FIELD_NONE          = 1, /* this device has no fields ... */
	V4L2_FIELD_TOP           = 2, /* top field only */
	V4L2_FIELD_BOTTOM        = 3, /* bottom field only */
	V4L2_FIELD_INTERLACED    = 4, /* both fields interlaced */
	V4L2_FIELD_SEQ_TB        = 5, /* both fields sequential into one
					 buffer, top-bottom order */
	V4L2_FIELD_SEQ_BT        = 6, /* same as above + bottom-top order */
	V4L2_FIELD_ALTERNATE     = 7, /* both fields alternating into
					 separate buffers */
	V4L2_FIELD_INTERLACED_TB = 8, /* both fields interlaced, top field
					 first and the top field is
					 transmitted first */
	V4L2_FIELD_INTERLACED_BT = 9, /* both fields interlaced, top field
					 first and the bottom field is
					 transmitted first */
};
/**
 * struct v4l2_mbus_framefmt - frame format on the media bus
 * @width:	image width
 * @height:	image height
 * @code:	data format code (from enum v4l2_mbus_pixelcode)
 * @field:	used interlacing type (from enum v4l2_field)
 * @colorspace:	colorspace of the data (from enum v4l2_colorspace)
 * @ycbcr_enc:	YCbCr encoding of the data (from enum v4l2_ycbcr_encoding)
 * @hsv_enc:	HSV encoding of the data (from enum v4l2_hsv_encoding)
 * @quantization: quantization of the data (from enum v4l2_quantization)
 * @xfer_func:  transfer function of the data (from enum v4l2_xfer_func)
 * @flags:	flags (V4L2_MBUS_FRAMEFMT_*)
 * @reserved:  reserved bytes that can be later used
 */
struct v4l2_mbus_framefmt {
	uint32_t			width;
	uint32_t			height;
	uint32_t			code;
	uint32_t			field;
	uint32_t			colorspace;
	union {
		/* enum v4l2_ycbcr_encoding */
		uint16_t			ycbcr_enc;
		/* enum v4l2_hsv_encoding */
		uint16_t			hsv_enc;
	};
	uint16_t			quantization;
	uint16_t			xfer_func;
	uint16_t			flags;
	uint16_t			reserved[10];
};

int ov5640_s_fmt(struct v4l2_mbus_framefmt *mf);
int ov5640_reg_writes(const struct ov5640_reg reglist[]);
#endif
