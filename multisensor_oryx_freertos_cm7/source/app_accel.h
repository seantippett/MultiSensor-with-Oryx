/*
 * app_accel.h
 *
 *  Created on: Jun. 18, 2022
 *      Author: tsnider
 */

#ifndef APP_ACCEL_H_
#define APP_ACCEL_H_

#include "MMA8451.h"
/*******************************************************************************
 * Config Definitions
 ******************************************************************************/


/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern void SwTimerCallback_accel(TimerHandle_t xTimer);
extern TimerHandle_t SwTimerHandle_accelFilter;
void tcp_accel_task(void *pvParameters);
/* The software timer period. */
#define SW_TIMER_PERIOD_accel_MS (100 / portTICK_PERIOD_MS)

//******************************  Event Codes  *********************************
#define INIT_EVENT              0
#define TAP_EVENT               1
#define TIMER_TICK_EVENT        2
#define DATA_READY_EVENT        3

//*************************  Build time config parameters  *********************
#define TICK_PER_SEC                    10              //Used to compute time base of the detect routine - if changing from 10 evaluate data width of timer variables

#define SUPERVISION_ACTIVE_LIMIT        15729           //about 0.06G difference in any axis (16384b / G and 16x filter gain)
#define SUPERVISION_INACTIVE_LIMIT      9473
#define SUPERVISION_DEBOUNCE_TIME       2               //historysis time

#define PLOT_TIMEOUT                    120 * TICK_PER_SEC                      //time the module will continue plotting when a plot is requested
#define AUX_BYPASS_DELAY                3 * TICK_PER_SEC                        //time delay when coming out of aux input bypass mode


extern int32_t XYZ_supervision_vals[3];
extern int32_t XYZ_supervision_filter[3];



extern struct detector accel_sensor_status;

extern uint8_t filter_flush_flag;

extern uint8_t profile_peak_hold;

extern uint32_t plot_timeout, histogram_plot_timeout;

#define PLOT_BUSY	((0 != plot_timeout) || (0 != histogram_plot_timeout))

extern uint8_t env_comp_matrix[4];  //stores noise cancellation data for the local sensor



//************************  Function Prototypes  *******************************
uint8_t GM_detect(uint8_t op_code );
uint8_t discrete_tap_tracker(uint8_t op_code);
uint8_t detector_supervision_check(void);

void plot_mode_init( void );
void histogram_plot_mode_init( uint8_t );

void EDAPT_calculate(uint8_t node_index); //used by coordinator
//***************************  Helper Macros  **********************************
#define AUX_INPUT_BYPASS_CHECK          if(GM_STATUS.alarm_sup & 0x04) { aux_bypass_timer = AUX_BYPASS_DELAY; } \
                                        else if(aux_bypass_timer){ aux_bypass_timer --; }

#define ALARM_BYPASS                    ((Proc_config.detect.SupMode & 0x80) && aux_bypass_timer)
#define SUPERVISION_BYPASS              ((Proc_config.detect.SupMode & 0x40) && aux_bypass_timer)


//*****************************  Log conversions  ******************************

#define LOG_TO_LINEAR(arg)		pow(10,( ((float)arg) *6/160 ) )
uint8_t ssc_log(uint32_t MAG);


//*****************************  Filter Macros  ********************************

#define HPF_MAX_INDEX           7
#define LPF_MAX_INDEX           6
#define OUTPUT_BOXCAR_LEN 	4
#define NUMBER_OF_FILTER_COEF   11

// Supervision filters - smoothing filter used to decimate 800Hz data down to 50Hz when in active mode
#define PRE_FILTER_XYZ(x,y,z)    \
XYZ_smoothing_filter[0] = (XYZ_smoothing_filter[0] - (XYZ_smoothing_filter[0] >>8)) + x; \
XYZ_smoothing_filter[1] = (XYZ_smoothing_filter[1] - (XYZ_smoothing_filter[1] >>8)) + y; \
XYZ_smoothing_filter[2] = (XYZ_smoothing_filter[2] - (XYZ_smoothing_filter[2] >>8)) + z;

#define PRE_FILTER_X_AXIS              (XYZ_smoothing_filter[0] >>8)
#define PRE_FILTER_Y_AXIS              (XYZ_smoothing_filter[1] >>8)
#define PRE_FILTER_Z_AXIS              (XYZ_smoothing_filter[2] >>8)

#define FILTER_XYZ(x,y,z)    \
XYZ_supervision_filter[0] = (XYZ_supervision_filter[0] - (XYZ_supervision_filter[0] >>4)) + x; \
XYZ_supervision_filter[1] = (XYZ_supervision_filter[1] - (XYZ_supervision_filter[1] >>4)) + y; \
XYZ_supervision_filter[2] = (XYZ_supervision_filter[2] - (XYZ_supervision_filter[2] >>4)) + z;

#define FILTER_XYZ_init(x,y,z)    \
XYZ_supervision_filter[0] =  16 * (int32_t)x; XYZ_smoothing_filter[0] = 256 * (int32_t)x; \
XYZ_supervision_filter[1] =  16 * (int32_t)y; XYZ_smoothing_filter[1] = 256 * (int32_t)y; \
XYZ_supervision_filter[2] =  16 * (int32_t)z; XYZ_smoothing_filter[2] = 256 * (int32_t)z;

#define LPF_X_AXIS              (XYZ_supervision_filter[0] >>4)
#define LPF_Y_AXIS              (XYZ_supervision_filter[1] >>4)
#define LPF_Z_AXIS              (XYZ_supervision_filter[2] >>4)

#define X_ORIENTATION           (INT8S)(XYZ_supervision_filter[0] / 655)                //filtered output is x16 so 16384 * 16 / 100 = 2621 which yelds .01G per bit, +/- 1.27G full scale INT8S
#define Y_ORIENTATION           (INT8S)(XYZ_supervision_filter[1] / 655)				 // 2621 at 2G scale, 1310 at 4G, 655 at 8G
#define Z_ORIENTATION           (INT8S)(XYZ_supervision_filter[2] / 655)

// Detection LPF - uses a cascade of bitshift filters
#define DETECT_LP_FILTER_XYZ(x,y,z,a,index)    \
XYZ_detect_LP_filter[0][index] = (XYZ_detect_LP_filter[0][index] - (XYZ_detect_LP_filter[0][index] >>a)) + x; \
XYZ_detect_LP_filter[1][index] = (XYZ_detect_LP_filter[1][index] - (XYZ_detect_LP_filter[1][index] >>a)) + y; \
XYZ_detect_LP_filter[2][index] = (XYZ_detect_LP_filter[2][index] - (XYZ_detect_LP_filter[2][index] >>a)) + z;

#define DETECT_LPF_X_AXIS(a,index)              (XYZ_detect_LP_filter[0][index] >>a)
#define DETECT_LPF_Y_AXIS(a,index)              (XYZ_detect_LP_filter[1][index] >>a)
#define DETECT_LPF_Z_AXIS(a,index)              (XYZ_detect_LP_filter[2][index] >>a)

//Detect HPF
//HPF biquad filter does not apply gain correction as gains are near unity
#define HPF_BIQUAD(index,input)	        HPF_last_sample[index][0] = HPF_last_sample[index][1]; HPF_last_sample[index][1] = HPF_last_sample[index][2]; \
                                        HPF_last_sample[index][2] = input ; \
                                        HPF_vals_last[index][0] = HPF_vals_last[index][1]; HPF_vals_last[index][1] = HPF_vals[index]; \
                                        diff_temp = (HPF_last_sample[index][0] + HPF_last_sample[index][2]) - 2 * HPF_last_sample[index][1]; \
                                        HPF_vals[index] = (float)diff_temp + ( HPF_biquads[0] * HPF_vals_last[index][0]) + ( HPF_biquads[1] * HPF_vals_last[index][1]);

#define HPF_BIQUAD_FLUSH(index,input)       HPF_vals_last[index][0] = 0; HPF_vals_last[index][1] = 0; HPF_last_sample[index][0] = input ; HPF_last_sample[index][1] = input ;


//LPF biquad filter does apply gain correction as gains are all over the place
#define LPF_BIQUAD(index,input)	        LPF_last_sample[index][0] = LPF_last_sample[index][1]; LPF_last_sample[index][1] = LPF_last_sample[index][2]; \
                                        LPF_last_sample[index][2] = input ; \
                                        LPF_vals_last[index][0] = LPF_vals_last[index][1]; LPF_vals_last[index][1] = LPF_vals[index]; \
                                        diff_temp = (LPF_last_sample[index][0] + LPF_last_sample[index][2]) + 2 * LPF_last_sample[index][1]; \
                                        LPF_vals[index] = ((float)diff_temp / LPF_biquads[2]) + ( LPF_biquads[0] * LPF_vals_last[index][0]) + ( LPF_biquads[1] * LPF_vals_last[index][1]);









uint32_t getAccelMagnitude(void);
int32_t	getAccelError(void);
/*******************************************************************************
 * Tasks
 ******************************************************************************/
void accel_polling_task(void *pvParameters);


#endif /* APP_ACCEL_H_ */

