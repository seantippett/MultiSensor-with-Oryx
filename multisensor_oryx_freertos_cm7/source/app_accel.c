/*
 * app_accel.c
 *
 *  Created on: Jun. 15, 2022
 *      Author: tsnider
 */


#include <string.h>
#include "cr_section_macros.h"

#include "app_audio.h"
#include "fsl_pdm.h"
#include "fsl_pdm_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpt.h"
#include "fsl_anatop_ai.h"

#include "app_shared.h"
//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"

#include "app_accel.h"
#include "MMA8451.h"
#include "app_jack.h"
#include "app_logging.h"
#include "app_alarm.h"
#include "app_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define COMPANDED_DETECTOR

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

uint32_t LM100_companding( uint32_t a_nInput);
/*******************************************************************************
 * Variables
 ******************************************************************************/

static TaskHandle_t xTaskToNotify_accel = NULL;
uint8_t filter_flush_flag, accel_mode;
uint8_t env_comp_matrix[4];  //stores noise cancellation data for the local sensor
struct detector accel_sensor_status;

// detection based globals
//******************************  GLOBALS  *************************************
int32_t HPF_ABS[3], env_comp_vals[3] = {0}, envelope_vals[3] = {0}, accumulator[3];
uint32_t output_boxcar[OUTPUT_BOXCAR_LEN], output;
uint8_t bc_head = OUTPUT_BOXCAR_LEN-1, bc_tail=0, output_scaled =0, output_max;

uint8_t detector_sample_number=0, supervision_sample_number=0, supervision_sample_decimate=0;

 //supervision filters
 int32_t XYZ_smoothing_filter[3] = {0};
 int32_t XYZ_supervision_filter[3] = {0};
 int32_t XYZ_supervision_vals[3] = {0};
 uint8_t supervision_startup_delay =0, supervision_decimate =0;

 //detect filters
 int32_t XYZ_detect_LP_filter[3][4] = {{0}};
 uint8_t detect_LPF_index=0;

 float HPF_biquads[2] = {0};
 float HPF_vals[3]={0}, HPF_vals_last[3][2]={{0}};
 int32_t HPF_last_sample[3][3]={{0}}, diff_temp;

 //float LPF_biquads[3];    //element 2 will hold 1 / gain
 //float LPF_vals[3]={0}, LPF_vals_last[3][2]={0};
 //int32_t LPF_last_sample[3][3]={0};


uint8_t supervision_debounce;
uint8_t supervision_mode = 0x03;


uint8_t plot_buffer[10], plot_sample_index=0, plot_max_events, plot_status_word_latch;
uint8_t histogram_plot_max_hold,plot_max_hold=0, histogram_plot_status_word_latch, histogram_plot_samples, histogram_plot_max_events, histogram_plot_sample_index, histogram_plot_send_index;

uint32_t histogram_plot_accumulate;

uint32_t aux_bypass_timer =0;
#define accel_limit_check
//************************  Filter Coeff. Bank  ********************************
float filter_bank[NUMBER_OF_FILTER_COEF][3] = {

    {   //0.1Hz
      -0.9988898959,         // A1
      1.9988892794,          // A2
      6.488157352e+06       // HPF gain
    } ,

    {  //0.5Hz
      -0.9944617891,     // A1
      1.9944464105,      // A2
      2.601028156e+05
    },

    {  //1Hz
      -0.9889542499,     // A1
      1.9888929059,      // A2
      6.520601554e+04
    },

    {   //2Hz
      -0.9780305085,     // A1
      1.9777864838,      // A2
      1.639178228e+04
    },

    {   //4Hz
      -0.9565436765,     // A1
      1.9555782403,      // A2
      4.143204922e+03
    },

    {  //8Hz
      -0.9149758348,     // A1
      1.9111970674,      // A2
      1.058546241e+03
    },

    { //16Hz
      -0.8371816513,     // A1
      1.8226949252,      // A2
      2.761148367e+02
    },

    {  //32Hz
      -0.7008967812,     // A1
      1.6474599811,      // A2
      7.485478157e+01
    },

    {  //64Hz
      -0.4918122372,
      1.3072850288,
      2.167702007e+01
    },

    {  //120Hz
      -0.2722149379,
      0.7477891783,
      7.627390391e+00
    },

    {  //240Hz
      -0.1958157127,
      -0.3695273774,
      2.555350342e+00
    }
    //next is 400Hz = =HPF disable

};


const uint8_t LPF_shifts[7][5] = {
	{0,0,0,0,0},	//400Hz
	{1,1,0,0,2},	//80Hz
	{1,1,1,0,3},	//42Hz
	{1,1,1,1,4},	//25Hz
	{2,2,2,1,7},	//12Hz
	{3,2,2,2,9},	//7.7Hz
	{3,3,3,3,12}	//3.7Hz
};
//the first 4 elements are number of bitshifts in each stage of the cascaded filter, fifth element is the overall gain in bit shifts (if this gets larger than 16b the filter vars will need to be INT64)

__BSS(BOARD_SDRAM) struct MMA_sample accel_data_fifo[AD_FIFO_SIZE];

SemaphoreHandle_t xAccelMutex;


/*******************************************************************************
 * Code
 ******************************************************************************/

int32_t	accelErrorFlag = 0;
int32_t	accelErrorFlagPrev = 0;

int32_t	getAccelError(void){
	int32_t err;
	err = accelErrorFlag;
	if (accelErrorFlag != 0) {
				return 1;
		} // the above is temp while we aren't caring about specific errors

	return err;
}













/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* The callback function. */
//void SwTimerCallback_accel(TimerHandle_t xTimer);


//TimerHandle_t SwTimerHandle_accelFilter = NULL;
//int timerFlag;
//void SwTimerCallback_accel(TimerHandle_t xTimer)
//{
//	xSemaphoreTake( xAccelMutex, ( TickType_t ) portMAX_DELAY );
//	timerFlag = 1;
//	xSemaphoreGive( xAccelMutex);
//}

volatile int resetAccel = 0;


struct STR_accelConfig accelSettings;

void setAccelConfig(struct STR_accelConfig *newConfig){

	memcpy(&accelSettings, newConfig, sizeof(accelSettings));
	resetAccel = 1;
}
void getAccelConfig(struct STR_accelConfig *config){

	memcpy(config, &accelSettings, sizeof(config));
}
uint32_t accelMagnitude = 0;
uint32_t getAccelMagnitude(void){
	uint32_t rv;
	xSemaphoreTake( xAccelMutex, ( TickType_t ) portMAX_DELAY );
		rv = accelMagnitude;
		accelMagnitude = 0;
	xSemaphoreGive( xAccelMutex);
	return rv;
}
float globalPitch = 0;
float globalRoll = 0;

float getRoll(void){
	return globalRoll;
}
float getPitch(void){
	return globalPitch;
}


void accel_polling_task(void *pvParameters){
	uint16_t count;
	int timerEventPrescaller = 0;
	int orientationPrescaller = 0;
	int16_t samples[96];
	int16_t logging[3][32];
	uint8_t status, i;//, j, k;
	TickType_t wakeTimer = 0;
	float pitch = 0, pitchPrevious = 0;
	float roll = 0, rollPrevious = 0;
	float orientation[6];
	uint32_t tamperDebounce = 20;

	accelSettings.HPFSetting =  configParam.accelConfig.HPFSetting; //Proc_config.detect.HPF_val;
	accelSettings.LPFSetting = configParam.accelConfig.LPFSetting; // Proc_config.detect.LPF_val;
	accelSettings.eventWindow_s = configParam.accelConfig.eventWindow_s; //Proc_config.detect.Event_Window;
	accelSettings.eventCount = configParam.accelConfig.eventCount;	//  Proc_config.detect.Event_Count;
	accelSettings.alarmWindow_s = configParam.accelConfig.alarmWindow_s; //Proc_config.detect.Alarm_Window;
	accelSettings.supervisionTolerance = configParam.accelConfig.supervisionTolerance;//  Proc_config.detect.SupMode;

//	memcpy(&accelSettings, &defaultAccelConfig, sizeof(accelSettings));

	do{
	// INIT
		memset(accel_data_fifo,0,sizeof(accel_data_fifo));


		accel_mode = 2;
		GM_detect(INIT_EVENT);

		do{
			vTaskDelay(20 / portTICK_PERIOD_MS );
			status = MMA8451_init();

			accelErrorFlagPrev = accelErrorFlag;
			accelErrorFlag = status;
			if (accelErrorFlag != accelErrorFlagPrev){
				// change in error state
				// so flag to the jack task that there's a change
				diagnostic_change_flag |= 1;
				if(accelErrorFlag != 0){
					sys_stats.sensorstat.faultAccel_count++;
				}
			}
			// TODO report error after reasonable time
		}while (status != 0);


		wakeTimer = xTaskGetTickCount();

		resetAccel = 0;
		while(resetAccel == 0){
	//		vTaskDelay(20 / portTICK_PERIOD_MS );   //McuWait_WaitOSms(1);
			xTaskDelayUntil( &wakeTimer, 	20 / portTICK_PERIOD_MS );

	//  Sample rate is 1.25ms.
	//  32 sample Fifo.  32 x 1.25 = 40mS to fill FIFO.
	//  T.S. 2022 07 24 -  Changed wait time from 10mS to 20mS to reduce I2C usage.
	//  Note that the timerFlag has a 100mS period.  THis is important to know.


			// SCAN
				status = MMA8451_read_FIFO(samples, &count);

			if((status == 0) && (count > 0))
			{
				xSemaphoreTake( xAccelMutex, ( TickType_t ) portMAX_DELAY );
				for(i = 0; i < count; i++){
					accel_data_fifo[AD_fifo_head].x_axis = samples[i*3 + 0];
					accel_data_fifo[AD_fifo_head].y_axis = samples[i*3 + 1];
					accel_data_fifo[AD_fifo_head].z_axis = samples[i*3 + 2];

					logging[0][i] = samples[i*3 + 0];
					logging[1][i] = samples[i*3 + 1];
					logging[2][i] = samples[i*3 + 2];

					AD_FIFO_PUSH;
				}
				GM_detect(DATA_READY_EVENT);
				xSemaphoreGive( xAccelMutex);

				enqueueAccelElement(logging[0], logging[1], logging[2], count);
			}
			else if(status != 0){
				// Accel.  Failure!
				do{
					vTaskDelay(20 / portTICK_PERIOD_MS );
					status = MMA8451_init();
					accelErrorFlagPrev = accelErrorFlag;
					accelErrorFlag = status;
					if (accelErrorFlag != accelErrorFlagPrev){
						// change in error state
						// so flag to the jack task that there's a change
						diagnostic_change_flag |= 1;
						if(accelErrorFlag != 0){
							sys_stats.sensorstat.faultAccel_count++;
						}
					}
				}while (status != 0);

			}

			xSemaphoreTake( xAccelMutex, ( TickType_t ) portMAX_DELAY );
			if(timerEventPrescaller++ >= 5){
				timerEventPrescaller = 0;
				GM_detect(TIMER_TICK_EVENT);
			}
	//		if(timerFlag == 1){
	//			GM_detect(TIMER_TICK_EVENT);
	//			timerFlag = 0;
	//		}

			if(orientationPrescaller++ > 500){
				orientationPrescaller = 0;
				orientation[0] = ((float)XYZ_supervision_filter[0]) / (float)655; //X_ORIENTATION;
				orientation[1] = ((float)XYZ_supervision_filter[1]) / (float)655; //Y_ORIENTATION;
				orientation[2] = ((float)XYZ_supervision_filter[2]) / (float)655; //Z_ORIENTATION;
				orientation[3] = orientation[0] * orientation[0];
				orientation[4] = orientation[1] * orientation[1];
				orientation[5] = orientation[2] * orientation[2];
				// some odd scenarios //
				float temp;
				temp = fabs((float)sqrt(orientation[3] + orientation[4] + orientation[5]));
				// temp should = 1g (i.e. 100) when stationary //
				if(tamperDebounce > 0){
					tamperDebounce--;
					if(tamperDebounce == 0){
						if((accelSettings.supervisionTolerance) != 0){
							tamper_flag = 1;
							diagnostic_flags |= 1;		// change in diagnostic status.
						}
					}else{
						if(tamper_flag == 1){
							if((accelSettings.supervisionTolerance) != 0){
								diagnostic_flags |= 1;
							}
						}
						tamper_flag = 0;
					}
				}else{
					if((accelSettings.supervisionTolerance) != 0){
						tamper_flag = 1;
					}
				}
				if(( temp > 120 ) || (temp  < 80)){
					// too much motion.

				}else{
					if(fabs(orientation[0]) < 2){
						// x axis is near flat... pitch is straight up
						pitch = 0;
					}else if((orientation[4] + orientation[5]) < 1){
						// protect against divide by zero
						pitch = 45;
					}else{
						temp = orientation[0] / (sqrt( orientation[4] + orientation[5]  ));
						if(fabs(temp) <= 1){
							pitch = atan( temp );
							pitch = pitch * (180 / 3.1415926535);
						}
					}
					if(fabs(orientation[1]) < 2){
						// y axis is near flat... roll is flat
						roll = 0;
					}else if((orientation[3] + orientation[5]) < 1){
						// protect against divide by zero
						roll = 45;
					}else{
						temp = orientation[1] / (sqrt( orientation[3] + orientation[5]  ));
						if(fabs(temp) <= 1){
							roll = atan(temp);
							roll = roll * (180 / 3.1415926535);
						}
					}

					if(accelSettings.supervisionTolerance == 1){
						// 3 deg
						if(((fabs(pitch - configParam.mountingPitch)) <= 3) &&  ((fabs(roll - configParam.mountingRoll)) <= 3)){
							tamperDebounce = 10;
						}
					}else if(accelSettings.supervisionTolerance == 2){
						// 5 deg
						if(((fabs(pitch - configParam.mountingPitch)) <= 5) &&  ((fabs(roll - configParam.mountingRoll)) <= 5)){
							tamperDebounce = 10;
						}
					}else if(accelSettings.supervisionTolerance == 3){
						// 7 deg
						if(((fabs(pitch - configParam.mountingPitch)) <= 7) &&  ((fabs(roll - configParam.mountingRoll)) <= 7)){
							tamperDebounce =10;
						}
					}else if(accelSettings.supervisionTolerance == 4){
						// 10 deg
						if(((fabs(pitch - configParam.mountingPitch)) <= 10) &&  ((fabs(roll - configParam.mountingRoll)) <= 10)){
							tamperDebounce = 10;
						}
					}else{
						tamperDebounce = 10;
					}
					globalRoll = roll;
					globalPitch = pitch;

				}




			}




			xSemaphoreGive( xAccelMutex);



		}
	}while(1);
}




uint8_t GM_detect(uint8_t op_code)
 {
  uint8_t ret_val = 0;
  uint16_t i,j;
//  uint16_t time_diff[2];
  uint16_t samples_processed=0;

  int32_t HPF_ABS_comp[3] = {0};

  switch (op_code){

    case(INIT_EVENT):

      //limit checks
#if(0)
      if(Proc_config.detect.HPF_val > HPF_MAX_INDEX){ Proc_config.detect.HPF_val = HPF_MAX_INDEX; }
      if(Proc_config.detect.LPF_val > LPF_MAX_INDEX){ Proc_config.detect.LPF_val = LPF_MAX_INDEX; }

      if(Proc_config.detect.Event_Threshold == 0){ Proc_config.detect.Event_Threshold = 1; }

      if(Proc_config.detect.Event_Window < 5){Proc_config.detect.Event_Window = 5;}
      if(Proc_config.detect.Event_Window > 100){Proc_config.detect.Event_Window = 100;}

      if(Proc_config.detect.Alarm_Window == 0){Proc_config.detect.Alarm_Window = 1;}
      if(Proc_config.detect.Alarm_Window > 100){Proc_config.detect.Alarm_Window = 100;}

      if(Proc_config.detect.Alarm_Hold_Time ==0) {Proc_config.detect.Alarm_Hold_Time = 1;}
      if(Proc_config.detect.Alarm_Hold_Time > 30) {Proc_config.detect.Alarm_Hold_Time = 30;}

      if(Proc_config.detect.Event_Count ==0) {Proc_config.detect.Event_Count =1;}
      if(Proc_config.detect.Event_Count > 10) {Proc_config.detect.Event_Count =10;}
#endif

//    	uint32_t	HPFSetting;					// 0 = 0.1Hz, 1=0.5Hz, 2=1Hz, 3=2Hz, 4=4Hz, 5=8Hz, 6=16Hz, 7=32Hz.
//    	uint32_t	LPFSetting;					// 0 = 400Hz, 1=80Hz, 2=40Hz, 3=25Hz, 4=12Hz, 5=8Hz, 6=4Hz, 7=Invalid.
//    	float_t		eventWindow_s;				// 0.5 to 10 (in 0.5s increments)
//    	uint32_t	eventCount;					// 1 to 10
//    	float_t		alarmWindow_s;				// 1 to 99 (in 1s increments)
//    	uint32_t	supervisionTolerance;		// 0=1x, 1=2x, 3=4x, 4=8x.


      if(accelSettings.HPFSetting > HPF_MAX_INDEX){ accelSettings.HPFSetting = HPF_MAX_INDEX; }
      if(accelSettings.LPFSetting > LPF_MAX_INDEX){ accelSettings.LPFSetting = LPF_MAX_INDEX; }

	  if(accelSettings.eventWindow_s < 0.5){ accelSettings.eventWindow_s = 0.5; }
	  if(accelSettings.eventWindow_s > 10){ accelSettings.eventWindow_s = 10; }

      if(accelSettings.alarmWindow_s < 1){ accelSettings.alarmWindow_s = 1; }
      if(accelSettings.alarmWindow_s > 99){ accelSettings.alarmWindow_s = 99; }

      if(accelSettings.eventCount < 1){ accelSettings.eventCount =1; }
      if(accelSettings.eventCount > 10){ accelSettings.eventCount = 10; }

// unsigned      if(accelSettings.supervisionTolerance < 0){ accelSettings.supervisionTolerance = 0; }
      if(accelSettings.supervisionTolerance > 4){ accelSettings.supervisionTolerance = 4; }

      //initialize alarm tracker
      ret_val = discrete_tap_tracker(INIT_EVENT);

      //initialize signal processing vars
      filter_flush_flag =1;

//      HPF_biquads[0] = filter_bank[Proc_config.detect.HPF_val][0];  HPF_biquads[1] = filter_bank[Proc_config.detect.HPF_val][1];
      HPF_biquads[0] = filter_bank[accelSettings.HPFSetting][0];
      HPF_biquads[1] = filter_bank[accelSettings.HPFSetting][1];

//      detect_LPF_index = Proc_config.detect.LPF_val;
	  detect_LPF_index = accelSettings.LPFSetting;

      detector_sample_number=0;
      supervision_sample_number=0;
      supervision_sample_decimate=0;

      accumulator[0] = 0;
      accumulator[1] = 0;
      accumulator[2] = 0;

      if(supervision_mode != (0x03 & accelSettings.supervisionTolerance)){  //(0x03 & Proc_config.detect.SupMode)){              //if changing supervision mode clear ref. values
          XYZ_supervision_vals[0] = 0;
          XYZ_supervision_vals[1] = 0;
          XYZ_supervision_vals[2] = 0;
          supervision_mode = (0x03 & accelSettings.supervisionTolerance); //(0x03 & Proc_config.detect.SupMode);
      }

	  env_comp_matrix[0] = 0;   //index 0 will hold an EDAPT desensitize factor
	  env_comp_matrix[1] = 0;	//this will be a short term noise level
	  env_comp_matrix[2] = 0;	//this will be a lpf'ed output signal - use this for common mode exclusion
	  env_comp_matrix[3] = Proc_config.RF.index_num; //put the index here because it is not know to the receivers of shared data - until a better way to communicate this is known


    break;

    case(TIMER_TICK_EVENT):

      if((ret_val = discrete_tap_tracker(TIMER_TICK_EVENT))){   //process the elapsed time in the event tracking state machine, non-zero return means status change that should be sent to the base
		//send status message
		//task.send_status_short =1;
	  }

// Multisenosr... commented out for now....      plot_driver();

	  if(0 != env_comp_matrix[0]){
		env_comp_matrix[0] --;
	  }

    break;

    case(DATA_READY_EVENT):


      if(filter_flush_flag){                                                    //flush detect filters
        filter_flush_flag =0;

        for(int8_t k = HPF_MAX_INDEX; k >= accelSettings.HPFSetting; k--){ //Proc_config.detect.HPF_val; k--){                        //walk down the HPF time constants

          HPF_biquads[0] = filter_bank[k][0];
          HPF_biquads[1] = filter_bank[k][1];

          for( j = 0; j < 32; j++){                                             //run some samples or ratherthe same sample through the filters a bunch of times to flush them

            if(detect_LPF_index != 0){                                           //index greater than 0 indicates we need the LPF, ==0 indicates the LPF is bypassed

               //first stage of a cascaded first order bit shift filter
              DETECT_LP_FILTER_XYZ(
                accel_data_fifo[AD_fifo_tail].x_axis,
                accel_data_fifo[AD_fifo_tail].y_axis,
                accel_data_fifo[AD_fifo_tail].z_axis,
                LPF_shifts[detect_LPF_index][0],0);

              //LPF cascade
              for(i =0; i < 3; i++){

                if(LPF_shifts[detect_LPF_index][i+1] == 0){ break; }              //some filter frequencies do not use the entire cascade, break on a 0 shift

                DETECT_LP_FILTER_XYZ(
                  XYZ_detect_LP_filter[0][i],
                  XYZ_detect_LP_filter[1][i],
                  XYZ_detect_LP_filter[2][i],
                  LPF_shifts[detect_LPF_index][i+1],i+1);

              }


              HPF_BIQUAD(0,DETECT_LPF_X_AXIS(LPF_shifts[detect_LPF_index][4],i)); //process HPFs with LPF cascade result as the input
              HPF_BIQUAD(1,DETECT_LPF_Y_AXIS(LPF_shifts[detect_LPF_index][4],i));
              HPF_BIQUAD(2,DETECT_LPF_Z_AXIS(LPF_shifts[detect_LPF_index][4],i));

            }

            else{ //a 0 LPF index disables the LPF so the samples are loaded directly into the HPF
              //2nd order filters take about 1ms per 32 samples in 3-axis, so 25ms per second @ 800Hz sampling
              HPF_BIQUAD(0,accel_data_fifo[AD_fifo_tail].x_axis);
              HPF_BIQUAD(1,accel_data_fifo[AD_fifo_tail].y_axis);
              HPF_BIQUAD(2,accel_data_fifo[AD_fifo_tail].z_axis);
            }
          }
        }
      }   //end filter flush



//      if(AD_FIFO_NOT_EMPTY){                                                 //fifo fills in ISR while this executes
     while(AD_FIFO_NOT_EMPTY){                                                 //fifo fills in ISR while this executes T.S.... not really... we hold the semaphore.

        //*** LPFs for position indication and supervision ***
        PRE_FILTER_XYZ(accel_data_fifo[AD_fifo_tail].x_axis,                    //pre filter decimates the active rate of 800Hz down to the sleep rate of 50Hz, downstream supervison processing runs at 50Hz always
                       accel_data_fifo[AD_fifo_tail].y_axis,
                       accel_data_fifo[AD_fifo_tail].z_axis);

         if(accel_mode < 2){                                                    // accel_mode < 2 = 50Hz data rate, at 50Hz process every sample directly into the supervision filter
            FILTER_XYZ(accel_data_fifo[AD_fifo_tail].x_axis,
                       accel_data_fifo[AD_fifo_tail].y_axis,
                       accel_data_fifo[AD_fifo_tail].z_axis);
            supervision_sample_number++;
         }
         else if(++supervision_sample_decimate >= 16) {                         //at 800Hz process every 16th sample using the prefilter output as the source
            supervision_sample_decimate =0;
            FILTER_XYZ(PRE_FILTER_X_AXIS, PRE_FILTER_Y_AXIS, PRE_FILTER_Z_AXIS);
            supervision_sample_number++;
         }

         if(supervision_sample_number >= 16) {                                  //every 16th sample set evaluate supervision, or rather flag it to be executed later
           supervision_sample_number =0;
// Not a thing on Multisensor           task.check_supervision =1;
//           	   USE THESE!!!
//#define X_ORIENTATION           (INT8S)(XYZ_supervision_filter[0] / 655)                //filtered output is x16 so 16384 * 16 / 100 = 2621 which yelds .01G per bit, +/- 1.27G full scale INT8S
//#define Y_ORIENTATION           (INT8S)(XYZ_supervision_filter[1] / 655)				 // 2621 at 2G scale, 1310 at 4G, 655 at 8G
//#define Z_ORIENTATION           (INT8S)(XYZ_supervision_filter[2] / 655)



         }

         accel_limit_check;

         //***  Detection Routine signal processing  ***
         if(accel_mode ==2){ //this indicates the module is in active mode, when not active data is not examined by the detection routine

           if(detect_LPF_index != 0){                                           //index greater than 0 indicates we need the LPF, ==0 indicates the LPF is bypassed

             //first stage of a cascaded first order bit shift filter
            DETECT_LP_FILTER_XYZ(
              accel_data_fifo[AD_fifo_tail].x_axis,
              accel_data_fifo[AD_fifo_tail].y_axis,
              accel_data_fifo[AD_fifo_tail].z_axis,
              LPF_shifts[detect_LPF_index][0],0);

            //LPF cascade
            for(i =0; i < 3; i++){

              if(LPF_shifts[detect_LPF_index][i+1] == 0){ break; }              //some filter frequencies do not use the entire cascade, break on a 0 shift

              DETECT_LP_FILTER_XYZ(
                XYZ_detect_LP_filter[0][i],
                XYZ_detect_LP_filter[1][i],
                XYZ_detect_LP_filter[2][i],
                LPF_shifts[detect_LPF_index][i+1],i+1);

            }


            HPF_BIQUAD(0,DETECT_LPF_X_AXIS(LPF_shifts[detect_LPF_index][4],i)); //process HPFs with LPF cascade result as the input
            HPF_BIQUAD(1,DETECT_LPF_Y_AXIS(LPF_shifts[detect_LPF_index][4],i));
            HPF_BIQUAD(2,DETECT_LPF_Z_AXIS(LPF_shifts[detect_LPF_index][4],i));

          }

          else{ //a 0 LPF index disables the LPF so the samples are loaded directly into the HPF
            //2nd order filters take about 1ms per 32 samples in 3-axis, so 25ms per second @ 800Hz sampling
            HPF_BIQUAD(0,accel_data_fifo[AD_fifo_tail].x_axis);
            HPF_BIQUAD(1,accel_data_fifo[AD_fifo_tail].y_axis);
            HPF_BIQUAD(2,accel_data_fifo[AD_fifo_tail].z_axis);
          }

          //abs of HPF
          if(HPF_vals[0] >= 0)  { HPF_ABS[0] =  HPF_vals[0]; }                  //HPF output is floating point and implicitly converted back to int32_t here
          else                  { HPF_ABS[0] = -HPF_vals[0]; }

          if(HPF_vals[1] >= 0)  { HPF_ABS[1] =  HPF_vals[1]; }
          else                  { HPF_ABS[1] = -HPF_vals[1]; }

          if(HPF_vals[2] >= 0)  { HPF_ABS[2] =  HPF_vals[2]; }
          else                  { HPF_ABS[2] = -HPF_vals[2]; }

		  //accumulator[0] += HPF_ABS[0];       //accumulate
          //accumulator[1] += HPF_ABS[1];
          //accumulator[2] += HPF_ABS[2];

			HPF_ABS_comp[0] = HPF_ABS[0]<<8;
			HPF_ABS_comp[1] = HPF_ABS[1]<<8;
			HPF_ABS_comp[2] = HPF_ABS[2]<<8;

			//env filter
		   if(HPF_ABS_comp[0] > env_comp_vals[0])       { env_comp_vals[0] += (HPF_ABS_comp[0] - env_comp_vals[0]) >> 10; }  //rising = slow filter (alpha = 2^-1)
          else						{ env_comp_vals[0] += (HPF_ABS_comp[0] - env_comp_vals[0]) >> 9; }  //falling = fast filter (alpha = 2^-6)

          if(HPF_ABS_comp[1] > env_comp_vals[1])	{ env_comp_vals[1] += (HPF_ABS_comp[1] - env_comp_vals[1]) >> 10; }  //rising = slow filter
          else						{ env_comp_vals[1] += (HPF_ABS_comp[1] - env_comp_vals[1]) >> 9; }  //falling = fast filter

          if(HPF_ABS_comp[2] > env_comp_vals[2])	{ env_comp_vals[2] += (HPF_ABS_comp[2] - env_comp_vals[2]) >> 10; }  //rising = slow filter
          else						{ env_comp_vals[2] += (HPF_ABS_comp[2] - env_comp_vals[2]) >> 9; }  //falling = fast filter


			HPF_ABS_comp[0] = HPF_ABS[0]<<8;
			HPF_ABS_comp[1] = HPF_ABS[1]<<8;
			HPF_ABS_comp[2] = HPF_ABS[2]<<8;

			if(HPF_ABS_comp[0] > (env_comp_vals[0] )){ HPF_ABS_comp[0] -= (env_comp_vals[0]); }  else {HPF_ABS_comp[0] = 0;}  //subract half the env comp value
			if(HPF_ABS_comp[1] > (env_comp_vals[1] )){ HPF_ABS_comp[1] -= (env_comp_vals[1]); }	 else {HPF_ABS_comp[1] = 0;}
			if(HPF_ABS_comp[2] > (env_comp_vals[2] )){ HPF_ABS_comp[2] -= (env_comp_vals[2]); }  else {HPF_ABS_comp[2] = 0;}

          //compute envelope
          if(HPF_ABS_comp[0] > (envelope_vals[0]))       { envelope_vals[0] += (HPF_ABS_comp[0] - envelope_vals[0]) >> 1; }  //rising = fast filter (alpha = 2^-1)
          else						{ envelope_vals[0] += (HPF_ABS_comp[0] - envelope_vals[0]) >> 6; }  //falling = slow filter (alpha = 2^-6)

          if(HPF_ABS_comp[1] > (envelope_vals[1]))	{ envelope_vals[1] += (HPF_ABS_comp[1] - envelope_vals[1]) >> 1; }  //rising = fast filter
          else						{ envelope_vals[1] += (HPF_ABS_comp[1] - envelope_vals[1]) >> 6; }  //falling = slow filter

          if(HPF_ABS_comp[2] > (envelope_vals[2]))	{ envelope_vals[2] += (HPF_ABS_comp[2] - envelope_vals[2]) >> 1; }  //rising = fast filter
          else						{ envelope_vals[2] += (HPF_ABS_comp[2] - envelope_vals[2]) >> 6; }  //falling = slow filter

          //accumulate
          accumulator[0] += envelope_vals[0] >>8;       //accumulate and drop envelope filter gain
          accumulator[1] += envelope_vals[1] >>8;
          accumulator[2] += envelope_vals[2] >>8;


          //perform boxcar integration accumulate and threshold compare at decimated rate
          if(++detector_sample_number >= 32){
            detector_sample_number = 0;

            //if(Proc_config.detect.SupMode & 0x20){ accumulator[2] = 0; }       //this bit indicates the Z-axis is bypassed from detection so zero its result

            output_boxcar[bc_head] = (uint32_t)((accumulator[0] + accumulator[1] + accumulator[2]));
#if(0)
			uint32_t env_comp_local = env_comp_routine_integer(output_boxcar[bc_head]);

			uint32_t env_comp_local_max = (output_boxcar[bc_head] * 6) /10;	//cap this at 60%
			if(env_comp_local > env_comp_local_max){
				env_comp_local = env_comp_local_max;
			}

			//Disable env comp
			//output_boxcar[bc_head] -= env_comp_local;

			//apply EDAPT scale factor
			//output_boxcar[bc_head] *= (512 - env_comp_matrix[0]); //this value is intended to come from the coordinator in the case of many nodes reporting events simultaniously
			//output_boxcar[bc_head] >>= 9; // / 512
#endif

            accumulator[0] = 0;
            accumulator[1] = 0;
            accumulator[2] = 0;

			output += output_boxcar[bc_head];                                   //boxcar filter
            output -= output_boxcar[bc_tail];

            if(++bc_head >= OUTPUT_BOXCAR_LEN){bc_head =0;}
            if(++bc_tail >= OUTPUT_BOXCAR_LEN){bc_tail =0;}

			#ifdef LOG_DETECTOR
				output_scaled = ssc_log(output >> 13);
			#else
				#ifdef COMPANDED_DETECTOR
					output_scaled = LM100_companding(output >> 6);
				#else
					output_scaled = output >> 14;
				#endif
			#endif


            if(output_max < output_scaled){ output_max = output_scaled; }                     //output max is used to max hold and report the event mag in the tap tracker
            if(plot_max_hold < output_scaled){ plot_max_hold = output_scaled; }               //this is used by the plot function and reported as the sensor response for each timer tick interval

            xSemaphoreTake( xJackMutex, ( TickType_t ) portMAX_DELAY );
            	accelDataForJack = output_scaled;
            xSemaphoreGive( xJackMutex);
			if(accelMagnitude < output_scaled){
				accelMagnitude = output_scaled;
			}


            if( getProxAlarm() == TRUE){
				if(output_scaled >= Proc_config.detect.Event_Threshold){ //Proc_config.detect.Event_Threshold){                   //if the output is over threshold then this is an event, pass this info on to the tap tracker and let it do its thing
				  if((ret_val = discrete_tap_tracker(TAP_EVENT))){ /*task.send_status_short =1;*/ } //again return !=0 indicates a state change that should be sent up the line
				  	  setAlarmFlag();
				}
            }else{
            	if(output_scaled >= (Proc_config.detect.Event_Threshold * (2))){                   //if the output is over threshold then this is an event, pass this info on to the tap tracker and let it do its thing
				  if((ret_val = discrete_tap_tracker(TAP_EVENT))){ /*task.send_status_short =1;*/ } //again return !=0 indicates a state change that should be sent up the line
				  	  setAlarmFlag();
            	}
            }


			//run LPF used for environmental comp
			//env_comp_matrix[0]

          }//end decimated portion

         }//end detect routine

         //increment FIFO tail
         AD_FIFO_PULL;                                                          //increment Accelerometer data fifo
         samples_processed++;                                                   //counter to be used with timing diagnostics
      }//end fifo unload loop

      break;

            // signal processing is now (01JUN2015) taking 1.70mS per 32 x 3-axis, so 43mS per second
            // signal processing is now (26JUN2015) taking 1.56mS per 32 x 3-axis, so 39mS per second
            //13JUL2015 - introduce LPF, went from 46.9mS per sec -> 80.9mS per sec (time now includes active I2C ISR unloading the accelerometer)
            //17JUL2015 - use 4 cascaded bitshift LPF on detector and add second bitshift LPF on supervision for rate hopping - 59.4mS per sec with LPF on, 48.7mS per sec without
            //22JUL2015 - replace floating point envelope computation with integer - 47.4mS/S with LPF & 35.3mS/S without

    default:
    break;

  }

  return ret_val;
 }


uint8_t discrete_tap_tracker(uint8_t op_code)
 {
	uint8_t ret_val = 0;//, new_alarm =0;
  switch (op_code){
    case(INIT_EVENT):

      //initialize internal vars
      accel_sensor_status.alarm = 0;
      accel_sensor_status.alarm_timer = 0;
      accel_sensor_status.event_count = 0;
      accel_sensor_status.event_timer = 0;
      accel_sensor_status.event_timer_reload = (TICK_PER_SEC * accelSettings.eventWindow_s)/10;   //(TICK_PER_SEC * Proc_config.detect.Event_Window)/10;           // EVENT_WINDOW;
      accel_sensor_status.alarm_timer_reload = TICK_PER_SEC * accelSettings.alarmWindow_s;    	//TICK_PER_SEC * Proc_config.detect.Alarm_Window;                //ALARM_WINDOW;
      accel_sensor_status.alarm_hold_timer_reload = TICK_PER_SEC * Proc_config.detect.Alarm_Hold_Time;        //ALARM_HOLD;
      accel_sensor_status.event_count_threshold = accelSettings.eventCount;						//Proc_config.detect.Event_Count;                             //EVENT_COUNT_ALARM;

      accel_sensor_status.liftime_event_count =0;
      accel_sensor_status.liftime_alarm_count =0;
      accel_sensor_status.liftime_event_TO_count =0;
      accel_sensor_status.uptime = 0;

    break;

    case(TAP_EVENT):    //an event has been declared by some other detection routine

      //Load the output max hold into sensor status struct
      if(output_max > 255){
//        LN_STATUS.event_mag = 255;
      }
      else{
//        LN_STATUS.event_mag = output_max;
      }

      //if this is not a new event do nothing else - this is equivalent to the peak hold state in flexzone
      if(accel_sensor_status.event_timer != 0){
        break;
      }

      //alarm bypass is triggered by a user configuration & the aux contact being in alarm (so if configured in such a way, dont alarm when the gate is opened)
      if(ALARM_BYPASS){ break; }

      //reaching this point indicates a new event
      accel_sensor_status.event_count ++;	accel_sensor_status.liftime_event_count ++;   //increment event counters
      accel_sensor_status.event_timer = accel_sensor_status.event_timer_reload;             //set event window timer
      accel_sensor_status.alarm_timer = accel_sensor_status.alarm_timer_reload;             //set alarm window timer
      ret_val =1;                                                               //new event - transmit to base
//	  task.send_status_short_no_ack =1;
      //debug
	  //task.send_routing_beacon =1;
      if(accel_sensor_status.event_count == accel_sensor_status.event_count_threshold){     //if the event counter == the event count threshold we have an alarm
             accel_sensor_status.alarm = 1;	accel_sensor_status.liftime_alarm_count ++;   //set alarm state and increment alarm counter
             accel_sensor_status.alarm_timer = accel_sensor_status.alarm_hold_timer_reload; //set alarm window timer to the alarm hold time
//			 task.send_status_short =1;
             ACCEL_LED_ON;
             accelConfidenceForJack = 100;
             accelConfidenceForLogging = 100;
//			 new_alarm = 1;
      }

      else if(accel_sensor_status.event_count > accel_sensor_status.event_count_threshold){ //if the event counter > the event count threshold we have a the continuation of a already declared alarm
             accel_sensor_status.event_count = accel_sensor_status.event_count_threshold;   //set the event count back to its threshold level so it doesn't increase forever
             accel_sensor_status.alarm_timer = accel_sensor_status.alarm_hold_timer_reload; //reset the alarm hold time
             ret_val =0;                                                        //retriggerign existing alarm - do not transmit to base
      }

      //MMA_8451_ACTIVITY;                                                        //clear the activity timeout timer for the accelerometer
    break;

    case(TIMER_TICK_EVENT):     //a timer tick has elapsed

      //if(MMA_sleep_timer){                                                      //process the accelerometer sleep timer
        //if(--MMA_sleep_timer ==0){                                              //if the timer rolls over to zero
           //LN_STATUS.event_mag = 0;
           //if(accel_mode == 2){                                                 //...and the accelerometer is in active mode
            //MMA_8451_ENTER_SLEEP;                                               //sleep the accelerometer to save power
            //accel_mode =1;                                                      //set the mode to sleep(50Hz data rate)
          //}
        //}
      //}

      if(accel_sensor_status.event_timer) {                                           //if the event window timer is set process it
        if(--accel_sensor_status.event_timer == 0){                                   //if the event window timer is expiring
          if(output_max > 255){                                                 //collect the peak hold one last time
//            LN_STATUS.event_mag = 255;
          }
          else{
//            LN_STATUS.event_mag = output_max;
          }
          ret_val =1;                                                           //transmit event mag to base
		  //task.send_status_short_no_ack =1;	//ship this to save airtime
          output_max =0;                                                        //clear peak hold
        }
      }

      if(accel_sensor_status.alarm_timer){                                            //if the alarm window timer is set process it
        //MMA_8451_ACTIVITY;                                                      //keep the accelerometer awake if there are any events on the books
        if(--accel_sensor_status.alarm_timer ==0){                                    //if the alarm window timer is expiring
          if(accel_sensor_status.alarm == 0){accel_sensor_status.liftime_event_TO_count ++;} //if no alarm was declared then increment a event timeout counter
          accel_sensor_status.alarm = 0;                                              //clear alarm status
          ACCEL_LED_OFF;
          accel_sensor_status.event_count = 0;                                        //clear event count
//          LN_STATUS.event_mag = 0;                                              //clear event mag
          ret_val =1;                                                           //alarm or events have cleared - transmit to base
//		  task.send_status_short =1;
          //if(accel_mode == 2){                                                  //if the accelerometer is active then sleep it
            //MMA_8451_ENTER_SLEEP;
            //accel_mode =1;
          //}
        }
      }
      accel_sensor_status.uptime ++;                                                  //increment diagnostic uptime counter
    break;

    default:
    break;

  }

//  LN_STATUS.event_count = sensor_status.event_count;                            //copy things from internal control struct into sensor status struct
//  LN_STATUS.alarm = 0x01 & sensor_status.alarm;



  //sensor_status.send_flag |= ret_val;	                                        //send flag mimics the ret_val but must be cleared upon sending update
  return ret_val;
 }




uint32_t LM100_companding( uint32_t a_nInput){
	 uint32_t op  = a_nInput;
	 uint32_t res = 0;
	 uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


	 //// "one" starts at the highest power of four <= than the argument.
	 //while (one > op)
	 //{
		 //one >>= 2;
	 //}

	one >>= __builtin_clz(op) & ~0x3;

	 while (one != 0)
	 {
		 if (op >= res + one)
		 {
			 op = op - (res + one);
			 res = res +  2 * one;
		 }
		 res >>= 1;
		 one >>= 2;
	 }

	 ///* Do arithmetic rounding to nearest integer */
	 //if (op > res)
	 //{
		 //res++;
	 //}

	 //shave off some noise
	// if (res < 8) res = 0;
	// else res -= 8;

	 return res;
 }



#define ACCEL_TCP_SAMPLES_TO_SEND	387
#define HEADER_SIZE 13
uint8_t	accel_sequence_num = 0;

union U_MMA_sample_withPaddingByte{
	struct {
		int16_t padding;				// I have no idea why we put this padding here.
		int16_t x_axis;
		int16_t y_axis;
		int16_t z_axis;
	};
	uint8_t raw[8];
};

__BSS(BOARD_SDRAM) union U_MMA_sample_withPaddingByte dataCopy[ACCEL_TCP_SAMPLES_TO_SEND];

#if(0)
void tcp_accel_task(void *pvParameters)
{

	struct netconn *conn, *newconn;
	error_t err;
//	uint32_t ulNotificationValue;
	uint32_t sampleCount,j;

//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 1000 );
	LWIP_UNUSED_ARG(pvParameters);
	/* Store the handle of the calling task. */
	xTaskToNotify_accel = xTaskGetCurrentTaskHandle();
	//xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY );

	/* Create a new connection identifier. */
	/* Bind connection to well known port number 8. */


	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, IP_ADDR_ANY, 8);
	LWIP_ERROR("Accelerometer task: invalid conn", (conn != NULL), return;);


	/* Tell connection to go into listening mode. */
	netconn_listen(conn);
	/* Main loop. Get sensor data and send via TCP */
	while (1)
	{
		uint32_t sent_frames = 0;
		ip_addr_t client_address;
		uint16_t client_port;
		uint8_t header[HEADER_SIZE];
		TaskHandle_t accel_polling_handle = NULL;
		/* Grab new connection. */
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Accelerometer task: awaiting new connection %p\r\n", newconn);
		xSemaphoreGive( xPrintMutex );

		err = netconn_accept(conn, &newconn);
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Accelerometer task: accepted new connection %p\r\n", newconn);
		xSemaphoreGive( xPrintMutex );

		err |= netconn_peer(newconn, &client_address, &client_port);
		/* Process the new connection. */
		vTaskDelay(50);
		if (err == NO_ERROR) {

			do {
				vTaskDelay(50);	// wait for samples.  This needs to be less than the time to gather ACCEL_TCP_SAMPLES_TO_SEND number of samples.
								//						128 samples at 800 samples / s = 128 * 1/800 = 160mS.

				xSemaphoreTake( xAccelMutex, ( TickType_t ) portMAX_DELAY );
				if(AD_fifo_head > AD_fifo_tail_tcpip){
					sampleCount =  AD_fifo_head - AD_fifo_tail_tcpip;
				} else if(AD_fifo_head < AD_fifo_tail_tcpip) {
					sampleCount =  AD_fifo_head + AD_FIFO_SIZE - AD_fifo_tail_tcpip;

				} else{
					sampleCount = 0;
				}

				if(sampleCount < ACCEL_TCP_SAMPLES_TO_SEND){
					xSemaphoreGive( xAccelMutex);
					continue;
				}
				memset(dataCopy,0,sizeof(dataCopy));
				// we have at least 128 samples in our FIFO.
				for(j = 0; j < ACCEL_TCP_SAMPLES_TO_SEND; j++){
					dataCopy[j].x_axis = accel_data_fifo[AD_fifo_tail_tcpip].x_axis;
					dataCopy[j].y_axis = accel_data_fifo[AD_fifo_tail_tcpip].y_axis;
					dataCopy[j].z_axis = accel_data_fifo[AD_fifo_tail_tcpip].z_axis;
					AD_FIFO_TCPIP_PULL;
				}
				xSemaphoreGive( xAccelMutex);

				/* Print out the angle data. */
				//PRINTF("x= %2d y= %2d z= %2d\r\n", g_xAngle, g_yAngle, g_zAngle);

				header[0] = 0xFF;
				header[1] = 0xFE;
				header[2] = 0xFF;
				header[3] = 0xFE;
				header[4] = 0xFF;
				header[5] = 0xFE;
				header[6] = 0xFF;
				header[7] = 0xFE;
				header[8] = accel_sequence_num;
				memcpy(&header[9], &timestamp_us, sizeof(timestamp_us));
				err = netconn_write(newconn, &header, HEADER_SIZE, NETCONN_NOCOPY);
				err |= netconn_write(newconn, (uint8_t *)&(dataCopy[0].raw[0]), (ACCEL_TCP_SAMPLES_TO_SEND * 2 * 4), NETCONN_NOCOPY);
				accel_sequence_num++;
				if(accel_sequence_num > MAX_SEQ_NUM){
					accel_sequence_num = 0;
				}
				if(err == NO_ERROR)
				{
					sent_frames++;
				}


			}
			while(err == NO_ERROR);
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Accelerometer task: Got EOF, looping. %u sent\r\n", sent_frames);
			xSemaphoreGive( xPrintMutex );

			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
			vTaskDelete(accel_polling_handle);
		}
		else{
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Accelerometer task: netconn connection error \"%s\"\r\n", lwip_strerr(err));
			xSemaphoreGive( xPrintMutex );
			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);


		}

	}
}
#endif
#if(0)
static void accel_polling_task(void *pvParameters){
	int16_t xData           = 0;
	int16_t yData           = 0;
	int16_t zData           = 0;
	fxos_data_t sensorData   = {0};
	uint8_t hz_counter = 4; // the loop is actually running at 1kHz so we skip every 5th sensor read to maintain 800hz (4/5)
	buffer_ping_pong = 0; // tells the poller in which buffer to put the accel data. range of 0-BUFFER_NUMBER
	buffer_readIndex = 0; // tells the tcp streamer which buffer of accel data to send. range of 0-BUFFER_NUMBER
	buffer_writeIndex = 0; // tells the poller where to write the data within a buffer. range of 0-BUFFER_SIZE
	while (1){
		if (hz_counter){
			xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY );
			/* Get new accelerometer data. */
			if (FXOS_ReadSensorData(&fxosHandle, &sensorData) != kStatus_Success)
			{
				xSemaphoreGive( xMutex );
				xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
				PRINTF("Accelerometer task: error reading from accelerometer.\r\n");
				xSemaphoreGive( xPrintMutex );

				vTaskDelay(100);
			}
			else{
				xSemaphoreGive( xMutex );
				/* Get the X and Y data from the sensor data structure in 14 bit left format data*/
				xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
				yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;
				zData = (int16_t)((uint16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB) / 4U;
				memset(&accel_data[buffer_ping_pong][(buffer_writeIndex + 0)], 0, sizeof(xData));
				memcpy(&accel_data[buffer_ping_pong][(buffer_writeIndex + 2)], &xData, sizeof(xData));
				memcpy(&accel_data[buffer_ping_pong][(buffer_writeIndex + 4)], &yData, sizeof(yData));
				memcpy(&accel_data[buffer_ping_pong][(buffer_writeIndex + 6)], &zData, sizeof(zData));
				buffer_writeIndex += 8;
				if (buffer_writeIndex == BUFFER_SIZE){
					// the buffer is full so we will now notify the other task to send it
					timestamp_us = udp_sync_us + udp_master_time_us;
					buffer_readIndex = buffer_ping_pong;

					xTaskNotifyGiveIndexed( xTaskToNotify_accel,
							xArrayIndex_accel);
					if (buffer_ping_pong) buffer_ping_pong--;
					else buffer_ping_pong++;
					buffer_writeIndex = 0;
				}
			}
			hz_counter--;
		}
		else hz_counter = 4; // skip this sensor read to maintain 800Hz
		McuWait_WaitOSms(1); //800Hz
	}
}

#endif
