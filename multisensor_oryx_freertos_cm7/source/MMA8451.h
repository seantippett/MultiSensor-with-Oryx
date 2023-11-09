#ifndef MMA8451_H
#define MMA8451_H

#include "app_shared.h"


 #define MMA8451_INT1			(0 == port_pin_get_input_level(MMA8451_IRQ1_PORT))
 #define MMA8451_INT2			MMA8451_INT1
 
struct MMA_sample{
	  int16_t x_axis;
      int16_t y_axis;
      int16_t z_axis;
    };

struct MMA_XY_sample{
		int16_t x_axis;
		int16_t y_axis;
	};

#define ACCEL_MAX_SAMPLES                       32	// max samples that the FIFO on the MMA8451 holds.
#define AD_FIFO_SIZE							1024	// FIFO in this application.
#define SIZE_OF_MMA_DATA_SET                    100



union MMA_data_set{
  uint8_t byte_array[4];
  struct{
	  uint8_t transient;
	  uint8_t rotation;
	  uint8_t pulse;
	  uint8_t reserved;
    //struct MMA_sample sample[ACCEL_MAX_SAMPLES];
  };
};

struct MMA_transient_params{
	uint8_t threshold;
	uint8_t debounce;
	uint8_t HPF_val;
	uint8_t detect;
};




extern struct MMA_sample accel_data_fifo[AD_FIFO_SIZE];

extern uint16_t AD_fifo_head, AD_fifo_tail, AD_fifo_tail_tcpip, eng_packet_fifo_tail, eng_packet_counter, MMA8451_initialized;
extern uint8_t accel_mode; //used to track sleep / run state 0 = use transient detect for alarming, 1 = sleep, 2 = active
extern struct MMA_transient_params wake_params;
extern union MMA_data_set accel_data;


#define AD_FIFO_NOT_EMPTY       (AD_fifo_tail != AD_fifo_head)
#define AD_FIFO_TCPIP_NOT_EMPTY (AD_fifo_tail_tcpip) != AD_fifo_head);
#define AD_FIFO_EMPTY			(AD_fifo_tail == AD_fifo_head)
#define AD_FIFO_TCPIP_EMPTY 	(AD_fifo_tail_tcpip) == AD_fifo_head);
#define AD_FIFO_PUSH            if(++AD_fifo_head >= AD_FIFO_SIZE){ AD_fifo_head =0; }
#define AD_FIFO_PULL            if(++AD_fifo_tail >= AD_FIFO_SIZE){ AD_fifo_tail =0; }
#define AD_FIFO_TCPIP_PULL            if(++AD_fifo_tail_tcpip >= AD_FIFO_SIZE){ AD_fifo_tail_tcpip =0; }


#define ENG_PACKET_ELEM_IN_QUEUE	((AD_fifo_head - eng_packet_fifo_tail) & 0x3F)  //this is hard coded for 64 element Queue length and doesn't handle wrap around, the eng packets are expected to keep up	
#define ENG_PACKET_PULL(n)			eng_packet_fifo_tail += n; eng_packet_fifo_tail &= 0x3F;
	


uint8_t MMA8451_read_FIFO(int16_t *samples, uint16_t *samplesCount);
uint8_t MMA8451_init( void );
void MMA8451_IRQ( void );
uint8_t MMA8451_data_streamer( void );

uint8_t MMA8451_reg_write(uint8_t reg_add, uint8_t reg_val);
uint8_t MMA8451_reg_read(uint8_t reg_add);
uint8_t MMA8451_sample_read( struct MMA_sample * sample);
uint8_t MMA8451_FIFO_read(struct MMA_sample * sample);
uint8_t MMA8451_set_transient_params(struct MMA_transient_params new_detect_params);


#define MMA8451_DEV_ADD                         0x1C
//#define MMA8451_DEV_ADD                         0x1E

#define MMA8451_ADDR_WHO_AM_I					0x0D
#define			MMA8451_WHO_AM_I_RESPONSE		0x1A

#define MMA8451_ADDR_CTRL_REG1                  0x2A
#define         MMA8451_ACTIVE_BIT              0x01
#define         MMA8451_DATA_RATE_800Hz         0x00
#define         MMA8451_DATA_RATE_400Hz         0x08
#define         MMA8451_DATA_RATE_200Hz         0x10
#define         MMA8451_DATA_RATE_100Hz         0x18
#define         MMA8451_DATA_RATE_50Hz          0x20
#define         MMA8451_DATA_RATE_12_5Hz        0x28
#define         MMA8451_DATA_RATE_6_25Hz        0x30
#define         MMA8451_DATA_RATE_1_56Hz        0x38

#define         MMA8451_FREAD_MSB_ONLY          0x02   

#define MMA8451_ADDR_CTRL_REG2                  0x2B
#define         MMA8451_SELF_TEST               0x80
#define         MMA8451_SOFT_RESET              0x40
#define         MMA8451_AUTO_SLEEP_EN           0x04

#define MMA8451_ADDR_CTRL_REG3                  0x2C
#define         MMA8451_FIFO_GATE               0x80
#define         MMA8451_WAKE_TRANS              0x40
#define         MMA8451_WAKE_LNDPRT             0x20
#define         MMA8451_WAKE_PULSE              0x10
#define         MMA8451_WAKE_FF_MT              0x08
#define         MMA8451_IPOL_HIGH               0x02
#define         MMA8451_INTR_OD                 0x01

#define MMA8451_ADDR_CTRL_REG4                  0x2D
#define MMA8451_ADDR_CTRL_REG5                  0x2E
#define MMA8451_ADDR_ASLP_COUNT_REG             0x29

#define MMA8451_ADDR_X_AXIS_DATA        0x01          
#define MMA8451_ADDR_STATUS             0x00
#define         MMA8451_DATA_RDY_BIT    0x08
#define MMA8451_ADDR_F_STATUS           0x00
#define         MMA8451_FIFO_FCNT_MASK  0x3F

#define MMA8451_ADDR_F_SETUP            0x09
#define         MMA8451_FMODE_FIFO_DISABLED             0x00
#define         MMA8451_FMODE_FIFO_CIRCULAR_MODE        0x40
#define         MMA8451_FMODE_FIFO_FILL_MODE            0x80
#define         MMA8451_FMODE_FIFO_TRIGGER_MODE         0xC0

#define         MMA8451_ADDR_INT_SOURCE         0x0C

#define         MMA8451_ADDR_SRC_ASLP           0x0B
#define         MMA8451_ADDR_SRC_TRANS          0x1E
#define         MMA8451_ADDR_SRC_LNDPRT         0x10
#define         MMA8451_ADDR_SRC_PULSE          0x22
#define         MMA8451_ADDR_SRC_FF_MT          0x16


#define MMA8451_ADDR_XYZ_DATA_CFG       0x0E
#define         MMA8451_RANGE_2G        0x00        
#define         MMA8451_RANGE_4G        0x01        
#define         MMA8451_RANGE_8G        0x02     

#define MMA8451_ADDR_INT_SOURCE         0x0C

//transient detection
#define MMA8451_TRANSIENT_CFG_REG       0x1D
#define MMA8451_TRANSIENT_SRC_REG       0x1E
#define MMA8451_TRANSIENT_THS_REG       0x1F
#define MMA8451_TRANSIENT_CNT_REG       0x20
#define MMA8451_HP_FILTER_REG           0x0F

//portrait / landscape detect
#define MMA8451_PL_STATUS_REG           0x10               
#define MMA8451_PL_CONFIG_REG           0x11
#define MMA8451_PL_COUNT_REG            0x12
#define MMA8451_PL_BF_ZCOMP_REG         0x13
#define MMA8451_PL_THS_REG              0x14

//pulse (tap) detection
#define MMA8451_PULSE_CFG_REG           0x21
#define MMA8451_PULSE_SRC_REG           0x22
#define MMA8451_PULSE_THSX_REG          0x23
#define MMA8451_PULSE_THSY_REG          0x24
#define MMA8451_PULSE_THSZ_REG          0x25
#define MMA8451_PULSE_TMLT_REG          0x26
#define MMA8451_PULSE_LTCY_REG          0x27
#define MMA8451_PULSE_WIND_REG          0x28




#endif
