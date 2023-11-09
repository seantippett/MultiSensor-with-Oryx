/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v9.0
processor: MIMXRT1176xxxxx
package_id: MIMXRT1176DVMAA
mcu_data: ksdk2_0
processor_version: 9.0.3
board: MIMXRT1170-EVK
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 673cf921-1ec2-4930-93f2-e0768a84092c
  called_from_default_init: true
  selectedCore: cm7
- name: SHIELD_InitPeripherals
  UUID: 3d2ff5d1-07ed-421d-afb4-d4740b8bab84
  id_prefix: SHIELD_
  selectedCore: cm7
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"
#include "app_pir.h"
#include "fsl_lpuart_edma.h"
/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * GPT2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPT2'
- type: 'gpt'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'gpt_e92a0cbd07e389b82a1d19b05eb9fdda'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPT2'
- config_sets:
  - fsl_gpt:
    - gpt_config:
      - clockSource: 'kGPT_ClockSource_Periph'
      - clockSourceFreq: 'BOARD_BootClockRUN'
      - oscDivider: '1'
      - divider: '1'
      - enableFreeRun: 'false'
      - enableRunInWait: 'true'
      - enableRunInStop: 'true'
      - enableRunInDoze: 'false'
      - enableRunInDbg: 'false'
      - enableMode: 'true'
    - input_capture_channels: []
    - output_compare_channels:
      - 0:
        - channel: 'kGPT_OutputCompare_Channel1'
        - mode: 'kGPT_OutputOperation_Disconnected'
        - compare_value_str: '255'
    - interrupt_requests: ''
    - isInterruptEnabled: 'false'
    - interrupt:
      - IRQn: 'GPT2_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'false'
    - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const gpt_config_t GPT2_config = {
  .clockSource = kGPT_ClockSource_Periph,
  .divider = 1UL,
  .enableFreeRun = false,
  .enableRunInWait = true,
  .enableRunInStop = true,
  .enableRunInDoze = false,
  .enableRunInDbg = false,
  .enableMode = true
};

static void GPT2_init(void) {
  /* GPT device and channels initialization */
  GPT_Init(GPT2_PERIPHERAL, &GPT2_config);
  GPT_SetOscClockDivider(GPT2_PERIPHERAL, 1);
  GPT_SetOutputCompareValue(GPT2_PERIPHERAL, kGPT_OutputCompare_Channel1, 255);
  GPT_SetOutputOperationMode(GPT2_PERIPHERAL, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected);
  /* Enable GPT interrupt sources */
  GPT_EnableInterrupts(GPT2_PERIPHERAL, 0);
}

/***********************************************************************************************************************
 * SEMC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SEMC'
- type: 'semc'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'semc_84a769c198c91c527e11dcec2f5b4b81'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'SEMC'
- config_sets:
  - fsl_semc:
    - enableDCD: 'true'
    - clockConfig:
      - clockSource: 'kSEMC_ClkSrcPeri'
      - clockSourceFreq: 'BOARD_BootClockRUN'
    - semc_config_t:
      - dqsMode: 'kSEMC_Loopbackdqspad'
      - cmdTimeoutCycles: '0'
      - busTimeoutCycles: '0'
      - queueWeight:
        - queueaEnable: 'false'
        - queueaWeight:
          - structORvalue: 'structure'
          - queueaConfig:
            - qos: '5'
            - aging: '8'
            - slaveHitSwith: '0x40'
            - slaveHitNoswitch: '0x10'
        - queuebEnable: 'false'
        - queuebWeight:
          - structORvalue: 'structure'
          - queuebConfig:
            - qos: '5'
            - aging: '8'
            - slaveHitSwith: '0x24'
            - weightPagehit: '0x60'
            - bankRotation: '0x40'
    - semc_sdram_config_t:
      - csxPinMux: 'kSEMC_MUXCSX0'
      - semcSdramCs: 'kSEMC_SDRAM_CS0'
      - address: '0x80000000'
      - memsize_input: '32MB'
      - portSize: 'kSEMC_PortSize16Bit'
      - burstLen: 'kSEMC_Sdram_BurstLen8'
      - columnAddrBitNum: 'kSEMC_SdramColunm_9bit'
      - casLatency: 'kSEMC_LatencyThree'
      - tPrecharge2Act_Ns: '36'
      - tAct2ReadWrite_Ns: '36'
      - tRefreshRecovery_Ns: '127'
      - tWriteRecovery_Ns: '20'
      - tCkeOff_Ns: '70'
      - tAct2Prechage_Ns: '90'
      - tSelfRefRecovery_Ns: '255'
      - tRefresh2Refresh_Ns: '120'
      - tAct2Act_Ns: '12'
      - tPrescalePeriod_Ns: '220'
      - tIdleTimeout_Ns: '0'
      - refreshPeriod_nsPerRow: '73000'
      - refreshUrgThreshold: '178000'
      - refreshBurstLen: '5'
      - delayChain: '0'
    - sdramArray: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void SEMC_init(void) {
} */

/***********************************************************************************************************************
 * LPI2C5_CODEC_ACCEL initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPI2C5_CODEC_ACCEL'
- type: 'lpi2c'
- mode: 'master'
- custom_name_enabled: 'true'
- type_id: 'lpi2c_db68d4f4f06a22e25ab51fe9bd6db4d2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPI2C5'
- config_sets:
  - main:
    - clockSource: 'Lpi2cClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
    - interrupt:
      - IRQn: 'LPI2C5_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - quick_selection: 'qs_interrupt'
  - master:
    - mode: 'transfer'
    - config:
      - enableMaster: 'true'
      - enableDoze: 'true'
      - debugEnable: 'false'
      - ignoreAck: 'false'
      - pinConfig: 'kLPI2C_2PinOpenDrain'
      - baudRate_Hz: '300000'
      - busIdleTimeout_ns: '0'
      - pinLowTimeout_ns: '0'
      - sdaGlitchFilterWidth_ns: '0'
      - sclGlitchFilterWidth_ns: '0'
      - hostRequest:
        - enable: 'false'
        - source: 'kLPI2C_HostRequestExternalPin'
        - polarity: 'kLPI2C_HostRequestPinActiveHigh'
      - edmaRequestSources: ''
    - transfer:
      - blocking: 'false'
      - enable_custom_handle: 'false'
      - callback:
        - name: ''
        - userData: ''
      - flags: ''
      - slaveAddress: '0'
      - direction: 'kLPI2C_Write'
      - subaddress: '0'
      - subaddressSize: '1'
      - blocking_buffer: 'false'
      - enable_custom_buffer: 'false'
      - dataSize: '1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpi2c_master_config_t ACCEL_LPI2C_masterConfig = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};
lpi2c_master_transfer_t ACCEL_LPI2C_masterTransfer = {
  .flags = kLPI2C_TransferDefaultFlag,
  .slaveAddress = 0,
  .direction = kLPI2C_Write,
  .subaddress = 0,
  .subaddressSize = 1,
  .data = ACCEL_LPI2C_masterBuffer,
  .dataSize = 1
};
lpi2c_master_handle_t ACCEL_LPI2C_masterHandle;
uint8_t ACCEL_LPI2C_masterBuffer[ACCEL_LPI2C_MASTER_BUFFER_SIZE];

static void ACCEL_LPI2C_init(void) {
  LPI2C_MasterInit(ACCEL_LPI2C_PERIPHERAL, &ACCEL_LPI2C_masterConfig, ACCEL_LPI2C_CLOCK_FREQ);
  LPI2C_MasterTransferCreateHandle(ACCEL_LPI2C_PERIPHERAL, &ACCEL_LPI2C_masterHandle, NULL, NULL);
}

/***********************************************************************************************************************
 * LPI2C6_CAMERA initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPI2C6_CAMERA'
- type: 'lpi2c'
- mode: 'master'
- custom_name_enabled: 'true'
- type_id: 'lpi2c_db68d4f4f06a22e25ab51fe9bd6db4d2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPI2C6'
- config_sets:
  - main:
    - clockSource: 'Lpi2cClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
    - interrupt:
      - IRQn: 'LPI2C6_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - quick_selection: 'qs_interrupt'
  - master:
    - mode: 'transfer'
    - config:
      - enableMaster: 'true'
      - enableDoze: 'true'
      - debugEnable: 'false'
      - ignoreAck: 'false'
      - pinConfig: 'kLPI2C_2PinOpenDrain'
      - baudRate_Hz: '100000'
      - busIdleTimeout_ns: '0'
      - pinLowTimeout_ns: '0'
      - sdaGlitchFilterWidth_ns: '0'
      - sclGlitchFilterWidth_ns: '0'
      - hostRequest:
        - enable: 'false'
        - source: 'kLPI2C_HostRequestExternalPin'
        - polarity: 'kLPI2C_HostRequestPinActiveHigh'
      - edmaRequestSources: ''
    - transfer:
      - blocking: 'false'
      - enable_custom_handle: 'false'
      - callback:
        - name: ''
        - userData: ''
      - flags: ''
      - slaveAddress: '0'
      - direction: 'kLPI2C_Write'
      - subaddress: '0'
      - subaddressSize: '1'
      - blocking_buffer: 'false'
      - enable_custom_buffer: 'false'
      - dataSize: '1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpi2c_master_config_t CAMERA_LPI2C_masterConfig = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};
lpi2c_master_transfer_t CAMERA_LPI2C_masterTransfer = {
  .flags = kLPI2C_TransferDefaultFlag,
  .slaveAddress = 0,
  .direction = kLPI2C_Write,
  .subaddress = 0,
  .subaddressSize = 1,
  .data = CAMERA_LPI2C_masterBuffer,
  .dataSize = 1
};
lpi2c_master_handle_t CAMERA_LPI2C_masterHandle;
uint8_t CAMERA_LPI2C_masterBuffer[CAMERA_LPI2C_MASTER_BUFFER_SIZE];


#if(0)
static void CAMERA_LPI2C_init(void) {
  LPI2C_MasterInit(CAMERA_LPI2C_PERIPHERAL, &CAMERA_LPI2C_masterConfig, CAMERA_LPI2C_CLOCK_FREQ);
  LPI2C_MasterTransferCreateHandle(CAMERA_LPI2C_PERIPHERAL, &CAMERA_LPI2C_masterHandle, NULL, NULL);
}
#endif
/***********************************************************************************************************************
 * GPT3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPT3'
- type: 'gpt'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'gpt_e92a0cbd07e389b82a1d19b05eb9fdda'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPT3'
- config_sets:
  - fsl_gpt:
    - gpt_config:
      - clockSource: 'kGPT_ClockSource_Periph'
      - clockSourceFreq: 'BOARD_BootClockRUN'
      - oscDivider: '1'
      - divider: '1'
      - enableFreeRun: 'false'
      - enableRunInWait: 'true'
      - enableRunInStop: 'true'
      - enableRunInDoze: 'false'
      - enableRunInDbg: 'false'
      - enableMode: 'true'
    - input_capture_channels: []
    - output_compare_channels:
      - 0:
        - channel: 'kGPT_OutputCompare_Channel1'
        - mode: 'kGPT_OutputOperation_Disconnected'
        - compare_value_str: '23'
    - interrupt_requests: 'kGPT_OutputCompare1InterruptEnable'
    - isInterruptEnabled: 'true'
    - interrupt:
      - IRQn: 'GPT3_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'true'
      - handler_custom_name: 'GPT3_IRQHandler'
    - EnableTimerInInit: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const gpt_config_t GPT3_config = {
  .clockSource = kGPT_ClockSource_Periph,
  .divider = 1UL,
  .enableFreeRun = false,
  .enableRunInWait = true,
  .enableRunInStop = true,
  .enableRunInDoze = false,
  .enableRunInDbg = false,
  .enableMode = true
};

static void GPT3_init(void) {
  /* GPT device and channels initialization */
  GPT_Init(GPT3_PERIPHERAL, &GPT3_config);
  GPT_SetOscClockDivider(GPT3_PERIPHERAL, 1);
  GPT_SetOutputCompareValue(GPT3_PERIPHERAL, kGPT_OutputCompare_Channel1, 23);
  GPT_SetOutputOperationMode(GPT3_PERIPHERAL, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected);
  /* Enable GPT interrupt sources */
  GPT_EnableInterrupts(GPT3_PERIPHERAL, kGPT_OutputCompare1InterruptEnable);
  /* Enable interrupt GPT3_IRQn request in the NVIC. */
  EnableIRQ(GPT3_GPT_IRQN);
}

/***********************************************************************************************************************
 * CM7_GPIO3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'CM7_GPIO3'
- type: 'igpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'igpio_b1c1fa279aa7069dca167502b8589cb7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'CM7_GPIO3'
- config_sets:
  - fsl_gpio:
    - enable_irq_comb_0_15: 'true'
    - gpio_interrupt_comb_0_15:
      - IRQn: 'CM7_GPIO2_3_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - enable_irq_comb_16_31: 'false'
    - gpio_interrupt_comb_16_31:
      - IRQn: 'CM7_GPIO2_3_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void CM7_GPIO3_init(void) {
  /* Make sure, the clock gate for CM7_GPIO3 is enabled (e. g. in pin_mux.c) */
  /* Enable interrupt CM7_GPIO2_3_IRQn request in the NVIC. */
  EnableIRQ(CM7_GPIO3_GPIO_COMB_0_15_IRQN);
}

/***********************************************************************************************************************
 * SHIELD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'SHIELD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void SHIELD_NVIC_init(void) {
} */







/***********************************************************************************************************************
 * LPSPI1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPSPI1'
- type: 'lpspi'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'lpspi_6e21a1e0a09f0a012d683c4f91752db8'
- functional_group: 'SHIELD_InitPeripherals'
- peripheral: 'LPSPI1'
- config_sets:
  - main:
    - mode: 'kLPSPI_Master'
    - clockSource: 'LpspiClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
    - master:
      - baudRate: '500000'
      - bitsPerFrame: '8'
      - cpol: 'kLPSPI_ClockPolarityActiveHigh'
      - cpha: 'kLPSPI_ClockPhaseFirstEdge'
      - direction: 'kLPSPI_MsbFirst'
      - pcsToSckDelayInNanoSec: '1000'
      - lastSckToPcsDelayInNanoSec: '1000'
      - betweenTransferDelayInNanoSec: '1000'
      - whichPcs: 'kLPSPI_Pcs0'
      - pcsActiveHighOrLow: 'kLPSPI_PcsActiveLow'
      - pinCfg: 'kLPSPI_SdiInSdoOut'
      - dataOutConfig: 'kLpspiDataOutRetained'
    - set_FifoWaterMarks: 'false'
    - fifoWaterMarks:
      - txWatermark: '0'
      - rxWatermark: '0'
    - quick_selection: 'qs_master'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpspi_master_config_t PHY_LPSPI_config = {
  .baudRate = 500000UL,
  .bitsPerFrame = 8UL,
  .cpol = kLPSPI_ClockPolarityActiveHigh,
  .cpha = kLPSPI_ClockPhaseFirstEdge,
  .direction = kLPSPI_MsbFirst,
  .pcsToSckDelayInNanoSec = 1000UL,
  .lastSckToPcsDelayInNanoSec = 1000UL,
  .betweenTransferDelayInNanoSec = 1000UL,
  .whichPcs = kLPSPI_Pcs0,
  .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,
  .pinCfg = kLPSPI_SdiInSdoOut,
  .dataOutConfig = kLpspiDataOutRetained
};

static void PHY_SPI_init(void) {
  LPSPI_MasterInit(PHY_LPSPI_PERIPHERAL, &PHY_LPSPI_config, PHY_LPSPI_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPUART2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPUART2'
- type: 'lpuart'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'lpuart_54a65a580e3462acdbacefd5299e0cac'
- functional_group: 'SHIELD_InitPeripherals'
- peripheral: 'LPUART2'
- config_sets:
  - lpuartConfig_t:
    - lpuartConfig:
      - clockSource: 'LpuartClock'
      - lpuartSrcClkFreq: 'BOARD_BootClockRUN'
      - baudRate_Bps: '512000'
      - parityMode: 'kLPUART_ParityEven'
      - dataBitsCount: 'kLPUART_EightDataBits'
      - isMsb: 'false'
      - stopBitCount: 'kLPUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - enableRxRTS: 'false'
      - enableTxCTS: 'false'
      - txCtsSource: 'kLPUART_CtsSourcePin'
      - txCtsConfig: 'kLPUART_CtsSampleAtStart'
      - rxIdleType: 'kLPUART_IdleTypeStartBit'
      - rxIdleConfig: 'kLPUART_IdleCharacter1'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpuart_config_t MICROWAVE_LPUART_config = {
  .baudRate_Bps = 512000UL,
  .parityMode = kLPUART_ParityEven,
  .dataBitsCount = kLPUART_EightDataBits,
  .isMsb = false,
  .stopBitCount = kLPUART_OneStopBit,
  .txFifoWatermark = 0U,
  .rxFifoWatermark = 1U,
  .enableRxRTS = false,
  .enableTxCTS = false,
  .txCtsSource = kLPUART_CtsSourcePin,
  .txCtsConfig = kLPUART_CtsSampleAtStart,
  .rxIdleType = kLPUART_IdleTypeStartBit,
  .rxIdleConfig = kLPUART_IdleCharacter1,
  .enableTx = true,
  .enableRx = true
};

static void MICROWAVE_LPUART_init(void) {
  LPUART_Init(MICROWAVE_LPUART_PERIPHERAL, &MICROWAVE_LPUART_config, MICROWAVE_LPUART_CLOCK_SOURCE);
}


/* must align with the modulo range */
AT_NONCACHEABLE_SECTION_ALIGN_INIT(uint32_t ADC_DMA_resultBuffer[ADC_DMA_BUFFER_LENGTH], 16) = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
																								0x00U, 0x00U, 0x00U, 0x00U,	0x00U, 0x00U, 0x00U, 0x00U};
//__DATA(SRAM_DTC_cm7)  uint32_t ADC_DMA_resultBuffer[ADC_DMA_BUFFER_LENGTH] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,																				0x00U, 0x00U, 0x00U, 0x00U,	0x00U, 0x00U, 0x00U, 0x00U};



/* Allocate TCD memory poll */
AT_NONCACHEABLE_SECTION_ALIGN_INIT(edma_tcd_t tcdMemoryPoolPtr[TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));
//__DATA(SRAM_DTC_cm7) edma_tcd_t tcdMemoryPoolPtr[TCD_QUEUE_SIZE + 1];



__DATA(SRAM_DTC_cm7)  lpuart_edma_handle_t g_lpuartEdmaHandle;

volatile bool rxBufferEmpty                                          = true;
volatile bool txBufferFull                                           = false;
volatile bool txOnGoing                                              = false;
volatile bool rxOnGoing                                              = false;
void LPUART_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);


void DMA_configuration(void){

	edma_config_t dmaConfig = {0};
    edma_transfer_config_t transferConfig;

    memset(ADC_DMA_resultBuffer,0,sizeof(ADC_DMA_resultBuffer));
/* PART 1 - Setup the DMA MUX.  */
	DMAMUX_Init(DMAMUX_ADC_BASEADDR);

	/* This points the ADC DMA signal to the DMA channel. */
	DMAMUX_SetSource(DMAMUX_ADC_BASEADDR, EDMA_ADC_CHANNEL, EDMA_ADC_REQUEST_SOURCE);
    DMAMUX_EnableChannel(DMAMUX_ADC_BASEADDR, EDMA_ADC_CHANNEL);


/* PART 2 - Setup the DMA itself */

	EDMA_GetDefaultConfig(&dmaConfig);
	EDMA_Init(EDMA_ADC_BASEADDR, &dmaConfig);

	/* Create EDMA handle */
	/*
	 * dmaConfig.enableRoundRobinArbitration = false;
	 * dmaConfig.enableHaltOnError = true;
	 * dmaConfig.enableContinuousLinkMode = false;
	 * dmaConfig.enableDebugMode = false;
	 */

	EDMA_CreateHandle(&ADC_EdmaHandle_0, EDMA_ADC_BASEADDR, EDMA_ADC_CHANNEL);
	EDMA_SetCallback(&ADC_EdmaHandle_0, EDMA_ADC_Callback, NULL);
    EDMA_ResetChannel(ADC_EdmaHandle_0.base, ADC_EdmaHandle_0.channel);


 /* PART 3 - SETUP TDC's */

    /* We need to create TCD's.  These give the specifics of the DMA transfer that will take place */
    /* 			Source is the ADC FIFO.  Note that the source address will never increment.  This  */
    /* 			is done with the PerhiperalToMemory Flag.  ADC_DMA_resultBuffer is the buffer that will accept */
    /*			the data.  Note the 4's in this msg are mostly to indicate that each transaction is*/
    /* 			4 bytes (32bits) in lenghth.  A total of 4*WATERMARK Threshold bytes will 			*/
    /*			be transfered.																		*/


    EDMA_PrepareTransfer(&transferConfig,
    						(void *)&(LPADC1->RESFIFO),		//srcAddr,
							4,						//sizeof(srcAddr[0]),
							ADC_DMA_resultBuffer,
							4,
							4, 								/* minor loop bytes: i.e. size of destination address */
							(4 * ADC_WATERMARK_THRESHOLD),  /* Number of bytes in a transfer  */
							kEDMA_PeripheralToMemory); //kEDMA_MemoryToMemory);

	// Note that when we install this one, we add the address of the next on as it's NEXTTCD address... Ping pong.
    EDMA_TcdSetTransferConfig(tcdMemoryPoolPtr, &transferConfig, &tcdMemoryPoolPtr[1]);

    /* prepare descriptor 1 */
    EDMA_PrepareTransfer(&transferConfig,
    						(void *)&(LPADC1->RESFIFO),		//&srcAddr[4],
							4,						// sizeof(srcAddr[0]),
							&(ADC_DMA_resultBuffer[ADC_WATERMARK_THRESHOLD]),
							4,
							4, 								/* minor loop bytes: i.e. size of destination address */
							(4 * ADC_WATERMARK_THRESHOLD),  /* Number of bytes in a transfer  */
							kEDMA_PeripheralToMemory); //kEDMA_MemoryToMemory);
	// Note that when we install this one, we add the address of the next on as it's NEXTTCD address... Ping pong.
    EDMA_TcdSetTransferConfig(&tcdMemoryPoolPtr[1], &transferConfig, tcdMemoryPoolPtr);

    /* in each TCD, we enable the ISR to execute on completion */
    EDMA_TcdEnableInterrupts(&tcdMemoryPoolPtr[0], kEDMA_MajorInterruptEnable);
    EDMA_TcdEnableInterrupts(&tcdMemoryPoolPtr[1], kEDMA_MajorInterruptEnable);

    /* We need to install a TCD in the DMA */
    EDMA_InstallTCD(ADC_EdmaHandle_0.base, EDMA_ADC_CHANNEL, tcdMemoryPoolPtr);



/////////////////// AUDIO ///////////////////
#if(1)
	/* PART 1 - Setup the DMA MUX.  */
	// already done above in the ADC init.	DMAMUX_Init(DMAMUX_PDM_BASEADDR);

	    /* This points the PDM DMA signal to the DMA channel */
		DMAMUX_SetSource(DMAMUX_PDM_BASEADDR, PDM_EDMA_CHANNEL, PDM_REQUEST_SOURCE);
	    DMAMUX_EnableChannel(DMAMUX_PDM_BASEADDR, PDM_EDMA_CHANNEL);


	/* PART 2 - Setup the DMA itself */
		EDMA_CreateHandle(&s_pdmDmaHandle_0, EDMA_PDM_BASEADDR, EDMA_PDM_CHANNEL);
	//	EDMA_SetCallback(&s_pdmDmaHandle_0, pdmCallBack, NULL);
	//  EDMA_ResetChannel(s_pdmDmaHandle_0.base, s_pdmDmaHandle_0.channel);




/* PART 3 - SETUP TDC's */
		/* Audio TCD's are done in their own way in the fsl_pdm_edma driver. */

    audio_init();

#endif

#define DMAMUX_LPUART_BASEADDR			DMAMUX0
#define EDMA_LPUART_BASEADDR			DMA0


    /// LPUART	//// - For the Radar
	/* PART 1 - Setup the DMA MUX.  */
	// already done above in the ADC init.	DMAMUX_Init(DMAMUX_PDM_BASEADDR);

	    /* This points the PDM DMA signal to the DMA channel */
		DMAMUX_SetSource(DMAMUX_LPUART_BASEADDR, LPUART_EDMA_CHANNEL_RX, LPUART_REQUEST_SOURCE_RX);
		DMAMUX_SetSource(DMAMUX_LPUART_BASEADDR, LPUART_EDMA_CHANNEL_TX, LPUART_REQUEST_SOURCE_TX);
	    DMAMUX_EnableChannel(DMAMUX_LPUART_BASEADDR, LPUART_EDMA_CHANNEL_RX);
	    DMAMUX_EnableChannel(DMAMUX_LPUART_BASEADDR, LPUART_EDMA_CHANNEL_TX);

		EDMA_CreateHandle(&LPUART_EdmaHandle_RX0, EDMA_LPUART_BASEADDR, LPUART_EDMA_CHANNEL_RX);
		EDMA_CreateHandle(&LPUART_EdmaHandle_TX0, EDMA_LPUART_BASEADDR, LPUART_EDMA_CHANNEL_TX);
//		EDMA_SetCallback(&LPUART_EdmaHandle_0, EDMA_ADC_Callback, NULL);
	    EDMA_ResetChannel(LPUART_EdmaHandle_RX0.base, LPUART_EdmaHandle_RX0.channel);
	    EDMA_ResetChannel(LPUART_EdmaHandle_TX0.base, LPUART_EdmaHandle_TX0.channel);


	    LPUART_TransferCreateHandleEDMA(LPUART2, &g_lpuartEdmaHandle, LPUART_UserCallback, NULL,  &LPUART_EdmaHandle_TX0, &LPUART_EdmaHandle_RX0);
	/* PART 2 - Setup the DMA itself */









	NVIC_SetPriority(DMA0_DMA16_IRQn, 5); // max syscall priority is set to 5. can't be lower to work with freeRTOS


	//  This mostly just marks the ERQ bit for this channel.  Think of it as an enable bit for the signal.
	//  Note that this uses the "SERQ" register.  It's a Setting register.  A convenient way to modify ERQ register.
	//	There's even a "CERQ" to clear a register... but it's all just modifying ERQ - which enables the DMA Requests.
	EDMA_StartTransfer(&ADC_EdmaHandle_0);
}


void XBARA_Configuration(void)
{
    /* Init xbara module. */

	XBARA_Init(XBARA_BASEADDR);


    /* Configure the XBARA signal connections. */
    XBARA_SetSignalsConnection(XBARA_BASEADDR, XBARA_INPUT_PITCH0, XBARA_OUTPUT_ADC_ETC);
}



uint32_t	currentPITPeriod = SAMPLING_PERIOD_uS;
void changePIT_Period_uS(uint32_t newTimerPeriod)
{	// Because PIT_SetTimerPeriod changes the period through the use of "LDVAL", this is a smooth transition to the new
	// period.  Talk about awesome!  No weird scenarios to deal with.
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT_CHANNEL, USEC_TO_COUNT(newTimerPeriod, BUS_CLK_FREQ));
    currentPITPeriod = newTimerPeriod;
}

void PIT_Configuration(void)
{
    pit_config_t pitConfig;

    PIT_GetDefaultConfig(&pitConfig);
    pitConfig.enableRunInDebug = false;
    PIT_Init(PIT_BASEADDR, &pitConfig);

    /* Set period is 500ms */
//    PIT_SetTimerPeriod(PIT_BASEADDR, PIT_CHANNEL, USEC_TO_COUNT(SAMPLING_PERIOD_uS, BUS_CLK_FREQ));
    changePIT_Period_uS(SAMPLING_PERIOD_uS);
}

/*!
 * @brief Configure LPADC to working with ADC_ETC.
 */

#define ADC_ETC_DONE0_Handler ADC_ETC_IRQ0_IRQHandler
#define ADC_USER_CMDID    1U
#define ADC_WATERMARK_THRESHOLD		9		// this is also the DMA threshold (what triggers a DMA activation).
void LPADC_Configuration(void)
{
    lpadc_config_t lpadcConfig;
    lpadc_conv_command_config_t lpadcCommandConfig;
    lpadc_conv_trigger_config_t lpadcTriggerConfig;

    /* Initialize the ADC module. */
    LPADC_GetDefaultConfig(&lpadcConfig);
    /* Set the Watermark - This is also the DMA threshold */
    lpadcConfig.FIFOWatermark = ADC_WATERMARK_THRESHOLD;
    LPADC_Init(LPADC1, &lpadcConfig);

#if (defined(FSL_FEATURE_LPADC_HAS_CFG_CALOFS) && FSL_FEATURE_LPADC_HAS_CFG_CALOFS)
    /* Do offset calibration. */
    LPADC_DoOffsetCalibration(DEMO_ADC_BASE, SystemCoreClock);
#endif /* FSL_FEATURE_LPADC_HAS_CFG_CALOFS */

    /* Set conversion CMD configuration. */
    // Note that this configures a 'chain' of conversions.  the .chainedNextCommandNumber in each of these
    //	command config structures points to the next command number.  Thus, when you execute the first
    //	command, all the following commands get exectued.  NOTE that command numbers can't be 0.  They
    //  start at 1.


    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = ADC_CHANNEL__V_NETWORK;
    lpadcCommandConfig.sampleChannelMode = ADC_CHANNEL_MODE__V_NETWORK;
    lpadcCommandConfig.chainedNextCommandNumber = ADC_COMMAND_NUMB__IMON_A;
    lpadcCommandConfig.hardwareAverageMode = kLPADC_HardwareAverageCount1,
	lpadcCommandConfig.sampleTimeMode = ADC_SAMPLE_TIME;
    LPADC_SetConvCommandConfig(LPADC1, ADC_COMMAND_NUMB__V_NETWORK, &lpadcCommandConfig);

    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = ADC_CHANNEL__IMON_A;
    lpadcCommandConfig.sampleChannelMode = ADC_CHANNEL_MODE__IMON_A;
    lpadcCommandConfig.chainedNextCommandNumber = ADC_COMMAND_NUMB__IMON_B;
    lpadcCommandConfig.hardwareAverageMode = kLPADC_HardwareAverageCount1,
	lpadcCommandConfig.sampleTimeMode = ADC_SAMPLE_TIME;
    LPADC_SetConvCommandConfig(LPADC1, ADC_COMMAND_NUMB__IMON_A, &lpadcCommandConfig);

    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = ADC_CHANNEL__IMON_B;
    lpadcCommandConfig.sampleChannelMode = ADC_CHANNEL_MODE__IMON_B;
    lpadcCommandConfig.chainedNextCommandNumber = 0;
    lpadcCommandConfig.hardwareAverageMode = kLPADC_HardwareAverageCount1,
	lpadcCommandConfig.sampleTimeMode = ADC_SAMPLE_TIME;
    LPADC_SetConvCommandConfig(LPADC1, ADC_COMMAND_NUMB__IMON_B, &lpadcCommandConfig);





    /* Set trigger configuration. */
    //	We setup here for a hardware trigger.
    //	When this HW trigger goes off (i.e. the ADC_ETC <- External trigger controller)
    //	It will execute "ADC_USER_CMDID".  This is the first ADC command that will
    //  execute.  These are in a 'chain'.
    LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
    lpadcTriggerConfig.targetCommandId       = ADC_USER_CMDID;
    lpadcTriggerConfig.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(LPADC1, 0U, &lpadcTriggerConfig);

    /* Enable the connection between the ADC and the DMA */
    // specifically, this makes it so that the high water mark flag
    // also turns on the DMA flag WITHIN the ADC module.
    LPADC_EnableFIFOWatermarkDMA(LPADC1, true);


}



// Configuration for the external trigger controller (ETC).
void ADC_ETC_Configuration(void)
{
    adc_etc_config_t adcEtcConfig;
    adc_etc_trigger_config_t adcEtcTriggerConfig;
    adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;

    /* Initialize the ADC_ETC. */
    /* Set the external XBAR trigger0 configuration. */
    ADC_ETC_GetDefaultConfig(&adcEtcConfig);
    adcEtcConfig.XBARtriggerMask = 1U; 					/* Enable the external XBAR trigger0. This is the mask that allows the XBAR switch to trigger the ETC controller*/
    ADC_ETC_Init(ADC_ETC, &adcEtcConfig);

    adcEtcTriggerConfig.enableSyncMode      = false;
    adcEtcTriggerConfig.enableSWTriggerMode = false;	// this is if we were going to trigger it with the use of a SW command.
    adcEtcTriggerConfig.triggerChainLength  = 0;  		/* Chain length is ZERO!!!  This is so critical!  This is not Chains of ADC conversions.  This is chains in the ETC controller */
    adcEtcTriggerConfig.triggerPriority     = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;		// this isn't relevant with B2B mode.
    adcEtcTriggerConfig.initialDelay        = 0U;		// This is a delay we can add after the trigger is fired.  Not useful for us here
    ADC_ETC_SetTriggerConfig(ADC_ETC, ADC_ETC_TRIGGER_GROUP, &adcEtcTriggerConfig);

    /* Set the external XBAR trigger0 chain0 configuration. */
    adcEtcTriggerChainConfig.enableB2BMode       = true; 							//	B2B= back to back... but This bothers me because it reminds me of backstreet boys for whatever reason.
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << ADC_CHANNEL_GROUP; 		/* Select ADC_HC0 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = ADC_ETC_CHANNEL; 					/* ADC_HC0 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = false;								//kADC_ETC_Done0InterruptEnable; /* Enable the Done0 interrupt. */
#if defined(FSL_FEATURE_ADC_ETC_HAS_TRIGm_CHAIN_a_b_IEn_EN) && FSL_FEATURE_ADC_ETC_HAS_TRIGm_CHAIN_a_b_IEn_EN
    adcEtcTriggerChainConfig.enableIrq = false;//true; 								/* Enable the IRQ if you want an ISR when the ETC is done.  We do it after the DMA transfer.  So disable this. */
#endif                                         /* FSL_FEATURE_ADC_ETC_HAS_TRIGm_CHAIN_a_b_IEn_EN */
    /* Configure the trigger group chain 0. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, ADC_ETC_TRIGGER_GROUP, 0U, &adcEtcTriggerChainConfig);

    /* Set NVIC priority - It's a freeRTOS thing */
	NVIC_SetPriority(ADC_ETC_IRQ0_IRQn, 5);			// this is leftover from when we were using the ETC to fire an ISR.  It's harmless to leave it here, and is required if we ever re-enable that for debugging.

}



/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  GPT2_init();
  ACCEL_LPI2C_init();
//  CAMERA_LPI2C_init();
  GPT3_init();				// Timer for udp sync.
  CM7_GPIO3_init();


	/* Initialize LPADC converter */
	LPADC_Configuration();


	/* cross bar switch */
	XBARA_Configuration();

	/* Initialize the External Trigger Controller (ETC) */
	/* This is what makes our ADC get triggered multiple times (i.e. our chain). */
	/* Note that the initial trigger still BEGINS FROM THE XBAR.  The PIT sends the*/
	/* trigger through the XBAR.  The XBAR triggers this, and then this triggers*/
	/* the ADC (multiple times, in our case)									*/
	ADC_ETC_Configuration();





	DMA_configuration();
//	audio_init();

	/* Periodic interrupt timer.  This does the timing for the ADC Samples */
// this needs to be on, it's off now for debugging.
	PIT_Configuration();
//	PIT_ClearStatusFlags(PIT_BASEADDR, PIT_CHANNEL, 1);	// not necessary.  But I left it here for future debug use.


  /* Enable the NVIC. */
  EnableIRQ(ADC_ETC_IRQ0_IRQn);


  PHY_SPI_init();			// LPSPI1

  MICROWAVE_LPUART_init();
}


/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
