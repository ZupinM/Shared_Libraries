/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner LoRa_LoRa
 */

#ifndef __LoRa_H__
#define __LoRa_H__

#include <stdint.h>
#include <stdbool.h>
#include "../main.h"
#include "config.h"

#define LoRa_MAX_PACKET	255
#define LoRa_DEFAULT_TIMEOUT		3000

//#define GREEN 1

#define MASTER 1
#define SLAVE 0
#define DISABLE 2     //bind mode
#define CANCEL 5     //bind mode
#define BY_CHANNEL 3  //bind mode
#define MASTER_BY_CHANNEL 4 //bind mode

#define LORA_ID_MASTER 100

#define CONV_MODE_SLAVE 0
#define CONV_MODE_MASTER 1
#define TRANSMISSION_FINISHED 1
#define PACKET_RECEIVED 2

#define ENABLE_INTERRUPTS 1
#define DISABLE_INTERRUPTS 0

extern uint8_t LoRa_bindMode_master;
extern uint8_t conv_mode;
extern uint8_t LoRa_route[165][MAX_ROUTE_HOPS];
extern uint8_t LoRa_id;

extern volatile uint8_t transmission;
extern volatile uint8_t lora_int_stat;

extern uint8_t LoRa_bindMode_slave;
extern uint8_t LoRa_channel_received;
extern uint8_t checkRouting;
extern uint8_t set_settings_flag;

extern uint8_t LoRa_route[165][MAX_ROUTE_HOPS];

#define LoRa_IRQ_ValidHeader            ( 1 << 4 )
#define LoRa_IRQ_PayloadCrcError	( 1 << 5 )
#define LoRa_IRQ_RxDone                 ( 1 << 6 )
#define LoRa_IRQ_RxTimeout              ( 1 << 7 )

//Error Coding rate (CR)setting
#define LoRa_CR_4_5
//#define LoRa_CR_4_6
//#define LoRa_CR_4_7
//#define LoRa_CR_4_8
#ifdef   LoRa_CR_4_5
#define LoRa_CR	0x01
#else
#ifdef   LoRa_CR_4_6
#define LoRa_CR    0x02
#else
#ifdef   LoRa_CR_4_7
#define LoRa_CR    0x03
#else
#ifdef   LoRa_CR_4_8
#define LoRa_CR    0x04
#endif
#endif
#endif
#endif

//CRC Enable
#define LoRa_CRC_EN
#ifdef  LoRa_CRC_EN
#define LoRa_CRC   0x01
#else
#define LoRa_CRC   0x00
#endif
//RFM98 Internal registers Address
/********************LoRa mode***************************/
#define LR_RegFifo                                  0x00
// Common settings
#define LR_RegOpMode                                0x01
#define LR_RegFrMsb                                 0x06
#define LR_RegFrMid                                 0x07
#define LR_RegFrLsb                                 0x08
// Tx settings
#define LR_RegPaConfig                              0x09
#define LR_RegPaRamp                                0x0A
#define LR_RegOcp                                   0x0B
// Rx settings
#define LR_RegLna                                   0x0C
// LoRa registers
#define LR_RegFifoAddrPtr                           0x0D
#define LR_RegFifoTxBaseAddr                        0x0E
#define LR_RegFifoRxBaseAddr                        0x0F
#define LR_RegFifoRxCurrentaddr                     0x10
#define LR_RegIrqFlagsMask                          0x11
#define LR_RegIrqFlags                              0x12
#define LR_RegRxNbBytes                             0x13
#define LR_RegRxHeaderCntValueMsb                   0x14
#define LR_RegRxHeaderCntValueLsb                   0x15
#define LR_RegRxPacketCntValueMsb                   0x16
#define LR_RegRxPacketCntValueLsb                   0x17
#define LR_RegModemStat                             0x18
#define LR_RegPktSnrValue                           0x19
#define LR_RegPktRssiValue                          0x1A
#define LR_RegRssiValue                             0x1B
#define LR_RegHopChannel                            0x1C
#define LR_RegModemConfig1                          0x1D
#define LR_RegModemConfig2                          0x1E
#define LR_RegSymbTimeoutLsb                        0x1F
#define LR_RegPreambleMsb                           0x20
#define LR_RegPreambleLsb                           0x21
#define LR_RegPayloadLength                         0x22
#define LR_RegMaxPayloadLength                      0x23
#define LR_RegHopPeriod                             0x24
#define LR_RegFifoRxByteAddr                        0x25
#define LR_RegModemConfig3                          0x26
// I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
// Version
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64

/********************FSK/ook mode***************************/
#define  RegFIFO                0x00
#define  RegOpMode              0x01
#define  RegBitRateMsb      	0x02
#define  RegBitRateLsb      	0x03
#define  RegFdevMsb             0x04
#define  RegFdevLsb             0x05
#define  RegFreqMsb             0x06
#define  RegFreqMid             0x07
#define  RegFreqLsb         	0x08
#define  RegPaConfig            0x09
#define  RegPaRamp              0x0a
#define  RegOcp                 0x0b
#define  RegLna                 0x0c
#define  RegRxConfig            0x0d
#define  RegRssiConfig      	0x0e
#define  RegRssiCollision 		0x0f
#define  RegRssiThresh      	0x10
#define  RegRssiValue           0x11
#define  RegRxBw                0x12
#define  RegAfcBw               0x13
#define  RegOokPeak             0x14
#define  RegOokFix              0x15
#define  RegOokAvg              0x16
#define  RegAfcFei              0x1a
#define  RegAfcMsb              0x1b
#define  RegAfcLsb              0x1c
#define  RegFeiMsb              0x1d
#define  RegFeiLsb              0x1e
#define  RegPreambleDetect  	0x1f
#define  RegRxTimeout1      	0x20
#define  RegRxTimeout2      	0x21
#define  RegRxTimeout3      	0x22
#define  RegRxDelay             0x23
#define  RegOsc                 0x24
#define  RegPreambleMsb     	0x25
#define  RegPreambleLsb     	0x26
#define  RegSyncConfig      	0x27
#define  RegSyncValue1      	0x28
#define  RegSyncValue2      	0x29
#define  RegSyncValue3      	0x2a
#define  RegSyncValue4      	0x2b
#define  RegSyncValue5      	0x2c
#define  RegSyncValue6      	0x2d
#define  RegSyncValue7      	0x2e
#define  RegSyncValue8      	0x2f
#define  RegPacketConfig1       0x30
#define  RegPacketConfig2       0x31
#define  RegPayloadLength       0x32
#define  RegNodeAdrs            0x33
#define  RegBroadcastAdrs       0x34
#define  RegFifoThresh      	0x35
#define  RegSeqConfig1      	0x36
#define  RegSeqConfig2      	0x37
#define  RegTimerResol      	0x38
#define  RegTimer1Coef      	0x39
#define  RegTimer2Coef      	0x3a
#define  RegImageCal            0x3b
#define  RegTemp                0x3c
#define  RegLowBat              0x3d
#define  RegIrqFlags1           0x3e
#define  RegIrqFlags2           0x3f
#define  RegDioMapping1			0x40
#define  RegDioMapping2			0x41
#define  RegVersion				0x42
#define  RegPllHop				0x44
#define  RegPaDac				0x4d
#define  RegBitRateFrac			0x5d

/**********************************************************
 **Parameter table define
 **********************************************************/
#define LoRa_433MHZ			0

//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0xD0, 0x00 }, }; //435 250 000 Hz
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x80, 0x00 }, }; //434MHz
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x48, 0x00 }, }; //433.125
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x53, 0x33 }, }; //433.300
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x56, 0x66 }, }; //433.350
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x5E, 0x66 }, }; //433.475
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x69, 0x9A }, }; //433.650
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x6B, 0x33 }, }; //433.675  +200
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x6E, 0x66 }, }; //433.725  	2x
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x6A, 0x66 }, }; //433.6625  	1.5x
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x76, 0x66 }, }; //433.850		3x
//static const uint8_t LoRa_Frequency[1][3] = { { 0x6C, 0x74, 0xCD }, }; //433.825


//static const uint8_t LoRa_Frequency[1][3] = { { 0xD9, 0x00, 0x00 }, };   //868MHz

#define LoRa_POWER_20DBM		0
#define LoRa_POWER_17DBM		1
#define LoRa_POWER_14DBM		2
#define LoRa_POWER_11DBM		3

static const uint8_t LoRa_Power[4] = { 0xFF, //20dbm
		0xFC, //17dbm
		0xF9, //14dbm
		0xF6, //11dbm
		};

#define LoRa_SF_6		6
#define LoRa_SF_7		7
#define LoRa_SF_8		8
#define LoRa_SF_9		9
#define LoRa_SF_10		10
#define LoRa_SF_11		11
#define LoRa_SF_12		12

static const uint8_t LoRa_SpreadFactor[7] = { 6, 7, 8, 9, 10, 11, 12 };

#define LoRa_BW_7_8KHZ		0
#define LoRa_BW_10_4KHZ		1
#define LoRa_BW_15_6KHZ		2
#define LoRa_BW_20_8KHZ		3
#define LoRa_BW_31_2KHZ		4
#define LoRa_BW_41_7KHZ		5
#define LoRa_BW_62_5KHZ		6
#define LoRa_BW_125KHZ		7
#define LoRa_BW_250KHZ		8
#define LoRa_BW_500KHZ		9

static const uint8_t LoRa_LoRaBandwidth[10] = { 0, //   7.8KHz,
		1, //  10.4KHz,
		2, //  15.6KHz,
		3, //  20.8KHz,
		4, //  31.2KHz,
		5, //  41.7KHz,
		6, //  62.5KHz,
		7, // 125.0KHz,
		8, // 250.0KHz,
		9  // 500.0KHz
		};

#define TxMode 1
#define RxMode 0		

typedef enum _LoRa_STATUS {
	SLEEP, STANDBY, TX, RX
} LoRa_Status_t;

typedef struct {
	int pin;
	void * port;
} LoRa_hw_dio_t;

typedef struct {
	LoRa_hw_dio_t reset;
	LoRa_hw_dio_t dio0;
	LoRa_hw_dio_t nss;
	void * spi;
} LoRa_hw_t;

typedef struct {
	LoRa_hw_t * hw;

	uint32_t frequency;
	uint8_t channel;
	uint8_t power;
	uint8_t spFactor;
	uint8_t LoRa_BW;
	uint8_t packetLength;

	LoRa_Status_t status;

	uint8_t rxBuffer[LoRa_MAX_PACKET];
	uint8_t readBytes;
	uint8_t packetReady;
    uint8_t routed;
} LoRa_t;

extern uint8_t transceiver;
extern LoRa_t module;
extern LoRa_t original;

//logic

void LoRa_Interrupt(uint8_t enable_disable);
void LoRa_standby(void);
void LoRa_sleep(void);
void LoRa_clearIrq(void);
void LoRa_reset(void);
uint8_t LoRa_config(uint8_t channel, uint8_t power, uint8_t LoRa_Rate, uint8_t LoRa_BW, uint8_t packetLength, uint8_t mode);
int LoRa_EntryRx(uint8_t length, uint32_t timeout);
int LoRa_EntryTx(uint8_t length, uint32_t timeout);	
uint8_t LoRa_tx_finished(void);
uint8_t LoRa_rx_finished(void);
void LoRa_TxPacket(uint8_t* txBuffer, uint8_t length, uint32_t timeout); 
void LoRa_Bind_Mode(uint8_t mode);
void rx_single(void);
void rx_continued(void);
uint8_t lookup_returned_length(uint8_t command);
uint8_t lookup_second_packet_length(uint8_t cmd);
void check_routing(void);
uint8_t LoRa_get_rssi(void);

void tx_finished(void);
void rx_finished(void);

void set_LED(uint8_t color, uint8_t state, uint16_t timeout);



//POŠILJANJE IZ SIGME

static const uint8_t cmd_tx_length4[64] = {
0x20,
0x21,
0x23,
0x24,
0x25,
0x26,
0x29,
0x2A,
0x2B,
0x2C,
0x78,
0xF5,
0x37,
0x38,
0x30,
0x39,
0x41,
0xA0,
0x50,
0x52,
0x54,
0x56,
0xFB,
0xFD,
0x5C,
0x5E,
0x90,
0x92,
0x58,
0x5A,
0x71,
0x73,
0x75,
0x77,
0x81,
0x83,
0xF7,
0xF9,
0x66,
0x68,
0x6A,
0x6C,
0x6E,
0x62,
0x64,
0x43,
0xB3,
0xB5,
0xB7,
0xBF,
0xB1,
0x7B,
0x7C,
0x85,
0x86,
0x87,
0x89,
0x8A,
0x8B,
0x8D,
0x8F,
0x05,
0x06,
0XBC//
};

static const uint8_t cmd_tx_length5[2]={0x22, 0xB0};
static const uint8_t cmd_tx_length7[1]={0xB2};

static const uint8_t cmd_tx_length8[43]={
0xF6,
0x3A,
0x3B,
0x40,
0x42,
0xA1,
0x51,
0x53,
0x55,
0x57,
0xFC,
0xFE,
0x5D,
0x5F,
0x91,
0x93,
0x59,
0x5B,
0x70,
0x72,
0x74,
0x76,
0x80,
0x82,
0xF8,
0xFA,
0x67,
0x69,
0x6B,
0x6D,
0x6F,
0x63,
0x65,
0x44,
0xF0,
0xF1,
0x7A,
0x84,
0x88,
0x8C,
0x8E
};

static const uint8_t cmd_tx_length9[3]={0x79,0x35,0x36};

static const uint8_t cmd_tx_length19[1]={0x0c};

static const uint8_t cmd_tx_length21[1]={0xB4};//



//SPREJEM NA SIGMO

static const uint8_t cmd_rx_length5[13]={
0x21,
0x22,
0x79,
0x37,
0x38,
0x30,
0x06,
0xB0,//
0xB2,//
0xB5,//
0x35,
0x36
};


static const uint8_t cmd_rx_length7[3]={
0xB1,//
0xB7,//
0xBF//
};


static const uint8_t cmd_rx_length8[47]={
0x23,
0x24,
0x26,
0x2A,
0x39,
0x41,
0xA0,
0x50,
0x52,
0x54,
0x56,
0xFB,
0xFD,
0x5C,
0x5E,
0x90,
0x92,
0x58,
0x5A,
0x71,
0x73,
0x75,
0x77,
0x81,
0x83,
0xF7,
0xF9,
0x66,
0x68,
0x6A,
0x6C,
0x6E,
0x62,
0x64,
0xF5,
0x43,
0x7B,
0x7C,
0x85,
0x86,
0x87,
0x89,
0x8A,
0x8B,
0x8D,
0x8F,  
0xB3//
};

static const uint8_t cmd_rx_length9[43]={  //(NANOD) ali 5 (NANOD-error+ostali):
0x51,
0x53,
0x55,
0x57,
0xFC,
0xFE,
0x59,
0x5B,
0x63,
0x65,
0x67,
0x69,
0x5D,
0x5F,
0x70,
0x72,
0x74,
0x76,
0x80,
0x82,
0xF6,
0x91,
0x93,
0xF8,
0xFA,
0x40,
0x44,
0x42,
0x6B,
0x6D,
0x6F,
0x3A,
0x3B,
0xA1,
0xF0,
0xF1,
0x7A,
0x84,
0x88,
0x8C,
0x8E,
};

static const uint8_t cmd_rx_length12[2]={0x20, 0xB4};
static const uint8_t cmd_rx_length20[3]={0x25, 0x29, 0xBC};
static const uint8_t cmd_rx_length36[2]={0x2B, 0x2C};
static const uint8_t cmd_rx_length108[1]={0x78};



#endif
