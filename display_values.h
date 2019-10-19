/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISPLAY_VALUES_H
#define __DISPLAY_VALUES_H

//fsta
#define VERSION "1.0"

// 6.60 = 3500 /2900

#define SLAVE_BOOT_VER  1500  //indicates micro

#ifndef SELECT_TIV27E
  #define SLAVE_VER       3503  //ver 6.62
#else
  #define SLAVE_VER       2903  //ver 6.62
#endif


#define MAX_RX_END_TIMER 400            //400 * 125us = 50ms   cas ko se caka na rs485 paketek. Potem zapre sprejem
#define MIN_RX_END_TIMER 16             //16 * 125us = 2ms   pred tem casom se caka na naslednji byte, nato zacne z izvajanjem komande


//#define SLAVE_ADDR			0x10			//MODBUS slave address

//system commands
//#define MCD_W_reset	  		0x01			//positioner does a reset

//general infos
#define MCMD_R_dinamic			0x10	//16	//read dinamic parameters.
#define	MCMD_R_static_A		   	0x11	//17	//read static parameters
#define	MCMD_R_static_B		   	0x12	//18	//
#define	MCMD_R_static_C		   	0x13	//19	//
#define	MCMD_R_static_D		   	0x14	//20	//
#define	MCMD_R_static_E		   	0x15	//21	//
#define MCMD_R_static_F                 0x16    //22
#define MCMD_R_static_G                 0x17
#define MCMD_R_static_H                 0x18

#define MCMD_W_value                    0x20    //32    //vpis vrednost
 





#define NO_BROADCAST 	0
#define BROADCAST_CALL	1

#define MCMD_W_mode_switch              0x06
//------ RS485 modbus --------
//system commands
#define MCD_W_reset	  		0x01	//positioner does a reset

//general infos
#define MCMD_R_status		        0x20	//32	//status flags, i.e. errors,..
#define	MCMD_W_status		   	0x21	//33	//clear status flags
#define MCMD_W_slave_addr		0x22	//34	//????? set slave address (broadcast call, the only slave connected??)
#define MCMD_R_Usupply			0x23	//35	//read supply voltage
#define MCMD_R_Imotor		        0x24	//36	//read motor current
#define MCMD_R_serial_numbers	        0x25	//37	//read serial numbers
#define MCMD_R_version			0x26	//38	//read software version
#define MCMD_R_remain_A			0x27	//39	//read how many impulses are left from at the refernce point at last reference doing
#define MCMD_R_remain_B			0x28	//40	
#define MCMD_R_boot_ver			0x29
#define MCMD_R_events			0x2A  //read active events
#define MCMD_R_errorA_stats		0x2B
#define MCMD_R_errorB_stats             0x2C

//commands
#define MCMD_W_stop_motor		0x30	//48	//stop motors immediatelly
#define MCMD_R_position_A		0x31	//49	//read currently position (impulses)
#define MCMD_R_position_B		0x32	//50	//
#define MCMD_R_destination_A	        0x33	//51	//read currently set destination (impulses)
#define MCMD_R_destination_B	        0x34	//52	//
#define MCMD_W_destination_A	        0x35	//53	//go to destination (impulses)
#define MCMD_W_destination_B	        0x36	//54	//
#define MCMD_W_ref_A	  	        0x37	//55	//synchonize by going to reference - clear "position" after getting there
#define MCMD_W_ref_B			0x38	//56	//
#define MCMD_R_AxisState 		0x39
#define MCMD_W_SetAxisState  	        0x40
#define MCMD_R_Hall_cntDown		0x41
#define MCMD_W_Hall_cntDown		0x42

//parameters
#define MCMD_R_min_range_A	        0x50	//80	//read minimum moving limit for axis A (impulses)
#define MCMD_W_min_range_A	        0x51	//81	//
#define MCMD_R_min_range_B		0x52	//82	//minimum moving limit for axis B (impulses)
#define MCMD_W_min_range_B		0x53	//83	//
#define MCMD_R_max_range_A		0x54	//84	//maximum moving limit for axis A (impulses)
#define MCMD_W_max_range_A		0x55	//85	//
#define MCMD_R_max_range_B		0x56	//86	//maximum moving limit for axis B (impulses)
#define MCMD_W_max_range_B		0x57	//87	//
#define MCMD_R_max_Imotor_A		0x58	//88	//Imotor limit for axis A (Amps)	
#define MCMD_W_max_Imotor_A		0x59	//89	//
#define MCMD_R_max_Imotor_B		0x5A	//90	//Imotor limit for axis B (Amps)
#define MCMD_W_max_Imotor_B 	        0x5B	//91	//

#define MCMD_R_gear_ratio_A		0x5C	//92  	//gear ratio
#define MCMD_W_gear_ratio_A		0x5D	//93
#define MCMD_R_gear_ratio_B		0x5E	//94
#define MCMD_W_gear_ratio_B		0x5F	//95

#define MCMD_R_Usupply_factor	        0x62	//98	//Usupply voltage measurement factor
#define MCMD_W_Usupply_factor	        0x63	//99	//
#define MCMD_R_Imotor_factor	        0x64	//100	//Imotor measurement factor
#define MCMD_W_Imotor_factor	        0x65	//101	//
#define MCMD_R_modbus_timeout_position_A	0x66	//102	//rest position A in impulses  (modbus idle)
#define MCMD_W_modbus_timeout_position_A	0x67	//103	//
#define MCMD_R_modbus_timeout_position_B	0x68	//104	//rest position B in impulses (modbus idle)
#define MCMD_W_modbus_timeout_position_B	0x69	//105	//
#define MCMD_R_modbus_timeout			0x6A	//106	//0 - no rest, 1,2,3...4320000 (50 days) - seconds of idle Modbus, after timeout it goes on rest position
#define MCMD_W_modbus_timeout			0x6B	//107	//
#define MCMD_R_modbus_timeout_delay		0x6C	//108	//0 - disable, 1,2,3....4320000 (50 days) - seconds before error appear for no MODbus communication
#define MCMD_W_modbus_timeout_delay		0x6D	//109	//
#define MCMD_R_ref_toolong			0x6E	//110	//60 ....4320000 (50 days) - seconds before too long running to home error 
#define MCMD_W_ref_toolong			0x6F	//111	//

#define MCMD_W_StratI_ratioA   0x70
#define MCMD_R_StartI_ratioA   0x71
#define MCMD_W_StratI_ratioB   0x72
#define MCMD_R_StartI_ratioB   0x73

#define MCMD_W_StratI_timeA    0x74
#define MCMD_R_StartI_timeA    0x75
#define MCMD_W_StratI_timeB    0x76
#define MCMD_R_StartI_timeB    0x77

#define MCMD_R_All_PARAM       0x78

#define MCMD_W_SERIAL_slave_addr  0x79

#define MCMD_W_Voltage_hall       0x7A        // Voltage supply for hall sensors
#define MCMD_R_Voltage_hall       0x7B
#define MCMD_R_Uhall_0            0x7C
#define MCMD_R_Uhall_1            0x7D

#define MCMD_R_Batt_voltage       0x7E

#define MCMD_W_Detection_I_A	  0x80
#define MCMD_R_Detection_I_A	  0x81
#define MCMD_W_Detection_I_B	  0x82
#define MCMD_R_Detection_I_B	  0x83

#define MCMD_R_Invert_motor       0xa0
#define MCMD_W_Invert_motor       0xa1

#define MCMD_R_NC_EndSwitch       0xa2
#define MCMD_W_NC_EndSwitch       0xa3

#define MCMD_W_Lock_Tracker	  0xf0
#define MCMD_W_UnLockTracker	  0xf1
#define MCMD_R_FlashWritCnt	  0xf2

#define MCMD_R_Line_Resistance	  0xf3
#define MCMD_C_Mesure_Line_Res	  0xf4

#define MCMD_R_MaxLine_Resistance 0xf5
#define MCMD_W_MaxLine_Resistance 0xf6

#define MCMD_R_EndSwithDetectA	  0xf7
#define MCMD_W_EndSwithDetectA 	  0xf8
#define MCMD_R_EndSwithDetectB	  0xf9
#define MCMD_W_EndSwithDetectB 	  0xfA

#define MCMD_R_ZeroOffsetA	  0xfB
#define MCMD_W_ZeroOffsetA 	  0xfC
#define MCMD_R_ZeroOffsetB	  0xfD
#define MCMD_W_ZeroOffsetB 	  0xfE

#define MCMD_R_RampA	  	  0x90
#define MCMD_W_RampA 		  0x91
#define MCMD_R_RampB		  0x92
#define MCMD_W_RampB 		  0x93

////////////////////////////////////////////////
/* Slave replies on every Master's MODBUS command
- if OK, reply contains ok + (cmd byte 0xxx xxxx)
- if ERR reply contains error message byte + (cmd byte 1xxx xxxx) */
#define MACK_OK				0x00
#define MACK_UNRECOGNIZED_CMD		0x01
#define MACK_VALUE_OUT_OF_LIMIT		0x02
#define MACK_NOT_USED_DURING_REF	0x03
#define MACK_SEE_STATUS_BYTE		0x04 
////////////////////////////////////////////////



//Prototypes
void USB_display(void);
void USB_write(void);
void modbus_cmd (void);
void Chip_USB_Init(void);
//void modbus_crc(unsigned char *buffer, int length);



//----------------------------------
#endif