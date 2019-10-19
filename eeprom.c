#include "eeprom.h"

/* IAP - flash write (parameters backup) */
unsigned int command[5]; 	//IAP - spremenljivki
unsigned int result[5];
float        flash_backup[0x100]; 	//polje spremenljivk, ki se zapisejo v flash
unsigned int FlashWriteCounter;

#define IAP_LOCATION 0x03000205UL

typedef void (*IAP)(unsigned int [],unsigned int[]);
IAP iap_entry;


void read_SN(){
  iap_entry = (IAP)IAP_LOCATION; //nastavi IAP lokacijo 

   __disable_irq();
  // read unique ID via IAP
  command[0] = 58;
  iap_entry (command, result);
  SN[0] = result[1];      //result [0] je rezultat branja, naprej so prebrani SN-ji
  SN[1] = result[2];
  SN[2] = result[3];
  SN[3] = result[4];
  __enable_irq();
}

/***********************************************************
  FLASH READ/WRITE

  sektor 15 - 0x00F000...0x00FFFF, 
  brise se cel sektor (ker se mora), prvih 256 byte-ov se zapise

************************************************************/
void flash_read (unsigned int read_address) {

  unsigned int *p = (unsigned int *)flash_backup;         // parameters
  unsigned int *addr = (unsigned int *)(read_address);				
  unsigned char size = 255;

  do {
    *p++ = *addr++;
  }
  while (--size);	

  // system settings          
  update_flash_backup();

  __disable_irq();
  // read unique ID via IAP
  command[0] = 58;
  iap_entry (command, result);
  SN[0] = result[1];   		//result [0] je rezultat branja, naprej so prebrani SN-ji
  SN[1] = result[2];
  SN[2] = result[3];
  SN[3] = result[4];
  __enable_irq();
}

void eeprom_read (unsigned int read_address) {

  unsigned int *p = (unsigned int *)flash_backup;         // parameters
  unsigned int *addr = (unsigned int *)(read_address);				
  unsigned int size = 0x100;

  do {
    *p++ = *addr++;
  }
  while (--size);	

  __disable_irq();
  // read unique ID via IAP
  command[0] = 62;
  command[1] = read_address;                   //address					
  command[2] = (unsigned int)&flash_backup;
  command[3] = 0x400;				//256 bytes to write (manj ne gre)
  command[4] = SystemCoreClock / 1000;
  iap_entry (command, result);

  __enable_irq();

  // system settings          
  update_flash_backup();

  tracker_status    = flash_backup_ui[2];
  tracker_exstatus  = flash_backup_ui[3];

  bldc_motor *m = bldc_Motor(0);
  bldc_motor *mb = bldc_Motor(1);
   m->position                 = flash_backup[19];
  mb->position                 = flash_backup[40];

  cflags=(int)f_cflags;                        //configuration flags (u32) <- parametri (double)
  bflags=(int)f_bflags;
  buyflags=(int)f_buyflags;                    //buying flags (u32) <- parametri (double)

  FocusMiddleA=(int)f_FocusMiddleA;                 //externi light sensor
  FocusMiddleB=(int)f_FocusMiddleB;

  // last sync time
  lastSyncTime.seconds = (int)lastSync_time % 100;
  lastSyncTime.minutes = ((int)lastSync_time / 100) % 100;
  lastSyncTime.hours = (int)lastSync_time / 10000;
  lastSyncTime.date = (int)lastSync_date % 100;
  lastSyncTime.month = ((int)lastSync_date / 100) % 100;
  lastSyncTime.year = (int)lastSync_year;

}

void update_flash_backup(){
  slave_addr = flash_backup[0];       
  FlashWriteCounter = flash_backup_ui[1];
  //tracker_status    = flash_backup_ui[2];
  //tracker_exstatus  = flash_backup_ui[3];
  bldc_initStatus( flash_backup_ui[4] );

  events            = flash_backup_ui[6];
  
  err_currentA      = flash_backup[7];
  err_positionA     = flash_backup[8];
  err_voltageA      = flash_backup[9];

  max_line_resistance = flash_backup[10];
  //auto_state     = flash_backup_ui[11];

  modbus_timeout       = flash_backup_ui[12];
  modbus_timeout_delay = flash_backup_ui[13];

  // Motor A driving settings
  bldc_motor *m = bldc_Motor(0);

  bldc_config()->UConvertRatio  = flash_backup[14];         
  bldc_config()->IConvertRatio  = flash_backup[15];
  bldc_config()->homing_timeout = flash_backup_ui[16];
  bldc_config()->BConvertRatio  = flash_backup[58];
  bldc_config()->HConvertRatio  = flash_backup[59];
  bldc_config()->H1ConvertRatio  = flash_backup[60];

  m->state                    = flash_backup[17]; 
  m->status                   = flash_backup[18]; 
  m->position                 = flash_backup[19]; 
  m->min_position             = flash_backup[20];
  m->max_position             = flash_backup[21];
  m->Idetection               = flash_backup[22];
  m->I_limit                  = flash_backup[23];   
  m->I_Inrush_ratio           = flash_backup[24];
  m->I_Inrush_time            = flash_backup[25];
  m->modbus_timeout_position  = flash_backup[26];
  m->gear_ratio               = flash_backup[27];
     
  m->home_remaining           = flash_backup[28];   
  m->home_offset              = flash_backup[29];
  m->end_switchDetect         = flash_backup[30];

  m->pid.pgain                = flash_backup[31];
  m->pid.igain                = flash_backup[32]; 
  m->pid.dgain                = flash_backup[33];
  m->pid.deadband             = flash_backup[34];

  // Motor B driving settings
  bldc_motor *mb = bldc_Motor(1);

  err_currentB      = flash_backup[35];
  err_positionB     = flash_backup[36];
  err_voltageB      = flash_backup[37];

  mb->state                    = flash_backup[38];  
  mb->status                   = flash_backup[39];  
  mb->position                 = flash_backup[40];  
  mb->min_position             = flash_backup[41];
  mb->max_position             = flash_backup[42];
  mb->Idetection               = flash_backup[43];
  mb->I_limit                  = flash_backup[44];    
  mb->I_Inrush_ratio           = flash_backup[45];
  mb->I_Inrush_time            = flash_backup[46];
  mb->modbus_timeout_position  = flash_backup[47];
  mb->gear_ratio               = flash_backup[48];
       
  mb->home_remaining           = flash_backup[49];    
  mb->home_offset              = flash_backup[50];
  mb->end_switchDetect         = flash_backup[51];
  
  mb->pid.pgain                = flash_backup[52];
  mb->pid.igain                = flash_backup[53];
  mb->pid.dgain                = flash_backup[54];
  mb->pid.deadband             = flash_backup[55];

  //longitude                    = flash_backup[57];
  //latitude                     = flash_backup[67];
  //time_zone                    = flash_backup[118];

  voltage_select_0             = flash_backup[61];
  voltage_select_1             = flash_backup[62];

  LoRa_id                      = flash_backup[63];
  module.channel               = flash_backup[64];
  module.power                 = flash_backup[65];
  module.spFactor              = flash_backup[66]; 
  module.LoRa_BW               = flash_backup[67];
  transceiver_saved            = flash_backup[68];

  ES_0_normallyOpenLo = ((unsigned int)flash_backup[149]) & (1 << 0);
  ES_1_normallyOpenLo = (((unsigned int)flash_backup[149]) >> 1) & (1 << 0);
  ES_0_normallyOpenHi = (((unsigned int)flash_backup[149]) >> 2) & (1 << 0);
  ES_1_normallyOpenHi = (((unsigned int)flash_backup[149]) >> 3) & (1 << 0);

//fsta
//debug_printf("r ES_0_normallyOpenLo:%d ES_0_normallyOpenHi:%d ES_1_normallyOpenLo:%d ES_1_normallyOpenHi:%d\n", ES_0_normallyOpenLo,ES_0_normallyOpenHi,ES_1_normallyOpenLo,ES_1_normallyOpenHi);


  if(f_pcb_version == 0)
    f_pcb_version = 0x1E4601;

  pcb_version3=(int)f_pcb_version;             //verzija TIV (u8) <- parametri (double)
  pcb_version2=(int)f_pcb_version/0x100;
  pcb_version1=(int)f_pcb_version/0x10000;

  reset_status = flash_backup[56];
}

void flash_erase() {

  __disable_irq();

  // prepare sectors
  command[0] = 50;		// ukaz "prepare"
  command[1] = 63;  		// zacetni sektor 63
  command[2] = 63;		// koncni sektor 63
  iap_entry (command, result);
  // while (result[0]) goto iap_error;

  // erase sectors
  command[0] = 52;		// ukaz "erase"
  command[1] = 63;  		// zacetni sektor 63
  command[2] = 63;		// koncni sektor 63
  command[3] = 48000;	   	// 48.000kHz main clock
  iap_entry (command, result);
  // while (result[0]) goto iap_error;
	
  __enable_irq();
}

void eeprom_write(unsigned int write_address) {

//fsta
//debug_printf("write_address: %x\n",  write_address);

  // upgrade indicator
  int upgrExe = 0;
  
  // system settings
  if(flash_backup[0] != (float)slave_addr) {
    flash_backup[0] = slave_addr;
    upgrExe = 1;
  }
  if(flash_backup_ui[1] != FlashWriteCounter) {		
    flash_backup_ui[1] = FlashWriteCounter;
    upgrExe = 1;
  }
  if(flash_backup_ui[2] != tracker_status) {
    flash_backup_ui[2] = tracker_status;
    upgrExe = 1;
  }
  if(flash_backup_ui[3] != tracker_exstatus) {
    flash_backup_ui[3] = tracker_exstatus;
    upgrExe = 1;
  }
  if(flash_backup_ui[4] != bldc_Status()) {
    flash_backup_ui[4] = bldc_Status();
    upgrExe = 1;
  }
  if(flash_backup_ui[6] != events) {
    flash_backup_ui[6] = events;
    upgrExe = 1;
  }
	
  if(flash_backup[7] != err_currentA) {
    flash_backup[7] = err_currentA;
    upgrExe = 1;
  }
  if(flash_backup[8] != err_positionA) {
    flash_backup[8] = err_positionA;
    upgrExe = 1;
  }
  if(flash_backup[9] != err_voltageA) {
    flash_backup[9] = err_voltageA;
    upgrExe = 1;
  }

  if(flash_backup[10] != max_line_resistance) {
    flash_backup[10] = max_line_resistance;
    upgrExe = 1;
  }
//  if(flash_backup_ui[11] != auto_state) {
//    flash_backup_ui[11] = auto_state;
//    upgrExe = 1;
//  }

  if(flash_backup_ui[12] != modbus_timeout) {
    flash_backup_ui[12] = modbus_timeout;
    upgrExe = 1;
  }
  if(flash_backup_ui[13] != modbus_timeout_delay) {
    flash_backup_ui[13] = modbus_timeout_delay;
    upgrExe = 1;
  }

  // Motor A driving settings
  bldc_motor *m = bldc_Motor(0);

  if(flash_backup[14] != bldc_config()->UConvertRatio) {			
    flash_backup[14] = bldc_config()->UConvertRatio;
    upgrExe = 1;
  }			
  if(flash_backup[15] != bldc_config()->IConvertRatio) {
    flash_backup[15] = bldc_config()->IConvertRatio;
    upgrExe = 1;
  }
  if(flash_backup_ui[16] != bldc_config()->homing_timeout) {
    flash_backup_ui[16] = bldc_config()->homing_timeout;
    upgrExe = 1;
  }
  if(flash_backup[58] != bldc_config()->BConvertRatio) {
    flash_backup[58] = bldc_config()->BConvertRatio;
    upgrExe = 1;
  }
  if(flash_backup[59] != bldc_config()->HConvertRatio) {
    flash_backup[59] = bldc_config()->HConvertRatio;
    upgrExe = 1;
  }
  if(flash_backup[60] != bldc_config()->H1ConvertRatio) {
    flash_backup[60] = bldc_config()->H1ConvertRatio;
    upgrExe = 1;
  }

  if(flash_backup[17] != (float)m->state) {
    flash_backup[17] = m->state;
    upgrExe = 1;
  }
  if(flash_backup[18] != (float)m->status) {
    flash_backup[18] = m->status;
    upgrExe = 1;
  }

//fsta
//debug_printf("%f %f   %f %f\n", flash_backup[19], (float)m->position, flash_backup[20], m->min_position);

  if(flash_backup[19] != (float)m->position) {
    flash_backup[19] = m->position;
    upgrExe = 1;
  }
  if(flash_backup[20] != m->min_position) {
    flash_backup[20] = m->min_position;
    upgrExe = 1;
  }
  if(flash_backup[21] != m->max_position) {
    flash_backup[21] = m->max_position;
    upgrExe = 1;
  }
  if(flash_backup[22] != m->Idetection) {
    flash_backup[22] = m->Idetection;
    upgrExe = 1;
  }
  if(flash_backup[23] != m->I_limit) {
    flash_backup[23] = m->I_limit;
    upgrExe = 1;
  }
  if(flash_backup[24] != m->I_Inrush_ratio) {
    flash_backup[24] = m->I_Inrush_ratio;
    upgrExe = 1;
  }
  if(flash_backup[25] != m->I_Inrush_time) {
    flash_backup[25] = m->I_Inrush_time;
    upgrExe = 1;
  }
  if(flash_backup[26] != m->modbus_timeout_position) {
    flash_backup[26] = m->modbus_timeout_position;
    upgrExe = 1;
  }
  if(flash_backup[27] != m->gear_ratio) {
    flash_backup[27] = m->gear_ratio;
    upgrExe = 1;
  }
     
  if(flash_backup[28] != (float)m->home_remaining) {
    flash_backup[28] = m->home_remaining;
    upgrExe = 1;
  }
  if(flash_backup[29] != m->home_offset) {
    flash_backup[29] = m->home_offset;
    upgrExe = 1;
  }
  if(flash_backup[30] != m->end_switchDetect) {
    flash_backup[30] = m->end_switchDetect;
    upgrExe = 1;
  }

  if(flash_backup[31] != m->pid.pgain) {
    flash_backup[31] = m->pid.pgain;
    upgrExe = 1;
  }
  if(flash_backup[32] != m->pid.igain) {
    flash_backup[32] = m->pid.igain;
    upgrExe = 1;
  }
  if(flash_backup[33] != m->pid.dgain) {
    flash_backup[33] = m->pid.dgain;
    upgrExe = 1;
  }
  if(flash_backup[34] != (float)m->pid.deadband) {
    flash_backup[34] = m->pid.deadband;
    upgrExe = 1;
  }

  // Motor B driving settings
  bldc_motor *mb = bldc_Motor(1);

  if(flash_backup[35] != err_currentB) {
    flash_backup[35] = err_currentB;
    upgrExe = 1;
  }
  if(flash_backup[36] != err_positionB) {
    flash_backup[36] = err_positionB;
    upgrExe = 1;
  }
  if(flash_backup[37] != err_voltageB) {
    flash_backup[37] = err_voltageB;
    upgrExe = 1;
  }

  if(flash_backup[38] != (float)mb->state) {
    flash_backup[38] = mb->state;
    upgrExe = 1;
  }
  if(flash_backup[39] != (float)mb->status) {
    flash_backup[39] = mb->status;
    upgrExe = 1;
  }
  if(flash_backup[40] != (float)mb->position) {
    flash_backup[40] = mb->position;
    upgrExe = 1;
  }
  if(flash_backup[41] != mb->min_position) {
    flash_backup[41] = mb->min_position;
    upgrExe = 1;
  }
  if(flash_backup[42] != mb->max_position) {
    flash_backup[42] = mb->max_position;
    upgrExe = 1;
  }
  if(flash_backup[43] != mb->Idetection) {
    flash_backup[43] = mb->Idetection;
    upgrExe = 1;
  }
  if(flash_backup[44] != mb->I_limit) {
    flash_backup[44] = mb->I_limit;
    upgrExe = 1;
  }
  if(flash_backup[45] != mb->I_Inrush_ratio) {
    flash_backup[45] = mb->I_Inrush_ratio;
    upgrExe = 1;
  }
  if(flash_backup[46] != mb->I_Inrush_time) {
    flash_backup[46] = mb->I_Inrush_time;
    upgrExe = 1;
  }
  if(flash_backup[47] != mb->modbus_timeout_position) {
    flash_backup[47] = mb->modbus_timeout_position;
    upgrExe = 1;
  }
  if(flash_backup[48] != mb->gear_ratio) {
    flash_backup[48] = mb->gear_ratio;
    upgrExe = 1;
  }
       
  if(flash_backup[49] != (float)mb->home_remaining) {
    flash_backup[49] = mb->home_remaining;
    upgrExe = 1;
  }
  if(flash_backup[50] != mb->home_offset) {
    flash_backup[50] = mb->home_offset;
    upgrExe = 1;
  }
  if(flash_backup[51] != mb->end_switchDetect) {
    flash_backup[51] = mb->end_switchDetect;
    upgrExe = 1;
  }
  
  if(flash_backup[52] != mb->pid.pgain) {
    flash_backup[52] = mb->pid.pgain;
    upgrExe = 1;
  }
  if(flash_backup[53] != mb->pid.igain) {
    flash_backup[53] = mb->pid.igain;
    upgrExe = 1;
  }
  if(flash_backup[54] != mb->pid.dgain) {
    flash_backup[54] = mb->pid.dgain;
    upgrExe = 1;
  }
  if(flash_backup[55] != (float)mb->pid.deadband) {
    flash_backup[55] = mb->pid.deadband;
    upgrExe = 1;
  }

//fsta
//debug_printf("AAA %d   %f %f\n", upgrExe, flash_backup[56], (float)reset_status);

  if(flash_backup[56] != (float)reset_status) {
    flash_backup[56] = reset_status;
    upgrExe = 1;
  }

  if(flash_backup[61] != (float)voltage_select_0) {
    flash_backup[61] = voltage_select_0;
    upgrExe = 1;
  }
  if(flash_backup[62] != (float)voltage_select_1) {
    flash_backup[62] = voltage_select_1;
    upgrExe = 1;
  }

  if(flash_backup[63] != (float)LoRa_id) {
    flash_backup[63] = LoRa_id;
    upgrExe = 1;
  }
  if(flash_backup[64] != (float)module.channel) {
    flash_backup[64] = module.channel;
    upgrExe = 1;
  }
  if(flash_backup[65] != (float)module.power) {
    flash_backup[65] = module.power;
    upgrExe = 1;
  }
  if(flash_backup[66] != (float)module.spFactor) {
    flash_backup[66] = module.spFactor;
    upgrExe = 1;
  }
  if(flash_backup[67] != (float)module.LoRa_BW) {
    flash_backup[67] = module.LoRa_BW;
    upgrExe = 1;
  }
  if(flash_backup[68] != (float)transceiver) {
    flash_backup[68] = transceiver;
    upgrExe = 1;
  }

//fsta
//debug_printf("w ES_0_normallyOpenLo:%d ES_0_normallyOpenHi:%d ES_1_normallyOpenLo:%d ES_1_normallyOpenHi:%d\n", ES_0_normallyOpenLo,ES_0_normallyOpenHi,ES_1_normallyOpenLo,ES_1_normallyOpenHi);

  if(flash_backup[149] != (float)(ES_0_normallyOpenLo + (ES_1_normallyOpenLo << 1) + (ES_0_normallyOpenHi << 2) + (ES_1_normallyOpenHi << 3))) {
    flash_backup[149] = ES_0_normallyOpenLo + (ES_1_normallyOpenLo << 1) + (ES_0_normallyOpenHi << 2) + (ES_1_normallyOpenHi << 3);
    upgrExe = 1;
  }

  if(f_cflags != (float)cflags) {
    f_cflags = cflags;                        //configuration flags (u32) -> parametri (double)
    upgrExe = 1;
  }
  if(f_bflags != (float)bflags) {
    f_bflags = bflags;
    upgrExe = 1;
  }
  if(f_buyflags != buyflags) {
    f_buyflags = buyflags;                    //buying flags (u32) -> parametri (double)
    upgrExe = 1;
  }

  // last sync time
  if(lastSync_year != (float)lastSyncTime.year) {
    lastSync_year = lastSyncTime.year;
    upgrExe = 1;
  }
  if(lastSync_date != (float)(lastSyncTime.month * 100 + lastSyncTime.date)) {
    lastSync_date = lastSyncTime.month * 100 + lastSyncTime.date;
    upgrExe = 1;
  }
  if(lastSync_time != (float)(lastSyncTime.hours * 10000 + lastSyncTime.minutes * 100 + lastSyncTime.seconds)) {
    lastSync_time = lastSyncTime.hours * 10000 + lastSyncTime.minutes * 100 + lastSyncTime.seconds;
    upgrExe = 1;
  }
     
  // no changes
  if(!upgrExe)
    return;
  __disable_irq();

  // write
  command[0] = 61;				//ukaz "write"
  command[1] = write_address;                   //address					
  command[2] = (unsigned int)&flash_backup;
  command[3] = 0x400;				//256 bytes to write (manj ne gre)
  command[4] = SystemCoreClock / 1000;
 // iap_entry (command, result);
// for(double nm=0 ; nm<=10000000 ; nm++)
//  asm("");
  iap_entry(command, result);
  while (result[0])
    goto iap_error;

  __enable_irq();
  return;

iap_error:					//napaka pri zapisu v flash - rdeca LED
  // GPIOSetValue( PORT_2,RED,1 );
  __enable_irq();
  return;

}