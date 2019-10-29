/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ASCIIBOX_H
#define __ASCIIBOX_H

                                        //RS485 paketi:
#define cpcb_version            37      //S1
#define cversion                38      //S1
#define cUsolar                 39      //Dinamic
#define chours                  40      //Dinamic
#define cminutes                41      //Dinamic
#define cseconds                42      //Dinamic
#define cdate                   43      //S1
#define cshow_angle_A           44      //Dinamic
#define cposition_A             45      //Dinamic
#define cdestination_A          46      //Dinamic
#define cImotor_A               47      //Dinamic
#define cshow_angle_B           48      //Dinamic
#define cposition_B             49      //Dinamic
#define cdestination_B          50      //Dinamic
#define cImotor_B               51      //Dinamic
#define clatitude               52      //S1
#define clongitude              53      //S1
#define cinterval               54      //S1
#define cboot                   55      // <--- write only (upgrading iz Helios-a)
#define cMode                   56      //Dinamic
#define cNavigation             57
#define cService                58      //S1
#define cmonth                  59      //S1
#define cyear                   60
#define ctarget_H               61      //S1
#define ctarget_V               62      //S1
#define cremain_A               63      //Dinamic
#define cremain_B               64      //Dinamic

//--------------- parametri ----------------
#define cA1_A                   70      //S2
#define cA2_A                   71      //S2
#define cA3_A                   72      //S2
#define cA4_A                   73      //S2
#define cA5_A                   74      //S2
#define cA6_A                   75      //S2
#define cB1_A                   76      //S2
#define cB2_A                   77      //S2
#define cgear_ratio_A           78      //S2
#define cmax_range_A            79      //S2
#define ccoordinate_mode_A      80      //S2
#define cgroup                  81      //S2
#define reserved_1              82      //S2
#define cgeometry_mode_A        83      //S2
#define cusolar_factor          84      //S2
#define cimotor_factor_A        85      //S2
#define ccflags                 86      //S3 -
#define cID_number              87      //S5
#define cslave_id               90      //S3
#define cbuyflags               91      //S3
#define crun_delay_home         92      //S3
#define crun_delay              93      //S3
#define cWidePanelA             94      //S3
#define cSpacePanelA            95      //S3
#define cA1_B                   96      //S3
#define cA2_B                   97      //S3
#define cA3_B                   98      //S3
#define cA4_B                   99      //S3
#define cA5_B                   100     //S3
#define cA6_B                   101     //S3
#define cB1_B                   102     //S4 -
#define cB2_B                   103     //S4
#define cgear_ratio_B           104     //S4
#define cmax_range_B            105     //S4
#define ccoordinate_mode_B      106     //S4
#define cgeometry_mode_B        107     //S4
#define chome_position_A        108     //S4
#define chome_position_B        109     //S4
#define cmin_range_A            110     //S4
#define cmin_range_B            111     //S4
#define cmax_Imotor_A           112     //S4
#define cmax_Imotor_B           113      //S4
#define cimotor_factor_B        114      //S4
#define clink_ok                115      //Dinamic
#define crtc_correction         116      //S4
#define cgoref_Nday_A           117      //S4
#define cgoref_Nday_B           118      //S5 -
#define cWindSpeed              119      //S5
#define cWindSpeedThreshold     120      //S5
#define cWindWaitOn             121      //S5 <- trenutno brez veze parameter
#define cWindDestinationA       122      //S5
#define cWindDestinationB       123      //S5
#define cWindFactor             124      //S5
#define cWindSensorType         125      //S5 <- trenutno bRrez veze parameter
#define cSnowDestinationA       126      //S5
#define cSnowDestinationB       127      //S5
#define cFocusSensorOutputA     128      //Dinamic
#define cFocusSensorOutputB     129      //Dinamic
#define cFocusMiddleA           130      //S1
#define cFocusMiddleB           131      //S1
#define reserved11              132      //S5
#define cWindWaitOff            133      //S1
#define cFocusOffsetA           134     //S1
#define cFocusOffsetB           135     //S1
#define cStatus                 136     //S5
#define cWidePanelB             137     //
#define cSpacePanelB            138     //
//S
#define cOverTempShift          139
#define cTime_out_of_focus      140
///S

#define cInrush_ratioA          141
#define cInrush_ratioB          142
#define cInrush_timeA           143
#define cInrush_timeB           144
#define cfocus_max_offset       145
#define cfocus_fine_offset      146


#define cdeviation_A      147
#define cinclination_A    148
#define cpanel_space_A    149
#define cpanel_width_A    150
#define cpanel_thick_A    151

#define cdeviation_B      152
#define cinclination_B    153
#define cpanel_space_B    154
#define cpanel_width_B    155
#define cpanel_thick_B    156

#define ctime_zone        157
#define csunrise          158
#define csunset           159

#define cNightMode_time   160
#define cDayMode_time     161

#define cHTarget2   162
#define cVTarget2   163
#define cHTarget3   164
#define cVTarget3   165

#define cHeliostatP1_start 166
#define cHeliostatP1_end   167
#define cHeliostatP2_start 168
#define cHeliostatP2_end   169
#define cHeliostatP3_start 170
#define cHeliostatP3_end   171

#define cSoftRTCCorrection 172

#define cGMT_sec  173
#define cGMT_min  174
#define cGMT_hour 175
#define cGMT_day  176
#define cGMT_mon  177
#define cGMT_year 178

#define clastSync_time     179
#define ctimeDst           180
#define csdate             181
#define csmonth            182
#define csyear             183
#define cshours            184
#define csminutes          185
#define csseconds          186

#define cmotor_speedA      187
#define cmotor_speedB      188

#define covervoltageOcc    189
#define chour_angle        190
#define celevation         191

#define czero_offsetA      192
#define czero_offsetB      193

#define cnormally_close_es 194

#define cVoltage_select_0	195
#define cVoltage_select_1	196
#define cUVccHALL_0			197
#define cUVccHALL_1			198
#define cBatteryVoltage		199

#define clflags      200
#define clock_pin    201
#define cUnlock      202
#define cLock        203

#define cbldc_Speed        204
#define cnumber_of_poles   205





//----------------------------------
#endif /*__ASCIIBOX_H*/