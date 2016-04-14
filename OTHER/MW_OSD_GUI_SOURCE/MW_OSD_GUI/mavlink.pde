float mavdata_roll=0;
float mavdata_pitch=0;
int mavdata_airspeed=0;
int mavdata_groundspeed=0;
int mavdata_altitude=0;
int mavdata_heading=0;
int mavdata_climbrate=0;
int mavdata_throttle=0;
int mavdata_rssi=0;
int mavdata_gps_fixtype=0;
int mavdata_gps_sats=0;
int mavdata_gps_lat=0;
int mavdata_gps_lon=0;
int mavdata_voltage=0;
int mavdata_amperage=0;
int mavdata_remaining=0;
int mavdata_sensors=0;


int mav_sequence=0;
int mavmillis=0;
int mav_magic=50;
int mav_message_length;
int mav_custom_mode=0;
int mav_base_mode=0;

long mav_serial_checksum=0;
  final int
    MAVLINK_MSG_ID_HEARTBEAT=0,
    MAVLINK_MSG_ID_HEARTBEAT_MAGIC=50,
    MAVLINK_MSG_ID_HEARTBEAT_LEN=9,
    MAVLINK_MSG_ID_VFR_HUD=74,
    MAVLINK_MSG_ID_VFR_HUD_MAGIC=20,
    MAVLINK_MSG_ID_VFR_HUD_LEN=20,
    MAVLINK_MSG_ID_ATTITUDE=30,
    MAVLINK_MSG_ID_ATTITUDE_MAGIC=39,
    MAVLINK_MSG_ID_ATTITUDE_LEN=28,
    MAVLINK_MSG_ID_GPS_RAW_INT=24,
    MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC=24,
    MAVLINK_MSG_ID_GPS_RAW_INT_LEN=30,
    MAVLINK_MSG_ID_RC_CHANNELS_RAW=35,
    MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC=244,
    MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN=22,    
    MAVLINK_MSG_ID_SYS_STATUS=1,
    MAVLINK_MSG_ID_SYS_STATUS_MAGIC=124,
    MAVLINK_MSG_ID_SYS_STATUS_LEN=31  
    ;

void SendCommandMAVLINK(int cmd){
//  System.out.println("CMD:"+ cmd);
    
  int custom_mode=mav_custom_mode; ///< A bitfield for use for autopilot-specific flags.
  int type=1; ///< Type of the MAV 1 - plane / 2 copter)
  int autopilot=3; ///< Autopilot type / class. defined in MAV_AUTOPILOT ENUM
  int base_mode=mav_base_mode; ///< System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
//80=armed ??
  int system_status=4; ///< System status flag, see MAV_STATE ENUM
  int mavlink_version=255; ///< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
  
  int icmd = (int)(cmd&0xFF);
  switch(icmd) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      PortIsWriting = true;
      mav_message_length=MAVLINK_MSG_ID_HEARTBEAT_LEN;
      mav_magic=MAVLINK_MSG_ID_HEARTBEAT_MAGIC;
      mav_headserial(MAVLINK_MSG_ID_HEARTBEAT);
      mav_serialize32(mav_custom_mode);
      mav_serialize8(type);
      mav_serialize8(autopilot);
      mav_serialize8(base_mode);
      mav_serialize8(system_status);
      mav_serialize8(mavlink_version);
      mav_tailserial();
      PortIsWriting = false;
    break;
    case MAVLINK_MSG_ID_ATTITUDE:
      PortIsWriting = true;
      mav_message_length=MAVLINK_MSG_ID_ATTITUDE_LEN;
      mav_magic=MAVLINK_MSG_ID_ATTITUDE_MAGIC;
      mav_headserial(MAVLINK_MSG_ID_ATTITUDE);
      mav_serialize32(millis());
      mav_serializefloat(mavdata_roll); //rad roll
      mav_serializefloat(mavdata_pitch); //rad  pitch
      mav_serializefloat(0);
      mav_serializefloat(0);
      mav_serializefloat(0);
      mav_serializefloat(0);
      mav_tailserial();
      PortIsWriting = false;
    break;
    case MAVLINK_MSG_ID_VFR_HUD: 
      PortIsWriting = true;
      mav_message_length=MAVLINK_MSG_ID_VFR_HUD_LEN;
      mav_magic=MAVLINK_MSG_ID_VFR_HUD_MAGIC;
      mav_headserial(MAVLINK_MSG_ID_VFR_HUD);
      mav_serializefloat(mavdata_airspeed); //AS m/s
      mav_serializefloat(mavdata_groundspeed); //GS m/s
      mav_serializefloat(mavdata_altitude); //Alt m
      mav_serializefloat(mavdata_climbrate); //Climb m/s
      mav_serialize16(mavdata_heading); //Heading deg
      mav_serialize16(mavdata_throttle); //throttle
      mav_tailserial();
      PortIsWriting = false;
    break;
    case MAVLINK_MSG_ID_GPS_RAW_INT: 
      PortIsWriting = true;
      mav_message_length=MAVLINK_MSG_ID_GPS_RAW_INT_LEN;
      mav_magic=MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC;
      mav_headserial(MAVLINK_MSG_ID_GPS_RAW_INT);
      mav_serialize32(0);
      mav_serialize32(0); // 2nd - make up to 64 bit
      mav_serialize32(mavdata_gps_lat); //lat
      mav_serialize32(mavdata_gps_lon); //lon
      mav_serialize32(0);
      mav_serialize16(0);
      mav_serialize16(0);
      mav_serialize16(mavdata_groundspeed);
      mav_serialize16(mavdata_heading);
      mav_serialize8(mavdata_gps_fixtype&0xFF);  //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
      mav_serialize8(mavdata_gps_sats); //sats     
      mav_tailserial();
      PortIsWriting = false;
    break;
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: 
      PortIsWriting = true;
      mav_message_length=MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN;
      mav_magic=MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC;
      mav_headserial(MAVLINK_MSG_ID_RC_CHANNELS_RAW);
      mav_serialize32(0);
      mav_serialize16(int(Pitch_Roll.arrayValue()[0])); //CH0
      mav_serialize16(int(Pitch_Roll.arrayValue()[1]));
      mav_serialize16(int(Throttle_Yaw.arrayValue()[0]));
      mav_serialize16(int(Throttle_Yaw.arrayValue()[1]));
      mav_serialize16(1100);
      mav_serialize16(1100);
      mav_serialize16(1100);
      mav_serialize16(1100);  //CH7
      mav_serialize8(1);
      mav_serialize8(mavdata_rssi);  //RSSI
      mav_tailserial();
      PortIsWriting = false;
    break;
    case MAVLINK_MSG_ID_SYS_STATUS: 
      PortIsWriting = true;
      mav_message_length=MAVLINK_MSG_ID_SYS_STATUS_LEN;
      mav_magic=MAVLINK_MSG_ID_SYS_STATUS_MAGIC;
      mav_headserial(MAVLINK_MSG_ID_SYS_STATUS);
      mav_serialize32(0);
      mav_serialize32(mavdata_sensors);
      mav_serialize32(0);
      mav_serialize16(0);
      mav_serialize16(mavdata_voltage); // units mv
      mav_serialize16(mavdata_amperage); // units 10ma
      mav_serialize16(0);
      mav_serialize16(0);
      mav_serialize16(0);
      mav_serialize16(0);
      mav_serialize16(0);
      mav_serialize16(0);
      mav_serialize8(mavdata_remaining); //% remaining
      mav_tailserial();
      PortIsWriting = false;
    break;
  }
}


void mav_headserial(int msg) {
  if (mav_sequence>254){
    mav_sequence=0;
  }
  else{
    mav_sequence++;
  }
  mav_serial_checksum=0xFFFF; //init
  serialize8ncs(0xFE);
  mav_serialize8(mav_message_length);
  mav_serialize8(mav_sequence&0xFF);
  mav_serialize8(1);
  mav_serialize8(1);
  mav_serialize8(msg&0xFF);  
}


void mav_tailserial(){
  mav_checksum(mav_magic);
  serialize8ncs((int)mav_serial_checksum&0xFF);
  serialize8ncs((int)(mav_serial_checksum>>8)&0xFF);
}


void serialize8ncs(int val) {
  try{
    g_serial.write(val);
  } 
  catch(java.lang.Throwable t) {
    System.out.println( t.getClass().getName() );
    t.printStackTrace();
  }
}


void mav_serialize8(int val) {
  mav_checksum(val);
  try{
    g_serial.write(val);
  } 
  catch(java.lang.Throwable t) {
    System.out.println( t.getClass().getName() );
    t.printStackTrace();
  }
}


void mav_checksum(int val) {
  long tmp;
  tmp = val ^ mav_serial_checksum &0xFF;
  tmp ^= (tmp<<4)&0xFF;
  mav_serial_checksum = (mav_serial_checksum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);  
}


void mav_serialize16(int a) {
  if (str(a)!=null ){
  mav_serialize8((a   ) & 0xFF);
  mav_serialize8((a>>8) & 0xFF);
  }
}


void mav_serialize32(int a) {
  if (str(a)!=null ){
    mav_serialize8((a    ) & 0xFF);
    mav_serialize8((a>> 8) & 0xFF);
    mav_serialize8((a>>16) & 0xFF);
    mav_serialize8((a>>24) & 0xFF);
  } 
}


void mav_serializefloat(float b) {
  int a = Float.floatToIntBits(b);
  if (str(a)!=null ){
    mav_serialize8((a    ) & 0xFF);
    mav_serialize8((a>> 8) & 0xFF);
    mav_serialize8((a>>16) & 0xFF);
    mav_serialize8((a>>24) & 0xFF);
  } 
}


/*
unsigned char * serialize_float(unsigned char *buffer, float value) 
{ 
    unsigned int ivalue = *((unsigned int*)&value); // warning assumes 32-bit "unsigned int"
    buffer[0] = ivalue >> 24;  
    buffer[1] = ivalue >> 16;  
    buffer[2] = ivalue >> 8;  
    buffer[3] = ivalue;  
    return buffer + 4; 
} 
*/

void process_mav_send(){
  syncmav();
  if ((int(SimControlToggle.getValue())!=0)&&(Simtype==2)) {
    if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_HEARTBEAT);
    MSP_sendOrder++;
    switch(MSP_sendOrder) {
      case 1:
        if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_ATTITUDE);
        break;
      case 2:
        if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_VFR_HUD);
        break;
      case 3:
        if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_GPS_RAW_INT); 
        break;
      case 4:
        if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_RC_CHANNELS_RAW);
        break;
      case 5:
        if (init_com==1)SendCommandMAVLINK(MAVLINK_MSG_ID_SYS_STATUS); 
        MSP_sendOrder=0;
        break;
      default:  
        MSP_sendOrder=0;
    }
    PortWrite = !PortWrite; // toggle TX LED every other    
  } 
}

void syncmav(){

// armed 
  mav_base_mode=0;
  if(toggleModeItems[0].getValue()> 0){ //armed
    mav_base_mode |=1<<7;
  }

// flight mode
  mav_custom_mode=0;
  if(toggleModeItems[1].getValue()> 0){//stab
    mav_custom_mode =2;
  }
  if(toggleModeItems[6].getValue()> 0){//RTH
    mav_custom_mode =11;
  }
  if(toggleModeItems[7].getValue()> 0){//HOLD
    mav_custom_mode =1; //circle
  }
  
// sensors active
  mavdata_sensors=0;
  if(toggleModeItems[1].getValue()> 0){//gyro 0
    mavdata_sensors |=1<<1;
  }
  if(toggleModeItems[1].getValue()> 0){//acc 1
    mavdata_sensors |=1<<1;
  }
  if(toggleModeItems[3].getValue()> 0){//mag 2
    mavdata_sensors |=1<<2;
  }
  if(toggleModeItems[4].getValue()> 0){//baro 3      
    mavdata_sensors |=1<<3;
  }
  //for others https://github.com/geeksville/arduleader/blob/master/thirdparty/org.mavlink.library/generated/org/mavlink/messages/MAV_SYS_STATUS_SENSOR.java 
  
  
  
  mavdata_roll=radians(int(MW_Pitch_Roll.arrayValue()[0]));
  mavdata_roll= -mavdata_roll;
  mavdata_pitch=radians(int(MW_Pitch_Roll.arrayValue()[1]));
//  System.out.println("Roll:"+ mavdata_roll);
//  System.out.println("Pitch:"+ mavdata_pitch);
  mavdata_airspeed=int(SGPS_speed.value())/100; //actual used on OSD
  mavdata_groundspeed=int(SGPS_speed.value())/100;
  mavdata_altitude=int(SGPS_altitude.value()/100);
  mavdata_heading=MwHeading;
  mavdata_climbrate=int(sVario);
//  System.out.println("Vario:"+ mavdata_climbrate);

  mavdata_throttle=int(map(Throttle_Yaw.arrayValue()[1],1000,2000,0,100));
  mavdata_rssi=int((sMRSSI*100)/1023);
  mavdata_gps_fixtype=int(SGPS_FIX.arrayValue()[0]);
  mavdata_gps_sats=int(SGPS_numSat.value());
  mavdata_gps_lat=GPSstartlat;
  mavdata_gps_lon=GPSstartlon;
//  System.out.println("Lat:"+ mavdata_gps_lat);
//  System.out.println("Lon:"+ mavdata_gps_lon);
  mavdata_voltage=int(sVBat * 1000);
  mavdata_amperage=mavdata_throttle*100;
  mavdata_remaining=66;
/*
        int(MW_Pitch_Roll.arrayValue()[0])*10) //pitch
        int(MW_Pitch_Roll.arrayValue()[1])*10) /roll
        MwHeading. 
        headSerialReply(MSP_ANALOG, 7);
        serialize8(int(sVBat * 10));
        serialize16(0);
        serialize16(int(sMRSSI));
        serialize16(int(map(Throttle_Yaw.arrayValue()[1],1000,2000,0,50000)));
        serialize8(wpno);
        headSerialReply(MSP_RAW_GPS,16);
        serialize8(int(SGPS_FIX.arrayValue()[0]));
        serialize8(int(SGPS_numSat.value()));
        GPSstartlat=GPSstartlat+100;
        GPSstartlon=GPSstartlon-100;
        serialize32(GPSstartlat);
        serialize32(GPSstartlon);
        serialize16(int(SGPS_altitude.value()/100));
        serialize16(int(SGPS_speed.value()));
        serialize16(MwHeading*10);     
        if(confItem[GetSetting("S_GPSTIME")].value()>0)
          headSerialReply(MSP_COMP_GPS,9);
        else
        headSerialReply(MSP_COMP_GPS,5);
        serialize16(int(SGPS_distanceToHome.value()));
        int GPSheading = int(SGPSHeadHome.value());
        if(GPSheading < 0) GPSheading += 360;
        serialize16(GPSheading);
        serialize8(0);
        serialize32(osdtime);
        serialize32(int(sAltitude) *100);
        serialize16(int(sVario) *10);     
*/
        GPSstartlat=GPSstartlat+100;
        GPSstartlon=GPSstartlon-100;

}


/*
apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);
                    uav_flightmode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                    if (apm_mav_type == 2)    //ArduCopter MultiRotor or ArduCopter Heli
                    {                         
                        switch (uav_flightmode) {
                            case 0: uav_flightmode = 2;   break;      //Stabilize 
                            case 1: uav_flightmode = 1;   break;      //Acro 
                            case 2: uav_flightmode = 8;   break;      //Alt Hold
                            case 3: uav_flightmode = 10;  break;      //Auto
                            case 4: uav_flightmode = 10;  break;      //Guided -> Auto
                            case 5: uav_flightmode = 9;   break;      //Loiter
                            case 6: uav_flightmode = 13;  break;      //RTL
                            case 7: uav_flightmode = 12;  break;      //Circle
                            case 8: uav_flightmode = 9 ;  break;      //Position -> Loiter
                            case 9: uav_flightmode = 15;  break;      //Land
                            case 10: uav_flightmode = 9;  break;      //OF LOiter
                            case 11: uav_flightmode = 5;  break;      //Drift -> Stabilize1
                            case 12: uav_flightmode = 6;  break;      //Sport -> Stabilize2
                            default: uav_flightmode = 19; break;      //Unknown                      
                        }
                    }
                    else if (apm_mav_type == 1)    //ArduPlane
                    {
                        switch (uav_flightmode) {
                            case 0:                       break;       //Manual
                            case 1:  uav_flightmode = 12; break;       //Circle
                            case 2:                       break;       //Stabilize
                            case 5:  uav_flightmode = 16; break;       //FlyByWire A
                            case 6:  uav_flightmode = 17; break;       //FlyByWire B
                            case 10:                      break;       //Auto
                            case 11: uav_flightmode = 13; break;       //RTH
                            case 12: uav_flightmode = 9;  break;       //Loiter
                            default: uav_flightmode = 19; break;       //Unknown
                        }
                    }
                    //Mode (arducoper armed/disarmed)
                    uav_arm = mavlink_msg_heartbeat_get_base_mode(&msg);
                    if (getBit(uav_arm,7)) 
                        uav_arm = 1;
                    else 
                        uav_arm = 0;
 */
 
