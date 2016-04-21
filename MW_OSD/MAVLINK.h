#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_MAGIC 50
#define AVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_VFR_HUD_MAGIC 20
#define MAVLINK_MSG_ID_VFR_HUD_LEN 20
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_MAGIC 39
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC 244
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22    
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_MAGIC 124
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31  
#define  LAT  0
#define  LON  1

uint8_t  mav_message_length;
uint8_t  mav_message_cmd;
uint16_t mav_serial_checksum;
int32_t  GPS_home[2];
int16_t  GPS_altitude_home;                            
uint8_t  GPS_fix_HOME;
float    GPS_scaleLonDown;


void mav_checksum(uint8_t val) {
  uint16_t tmp;
  tmp = val ^ mav_serial_checksum &0xFF;
  tmp ^= (tmp<<4)&0xFF;
  mav_serial_checksum = (mav_serial_checksum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);  
}


float serialbufferfloat(uint8_t offset){
  float f_temp;
  byte * b = (byte *) &f_temp;
  for(uint8_t i=0;i<4;i++){
    b[i]=serialBuffer[offset+i];
  }
  return f_temp;
}


int32_t serialbufferint(uint8_t offset){
  int32_t i_temp;
  byte * b = (byte *) &i_temp;
  for(uint8_t i=0;i<4;i++){
    b[i]=serialBuffer[offset+i];
  }
  return i_temp;
}


void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;

  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}


void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  GPS_scaleLonDown = cos(rads);
}


void GPS_reset_home_position() {
  GPS_home[LAT] = GPS_latitude;
  GPS_home[LON] = GPS_longitude;
  GPS_altitude_home = GPS_altitude;
//  GPS_calc_longitude_scaling(GPS_home[LAT]);
}


void serialMAVCheck(){
#ifdef MSPACTIVECHECK
  timer.MSP_active=MSPACTIVECHECK; // getting valid MAV on serial port
#endif //MSPACTIVECHECK
  int16_t MwHeading360;
  uint8_t apm_mav_type=0;
  uint8_t osd_mode=serialbufferint(0);
  String mode_str;
  switch(mav_message_cmd) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    mode.armed      = (1<<0);
    mode.gpshome    = (1<<4);
    mode.gpshold    = (1<<5);
    mode.gpsmission = (1<<6);
    MwSensorActive&=0xFFFFFF8E;
    apm_mav_type=serialBuffer[4];   
        if (apm_mav_type == 2){ //ArduCopter MultiRotor or ArduCopter Heli
        if (osd_mode == 0)       mode_str = "stab"; //Stabilize: hold level position
        else if (osd_mode == 1)  mode_str = "acro"; //Acrobatic: rate control
        else if (osd_mode == 2)  mode_str = "alth"; //Altitude Hold: auto control
        else if (osd_mode == 3)  mode_str = "auto"; //Auto: auto control
        else if (osd_mode == 4)  mode_str = "guid"; //Guided: auto control
        else if (osd_mode == 5)  mode_str = "loit"; //Loiter: hold a single location
        else if (osd_mode == 6)  mode_str = "retl"; //Return to Launch: auto control
        else if (osd_mode == 7)  mode_str = "circ"; //Circle: auto control
        else if (osd_mode == 8)  mode_str = "posi"; //Position: auto control
        else if (osd_mode == 9)  mode_str = "land"; //Land:: auto control
        else if (osd_mode == 10) mode_str = "oflo"; //OF_Loiter: hold a single location using optical flow sensor
        else if (osd_mode == 11) mode_str = "drif"; //Drift mode: 
        else if (osd_mode == 13) mode_str = "sprt"; //Sport: earth frame rate control
        else if (osd_mode == 14) mode_str = "flip"; //Flip: flip the vehicle on the roll axis
        else if (osd_mode == 15) mode_str = "atun"; //Auto Tune: autotune the vehicle's roll and pitch gains
        else if (osd_mode == 16) mode_str = "hybr"; //Hybrid: position hold with manual override
    } else if(apm_mav_type == 1){ //ArduPlane
        if (osd_mode == 0)       mode_str = "manu"; //Manual
        else if (osd_mode == 1)  mode_str = "circ"; //Circle
        else if (osd_mode == 2)  mode_str = "stab"; //Stabilize
        else if (osd_mode == 3)  mode_str = "trng"; //Training
        else if (osd_mode == 4)  mode_str = "acro"; //Acro
        else if (osd_mode == 5)  mode_str = "fbwa"; //Fly_By_Wire_A
        else if (osd_mode == 6)  mode_str = "fbwb"; //Fly_By_Wire_B
        else if (osd_mode == 7)  mode_str = "crui"; //Cruise
        else if (osd_mode == 8)  mode_str = "atun"; //Auto Tune
        else if (osd_mode == 10) mode_str = "auto"; //Auto
        else if (osd_mode == 11) mode_str = "retl"; //Return to Launch
        else if (osd_mode == 12) mode_str = "loit"; //Loiter
        else if (osd_mode == 15) mode_str = "guid"; //Guided
        else if (osd_mode == 16) mode_str = "init"; //Initializing
    }
    if (serialbufferint(0)==11)      //RTH
      MwSensorActive|=(1<<4);
    if (serialbufferint(0)==1)       //HOLD
      MwSensorActive|=(1<<5);
    if (serialbufferint(0)==99)      //MISSION
      MwSensorActive|=(1<<6);
    if (serialBuffer[6]&(1<<7)){     //armed
      MwSensorActive|=(1<<0);
      armed=1;
    }
    else{
      armed=0;
      GPS_fix_HOME=0;
    }
    break;
  case MAVLINK_MSG_ID_VFR_HUD:
    GPS_speed=(int16_t)serialbufferfloat(4)*100;    // m/s-->cm/s 
    GPS_altitude=(int16_t)serialbufferfloat(8);     // m-->m
    if (GPS_fix_HOME == 0){
      GPS_reset_home_position();
    }
    GPS_altitude=GPS_altitude - GPS_altitude_home;
    MwAltitude = (int32_t) GPS_altitude *100;       // m--cm gps to baro
    MwHeading=serialBuffer[16]|serialBuffer[17]<<8; // deg (-->deg*10 if GPS heading)
    MwHeading360=MwHeading;
    if (MwHeading360>180)
      MwHeading360 = MwHeading360-360;
    MwHeading   = MwHeading360;
    MwVario=(int16_t)serialbufferfloat(12)*100;     // m/s-->cm/s
    break;
  case MAVLINK_MSG_ID_ATTITUDE:
    MwAngle[0]=(int16_t)(serialbufferfloat(4)*57296/10);     // rad-->0.1deg
    MwAngle[1]=(int16_t)(serialbufferfloat(8)*57296/10);     // rad-->0.1deg
    break;
  case MAVLINK_MSG_ID_GPS_RAW_INT:
#ifdef GPSACTIVECHECK
    timer.GPS_active=GPSACTIVECHECK;
#endif //GPSACTIVECHECK
    GPS_numSat=serialBuffer[29];                                                                         
    GPS_fix=serialBuffer[28];                                                                            
    GPS_ground_course = (serialBuffer[26]|(serialBuffer[27]<<8))/10;
    GPS_latitude =serialbufferint(8);
    GPS_longitude=serialbufferint(12);
    if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX)) {
      uint32_t dist;
      int32_t  dir;
      GPS_distance_cm_bearing(&GPS_latitude,&GPS_longitude,&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
      GPS_distanceToHome = dist/100;
      GPS_directionToHome = dir/100;
      //      GPS_altitude =  GPS_altitude- GPS_altitude_home;
      //      MwAltitude = (int32_t)GPS_altitude *100;
//      int16_t MwHeading360=GPS_ground_course/10;
//      if (MwHeading360>180)
//        MwHeading360 = MwHeading360-360;
 //     MwHeading   = MwHeading360;
//      gpsvario(); 
    } 
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    MwRssi=(uint16_t)(((103)*serialBuffer[21])/10);
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = (int16_t)(serialBuffer[4+(i*2)]|(serialBuffer[5+(i*2)]<<8));
    handleRawRC();
    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    mode.stable = 2;
    mode.baro   = 4;
    mode.mag    = 8;
    MwSensorActive&=0xFFFFFFF1;
    if ((serialbufferint(4)&(1<<1))>0) //acc
      MwSensorActive|=(1<<1);
    if ((serialbufferint(4)&(1<<3))>0) //baro1
      MwSensorActive|=(1<<2);
    if ((serialbufferint(4)&(1<<4))>0) //baro2
      MwSensorActive|=(1<<2);
    if ((serialbufferint(4)&(1<<2))>0) //mag
      MwSensorActive|=(1<<3);
    MwVBat=(serialBuffer[14]|(serialBuffer[15]<<8))/100;
    MWAmperage=serialBuffer[16]|(serialBuffer[17]<<8);
    break;
  }
  if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX) && armed){
    GPS_fix_HOME = 1;
  }
  else{
    GPS_altitude = 0 ;
    MwAltitude = 0 ;          
    GPS_distanceToHome = 0;
    GPS_directionToHome = 0;
    GPS_fix_HOME = 0;
  }
}


void serialMAVreceive(uint8_t c)
{
  static uint8_t mav_payload_index; 
  static uint16_t mav_checksum_rcv; 

  static enum _serial_state {
    MAV_IDLE,
    MAV_HEADER_START,
    MAV_HEADER_LEN,
    MAV_HEADER_SEQ,
    MAV_HEADER_SYS,
    MAV_HEADER_COMP,
    MAV_HEADER_MSG,
    MAV_PAYLOAD,
    MAV_CHECKSUM,
  }

  mav_state = MAV_IDLE;

  if ((mav_state == MAV_IDLE)||(mav_state == MAV_PAYLOAD))
  {
  }
  else
  {
    mav_checksum(c);
  }

  if (mav_state == MAV_IDLE)
  {
    if (c==0xFE)
    {
      mav_serial_checksum=0xFFFF;
      mav_payload_index=0;
      mav_state = MAV_HEADER_START;
    }
    else
    {
      mav_state = MAV_IDLE;
    } 
  }
  else if (mav_state == MAV_HEADER_START)
  {
    mav_message_length = c;
    mav_state = MAV_HEADER_LEN;
    if ((mav_payload_index) > SERIALBUFFERSIZE){  // too much data so reset check
      mav_state = MAV_IDLE;
    }
  }
  else if (mav_state == MAV_HEADER_LEN)
  {
    mav_state = MAV_HEADER_SEQ;
  }
  else if (mav_state == MAV_HEADER_SEQ)
  {
    mav_state = MAV_HEADER_SYS;
  }
  else if (mav_state == MAV_HEADER_SYS)
  {
    mav_state = MAV_HEADER_COMP;
  }
  else if (mav_state == MAV_HEADER_COMP)
  {
    mav_message_cmd = c;
    mav_state = MAV_HEADER_MSG;
  }
  else if (mav_state == MAV_HEADER_MSG) // header received, its a packet!
  {
    serialBuffer[mav_payload_index]=c;
    mav_payload_index++;
    if (mav_payload_index==mav_message_length){  // end of data
      mav_state = MAV_PAYLOAD;
    }

  }
  else if (mav_state == MAV_PAYLOAD)
  {
    if (mav_payload_index==mav_message_length){
      mav_checksum_rcv=c;
      mav_payload_index++;
    }
    else{
      mav_checksum_rcv+=(c<<8);
      int8_t mav_magic;
      switch(mav_message_cmd) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        mav_magic = MAVLINK_MSG_ID_HEARTBEAT_MAGIC;
        break;
      case MAVLINK_MSG_ID_VFR_HUD:
        mav_magic = MAVLINK_MSG_ID_VFR_HUD_MAGIC;
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        mav_magic = MAVLINK_MSG_ID_ATTITUDE_MAGIC;
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        mav_magic = MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC;
        break;
      case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        mav_magic = MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC;
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        mav_magic = MAVLINK_MSG_ID_SYS_STATUS_MAGIC;
        break;
      }

      mav_checksum(mav_magic);
      if(mav_checksum_rcv == mav_serial_checksum) {
        serialMAVCheck();
      }
      mav_state = MAV_IDLE;
    }
  }
}




























