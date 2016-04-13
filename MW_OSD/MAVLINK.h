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

uint8_t mav_message_length;
uint8_t mav_message_cmd;
uint16_t mav_serial_checksum;

#define  LAT  0
#define  LON  1
#define  GPS_BAUD BAUDRATE
uint32_t GPS_home_timer=0;
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
//  uint16_t GPS_ground_course = 0;                       
int16_t  GPS_altitude_home;                            
uint8_t  GPS_Present = 0;                             
//  uint8_t  GPS_SerialInitialised=5;
uint8_t  GPS_armedangleset = 0;
uint8_t  GPS_active=5; 
uint8_t  GPS_fix_HOME=0;
const char satnogps_text[] PROGMEM = " NO GPS ";


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

void serialMAVCheck(){
  //testing only...
  //  GPS_fix=3;
  //  GPS_numSat=6;
  armed=1;
  configMode==1;

  switch(mav_message_cmd) {
  case MAVLINK_MSG_ID_HEARTBEAT:
#ifdef MSPACTIVECHECK
    timer.MSP_active=MSPACTIVECHECK; // getting valid MAV on serial port
#endif
    //    armed = (MwSensorActive & 0x80) != 0;  //armed status check
    debug[0]=armed;
    //debug[1]=GPS_altitude;
    //debug[2]=MwAltitude;
    //debug[3]=MwVario;

    break;
  case MAVLINK_MSG_ID_VFR_HUD:
    GPS_speed=(int16_t)serialbufferfloat(4)*100;    // m/s-->cm/s 
    GPS_altitude=(int16_t)serialbufferfloat(8);     // m-->m
    GPS_altitude=GPS_altitude-0;
    MwAltitude = (int32_t) GPS_altitude *100;       // m--cm gps to baro
    MwHeading=serialBuffer[16]|serialBuffer[17]<<8; // deg (-->deg*10 if GPS heading)
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

    GPS_latitude =serialbufferint(8);    // decimal 10,000,000
    GPS_longitude=serialbufferint(12);  // decimal 10,000,000
    /*   
     if ((GPS_fix>2) && (GPS_numSat >= MINSATFIX)) {
     if (GPS_fix_HOME == 0){
     GPS_reset_home_position();
     }
     GPS_fix_HOME=1;
     if (!GPS_fix_HOME) {
     GPS_distanceToHome = 0;
     GPS_directionToHome = 0;
     GPS_altitude = 0 ;
     MwAltitude = 0 ;          
     }
     //      GPS_speed=serialBuffer[24]|(serialBuffer[25]<<8);
     
     //calculate distance. bearings etc
     uint32_t dist;
     int32_t  dir;
     //      GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
     //      GPS_distanceToHome = dist/100;
     //      GPS_directionToHome = dir/100;
     //      GPS_altitude =  GPS_altitude- GPS_altitude_home;
     //      MwAltitude = (int32_t)GPS_altitude *100;
     int16_t MwHeading360=GPS_ground_course/10;
     if (MwHeading360>180)
     MwHeading360 = MwHeading360-360;
     MwHeading   = MwHeading360;
     gpsvario(); 
     
     // set armed direction (review as prob not needed...
     if (GPS_armedangleset==0)  
     armedangle=MwHeading;
     if (GPS_distanceToHome>GPSOSDARMDISTANCE){
     GPS_armedangleset = 1;
     armed=1;
     }
     
     if (GPS_armedangleset==1){
     if ((GPS_distanceToHome<GPSOSDHOMEDISTANCE)&&(GPS_speed<75)){
     if ((GPS_home_timer+7000)>millis()){
     }
     else if((GPS_home_timer+22000)>millis()){
     configPage=0;
     armed=0;
     }      
     else{
     configMode=0;
     GPS_armedangleset=0;
     previousarmedstatus=0;
     }      
     }
     else {
     GPS_home_timer=millis();
     }    
     }
     }    
     */
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    MwRssi=(uint16_t)serialBuffer[21]*1024/100;
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = (int16_t)(serialBuffer[4+(i*2)]|(serialBuffer[5+(i*2)]<<8));
    handleRawRC();
    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    MwVBat=serialBuffer[14]|(serialBuffer[15]<<8)/100;
    MWAmperage=serialBuffer[16]|(serialBuffer[17]<<8);
    break;
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
    if (mav_payload_index==mav_message_length){  // CKbyte1
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
        serialMAVCheck(); // valid packet go MAVshit
      }
      mav_state = MAV_IDLE;
    }
  }
}



void GPS_reset_home_position() {
  GPS_home[LAT] = GPS_coord[LAT];
  GPS_home[LON] = GPS_coord[LON];
  GPS_altitude_home = GPS_altitude;
  //  GPS_calc_longitude_scaling(GPS_coord[LAT]);  //need an initial value for distance and bearing calc
  GPS_fix_HOME = 1;
}









