#include <ICM_20948.h>  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#include "acl_serial.h"


#define SPI_FREQ 6000000 // ICM20948 has max 7 MHz SPI, but >6 MHz breaks
#define CS_PIN 10        // Which pin you connect chip select (CS) to

ICM_20948_SPI myICM;     // SPI object to talk to ICM20948 IMU

//=============================================================================
// configuration options
//=============================================================================

// sensor polling interval (micros)
uint32_t SENSOR_POLL_INTERVAL_US = 1000; // default, can be changed online
// note that ICM20948 has max Fs,accel = 4500 Hz; Fs,gyro = 9000 Hz


//=============================================================================
// global variables
//=============================================================================

// serial stuff
uint8_t out_buf[ACL_SERIAL_MAX_MESSAGE_LEN];
acl_serial_message_t msg_buf;

// timing
uint32_t sensor_poll_previous_us = 0;

// computed constants
static constexpr double DEG2RAD = M_PI/180.;

//=============================================================================
// Helper functions
//=============================================================================

void update_sample_rate(uint16_t rate)
{
  SENSOR_POLL_INTERVAL_US = static_cast<uint32_t>(1e6 / rate);
  
  // pack and ship rate info
  acl_serial_rate_msg_t rate_msg;
  rate_msg.frequency = rate;
 
  const size_t len = acl_serial_rate_msg_send_to_buffer(out_buf, &rate_msg);
  Serial.write(out_buf, len);
}

//=============================================================================
// initialize
//=============================================================================

void setup()
{
  // set up serial communication
  // baud doesn't really matter since USB
  // (make sure selected in menu: Tools > USB Type > Serial)
  Serial.begin(115200);

  SPI.begin();
  bool initialized = false;
  while (!initialized) {
    myICM.begin(CS_PIN, SPI, SPI_FREQ);
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset( );
  delay(250);
 
  // Now wake the sensor up
  myICM.sleep( false );
  myICM.lowPower( false );

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );
  if( myICM.status != ICM_20948_Stat_Ok){
//    Serial.print(F("setSampleMode returned: "));
//    Serial.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
 
  myFSS.a = gpm16;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
                         
  myFSS.g = dps2000;      // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
                         
  myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5
                                         
  myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, false );
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Gyr, false );

  //
  // Transmit configuration info
  //

  update_sample_rate(static_cast<uint16_t>(1e6/SENSOR_POLL_INTERVAL_US));
}

//=============================================================================
// loop
//=============================================================================

void loop()
{
  uint32_t current_time_us = micros();
 
  if (current_time_us >= sensor_poll_previous_us + SENSOR_POLL_INTERVAL_US) {

    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'

    // pack and ship IMU data
    acl_serial_imu_msg_t imu_msg;
    imu_msg.t_us = current_time_us;
    imu_msg.accel_x = myICM.accX()*1e-3;
    imu_msg.accel_y = myICM.accY()*1e-3;
    imu_msg.accel_z = myICM.accZ()*1e-3;
    imu_msg.gyro_x = myICM.gyrX() * DEG2RAD;
    imu_msg.gyro_y = myICM.gyrY() * DEG2RAD;
    imu_msg.gyro_z = myICM.gyrZ() * DEG2RAD;
   
    const size_t len = acl_serial_imu_msg_send_to_buffer(out_buf, &imu_msg);
    Serial.write(out_buf, len);

    sensor_poll_previous_us = current_time_us;
  }
}

//=============================================================================
// handle received serial data
//=============================================================================

void serialEvent()
{
  while (Serial.available()) {
    uint8_t in_byte = (uint8_t) Serial.read();
    if (acl_serial_parse_byte(in_byte, &msg_buf)) {
      switch (msg_buf.type) {
      case ACL_SERIAL_MSG_RATE:
        acl_serial_rate_msg_t rate_msg;
        acl_serial_rate_msg_unpack(&rate_msg, &msg_buf);
        handle_rate_msg(rate_msg);
        break;
      }
    }
  }
}

//=============================================================================
// handle received messages
//=============================================================================

void handle_rate_msg(const acl_serial_rate_msg_t& msg)
{
  update_sample_rate(msg.frequency);
}
