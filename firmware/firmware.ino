#include <ICM_20948.h>  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#include "teensyimu_serial.h"


#define SPI_FREQ 4000000 // ICM20948 has max 7 MHz SPI, but >6 MHz breaks
#define CS_PIN 10        // Which pin you connect chip select (CS) to

ICM_20948_SPI myICM;     // SPI object to talk to ICM20948 IMU

static constexpr int MOTOR_PIN = 15; // must be PWM capable, see:
                                  // https://www.pjrc.com/teensy/td_pulse.html
static constexpr int MOTOR_PWM_HZ = 2000; // PWM frequency
static constexpr int PWM_RES = 8; // PWM resolution (default 8 bits)
static constexpr int PWM_MAX = (1 << PWM_RES) - 1; // maps to 100% duty cycle

//=============================================================================
// configuration options
//=============================================================================

// sensor polling interval (micros)
uint32_t SENSOR_POLL_INTERVAL_US = 1000; // default, can be changed online
// note that ICM20948 has max Fs,accel = 4500 Hz; Fs,gyro = 9000 Hz

static constexpr float g = 9.80665f;

//=============================================================================
// global variables
//=============================================================================

// serial stuff
uint8_t out_buf[TI_SERIAL_MAX_MESSAGE_LEN];
ti_serial_message_t msg_buf;

// timing
uint32_t sensor_poll_previous_us = 0;
uint32_t start_time_us = 0;

// computed constants
static constexpr double DEG2RAD = M_PI/180.;

//=============================================================================
// Helper functions
//=============================================================================

/**
 * If Data Terminal Ready (DTR) is not high (i.e., no device connected),
 * then the serialEvent may fire and read back what was wrote one byte at
 * a time. This is undesirable as it will put the parser in an unknown state
 * (or something else breaks? not sure exactly). This function checks that
 * there is a device listening for USB serial before writing. If not, the data
 * is simply thrown away.
 */
bool safe_serial_write(const uint8_t* buf, size_t len)
{
  if (Serial.dtr()) {
    Serial.write(buf, len);
    return true;
  } else {
    return false;
  }
}

void update_sample_rate(uint16_t rate)
{
  SENSOR_POLL_INTERVAL_US = static_cast<uint32_t>(1e6 / rate);
  
  // pack and ship rate info
  ti_serial_rate_msg_t rate_msg;
  rate_msg.frequency = rate;
 
  const size_t len = ti_serial_rate_msg_send_to_buffer(out_buf, &rate_msg);
  safe_serial_write(out_buf, len);
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

  // Setup PWM pin for motor control
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(MOTOR_PIN, MOTOR_PWM_HZ);
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

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
  myICM.enableDLPF( ICM_20948_Internal_Acc, false );
  myICM.enableDLPF( ICM_20948_Internal_Gyr, false );

  myICM.startupMagnetometer();

  //
  // Transmit configuration info
  //

  // this seems to break serial / not work anyways (see safe_serial_write)
//  update_sample_rate(static_cast<uint16_t>(1e6/SENSOR_POLL_INTERVAL_US));
}

//=============================================================================
// loop
//=============================================================================

void loop()
{
  uint32_t current_time_us = micros() - start_time_us;
 
  if (current_time_us >= sensor_poll_previous_us + SENSOR_POLL_INTERVAL_US) {

    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'

    // pack and ship IMU data
    ti_serial_imu_msg_t imu_msg;
    imu_msg.t_us = current_time_us;
    imu_msg.accel_x = myICM.accX()*1e-3 * g;
    imu_msg.accel_y = myICM.accY()*1e-3 * g;
    imu_msg.accel_z = myICM.accZ()*1e-3 * g;
    imu_msg.gyro_x = myICM.gyrX() * DEG2RAD;
    imu_msg.gyro_y = myICM.gyrY() * DEG2RAD;
    imu_msg.gyro_z = myICM.gyrZ() * DEG2RAD;
    imu_msg.mag_x = myICM.magX();
    imu_msg.mag_y = myICM.magY();
    imu_msg.mag_z = myICM.magZ();
   
    const size_t len = ti_serial_imu_msg_send_to_buffer(out_buf, &imu_msg);
    safe_serial_write(out_buf, len);

    // // could also use a more lightweight version with only 3 fields:
    // ti_serial_imu_3dof_msg_t imu_msg;
    // imu_msg.t_us = current_time_us;
    // imu_msg.accel_x = myICM.accX()*1e-3 * g;
    // imu_msg.accel_y = myICM.accY()*1e-3 * g;
    // imu_msg.gyro_z = myICM.gyrZ() * DEG2RAD;

    // const size_t len = ti_serial_imu_3dof_msg_send_to_buffer(out_buf, &imu_msg);
    // safe_serial_write(out_buf, len);

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
    if (ti_serial_parse_byte(in_byte, &msg_buf)) {
      switch (msg_buf.type) {
        case TI_SERIAL_MSG_RATE:
        {
          ti_serial_rate_msg_t msg;
          ti_serial_rate_msg_unpack(&msg, &msg_buf);
          handle_rate_msg(msg);
          break;
        }
        case TI_SERIAL_MSG_MOTORCMD:
        {
          ti_serial_motorcmd_msg_t msg;
          ti_serial_motorcmd_msg_unpack(&msg, &msg_buf);
          handle_motorcmd_msg(msg);
          break;
        }
      }
    }
  }
}

//=============================================================================
// handle received messages
//=============================================================================

void handle_rate_msg(const ti_serial_rate_msg_t& msg)
{
  start_time_us = micros(); // reset start time
  sensor_poll_previous_us = 0;
  update_sample_rate(msg.frequency);
}

void handle_motorcmd_msg(const ti_serial_motorcmd_msg_t& msg)
{
  uint16_t value = static_cast<uint16_t>((msg.percentage / 1000.) * PWM_MAX);
  analogWrite(MOTOR_PIN, value);
}
