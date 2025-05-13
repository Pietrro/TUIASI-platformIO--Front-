#include <Arduino.h>
#include "ArduinoLog.h"
#include "adcObj/adcObj.hpp"
#include "driver/can.h"
#include <TinyGPS++.h>
#include "Aditional/aditional.hpp"
#include <HardwareSerial.h>
#include <Arduino.h>
//#include "MPU6050/MPU6050.h"
#include "MPU9250/MPU9250c.hpp"


#define TX_GPIO_NUM   GPIO_NUM_14 //inversate??
#define RX_GPIO_NUM   GPIO_NUM_27
#define LOG_LEVEL LOG_LEVEL_ERROR
#define GPS_BAUDRATE 9600 

MPU9250c MPU;
u_int16_t status;
adcObj steeringAngle(ADC1_CHANNEL_7);
adcObj damperLeftFront(ADC1_CHANNEL_4);
adcObj damperRightFront(ADC1_CHANNEL_5);

int latitude = 0;
int longitude = 0;
int speed = 0;

int dleft;
int dright;
int dsteeringAngle;

float roll;
float pitch;
float yaw;

float gx;
float gy;
float gz;

float ax;
float ay;
float az;

TinyGPSPlus gps;

twai_message_t tx_msg_mpu;
twai_message_t tx_msg_gps;
twai_message_t tx_msg_adc;
twai_message_t tx_msg_gyro;
twai_message_t tx_msg_acc;

static const can_general_config_t g_config = {
  .mode = TWAI_MODE_NO_ACK,
  .tx_io = TX_GPIO_NUM,
  .rx_io = RX_GPIO_NUM,
  .clkout_io = TWAI_IO_UNUSED,
  .bus_off_io = TWAI_IO_UNUSED,
  .tx_queue_len = 1000,
  .rx_queue_len = 5,
  .alerts_enabled = TWAI_ALERT_ALL,
  .clkout_divider = 0,
  .intr_flags = ESP_INTR_FLAG_LEVEL1
};

static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();


void setup() {

  Serial.begin(115200);
  Serial2.begin(GPS_BAUDRATE);
  Log.begin(LOG_LEVEL, &Serial);
  
  MPU.read();


  status = can_driver_install(&g_config, &t_config, &f_config);
  if (status == ESP_OK) {
    Log.noticeln("Can driver installed");
  } else {
    Log.errorln("Can driver installation failed with error: %s", esp_err_to_name(status));
  }

  status = can_start();
  if (status == ESP_OK) {
    Log.noticeln("Can started");
  } else {
    Log.errorln("Can starting procedure failed with error: %s", esp_err_to_name(status));
  }

  // Inițializare mesaje CAN
  tx_msg_mpu.data_length_code = 8;
  tx_msg_mpu.identifier = 0x115;
  tx_msg_mpu.flags = CAN_MSG_FLAG_NONE;

  tx_msg_adc.data_length_code = 8;
  tx_msg_adc.identifier = 0x116;
  tx_msg_adc.flags = CAN_MSG_FLAG_NONE;

  tx_msg_gps.data_length_code = 5;
  tx_msg_gps.identifier = 0x117;
  tx_msg_gps.flags = CAN_MSG_FLAG_NONE;

  tx_msg_acc.data_length_code=6;
  tx_msg_acc.identifier=0x118;
  tx_msg_acc.flags=CAN_MSG_FLAG_NONE;

  tx_msg_gyro.data_length_code=6;
  tx_msg_gyro.identifier=0x119;
  tx_msg_gyro.flags=CAN_MSG_FLAG_NONE;

}

void loop() {
  

  dleft = damperLeftFront.getVoltage();
  dright = damperRightFront.getVoltage();
  dsteeringAngle = steeringAngle.getVoltage();

  tx_msg_adc.data[0]=dleft/100;
  tx_msg_adc.data[1]=dleft%100;
  tx_msg_adc.data[2]=dright/100;  
  tx_msg_adc.data[3]=dright%100; 
  tx_msg_adc.data[4]=dsteeringAngle/100;
  tx_msg_adc.data[5]=dsteeringAngle%100; 


  roll = MPU.getRoll();
  pitch = MPU.getPitch();
  yaw = MPU.getYaw();

  tx_msg_mpu.data[6]=0;

  if(roll >= 0) {
    convert(roll, tx_msg_mpu.data);

  }
  else {
    convert(roll*-1, tx_msg_mpu.data);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+1; 
  }

  if(pitch >= 0) {
    convert(pitch, tx_msg_mpu.data+2);
  }
  else {
    convert(pitch*-1, tx_msg_mpu.data+2);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+2; 
  }

  if(yaw >= 0) {
    convert(yaw, tx_msg_mpu.data+4);
  }
  else {
    convert(yaw*-1, tx_msg_mpu.data+4);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+4; 
  }


  gx = MPU.currentData.gyroX;
  gy = MPU.currentData.gyroY; 
  gz = MPU.currentData.gyroZ;
  convert(gx, tx_msg_gyro.data);
  convert(gy, tx_msg_gyro.data+2);
  convert(gz, tx_msg_gyro.data+4);

  ax = MPU.currentData.accelX;
  ay = MPU.currentData.accelY; 
  az = MPU.currentData.accelZ;
  convert(ax, tx_msg_acc.data);
  convert(ay, tx_msg_acc.data+2);
  convert(az, tx_msg_acc.data+4);

  convert(MPU.currentData.accelX, tx_msg_acc.data);
  convert(MPU.currentData.accelY, tx_msg_acc.data+2);
  convert(MPU.currentData.accelZ, tx_msg_acc.data+4);

  //gyro
  convert(MPU.currentData.gyroX, tx_msg_gyro.data);
  convert(MPU.currentData.gyroY, tx_msg_gyro.data+2);
  convert(MPU.currentData.gyroZ, tx_msg_gyro.data+4);


  if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
      if (gps.speed.isValid()) {
        speed = gps.speed.kmph();
      }
    }
  }
  tx_msg_gps.data[0] = double(latitude / 100);
  tx_msg_gps.data[1] = double(latitude % 100);
  tx_msg_gps.data[2] = double(longitude / 100);
  tx_msg_gps.data[3] = double(longitude % 100);
  tx_msg_gps.data[4] = uint8_t(speed);

  status = can_transmit(&tx_msg_gps, pdMS_TO_TICKS(1000));
  if (status == ESP_OK) {
  } else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if (status == ESP_OK) Log.errorln("Can driver restarted");
  }

 
  
  status = can_transmit(&tx_msg_adc, pdMS_TO_TICKS(1000));
  if (status == ESP_OK) {
    //Log.noticeln("Can message ADC sent");
  } else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if (status == ESP_OK) Log.errorln("Can driver restarted");
  }
  status = can_transmit(&tx_msg_mpu, pdMS_TO_TICKS(1000));
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.error("Can driver restarted");
  }

  status = can_transmit(&tx_msg_acc, pdMS_TO_TICKS(1000));
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.error("Can driver restarted");
  }

  status = can_transmit(&tx_msg_gyro, pdMS_TO_TICKS(1000));
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.error("Can driver restarted");
  }
  
  
  delay(800);  
}