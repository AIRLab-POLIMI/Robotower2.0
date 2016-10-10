#include <ros.h> 
#include<robogame_imu_sensor/imu_state.h>
//#include <SPI.h>
#include "RF24.h"
#include "MPU6050_6Axis_MotionApps20.h" 

// ROS settings
ros::NodeHandle  nh;
robogame_imu_sensor::imu_state imu_state;
ros::Publisher pub("robogame/imu_state", &imu_state);
/* ..... */

int WARNING_LED = 2;

RF24 RFtransmitter (7, 8); 
byte addresses[][6] = {"0"}; 

struct package{
  Quaternion q;
  VectorInt16 aaWorld;
  VectorInt16 gyro;
};

typedef struct package IMU_package;
IMU_package data;

void setup() 
{
  Serial.begin(57600);
  // Ros init
  nh.initNode();
  nh.advertise(pub);
  
  pinMode(WARNING_LED, OUTPUT);
  RFtransmitter.begin(); 
  RFtransmitter.setChannel(115); 
  RFtransmitter.setPALevel(RF24_PA_MAX);
  RFtransmitter.setDataRate( RF24_250KBPS ) ; 
  RFtransmitter.openReadingPipe(1, addresses[0]);
  delay(1000);
  RFtransmitter.startListening();
}


void loop()  
{

  unsigned long started_waiting_at = millis();
  bool timeout = false;

  while(!RFtransmitter.available() && !timeout){
    if (millis() - started_waiting_at > 250)
      timeout = true;
  }
  
  if (timeout) {
    //Serial.println("Failed, reponse time out.");
    digitalWrite(WARNING_LED, HIGH);
  }else{
    while (RFtransmitter.available())
    {
      RFtransmitter.read( &data, sizeof(data) );
    }

    digitalWrite(WARNING_LED, LOW);

    // deine new ros topic data.

    imu_state.linear_acc.x = data.aaWorld.x;
    imu_state.linear_acc.y = data.aaWorld.y;
    imu_state.linear_acc.z = data.aaWorld.z;

    imu_state.gyro.x = data.gyro.x;
    imu_state.gyro.y = data.gyro.y;
    imu_state.gyro.z = data.gyro.z;

    imu_state.q[0] = data.q.w;
    imu_state.q[1] = data.q.x;
    imu_state.q[2] = data.q.y;
    imu_state.q[3] = data.q.z;
    
    /* PUBLISH RELEVANT TOPICS */
    pub.publish(&imu_state);
  }
    nh.spinOnce();  // this has always to be called at the end when using ros (huge problems occurs, otherwise).
    delay(5);
}
