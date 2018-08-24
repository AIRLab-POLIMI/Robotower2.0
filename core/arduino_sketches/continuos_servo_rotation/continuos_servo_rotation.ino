/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, June 18, 2018
 * 
 */
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#define ANGLE_PUBLISH_INTERVAL 40
#define SERVO_ACTUATION_TIME 100
#define SERVO_PIN  9
#define BUZZER_PIN  6
#define ROSBAG_LED 8
#define BATTERY_PIN A0
#define OFF 0
#define ON 1
#define BAT_CHK_TIMEOUT_TRH 3000     // publishes battery data every 3 secs.
#define BEEPING_INTERVAL 400
#define LOW_BATTERY_VOLTAGE 22
#define VOLTAGE_TRH 1.947   // A voltage less then 1.77 correspond to battery 
                            // level at 20V, 1.947 correspond to battery level at 22V.

unsigned long last_battery_check_time;
unsigned long last_battery_beep_time;
unsigned long last_acc_time;          // last acc sample time that we saw. 
unsigned long last_angle_time;          // last acc sample time that we saw.
unsigned long angle_publish_time;
int should_beep;

Servo myservo;  // create servo object to control a servo

ros::NodeHandle nh;
std_msgs::Int8 bat_msg;
std_msgs::Int16 current_angle_msg;

void angleCallback( const std_msgs::Int16 &angle_msg);
void rosbagCallback( const std_msgs::Bool &amsg);

ros::Publisher bat_pub("arduino/battery", &bat_msg);
ros::Publisher angle_pub("arduino/current_servo_angle", &current_angle_msg);

ros::Subscriber<std_msgs::Bool> sub_rosbag("/game/is_recording", &rosbagCallback);
ros::Subscriber<std_msgs::Int16> sub("/onboard_cam/rotate", &angleCallback);


bool rosbag_active = false;
bool listening = true;
float servo_current_angle = 90.0;

const int MAGNET_SENSOR_PIN = 2;

const int STOP = 0;
const int CLOCKWISE = 1;
const int COUNTERCLOCKWISE = 2;

const float ANGULAR_SPEED_COUNTERCLOCKWISE = 150.0; // Experienced speed of servo in deg/s
const float ANGULAR_SPEED_CLOCKWISE = 150.0;

unsigned long starting_time;
unsigned long now;
int last_rotating_direction;
float current_angle;

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(1000);
  nh.advertise(bat_pub);
  nh.advertise(angle_pub);
  nh.subscribe(sub);
  nh.subscribe(sub_rosbag);
  delay(1000);
  
  myservo.attach(SERVO_PIN);  // attaches the servo to the servo object
  myservo.write(servo_current_angle);
  
  pinMode(ROSBAG_LED, OUTPUT); 
  pinMode(BUZZER_PIN, OUTPUT);


  starting_time = millis();
  last_rotating_direction = STOP;
  attachInterrupt(digitalPinToInterrupt(MAGNET_SENSOR_PIN), sensor_callback, FALLING);
  
}
 
void loop(){
  
    // Publish battery state every 3 seconds!
    if (millis() - last_battery_check_time > BAT_CHK_TIMEOUT_TRH){
        last_battery_check_time = millis();
        float voltage = checkBatteryLevel();
        bat_msg.data = voltage;
        bat_pub.publish(&bat_msg);
    }

    digitalWrite(ROSBAG_LED, rosbag_active);

    if (millis() - angle_publish_time > ANGLE_PUBLISH_INTERVAL){
      current_angle_msg.data = servo_current_angle;
      angle_pub.publish(&current_angle_msg);
      angle_publish_time = millis();
    }
    
    nh.spinOnce();
    delay(3);
}


// Reads voltage level at battery pin (analog). This
// voltage comes from the voltage divider circuit used to monitor the battery.
float getVoltageLevel(){
  float voltage = (analogRead(BATTERY_PIN) * 5.015) / 1024.0;
  return (LOW_BATTERY_VOLTAGE*voltage)/VOLTAGE_TRH;  // Converted voltage (approx).
}

// Checks the battery level and produces a beep in case it is low. Also writes
// the voltage level to serial port.
float checkBatteryLevel(){
  float voltage = getVoltageLevel();
  
  if (voltage <= LOW_BATTERY_VOLTAGE && voltage >= (LOW_BATTERY_VOLTAGE/2)){
    if ((millis() - last_battery_beep_time) > BEEPING_INTERVAL){
      last_battery_beep_time = millis();
      if (should_beep == OFF) {
        should_beep = ON;
        beep(200);
      } else {
        should_beep = OFF;
        beep(0);
      }
    }
  }else{
    digitalWrite(BUZZER_PIN, LOW);
  }
  return voltage;
}

//void angleCallback( const std_msgs::Int16 &angle_msg){
//  if (listening){
//    last_angle_time = millis();
//    float offset = angle_msg.data;
//    int new_angle_servo = servo_current_angle + offset;
//    if (new_angle_servo < 0){
//      new_angle_servo = 0;
//    }else if (new_angle_servo > 180){
//      new_angle_servo = 180;
//    }
//    myservo.write(new_angle_servo);
//    servo_current_angle = new_angle_servo;
//    listening = !listening;
//  }else{
//    if ((millis() - last_angle_time) > SERVO_ACTUATION_TIME){
//      listening = !listening;
//    }
//  }
//}

void rosbagCallback( const std_msgs::Bool &msg){
  rosbag_active = msg.data;
}

void beep(unsigned char delayms){
  analogWrite(BUZZER_PIN, delayms);      // Almost any value can be used except 0 and 255
}

//void setup() {
//  // put your setup code here, to run once:
//  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
//  myservo.write(90);
//  starting_time = millis();
//  last_rotating_direction = STOP;
//  current_angle = STARTING_ANGLE;
//  Serial.begin(9600);
//  attachInterrupt(digitalPinToInterrupt(MAGNET_SENSOR_PIN), sensor_callback, FALLING);
//}

void rotate(int direction){
  if(direction == CLOCKWISE){
    myservo.write(77);
  } else if (direction == COUNTERCLOCKWISE){
    myservo.write(100);
  } else if(direction == STOP){
    myservo.write(90);
  }
  return;
}

int decode_rotating_direction(float offset){
  // DEADBAND [-5, +5]
  if(offset > 10.0f){
    return CLOCKWISE;
  } else if(offset < -10.0f){
   return COUNTERCLOCKWISE;
  } else {
    return STOP;
  }
}

void angleCallback(const std_msgs::Int16 &angle_msg){
  float offset = angle_msg.data;
  int current_rotating_direction = decode_rotating_direction(offset);

  if(last_rotating_direction != current_rotating_direction){
    // WE'VE CHANGED OUR MOTION
    float angle = evaluate_angle(last_rotating_direction);
    update_current_angle(angle);
    Serial.println(current_angle);
    starting_time = millis();
    last_rotating_direction = current_rotating_direction;
    rotate(current_rotating_direction);
    }
   else{
    rotate(last_rotating_direction);
   }
}

float evaluate_angle(int last_rotating_direction){
  float speed = 0.0;
  if(last_rotating_direction == CLOCKWISE){
    speed = - ANGULAR_SPEED_CLOCKWISE;
  }
  else if(last_rotating_direction == COUNTERCLOCKWISE){
    speed = + ANGULAR_SPEED_COUNTERCLOCKWISE;
  }
  else{
    return 0.0;
  }
  unsigned long elapsed_time = millis() - starting_time;
  float travelled_degrees = (elapsed_time/1000.0) * speed;

  return travelled_degrees;
}

void update_current_angle(float offset){
  servo_current_angle += offset;
  if(servo_current_angle < 0.0f){
    servo_current_angle = 360.0 + current_angle;
  }
  else if(servo_current_angle > 360.0f){
    servo_current_angle = servo_current_angle - 360.0;
  }
}

void sensor_callback(){
  // When passing next to the sensor we know the camera is rotated to 0 degrees
  servo_current_angle = 0.0;
  starting_time = millis(); // Start counting from now
}
