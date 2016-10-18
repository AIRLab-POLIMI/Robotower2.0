#include <ros.h>
#include<robogame_imu_sensor/imu_state.h>
#include<robogame_arduino/Voltage.h>
#include<robogame_arduino/interState.h>
#include "RF24.h"
#include "time.h"
#include "MPU6050_6Axis_MotionApps20.h" 


int WARNING_LED = 9;
int BUZZER_PIN = 6;
//RGB LED pins
int RGB_PINS[] = {5, 4, 3}; //the three digital pins of the digital LED 
                                   //10 = redPin, 11 = greenPin, 9 = bluePin

const boolean ON = HIGH;     //Define on as LOW (this is because we use a common 
                            //Anode RGB LED (common pin is connected to +5 volts)
const boolean OFF = LOW;   //Define off as HIGH
boolean ledState = OFF;     // ledState used to set the LED
boolean battery_buzzer_state = OFF;


boolean isPlayerLost;
boolean isPlayerTooClose;

//Predefined Colors
const boolean RED[] = {ON, OFF, OFF};    
const boolean GREEN[] = {OFF, ON, OFF}; 
const boolean BLUE[] = {OFF, OFF, ON}; 
const boolean YELLOW[] = {ON, ON, OFF}; 
const boolean CYAN[] = {OFF, ON, ON}; 
const boolean MAGENTA[] = {ON, OFF, ON}; 
const boolean WHITE[] = {ON, ON, ON}; 
const boolean COLOROFF[] = {OFF, OFF, OFF};

//An Array that stores the predefined colors (allows us to later randomly display a color)
const boolean* COLORS[] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, COLOROFF};

int currentColor=4;
int blinking_interval = 250;           // interval at which to blink (milliseconds)

// Generally, we should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previous_time = 0;        // will store last time LED was updated;
unsigned long timeForBuzzer = 0;
unsigned long previousTimeForBatBeeping = 0;

/* MANAGES WIIMOTE INCOME MESSAGES*/
void handleGameState(const robogame_arduino::interState& msg){

  currentColor = msg.LEDColorindex;
//  blinking_interval = msg.blinkInterval;
  if ((msg.beep == true)){
      beep(20);
  }else{
      beep(0);
  }
}

/* ROS settings */
ros::NodeHandle  nh;
robogame_imu_sensor::imu_state imu_state;
robogame_arduino::Voltage voltageState;
//ros::Subscriber<robogame_arduino::interState> sub("robogame/iteraction_state", &handleGameState);
ros::Publisher voltagePub("robogame/battery_state", &voltageState);
ros::Publisher pub("robogame/imu_state", &imu_state);
/* ..... */

/*****************************************************/

/*DEFINING RF TRANSMITTER OBJECT AND DATA PACKAGE*/
RF24 RFtransmitter (10, 8); 
byte addresses[][6] = {"0"}; 

struct package{
  Quaternion q;
  VectorInt16 aaWorld;
  VectorInt16 gyro;
};

typedef struct package IMU_package;
IMU_package data;
/* ****************************** */



void setup() 
{
  Serial.begin(57600);
  // Ros init
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(voltagePub);
  
  pinMode(WARNING_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
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

  /* Checking battery (a voltage less then 1.77 correspond to battery level at 20V */
  float voltage = (analogRead(A2) * 5.015) / 1024.0;
  voltageState.raw_voltage = voltage;
  voltageState.voltage = (20*voltage)/1.77;         // Converted voltage (approx).
  if (voltageState.voltage <= 20 && voltageState.voltage >= 10){
    unsigned long currentBatMillis = millis();
    if ((millis() - previousTimeForBatBeeping) > 400){
      previousTimeForBatBeeping = currentBatMillis;
      if (battery_buzzer_state == OFF) {
        battery_buzzer_state = ON;
        beep(200);
      } else {
        battery_buzzer_state = OFF;
        beep(0);
      }
    }
  }else{
    digitalWrite(BUZZER_PIN, LOW);
  }
  /************************************************/
  // HANDLE INTERACTION LED ON THE ROBOT
  // check to see if it's time to blink the LED; that is, if the
  // difference between the current time and last time you blinked
  // the LED is bigger than the interval at which you want to
  // blink the LED.
  unsigned long current_time = millis();
  if((current_time - previous_time) >= blinking_interval) {
      // save the last time you blinked the LED
      previous_time = current_time;

      // if the LED is off turn it on and vice-versa:
      if (ledState == OFF) {
        ledState = ON;
        setColor(RGB_PINS, COLORS[currentColor]);    //Set the color of LED one
      } else {
        ledState = OFF;
        setColor(RGB_PINS, COLOROFF); // turn off light
      }
  }

  /*****************************************************/
  // HANDLE RF TRANSMISSION
  
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

    imu_state.header.stamp = nh.now();    
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
    
    /* PUBLISH RECEIVED IMU DATA */
    pub.publish(&imu_state);
  }
  /*****************************************************/
  voltagePub.publish(&voltageState);            // Publish robot battery info.
  nh.spinOnce();                                // this has always to be called at the end when using ros
                                                //    (huge problems occurs, otherwise).
  delay(5);
}

/* Sets an led to any color
   led - a three element array defining the three color pins (led[0] = redPin, led[1] = greenPin, led[2] = bluePin)
   color - a three element boolean array (color[0] = red value (LOW = on, HIGH = off), color[1] = green value, color[2] =blue value)
*/
void setColor(int* led, boolean* color){
 for(int i = 0; i < 3; i++){
   digitalWrite(led[i], color[i]);
 }
}

/* A version of setColor that allows for using const boolean colors
*/
void setColor(int* led, const boolean* color){
  boolean tempColor[] = {color[0], color[1], color[2]};
  setColor(led, tempColor);
}

void beep(unsigned char delayms){
  analogWrite(BUZZER_PIN, delayms);      // Almost any value can be used except 0 and 255
}
