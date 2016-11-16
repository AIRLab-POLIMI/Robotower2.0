#include <ros.h>
#include<robogame_arduino/interState.h>
#include<robogame_arduino/Voltage.h>

ros::NodeHandle  nh;

//RGB LED pins
int ledDigitalOne[] = {10, 11, 9}; //the three digital pins of the digital LED 
                                   //10 = redPin, 11 = greenPin, 9 = bluePin

const boolean ON = HIGH;     //Define on as LOW (this is because we use a common 
                            //Anode RGB LED (common pin is connected to +5 volts)
const boolean OFF = LOW;   //Define off as HIGH
boolean ledState = OFF;     // ledState used to set the LED
volatile boolean isBatButtonPressed = false;
boolean batBuzzerState = OFF;


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
boolean gameOverSound = false;
int blinkInterval = 250;           // interval at which to blink (milliseconds)
float distanceThreshold = 1.10;

// Generally, we should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated;
unsigned long timeForBuzzer = 0;
unsigned long previousTimeForBatBeeping = 0;

/* MANAGES WIIMOTE INCOME MESSAGES*/
void handleGameState(const robogame_arduino::interState& msg){

  currentColor = msg.LEDColorindex;
  blinkInterval = msg.blinkInterval;
  if ((msg.beep == true)){
      beep(20);
  }else{
      beep(0);
  }
}

/* SET PUBLISHERS AND SUBSCRIBERS*/
ros::Subscriber<robogame_arduino::interState> sub("robogame/iteraction_state", &handleGameState);
robogame_arduino::Voltage voltageState;
ros::Publisher voltagePub("robogame/battery_state", &voltageState);
/* ..... */

void setup(){
  Serial.begin(57600);
  for(int i = 0; i < 3; i++){
   pinMode(ledDigitalOne[i], OUTPUT);   //Set the three LED pins as outputs
  }

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));
  
  attachInterrupt(1,batButtonPressed,RISING);
  // declare pin 5 to be an output (for buzzer)
  pinMode(5, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(voltagePub);
}

void loop(){

  /* Checking battery (a voltage less then 1.77 correspond to battery level at 20V */
  float voltage = (analogRead(A2) * 5.015) / 1024.0;
  voltageState.raw_voltage = voltage;
  voltageState.voltage = (20*voltage)/1.77;
  if (voltageState.voltage <= 20 && voltageState.voltage > 1){
    unsigned long currentBatMillis = millis();
    if (!isBatButtonPressed){
      if ((millis() - previousTimeForBatBeeping) > 400){
        previousTimeForBatBeeping = currentBatMillis;
        if (batBuzzerState == OFF) {
          batBuzzerState = ON;
          beep(200);
        } else {
          batBuzzerState = OFF;
          beep(0);
        }
      }
    }
  }
  //.... 
  
  


  /* PUBLISH RELEVANT TOPICS */
  voltagePub.publish(&voltageState);            // Publish robot battery info.
  nh.spinOnce();
  delay(1);

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
  analogWrite(5, delayms);      // Almost any value can be used except 0 and 255
}

void batButtonPressed() {
  static unsigned long lastMillis = 0;
  unsigned long newMillis = millis();
  if(newMillis - lastMillis < 60){}
  else{
    isBatButtonPressed = !isBatButtonPressed;
  }
}
