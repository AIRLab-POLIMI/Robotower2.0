/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, January 3, 2017
 * 
 */

#include <SPI.h>
#include "RF24.h" 
#include <NewPing.h>

/////// CONSTANTS /////////////
#define TRIGGER_DELAY   20
#define N_CHARGE_LEDS   4             // How many LEDs are for display charging time.
#define SCHMITT_TRIGGER_INTERRUPT 0   //Digital pin 2 on Arduino UNO
#define LDR_THRESHOLD   614           // A light intensity less them the threshold will acuse tower has fallen.
#define LED_CHARGING_TIME 1000        // How much time (IN MILLIS) for turning on a LED on button press. 
#define TOTAL_CHARGING_TIME 10        // Total time for charging the tower (in SECONDS).
#define BLINKING_INTERVAL 500         // RED LED blinking interval (IN MILLIS). 
///////////////////////////////

/////// LED PINS //////////
#define GREEN_LED       1
#define RED_LED         A1
#define CHARGE_LED1     A2
#define CHARGE_LED2     A3
#define CHARGE_LED3     A4
#define CHARGE_LED4     A5
///////////////////////////////

/////// GENERAL PINS //////////
#define LDR_PIN             A0
#define SCHMITT_TRIGGER_PIN 2
///////////////////////////////

///////// ULTRASOUND PINS /////
#define TRIGGER_RIGHT   3
#define ECHO_RIGHT      4
#define TRIGGER_LEFT    5
#define ECHO_LEFT       6
#define TRIGGER_CENTER  7
#define ECHO_CENTER     8
///////////////////////////////

////// TRANSCEIVER PINS ///////
#define CSN_PIN         9
#define CE_PIN          10
#define MOSI_PIN        11
#define MISO_PIN        12
#define SCK_PIN         13
/*TRANSCEIVER PIN INFO
 * VCC – power – from 1.9 up to 3.3 V
 * GND – ground
 * MOSI – SPI serial data input
 * MISO – SPI serial data output
 * SCK – SPI clock
 * CSN – low state on this pin indicates that with this module the controller wants to communicate.
 * CE – signal activating receiving and transmitting. In receive mode, the high state indicates that he wants to receive. In transmit mode, pulse sends one packet of data.
 * IRQ – interrupt output. It does a low state pulse when data is waiting to receive, or when the data was properly sent.
 */
///////////////////////////////

// RF Trasmitter definition.
RF24 RFtransmitter(CE_PIN, CSN_PIN);
byte addresses[][6] = {"0"};  // RF pipe at address 0.

// Ultrasound sensors.
NewPing sonar_center(TRIGGER_CENTER, ECHO_CENTER, 200);
NewPing sonar_right(TRIGGER_RIGHT , ECHO_RIGHT, 200);
NewPing sonar_left(TRIGGER_LEFT  , ECHO_LEFT, 200);

// Enum to control how the timer evolves. 
enum timerOrientation{
  increase,         // Charger timer increase.
  decrease,         // Charger timer decrease.
  halt              // Charger timer halt.
};

// Data to be sent by the RF module.
struct package{
  boolean isTowerDown;
  boolean hasWon;
  boolean isGameActive;
  float distances[3]; // 0 - left sensor, 1- center sensor, 2- right sensor
  int ledMask[4];   // which LEDs are on.
}data;

//// LEDs VARIABLES ///////
int led_array[] = {GREEN_LED, RED_LED, CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, CHARGE_LED4 };       // All LEDs
int charge_LEDs[] = {CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, CHARGE_LED4};                         // All charge LEDs.
int ledMask[N_CHARGE_LEDS];                 // which charge LEDs are on.
int blink_state   = LOW;                    // keeps the RED/GREEN LED blink state.
///////////////////////////

///// BUTTON CONTROL //////
volatile int button_state = LOW;            // Keeps the current button state (LOW/HIGH)
int previous_button_state = LOW;            // Controls the previous state of the button (LOW/HIGH)
///////////////////////////    

///// TIMER ///////////////
static unsigned long led_timer = 0;         // Controls "turn on" time between LEDs.
static unsigned long blink_timer = 0;       // Control the blink time of the tower RED/GREEN LED.
int secs_counter = 0;                       // keeps how many seconds has been passed.
timerOrientation timerState = halt;         // Controls in which direction to update the LEDs charging timer.
///////////////////////////

///// GAME VARIABLES //////
bool isGameActive = true;                   // Keeps the state of the game: True (Game is ON).
bool isTowerDown = false;                   // True if the tower has fallen.
bool hasWon = false;                        // True if player has charged the LEDs and the tower did not fall.
///////////////////////////

void setup() {
  // set all LEDS pins to OUTPUT mode
  for (int count=0; count < 6;count++){
      pinMode(led_array[count], OUTPUT);
  }
  // initialize the pushbutton pin as an input:
  pinMode(SCHMITT_TRIGGER_PIN, INPUT);
  // set the interrupt pin to the tower button.
  attachInterrupt(SCHMITT_TRIGGER_INTERRUPT, handleButton, CHANGE);
  // set the RED_LED ON, at the beginning.
  digitalWrite(RED_LED,HIGH);

  // Set the transceiver properties.
  delay(1000);
  RFtransmitter.begin();  
  RFtransmitter.setChannel(115); 
  RFtransmitter.setPALevel(RF24_PA_MAX);
  RFtransmitter.setDataRate( RF24_250KBPS ) ; 
  RFtransmitter.openWritingPipe( addresses[0]);
  delay(1000);
}

void loop() {

    // Calls a function for calculating the distance measured by each Ultrasonic sensor
    delay(TRIGGER_DELAY);
    data.distances[0] = sonar_left.ping_cm();
    delay(TRIGGER_DELAY);
    data.distances[1] = sonar_center.ping_cm();
    delay(TRIGGER_DELAY);
    data.distances[2] = sonar_right.ping_cm();
    
    // Setting properties when LED_CHARGING_TIME is reached.
    if (secs_counter == (TOTAL_CHARGING_TIME-1)){
      if((millis() - led_timer) > LED_CHARGING_TIME){
        led_timer = millis();
        secs_counter = (secs_counter + 1) % TOTAL_CHARGING_TIME;
        updateLEDs(secs_counter);
        digitalWrite(RED_LED,LOW);
        turnONChargeLEDs();
        digitalWrite(GREEN_LED,HIGH);
        isGameActive = false;
        hasWon = true;
      }
    }
    
    if(isGameActive) {
      // Check interval for RED LED blinking.
      if((millis() - blink_timer) > BLINKING_INTERVAL){
          blink_timer = millis();           // reset the timer
          blink_state = !blink_state;       // invert the blinking_state.
          digitalWrite(RED_LED,blink_state);
      }
      
      // Check, using the LDR, whether the tower has fallen. 
      float ldr = analogRead(LDR_PIN);
      if (ldr < LDR_THRESHOLD){
        digitalWrite(RED_LED,LOW);
        isGameActive = false;
        isTowerDown = true;
        hasWon = false;
        turnOFFChargeLEDs();
      }else{
        // If tower was not detected as a "fallen tower", check and update LEDs.
        
        if (((previous_button_state == LOW) && (button_state == HIGH)) || ((previous_button_state == HIGH) && (button_state == HIGH))){
          timerState = increase;
        }else if (previous_button_state == HIGH && button_state == LOW){
          timerState = halt;
        }else if ((previous_button_state == LOW) && (button_state == LOW) && secs_counter != 0){
          timerState=halt;
        }else{
          timerState=halt;
        }
        // Save current button state
        previous_button_state = button_state;
        //
        updateTimer(timerState);
        updateLEDs(secs_counter);
      }
    }

    //Setting the remaining data to be transmitted.
    data.isGameActive = isGameActive;
    data.hasWon = hasWon;
    data.isTowerDown = isTowerDown;
    for(int i=0;i<N_CHARGE_LEDS; i++){
      data.ledMask[i] = ledMask[i]; 
    }

    // Transmit data.
    RFtransmitter.write(&data, sizeof(data));
}

/* Updates the timer (secs_counter) based on how many time 
 * has been passed between leds. Basically, controls the 
 * charging time for the tower.
 */
void updateTimer(timerOrientation orientation){ 
  if (orientation == increase){
    if((millis() - led_timer) > LED_CHARGING_TIME){
        led_timer = millis();
        secs_counter = (secs_counter + 1) % TOTAL_CHARGING_TIME;
    }
  }else if (orientation == decrease){
    if((millis() - led_timer) > LED_CHARGING_TIME){
        led_timer = millis();
        secs_counter = (secs_counter - 1) % TOTAL_CHARGING_TIME;
    }
  }
}

/*
 * Turns OFF ALL the charge LEDs by setting them to LOW.
 */
void turnOFFChargeLEDs(){
  for(int i=0; i < N_CHARGE_LEDS; i++){
    digitalWrite(charge_LEDs[i],LOW);
  }
}

/*
 * Turns ON ALL the charge LEDs by setting them to HIGH.
 */
void turnONChargeLEDs(){
  for(int i=0; i < N_CHARGE_LEDS; i++){
    digitalWrite(charge_LEDs[i],HIGH);
  }
}

// Function for calculating the distance measured by the Ultrasonic sensor (in cm)
int calculateDistance(int triggerPin, int echoPin){ 
  digitalWrite(triggerPin, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(triggerPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  float duration = pulseIn(echoPin, HIGH, 10000); // Reads the echoPin, returns the sound wave travel time in microseconds
  float distance= duration*0.034/2;
  return distance;
}

// Updates Charge LEDs based on the timer. 
void updateLEDs(int counting){
  int up = (N_CHARGE_LEDS*counting)/9;
  for(int i=0; i < N_CHARGE_LEDS; i++){
    if (i < up){
      digitalWrite(charge_LEDs[i],HIGH);
      ledMask[i] = 1;
    }else{
      digitalWrite(charge_LEDs[i],LOW);
      ledMask[i] = 0;
    }
  }
}

/*
 * Tower button callback.
 * Updates the state of the button. LOW (Not pressed), HIGH (Pressed).
 */
void handleButton() {
    button_state = !button_state;
}
