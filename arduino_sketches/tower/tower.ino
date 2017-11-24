/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, January 3, 2017
 * 
 */

#include <SPI.h>
#include "RF24.h"

/////// CONSTANTS /////////////
#define TRIGGER_DELAY   20
#define N_CHARGE_LEDS   4             // How many LEDs are for display charging time.
#define SCHMITT_TRIGGER_INTERRUPT 0   // Digital pin 2 on Arduino UNO
#define LED_CHARGING_TIME 2500        // How much time (IN MILLIS) for turning on a LED on button press. 
#define TOTAL_CHARGING_TIME 10        // Total time for charging the tower (in SECONDS).
#define BLINKING_INTERVAL 500         // RED LED blinking interval (IN MILLIS).
#define CHARGE_BLINKING_INTERVAL 150  // Charging LED blinking interval (IN MILLIS). 
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
#define TILT_SENSOR_PIN     A0
#define SCHMITT_TRIGGER_PIN 2  
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
const uint64_t wAddress = 0xF0F0F0F0A1LL;              // Pipe to write or transmit on
const uint64_t rAddress = 0xB00B1E50D2LL;

// Enum to control how the timer evolves. 
enum timerOrientation{
  increase,         // Charger timer increase.
  decrease,         // Charger timer decrease.
  halt              // Charger timer halt.
};

// Data to be sent by the RF module.
struct Package{
  boolean isTowerDown;
  boolean isCaptured;
  boolean isTowerEnable;
  boolean isButtonPressed;
  int ledMask[4];   // which LEDs are on.
  int pressCounter;
};

//// CONFIG VARIABLES ///////
int led_array[] = {GREEN_LED, RED_LED, CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, CHARGE_LED4 }; // All LEDs
int charge_LEDs[] = {CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, CHARGE_LED4};                    // All charge LEDs.
int ledMask[N_CHARGE_LEDS];                                                                  // which charge LEDs are on.

/* GAME CONTROL VARIABLES */
int blink_state;                        // keeps the RED/GREEN LED blink state.
int charge_blink_state;                 // keeps the charging LED blink state.

volatile int button_state;              // Keeps the current button state (LOW/HIGH)
int previous_button_state;              // Controls the previous state of the button (LOW/HIGH)
int press_counter;                      // Counts the ammount of button presses;

static unsigned long press_timer;       // Controls "turn on" time between LEDs.
static unsigned long blink_timer;       // Control the blink time of the tower RED/GREEN LED.
static unsigned long charge_blink_timer;// Control the blink time of the tower charging LED.
int n_leds_ON;                          // keeps how many charging LEDs are on.

bool isTowerEnable;                     // Keeps the state of the game: True (Game is ON).
bool isTowerDown;                       // True if the tower has fallen.
bool isCaptured;                        // True if player has charged the LEDs and the tower did not fall.

// FUNCTION PROTOTYPE
void turnOFFAllChargeLEDs();
void turnONAllChargeLEDs(); 
void resetTower();
void updateChargeLEDs();
void handleButton();

void setup() {

  //Set the Low pass filter input for the tilt sensor.
  pinMode(TILT_SENSOR_PIN, INPUT);
  
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
  digitalWrite(SCHMITT_TRIGGER_PIN,LOW);

  // Reset Variable.
  resetTower();
  
  // Set the transceiver properties.
  delay(1000);
  RFtransmitter.begin();
  //RFtransmitter.setChannel(115); 
  RFtransmitter.setPALevel(RF24_PA_MAX);
  RFtransmitter.setDataRate( RF24_250KBPS ) ; 
  RFtransmitter.openWritingPipe(wAddress);
  delay(1000);
}

void loop() {

    Package data;       // RF package
    
    if(isTowerEnable) {
      // Check interval for RED LED blinking.
      if((millis() - blink_timer) > BLINKING_INTERVAL){
          blink_timer = millis();           // reset the timer
          blink_state = !blink_state;       // invert the blinking_state.
          digitalWrite(RED_LED,blink_state);
      }
      
      // Check using the TILT Sensor, whether the tower has fallen. 
      if ((digitalRead(TILT_SENSOR_PIN) == HIGH) && isTowerEnable){
          digitalWrite(RED_LED,HIGH);
          isTowerEnable = false;
          isTowerDown   = true;
          turnOFFAllChargeLEDs();
      }else{

          // Count button presses by monitoring changes in the button
          if (previous_button_state != digitalRead(SCHMITT_TRIGGER_PIN)){
              if (digitalRead(SCHMITT_TRIGGER_PIN) == HIGH){
                  press_counter += 1;
              }
          }

          // If tower was not detected as a "fallen tower", check and update LEDs. 
          if (digitalRead(SCHMITT_TRIGGER_PIN) == HIGH){
              previous_button_state = HIGH;
              
              if((millis() - charge_blink_timer) > CHARGE_BLINKING_INTERVAL){
                  charge_blink_timer = millis();                  // reset the timer
                  charge_blink_state = !charge_blink_state;       // invert the blinking_state.
                  digitalWrite(GREEN_LED,charge_blink_state);
              }
               
              if ((millis() - press_timer) > LED_CHARGING_TIME){
                press_timer = millis();
                n_leds_ON += 1;
                if (n_leds_ON == N_CHARGE_LEDS){
                  // Setting properties when LED_CHARGING_TIME is reached.
                  digitalWrite(RED_LED,LOW);
                  turnONAllChargeLEDs();
                  digitalWrite(GREEN_LED,HIGH);
                  isTowerEnable = false;
                  isCaptured = true;
                  turnONAllChargeLEDs();
                }
              }
          }else{
              digitalWrite(GREEN_LED,LOW);
              press_timer = millis();
              previous_button_state = LOW;
          }
      }
    }

    updateChargeLEDs();
    
    //Setting the remaining data to be transmitted.
    data.isTowerEnable = isTowerEnable;
    data.isCaptured = isCaptured;
    data.isButtonPressed = digitalRead(SCHMITT_TRIGGER_PIN);
    data.isTowerDown = isTowerDown;
    data.pressCounter = press_counter;
    
    for(int i=0;i < N_CHARGE_LEDS; i++){
      data.ledMask[i] = ledMask[i]; 
    }

    // Transmit data.
    RFtransmitter.write(&data, sizeof(data));
}

/*
 * Reset tower
 */
void resetTower(){
	
  blink_state          = LOW;   // keeps the RED/GREEN LED blink state.
  charge_blink_state   = LOW;   // keeps the charging LED blink state.
  
  button_state         = LOW;   // Keeps the current button state (LOW/HIGH)
  previous_button_state= LOW;   // Controls the previous state of the button (LOW/HIGH)
  press_counter        = 0;     // Counts the ammount of button presses;
  
  press_timer          = 0;     // Controls "turn on" time between LEDs.
  blink_timer          = 0;     // Control the blink time of the tower RED/GREEN LED.
  charge_blink_timer   = 0;     // Control the blink time of the tower charging LED.
  n_leds_ON            = 0;     // keeps how many charging LEDs are on.
  
  isTowerEnable        = true;  // Keeps the state of the game: True (Game is ON).
  isTowerDown          = false; // True if the tower has fallen.
  isCaptured           = false; // True if player has charged the LEDs and the tower did not fall.
	
	turnOFFAllChargeLEDs();        // turn all LEDs OFF.
}


/*
 * Turns OFF ALL the charge LEDs by setting them to LOW.
 */
void turnOFFAllChargeLEDs(){
  for(int i=0; i < N_CHARGE_LEDS; i++){
    ledMask[i] = 0;
  }
}


/*
 * Turns ON ALL the charge LEDs by setting them to HIGH.
 */
void turnONAllChargeLEDs(){
  for(int i=0; i < N_CHARGE_LEDS; i++){
    ledMask[i] = 1;
  }
}

/* 
 * Updates Charge LEDs based on the current settings. 
 */ 
void updateChargeLEDs(){
  for(int i=0; i < N_CHARGE_LEDS; i++){
    if (i < n_leds_ON){
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
void handleButton(){
    button_state = digitalRead(SCHMITT_TRIGGER_PIN);
}
