/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, December 3, 2017
 * 
 * This is the code for managing game towers. It sends information to
 * the onboard arduino receiver via the NRF24L01 module.
 * 
 * ATTENTION: SET THE "TOWER_NUMBER" (1-4) VARIABLE PRIOR TO UPLOADING THE CODE!
 */ 

#include <SPI.h>
#include <RF24.h>

#define TOWER_NUMBER 1                // <-- SET THIS PRIOR TO UPLOAD THE CODE. RANGE MUST BE [1-4] INT VALUES.

/***** GENERAL PINS *****/
#define TRIGGER_DELAY   20
#define N_CHARGE_LEDS   4             // How many LEDs are for display charging time.
#define SCHMITT_TRIGGER_INTERRUPT 0   // Digital pin 2 on Arduino UNO
#define LED_CHARGING_TIME 2500        // How much time (IN MILLIS) for turning on a LED on button press. 
#define TOTAL_CHARGING_TIME 10        // Total time for charging the tower (in SECONDS).
#define BLINKING_INTERVAL 500         // RED LED blinking interval (IN MILLIS).
#define TRANSMISSION_INTERVAL 3000    // The time between data transmission.
#define CHARGE_BLINKING_INTERVAL 150  // Charging LED blinking interval (IN MILLIS). 
#define TILT_SENSOR_PIN     A0
#define SCHMITT_TRIGGER_PIN 2  
#define TIME_LISTENING_MASTER 100     // time waiting for master arduino.

/***** LED PINS *****/
#define GREEN_LED       1
#define RED_LED         A1
#define CHARGE_LED1     A2
#define CHARGE_LED2     A3
#define CHARGE_LED3     A4
#define CHARGE_LED4     A5

/***** TRANSCEIVER PINS *****/
#define CSN_PIN         9
#define CE_PIN          10
#define MOSI_PIN        11
#define MISO_PIN        12
#define SCK_PIN         13

/**************************************************************
 * TRANSCEIVER PIN INFO
 *  -------------------------
 * | vcc | csn | mosi | irq  |
 * | gnd | ce  | sck  | miso |
 *  -------------------------
 * 
 * VCC  – power – from 1.9 up to 3.3 V
 * GND  – ground
 * MOSI – SPI serial data input
 * MISO – SPI serial data output
 * SCK  – SPI clock
 * CSN  – low state on this pin indicates that with 
 *        this module the controller wants to communicate.
 * CE   – signal activating receiving and transmitting. 
 *        In receive mode, the high state indicates that 
 *        he wants to receive. In transmit mode,
 *        pulse sends one packet of data.
 * IRQ  – interrupt output. It does a low state pulse 
 *        when data is waiting to receive, or when the 
 *        data was properly sent.
 ***************************************************************/

// Enum to control how the timer evolves. 
enum timerOrientation{
  increase,         // Charger timer increase.
  decrease,         // Charger timer decrease.
  halt              // Charger timer halt.
};

// Data to be sent by the RF module.
struct Package{
  boolean button;
  int t_status;
  int leds[4];
  int num_presses;
};

/***** CONFIG VARIABLES *****/
int led_array[] = {GREEN_LED, RED_LED, CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, CHARGE_LED4 }; // All LEDs
int charge_LEDs[] = {CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, CHARGE_LED4};                    // All charge LEDs.
int ledMask[N_CHARGE_LEDS];                                                                  // which charge LEDs are on.

/***** TOWER CONTROL VARIABLES *****/
int feedback_blink_state;                 // keeps the RED/GREEN LED blink state.
int charge_blink_state;                   // keeps the charging LED blink state.

volatile int button_state;                // Keeps the current button state (LOW/HIGH)
int previous_button_state;                // Controls the previous state of the button (LOW/HIGH)
int press_counter;                        // Counts the ammount of button presses;

static unsigned long listening_timer;      // Controls time when to listen to the master
static unsigned long transmit_timer;      // Controls time between data transmission
static unsigned long press_timer;         // Controls "turn on" time between LEDs.
static unsigned long blink_timer;         // Control the blink time of the tower RED/GREEN LED.
static unsigned long charge_blink_timer;  // Control the blink time of the tower charging LED.
int n_leds_ON;                            // keeps how many charging LEDs are on.

bool isGameRunning;                       // Keeps the state of the game: True (Game is ON).
bool has_fallen;                          // True if the tower has fallen.
bool is_captured;                         // True if player has charged the LEDs and the tower did not fall.

/***** FUNCTION PROTOTYPES *****/
void turnOFFAllChargeLEDs();
void turnONAllChargeLEDs(); 
void resetTower();
void updateChargeLEDs();
void handleButton();
void sendData();

/***** RF Trasmitter definition *****/
RF24 RFtransmitter(CE_PIN, CSN_PIN);            
const uint64_t com_addresses[] = {0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0E9LL, 0xF0F0F0F0B9LL};    // robot (master) receiver.

const uint64_t comAddress = com_addresses[TOWER_NUMBER-1];  // address to transmit on

int control_msg[1];      // cmd received from the master.

void setup() {

  pinMode(SCHMITT_TRIGGER_PIN, INPUT);        // initialize the pushbutton pin as an input:
  pinMode(TILT_SENSOR_PIN, INPUT);            //Set the Low pass filter input for the tilt sensor.
  
  for (int count=0; count < 6;count++){       // set all LEDS pins to OUTPUT mode
      pinMode(led_array[count], OUTPUT);
  }
  
  attachInterrupt(SCHMITT_TRIGGER_INTERRUPT,  // set the interrupt pin to the tower button.
                  handleButton, CHANGE);    

  // Set the transceiver properties.
  RFtransmitter.begin();
  RFtransmitter.setChannel(120); 
  RFtransmitter.setPALevel(RF24_PA_MIN);
  RFtransmitter.setDataRate(RF24_250KBPS); 
  RFtransmitter.setAutoAck(true);
  RFtransmitter.enableAckPayload();
  RFtransmitter.enableDynamicPayloads();
  RFtransmitter.openReadingPipe(1,comAddress);
  RFtransmitter.startListening();
  RFtransmitter.setRetries(15,15);
  RFtransmitter.setPayloadSize( sizeof(Package)  );

  
  delay(1000);
  
  // Reset Variable.
  resetTower();
}

/*
 * Checks whether tower has fallen
 */
void checkFall(){
  if (digitalRead(TILT_SENSOR_PIN) == HIGH){
      digitalWrite(RED_LED,HIGH);
      has_fallen = true;
      turnOFFAllChargeLEDs();
  }
}

/*
 * Prepare data to be transmitted to robot receiver
 */
Package prepareData(){
    Package data;
  
    // Tower status
    // 0 = Uncaptured
    // 1 = Captured
    // -1 = Fallen

    if (!is_captured && !has_fallen){
        data.t_status = 0;
    }else{
        data.t_status = (is_captured ? 1 : -1);
    }
    data.button = digitalRead(SCHMITT_TRIGGER_PIN);
    data.num_presses = press_counter;
    
    for(int i=0;i < N_CHARGE_LEDS; i++){
      data.leds[i] = ledMask[i]; 
    }

    return data;
}

/*
 * Check whether the player has captured the tower by
 * charging all LEDs.
 */
void checkCapture(){
  if (n_leds_ON == N_CHARGE_LEDS){
      digitalWrite(RED_LED,LOW);
      digitalWrite(GREEN_LED,HIGH);
      turnONAllChargeLEDs();
      is_captured = true;
   }
}

/*
 * Receive cmd from master
 */
void receiveCmd(){
  if (RFtransmitter.available()) {
    Package pkg = prepareData();
    RFtransmitter.writeAckPayload(1, &pkg, sizeof(Package));
    RFtransmitter.read(&control_msg,sizeof(control_msg));
    Serial.print("integer got is : ");
    Serial.println(control_msg[0]);
  }
}

void loop() {

    if((millis() - listening_timer) > TIME_LISTENING_MASTER){
        listening_timer = millis();
        receiveCmd();
    }else{
        if(isGameRunning) {
          if (!has_fallen || !is_captured){
              checkFall();     // Checks whether tower has fallen
        
              if (has_fallen){
                digitalWrite(RED_LED,HIGH);
              }else if (!is_captured){ 
                  if (button_state == HIGH){
        
                      // Pressing feedback: blinks LED to notify the button is being pressed.
                      if((millis() - charge_blink_timer) > CHARGE_BLINKING_INTERVAL){
                          charge_blink_timer = millis();                  // reset the timer
                          charge_blink_state = !charge_blink_state;       // invert the blinking_state.
                          digitalWrite(GREEN_LED,charge_blink_state);
                      }
        
                      // Count time of button press
                      if ((millis() - press_timer) > LED_CHARGING_TIME){
                        press_timer = millis();
                        n_leds_ON += 1;           // increase LED counter
                      }
        
                      checkCapture();  
                  }
              }
          }
          
        }else{        // Game is not running
            // Check interval for RED LED blinking.
            if((millis() - blink_timer) > BLINKING_INTERVAL){
                blink_timer = millis();           // reset the timer
                feedback_blink_state = !feedback_blink_state;       // invert the blinking_state.
                digitalWrite(RED_LED,feedback_blink_state);
            }
        }
    
        updateChargeLEDs();
    }
}

/*
 * Reset tower
 */
void resetTower(){
	
  feedback_blink_state  = LOW;   // keeps the RED/GREEN LED blink state.
  charge_blink_state    = LOW;   // keeps the charging LED blink state.
  
  button_state          = LOW;   // Keeps the current button state (LOW/HIGH)
  previous_button_state = LOW;   // Controls the previous state of the button (LOW/HIGH)
  press_counter         = 0;     // Counts the ammount of button presses;
  
  press_timer           = 0;     // Controls "turn on" time between LEDs.
  blink_timer           = 0;     // Control the blink time of the tower RED/GREEN LED.
  charge_blink_timer    = 0;     // Control the blink time of the tower charging LED.
  n_leds_ON             = 0;     // keeps how many charging LEDs are on.
  
  isGameRunning         = false;     // Keeps the state of the game: True (Game is ON).
  has_fallen            = false; // True if the tower has fallen.
  is_captured           = false; // True if player has charged the LEDs and the tower did not fall.
  turnOFFAllChargeLEDs();        // turn all LEDs OFF.

  digitalWrite(RED_LED,HIGH);               // set the RED_LED ON, at the beginning.
  digitalWrite(SCHMITT_TRIGGER_PIN,LOW);    
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
    if (button_state && !has_fallen){
        press_counter += 1;
    }
}
