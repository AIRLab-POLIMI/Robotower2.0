#include <ESP8266WiFi.h>
#include <ros.h>
#include <WString.h>
#include <std_msgs/Bool.h>
#include <game_manager/ChangeLEDs.h>
#include <game_manager/TiltSensor.h>
#include <game_manager/ButtonState.h>

#define TOWER_ID 1

#define DEBOUNCE_DELAY 50
#define N_CHARGE_LEDS 3

/***** INPUT PINS *****/
#define TILT_SENSOR_PIN 13
#define INPUT_PIN       10

/***** LED PINS *****/
#define RED_LED         14
#define GREEN_LED       12
#define CHARGE_LED1     16
#define CHARGE_LED2     5
#define CHARGE_LED3     4

int RED_COLOR[] = {1,0};
int GREEN_COLOR[] = {0,1};
int BLANK_COLOR[] = {0,0};

/***** CONFIG VARIABLES *****/
int led_array[] = {CHARGE_LED1, CHARGE_LED2, CHARGE_LED3, GREEN_LED, RED_LED}; // All LEDs
int charge_LEDs[] = {CHARGE_LED1, CHARGE_LED2, CHARGE_LED3};                            // All charge LEDs.
int STATUS_LED[] = {RED_LED,GREEN_LED};


///////////////////////
// Input Definitions //
///////////////////////
int button_state;                   // the current reading from the input pin
int last_button_state;              // the previous reading from the input pin
unsigned long last_debounce_time;   // the last time the output pin was toggled. Remember that time variables 
                                    // are unsigned longs because the time, measured in  milliseconds, will quickly 
                                    // become a bigger number than can be stored in an int.


//////////////////////
// feedback LED     //
int feedback_blink_state;                 // keeps the RED/GREEN LED blink state.
int charge_blink_state;                   // keeps the charging LED blink state.

static unsigned long blink_timer;         // Control the blink time of the tower RED/GREEN LED.
static unsigned long charge_blink_timer;  // Control the blink time of the tower charging LED.

bool isGameRunning;                       // Keeps the state of the game: True (Game is ON).
bool has_fallen;                          // True if the tower has fallen.
bool is_captured;                         // True if player has charged the LEDs and the tower


//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = "Robogame";
const char* password = "aerolabio";


IPAddress ip_ros_master(10,0,0,122); // ip of your ROS server
int status = WL_IDLE_STATUS;

String Statuses[] =  { "WL_IDLE_STATUS=0", "WL_NO_SSID_AVAIL=1", "WL_SCAN_COMPLETED=2", "WL_CONNECTED=3", "WL_CONNECT_FAILED=4", "WL_CONNECTION_LOST=5", "WL_DISCONNECTED=6"};


WiFiClient client;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(ip_ros_master, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};

void updateChargeLEDs(const bool led_state[]){
  for(int i=0;i<N_CHARGE_LEDS;i++){
    digitalWrite(charge_LEDs[i],led_state[i]);
  }
}

void ledCallback(const game_manager::ChangeLEDs &msg){
  if (msg.id == TOWER_ID){
    updateChargeLEDs(msg.charge_leds);
    changeStatusLED(msg.status_led_color);
  }
}

void resetCallback(const std_msgs::Bool& reset_msg){
  if (reset_msg.data){
    resetVariables();
  }
}

game_manager::ButtonState bt_msg;
game_manager::TiltSensor tilt_msg;

ros::Publisher bt_pub("tower/button_state", &bt_msg);
ros::Publisher tilt_pub("tower/tilt_sensor", &tilt_msg);

ros::Subscriber <game_manager::ChangeLEDs> sub("game_manager/towers/ChangeLEDs", &ledCallback); 
ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi(){
  
  WiFi.begin(ssid, password);

  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to "); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void changeStatusLED(const bool color[]){
  for (int i=0; i<2; i++){
    digitalWrite(STATUS_LED[i],color[i]);
  }
}



void resetVariables(){
  
  feedback_blink_state  = LOW;   // keeps the RED/GREEN LED blink state.
  charge_blink_state    = LOW;   // keeps the charging LED blink state.
  
  button_state          = LOW;   // Keeps the current button state (LOW/HIGH)
  last_button_state     = LOW;   // Controls the previous state of the button (LOW/HIGH)
  last_button_state  
  
  blink_timer           = 0;     // Control the blink time of the tower RED/GREEN LED.
  charge_blink_timer    = 0;     // Control the blink time of the tower charging LED.
  n_leds_ON             = 0;     // keeps how many charging LEDs are on.
  
  isGameRunning         = false;  // Keeps the state of the game: True (Game is ON).
  has_fallen            = false; // True if the tower has fallen.
  is_captured           = false; // True if player has charged the LEDs and the tower did not fall.
  turnOFFAllChargeLEDs();        // turn all LEDs OFF.last_debounce_time = 0;
  
  charge_blink_timer = 0;
  blink_state = false;
  
  changeStatusLED(RED_COLOR);               // set the RED_LED ON, at the beginning.
  digitalWrite(SCHMITT_TRIGGER_PIN,LOW);    
}

/*
 * Handles the button inputs avoiding bouncing.
 */
bool isNewButtonState(){
  // read the state of the switch into a local variable:
  int reading = digitalRead(INPUT_PIN);

  bool is_new_state = false;

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != last_button_state) {
    // reset the debouncing timer
    last_debounce_time = millis();
  }

  if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != button_state) {
      button_state = reading;
      is_new_state = true;
    }

  }

  // save the reading. Next time through the loop, it'll be the last_button_state:
  last_button_state = reading;
  return is_new_state;
}

/*
 * Checks whether tower has fallen
 */
int hasFallen(){
  has_fallen = digitalRead(TILT_SENSOR_PIN);
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  
  pinMode(INPUT_PIN, INPUT);              
  pinMode(TILT_SENSOR_PIN, INPUT);
  
  for (int i=0; i < 6;i++){       // set all LEDS pins to OUTPUT mode
      pinMode(led_array[i], OUTPUT);
  }

  nh.initNode();
  nh.advertise(bt_pub);
  nh.advertise(tilt_pub);

  resetVariables();
}

void publisHasFallen(){
  tilt_msg.header.stamp = nh.now();
  tilt_msg.id = TOWER_ID;
  tilt_msg.value = true;
  tilt_pub.publish(&tilt_msg);
}

void loop() {

  // Checks whether button was pressed
  if (isNewButtonState()){
    bt_msg.header.stamp = nh.now();
    bt_msg.id = TOWER_ID;
    bt_msg.value = button_state;
    bt_pub.publish(&bt_msg);
  }

  // Check whether tower has fallen
  if (hasFallen()){
    publisHasFallen();
  }

  if(isGameRunning) {
      if (!has_fallen || !is_captured){
          checkFall();     // Checks whether tower has fallen
    
          if (has_fallen){
            changeStatusLED(RED_COLOR);
          }else if (!is_captured){ 
              if (button_state == HIGH){
                  // Pressing feedback: blinks LED to notify the button is being pressed.
                  if((millis() - charge_blink_timer) > CHARGE_BLINKING_INTERVAL){
                      charge_blink_timer = millis();                  // reset the timer
                      charge_blink_state = !charge_blink_state;       // invert the blinking_state.
    
                      if (charge_blink_state){
                        changeStatusLED(BLUE_COLOR);
                      }else{
                        changeStatusLED(BLANK_COLOR);
                      }
                  }
                  
                  // Count time of button press
                  if ((millis() - press_timer) > LED_CHARGING_TIME){
                    press_timer = millis();
                    n_leds_ON += 1;           // increase LED counter
                  }
    
                  checkCapture();  
              }else{
                changeStatusLED(RED_COLOR);
              }
          }
      }
      
    }else{        // Game is not running
        // Check interval for RED LED blinking.
        if((millis() - blink_timer) > BLINKING_INTERVAL){
            blink_timer = millis();           // reset the timer
            feedback_blink_state = !feedback_blink_state;       // invert the blinking_state.
            if (feedback_blink_state){
              changeStatusLED(RED_COLOR);
            }else{
              changeStatusLED(BLANK_COLOR);
            }
        }
    }
    
    updateChargeLEDs();
  
  nh.spinOnce();  // has to be always present otherwise we run into sync problems with ros.
  delay(5);
}
