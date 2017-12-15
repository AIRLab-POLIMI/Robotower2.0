/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, December 13, 2017
 * 
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "MPU6050_6Axis_MotionApps20.h" // for acc_package type definitions


#define TOWER_SAMPLE_TIME 500
#define ACC_WARNING_LED 8
#define BUZZER_PIN  6
#define BATTERY_PIN A2
#define NUM_TOWERS 4
#define OFF 0
#define ON 1
#define BAT_CHK_TIMEOUT_TRH 5000     // publishes battery data every 5 secs.
#define NUM_TOWERS 5
#define ACC_TIMEOUT_TRH 100
#define BEEPING_INTERVAL 400
#define LOW_BATTERY_VOLTAGE 22
#define ACC_PIPE_INDEX 5    // accelerometer r_addresses index.
#define BAT_INDEX  6 
#define VOLTAGE_TRH 1.947   // A voltage less then 1.77 correspond to battery 
                            // level at 20V, 1.947 correspond to battery level at 22V.

/*** TRANSCEIVER PINS 
 -------------------------
| vcc | csn | mosi | irq  |
| gnd | ce  | sck  | miso |
 -------------------------  ***/
#define CSN_PIN         9    
#define CE_PIN          10
#define MOSI_PIN        11
#define MISO_PIN        12
#define SCK_PIN         13

struct tower_package{
  boolean button;
  int t_status;
  int leds[4];
  int num_presses;
};

struct acc_package{
  VectorInt16 aaWorld;
  VectorInt16 gyro;
};

struct tower_package tower_data;
struct acc_package acc_data;
static unsigned long last_tw_sample_time;
static unsigned long last_battery_check_time;
static unsigned long last_battery_beep_time;
static unsigned long last_acc_time;          // last acc sample time that we saw. 
int should_beep;
bool is_notify;


RF24 RFtransmitter(CE_PIN, CSN_PIN);

/*r_addresses and w_addresses are com pipe addresses for the towers and accelerometer.
 * w_addresses[] = {tower, tower, tower, tower, accelerometer};
 */
const uint64_t w_addresses[] = {0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0E9LL, 0xF0F0F0F0B9LL};
const uint64_t r_addresses[] = {0xB00B1E50D2LL};


int com_counter= 0;     // keeps track of which tower to sample next

int msg[1] = {1};

void setup(){
  Serial.begin(57600);
  RFtransmitter.begin();
  RFtransmitter.setChannel(120);
  RFtransmitter.setPALevel(RF24_PA_MIN);
  RFtransmitter.setDataRate(RF24_250KBPS);

  RFtransmitter.setAutoAck(true);
  RFtransmitter.enableAckPayload();
  RFtransmitter.enableDynamicPayloads();
  RFtransmitter.stopListening();
  RFtransmitter.setRetries(15,15);
  RFtransmitter.setPayloadSize(sizeof(tower_package));
  
  pinMode(ACC_WARNING_LED, OUTPUT); 
  pinMode(BUZZER_PIN, OUTPUT);
  
  last_acc_time = millis();
  is_notify = false;
}


void writeAccToSerial(){
    Serial.print(5);        //ACC code is 4.
    Serial.print(F(","));
    Serial.print(acc_data.aaWorld.x);
    Serial.print(F(","));
    Serial.print(acc_data.aaWorld.y);
    Serial.print(F(","));
    Serial.print(acc_data.aaWorld.z);
    Serial.print(F(","));
    Serial.print(acc_data.gyro.x);
    Serial.print(F(","));
    Serial.print(acc_data.gyro.y);
    Serial.print(F(","));
    Serial.print(acc_data.gyro.z);
    Serial.print(F("\n"));
}

void writeTowerToSerial(int id){
    Serial.print(id);
    Serial.print(F(","));
    Serial.print(tower_data.button);
    Serial.print(F(","));
    Serial.print(tower_data.t_status);
    Serial.print(F(","));
    Serial.print(tower_data.leds[0]);
    Serial.print(F(","));
    Serial.print(tower_data.leds[1]);
    Serial.print(F(","));
    Serial.print(tower_data.leds[2]);
    Serial.print(F(","));
    Serial.print(tower_data.leds[3]);
    Serial.print(F(","));
    Serial.print(tower_data.num_presses);
    Serial.print(F("\n"));
}

void loop(){
  
    float voltage = checkBatteryLevel();

    //Get player acceleration data
    RFtransmitter.openWritingPipe(w_addresses[4]);
    RFtransmitter.setPayloadSize(sizeof(acc_package));
    delay(3);
    if(RFtransmitter.write(msg,sizeof(msg))){
        if(RFtransmitter.isAckPayloadAvailable()){
            RFtransmitter.read(&acc_data,sizeof(acc_data));
                last_acc_time = millis();
                digitalWrite(ACC_WARNING_LED, HIGH);
                writeAccToSerial();
        }
    }


    // Get tower data
    if (millis() - last_tw_sample_time > TOWER_SAMPLE_TIME){
        last_tw_sample_time = millis();

        int com_pipe_to_sample = com_counter % (NUM_TOWERS + 1);

        if (com_pipe_to_sample == 5){
            Serial.print(BAT_INDEX);
            Serial.print(F(","));
            Serial.print(voltage);
            Serial.print(F("\n"));
        }else{    // Sample towers
            //Get tower data
            RFtransmitter.openWritingPipe(w_addresses[com_pipe_to_sample]);
            delay(5);
            RFtransmitter.setPayloadSize(sizeof(tower_package));
            if(RFtransmitter.write(msg,sizeof(msg))){
                if(RFtransmitter.isAckPayloadAvailable()){
                  RFtransmitter.read(&tower_data,sizeof(tower_data));
                      writeTowerToSerial(com_pipe_to_sample+1);
                  }
            }
        }

        com_counter++;
    }

    // if acc data timed out turn OFF ACC_WARNING_LED.
    if (millis() - last_acc_time > ACC_TIMEOUT_TRH){
        last_acc_time = millis();
        digitalWrite(ACC_WARNING_LED, LOW);
    }
}

//This function turns the reciever into a transmitter briefly to tell one of the nRF24s
//in the network that it guessed the right number. Returns true if write to module was
//successful
/*
bool sendCorrectNumber(byte xMitter) {
    bool success; 												//variable to track if write was successful
    RFtransmitter.stopListening();								//Stop listening, stop recieving data.
    RFtransmitter.openWritingPipe(w_addresses[xMitter-1]);			//Open writing pipe to the nRF24 that guessed the right number
    if(!RFtransmitter.write(&daNumber, 1))  worked = false;		//write the correct number to the nRF24 module, and check that it was recieved
    else success = true; 										//it was recieved
    RFtransmitter.startListening(); 							//Switch back to a reciever
    return success;  											//return whether write was successful
}*/


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

void beep(unsigned char delayms){
  analogWrite(BUZZER_PIN, delayms);      // Almost any value can be used except 0 and 255
}
