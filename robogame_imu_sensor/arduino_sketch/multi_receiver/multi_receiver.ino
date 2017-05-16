//This sketch is from a tutorial video for networking more than two nRF24L01 tranciever modules on the ForceTronics YouTube Channel
//the code was leverage from the following code http://maniacbug.github.io/RF24/starping_8pde-example.html
//This sketch is free to the public to use and modify at your own risk
#define BUZZER_PIN 6
#define WARNING_LED 9

#define OFF 0
#define ON 1

////// TRANSCEIVER PINS ///////
#define CSN_PIN         8//This pin is used to set the nRF24 to standby (0) or active mode (1)
#define CE_PIN          10   //This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out
#define MOSI_PIN        11
#define MISO_PIN        12
#define SCK_PIN         13

#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include "MPU6050_6Axis_MotionApps20.h" // for acc_package type definitions

struct tower_package{
  boolean isTowerDown;
  boolean isCaptured;
  boolean isTowerEnable;
  boolean isButtonPressed;
  float distances[3]; // 0 - left sensor, 1- center sensor, 2- right sensor
  int ledMask[4];   // which LEDs are on.
  int pressCounter;
};

struct acc_package{
  Quaternion q;
  VectorInt16 aaWorld;
  VectorInt16 gyro;
};

struct tower_package tower_data;
struct acc_package acc_data;

RF24 RFtransmitter(CE_PIN, CSN_PIN);
int timeout_limit = 250; // in millis
int nTransmitters = 5; // the number of transmitters;
boolean battery_buzzer_state = OFF;

unsigned long previous_time = 0;        // will store last time LED was updated;
unsigned long timeForBuzzer = 0;
unsigned long previousTimeForBatBeeping = 0;

/*rAddress and wAddress are com pipe addresses for the towers and accelerometer.
 * rAddress[] = {tower, tower, tower, tower, accelerometer};
 * wAddress[] = {tower, tower, tower, tower, accelerometer};
 */
const uint64_t rAddress[] = {0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0E9LL, 0xF0F0F0F0B9LL}; //Create pipe addresses for the 4 nodes to recieve data, the "LL" is for LongLong type
const uint64_t wAddress[] = {0xB00B1E50D2LL, 0xB00B1E50C3LL, 0xB00B1E50B1LL, 0xB00B1E50A4LL, 0xB00B1E50C4LL};

const int ACC_PIPE_INDEX = 5;   // accelerometer communication pipe rAddress index.

void setup()   
{
  Serial.begin(57600);  //start serial to communication
  RFtransmitter.begin();  //Start the nRF24 module

  RFtransmitter.setPALevel(RF24_PA_MAX);
  RFtransmitter.setDataRate( RF24_250KBPS ) ;

  pinMode(WARNING_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  RFtransmitter.startListening();                 // Start listening for messages
}

void loop()  
{   
    byte pipeNum = 0; //variable to hold which reading pipe sent data
    
    unsigned long started_waiting_at = millis();
    bool timeout = false;
  
    /* Checking battery (a voltage less then 1.77 correspond to battery level at 20V, 1.947 correspond to battery level at 22V) */
    float voltage = (analogRead(A2) * 5.015) / 1024.0;
    voltage = (22*voltage)/1.947;         // Converted voltage (approx).
    if (voltage <= 22 && voltage >= 10){
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


    while(!RFtransmitter.available(&pipeNum) && !timeout){
      if (millis() - started_waiting_at > 250)
        timeout = true;
    }
  
    if (timeout) {
      //Serial.println("Failed, reponse time out.");
      digitalWrite(WARNING_LED, HIGH);
    }else{
    
      //Check if received data from transmitters.
      while(RFtransmitter.available(&pipeNum)){

          digitalWrite(WARNING_LED, LOW);
           
          /* 0-3 represent the towers.*/
          if (pipeNum != ACC_PIPE_INDEX){ 
            RFtransmitter.read(&tower_data, sizeof(tower_data));
            Serial.print(pipeNum);
            Serial.print(F(","));
            Serial.print(tower_data.isTowerEnable);
            Serial.print(F(","));
            Serial.print(tower_data.isButtonPressed);
            Serial.print(F(","));
            Serial.print(tower_data.isCaptured);
            Serial.print(F(","));
            Serial.print(tower_data.isTowerDown);
            Serial.print(F(","));
            Serial.print(tower_data.distances[0]);
            Serial.print(F(","));
            Serial.print(tower_data.distances[1]);
            Serial.print(F(","));
            Serial.print(tower_data.distances[2]);
            Serial.print(F(","));
            Serial.print(tower_data.ledMask[0]);
            Serial.print(F(","));
            Serial.print(tower_data.ledMask[1]);
            Serial.print(F(","));
            Serial.print(tower_data.ledMask[2]);
            Serial.print(F(","));
            Serial.print(tower_data.ledMask[3]);
            Serial.print(F(","));
            Serial.print(tower_data.pressCounter);
            Serial.print(F("\n"));
          }else{
            RFtransmitter.read(&acc_data, sizeof(acc_data));
            Serial.print(pipeNum);
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
            Serial.print(F(","));
            Serial.print(acc_data.q.w);
            Serial.print(F(","));
            Serial.print(acc_data.q.x);
            Serial.print(F(","));
            Serial.print(acc_data.q.y);
            Serial.print(F(","));
            Serial.print(acc_data.q.z);
            Serial.print(F("\n"));
          }
      }
    }
}

//This function turns the reciever into a transmitter briefly to tell one of the nRF24s
//in the network that it guessed the right number. Returns true if write to module was
//successful
/*
bool sendCorrectNumber(byte xMitter) {
    bool worked; //variable to track if write was successful
    RFtransmitter.stopListening(); //Stop listening, stop recieving data.
    RFtransmitter.openWritingPipe(wAddress[xMitter-1]); //Open writing pipe to the nRF24 that guessed the right number
    if(!RFtransmitter.write(&daNumber, 1))  worked = false; //write the correct number to the nRF24 module, and check that it was recieved
    else worked = true; //it was recieved
    RFtransmitter.startListening(); //Switch back to a reciever
    return worked;  //return whether write was successful
}*/

void beep(unsigned char delayms){
  analogWrite(BUZZER_PIN, delayms);      // Almost any value can be used except 0 and 255
}
