//This sketch is from a tutorial video for networking more than two nRF24L01 tranciever modules on the ForceTronics YouTube Channel
//the code was leverage from the following code http://maniacbug.github.io/RF24/starping_8pde-example.html
//This sketch is free to the public to use and modify at your own risk
#define ACC_WARNING_LED 8
#define BUZZER_PIN  6
#define OFF 0
#define ON 1

////// TRANSCEIVER PINS ///////
#define CSN_PIN         9    //This pin is used to set the nRF24 to standby (0) or active mode (1)
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
int nTransmitters = 5; // the number of transmitters
unsigned long started_waiting_at;
unsigned long previousTimeForBatBeeping;
unsigned long previousACCtime;     // last acc sample time that we saw. 
int battery_buzzer_state;
bool timeout;
float voltage;


/*rAddress and wAddress are com pipe addresses for the towers and accelerometer.
 * rAddress[] = {tower, tower, tower, tower, accelerometer};
 * wAddress[] = {tower, tower, tower, tower, accelerometer};
 */
const uint64_t rAddress[] = {0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0E9LL, 0xF0F0F0F0B9LL}; //Create pipe addresses for the 4 nodes to recieve data, the "LL" is for LongLong type
const uint64_t wAddress[] = {0xB00B1E50D2LL, 0xB00B1E50C3LL, 0xB00B1E50B1LL, 0xB00B1E50A4LL, 0xB00B1E50C4LL};

const int ACC_PIPE_INDEX = 5;   // accelerometer communication pipe rAddress index.
const int ACC_BAT_INDEX  = 6;

void setup()   
{
  Serial.begin(57600);  //start serial to communication
  RFtransmitter.begin();  //Start the nRF24 module

  RFtransmitter.setPALevel(RF24_PA_MAX);
  RFtransmitter.setDataRate( RF24_250KBPS ) ;
  
  for (int i=1;i<nTransmitters+1;i++){
    RFtransmitter.openReadingPipe(i,rAddress[i-1]);      //open pipe o for recieving meassages with pipe address
  }
  
  RFtransmitter.startListening();                 // Start listening for messages
  pinMode(ACC_WARNING_LED, OUTPUT); 
  pinMode(BUZZER_PIN, OUTPUT);  
}

void loop()  
{   
    byte pipeNum = 0; //variable to hold which reading pipe sent data
    started_waiting_at = millis();
    timeout = false;
    
    /* Checking battery (a voltage less then 1.77 correspond to battery level at 20V, 1.947 correspond to battery level at 22V) */
    voltage = (analogRead(A2) * 5.015) / 1024.0;
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


    /* PUBLIC THE VOLTAGE */
    Serial.print(ACC_BAT_INDEX);
    Serial.print(F(","));
    Serial.print(voltage);
    Serial.print(F("\n"));

    while (!RFtransmitter.available() && !timeout){
      if (previousACCtime - started_waiting_at > 250)
        timeout = true;
    }
  
    if (timeout) {
      digitalWrite(ACC_WARNING_LED, LOW);
    }else{
    
      //Check if received data from transmitters.
      while(RFtransmitter.available(&pipeNum)){
  
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
            Serial.print(F(","));
            Serial.print(voltage);
            Serial.print(F("\n"));
          }else{
            RFtransmitter.read(&acc_data, sizeof(acc_data));
            previousACCtime = millis();
            digitalWrite(ACC_WARNING_LED, HIGH);
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
            Serial.print(F(","));
            Serial.print(voltage);
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
    bool success; 												//variable to track if write was successful
    RFtransmitter.stopListening();								//Stop listening, stop recieving data.
    RFtransmitter.openWritingPipe(wAddress[xMitter-1]);			//Open writing pipe to the nRF24 that guessed the right number
    if(!RFtransmitter.write(&daNumber, 1))  worked = false;		//write the correct number to the nRF24 module, and check that it was recieved
    else success = true; 										//it was recieved
    RFtransmitter.startListening(); 							//Switch back to a reciever
    return success;  											//return whether write was successful
}/*

void beep(unsigned char delayms){
  analogWrite(BUZZER_PIN, delayms);      // Almost any value can be used except 0 and 255
}
