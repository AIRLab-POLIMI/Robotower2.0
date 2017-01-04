/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, January 3, 2017
 * This code shall be used only for testing the data transmission 
 * by sending dummy data across a nRF24L01 – 2.4GHz RF Transceiver Module 
 * link connection. Please, set up the wiring following the transceiver 
 * pins defines.
 */
#include <SPI.h>
#include "RF24.h" 
#include <NewPing.h>

////// TRANSCEIVER PINS ///////
#define CSN_PIN         9
#define CE_PIN          10
#define MOSI_PIN        11
#define MISO_PIN        12
#define SCK_PIN         13
///////////////////////////////

// RF Trasmitter definition.
RF24 RFtransmitter(CE_PIN, CSN_PIN);
byte addresses[][6] = {"0"};  // RF pipe at address 0.

// Data to be sent by the RF module.
struct package{
  boolean isTowerDown;
  boolean hasWon;
  boolean isGameActive;
  float distances[3]; // 0 - left sensor, 1- center sensor, 2- right sensor
  int ledMask[4];   // which LEDs are on.
}data;


void setup() {
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
    //Setting dummy data to be transmitted.
    data.isGameActive = true;
    data.hasWon = false;
    data.isTowerDown = isTowerDown;
    for(int i=0;i<N_CHARGE_LEDS; i++){
      data.ledMask[i] = false; 
    }

    // Transmit data.
    RFtransmitter.write(&data, sizeof(data));
}
