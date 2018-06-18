
/*
 * Author: Ewerton Lopes
 * Politecnico di Milano, June 18, 2018
 * 
 */
#include <ros.h>
#include <std_msgs/Int8.h>

#define BUZZER_PIN  6
#define BAT_PROBE_LED 8
#define BATTERY_PIN A2
#define OFF 0
#define ON 1
#define BAT_CHK_TIMEOUT_TRH 3000     // publishes battery data every 3 secs.
#define BEEPING_INTERVAL 400
#define LOW_BATTERY_VOLTAGE 22
#define VOLTAGE_TRH 1.947   // A voltage less then 1.77 correspond to battery 
                            // level at 20V, 1.947 correspond to battery level at 22V.

unsigned long last_battery_check_time;
unsigned long last_battery_beep_time;
unsigned long last_acc_time;          // last acc sample time that we saw. 
int should_beep;


ros::NodeHandle nh;
std_msgs::Int8 bat_msg;

ros::Publisher bat_pub("arduino/battery", &bat_msg);


void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(1000);
  nh.advertise(bat_pub);
  delay(1000);

  pinMode(BAT_PROBE_LED, OUTPUT); 
  pinMode(BUZZER_PIN, OUTPUT);
  
}

void loop(){

    // Publish battery state every 3 seconds!
    if (millis() - last_battery_check_time > BAT_CHK_TIMEOUT_TRH){
        last_battery_check_time = millis();
        float voltage = checkBatteryLevel();
        if (voltage != 0){
          digitalWrite(BAT_PROBE_LED, HIGH);
        }
        bat_msg.data = voltage;
        bat_pub.publish(&bat_msg);
    }
    
    nh.spinOnce();
    delay(3);
    digitalWrite(BAT_PROBE_LED, LOW);
}


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
