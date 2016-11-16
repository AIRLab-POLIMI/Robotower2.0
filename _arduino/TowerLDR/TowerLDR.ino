int pinArray[] = {8,9,10,11,12,13};
int countPinArray[] = {8,9,10,11};
int pinRedLED = 12;
int pinGreenLED = 13;
int analogPin = A0;

enum timerOrientation{
  increase,
  decrease,
  keepIncreasing,
  halt
};

timerOrientation timerState = halt;
volatile int buttonState = LOW;
static unsigned long lastTickMillis = 0;

void setup() {
  for (int count=0; count < 6;count++){
      pinMode(pinArray[count], OUTPUT);
  }
  attachInterrupt(0, ledOnOff, CHANGE);
  Serial.begin(9600);
  digitalWrite(pinRedLED,HIGH);

  pinMode(3,INPUT);
}

int counter = 0;
bool playing = true;
int last_input_value = LOW;
 
void loop() {


    float ldr = analogRead(analogPin);    // read the input pin
    Serial.println(ldr);
    if (ldr < 614){
      digitalWrite(pinRedLED,LOW);
      playing = false;
      turnOffDisplay();
    }
    
    if (counter == 9){
      digitalWrite(7,HIGH);
      if((millis() - lastTickMillis) > 1000){
        lastTickMillis = millis();
        counter = (counter + 1) % 10;
        updateLEDs(counter);
        digitalWrite(pinRedLED,LOW);
        digitalWrite(pinGreenLED,HIGH);
        turnOffDisplay();
        playing = false;
      }
    }
    
    if(playing) {
      int switch_input_value = buttonState;
      //Serial.println(switch_input_value);
      if (((last_input_value == LOW) && (switch_input_value == HIGH)) || ((last_input_value == HIGH) && (switch_input_value == HIGH))){
        timerState = increase;
      }else if (last_input_value == HIGH && switch_input_value == LOW){
        timerState = halt;
      }else if ((last_input_value == LOW) && (switch_input_value == LOW) && counter != 0){
        timerState=halt;
      }else{
        timerState=halt;
      }
      last_input_value = switch_input_value;
      modifyDisplay(timerState);
      updateLEDs(counter);
    }
    
}

void modifyDisplay(timerOrientation orientation){ 
  if (orientation == increase){
    if((millis() - lastTickMillis) > 1000){
        lastTickMillis = millis();
        counter = (counter + 1) % 10;
    }
  }else if (orientation == decrease){
    if((millis() - lastTickMillis) > 1000){
        lastTickMillis = millis();
        counter = (counter - 1) % 10;
    }
  }
}

void turnOffDisplay(){
  for(int i=0; i < 4; i++){
    digitalWrite(countPinArray[i],LOW);
  }
}

void updateLEDs(int counting){
  int up = (4*counting)/9;
  for(int i=0; i < 4; i++){
    if (i < up){
      digitalWrite(countPinArray[i],HIGH);
    }else{
      digitalWrite(countPinArray[i],LOW); 
    }
  }
}

void ledOnOff() {
    buttonState = !buttonState;
}
