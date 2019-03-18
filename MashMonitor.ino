#include <RotaryEncoder.h>

//LCD Display
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h> // https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
//                    addr, en,rw,rs,d4,d5,d6,d7
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// On most Arduino boards, SDA (data line) is on analog input pin 4, and SCL (clock line) is on analog input pin 5.
// On the newer Arduino UNO (The "V3" pinout), the SCL and SDA pins are also on two new leftmost top pins.

//https://github.com/mathertel/RotaryEncoder
RotaryEncoder encoder(A2, A3);

//Temp Sensor includes
#include <OneWire.h>
#include <DallasTemperature.h>

/*                   1111111111
           01234567890123456789
           Boiler 1   Boiler 2
Current     68.4˚C *  144.9˚C *    Current
Target     100.0˚C <<  43.9˚C <-   Target
Tolerance    0.5˚C <-   0.5˚C <-   Tolerance
Output     100% <-  Priority1 <-   Mode
*/

#define ONE_WIRE_BUS 5 //
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress b1Probe = { 0x28, 0xFF, 0xFC, 0x69, 0x22, 0x17, 0x03, 0x77 }; 
DeviceAddress b2Probe = { 0x28, 0xFF, 0x01, 0x6F, 0x22, 0x17, 0x03, 0x34 }; 

const char degree = (char)223;
const byte backPin = 3;  //back button
const byte selectPin = 2;  //next button
const byte powerPin = 9;   //AC triac | 488 Hz PWM
const byte boilerSelectPin = 4; //relay


const unsigned long selectDuration =  300; //ms between counting button presses
unsigned long lastSelect = 0;

const unsigned long postponeDuration =  500; //ms after input, before monitoring resumes
unsigned long lastPostpone = 0;

const unsigned long monitorDuration =  2000; // ms between temperature updates
unsigned long lastMonitor = 0;

float b1Temp;
float b2Temp;

int inputs[10] = {67, 0, 1, 0, 80, 0, 1, 0, 0, 0}; //default values
enum inputMode {
  B1_TARGET, B1_TARGET_D, B1_TOLERANCE, B1_TOLERANCE_D, B2_TARGET, B2_TARGET_D, B2_TOLERANCE, B2_TOLERANCE_D,  POWER, MODE
};
int inputLimit[] = {
  100,       9,           10,           9,              100,       9,           10,           9,               100,   6};
int currentInput = MODE;
int previousInput = POWER;
boolean switchedToBoiler2 = false;

const byte X = 0;
const byte Y = 1;
int inputPosn[10][2] = { {0,1},  //B1 Target
                         {0,1},
                         {0,2},  //B1 Tolerance
                         {0,2},
                         {10,1}, //B2 Target
                         {10,1},
                         {10,2}, //B2 Tolerance
                         {10,2},
                         {0,3},  //Power
                         {7,3}   //Mode
                         };

#define U_LEFT 0  //unitsLeft
#define U_RIGHT 1 //unitsRight
#define D_LEFT 2  //decimalsLeft
#define D_RIGHT 3 //decimalsRight

String blankArrow = "  ";
const String arrowType[]={"<-", "<-", "<<", "<<"};

//x,y,arrowType
int arrow[10][3] = { {8,1,U_LEFT},     //B1 Target
                     {8,1,D_LEFT},     //B1 Target decimal
                     {8,2,U_LEFT},     //B1 Tolerance
                     {8,2,D_LEFT},     //B1 Tolerance decimal
                     {18,1,U_RIGHT},   //B2 Target
                     {18,1,D_RIGHT},   //B2 Target decimal
                     {18,2,U_RIGHT},   //B2 Tolerance
                     {18,2,D_RIGHT},   //B2 Tolerance decimal
                     {5,3,U_LEFT},     // %OUTPUT
                     {18,3,U_RIGHT}    //Mode 
                   };


enum runMode{
  OFF, MANUAL1, MANUAL2, AUTO1, AUTO2, PRIORITY1, PRIORITY2
};
const String modes[]={"       Off",
                      "  Manual 1",
                      "  Manual 2",
                      "    Auto 1",
                      "    Auto 2",
                      "Priority 1",
                      "Priority 2"};

void setup() {
  Serial.begin(115200);  // Serial connection from ESP-01 via 3.3v console cable
  Serial.print("\n\r \n\r Started...");
  
  // put your setup code here, to run once:
  lcd.begin(20,4);
  lcd.backlight();

  RotaryEncoder encoder(A2, A3);
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11); // This enables the interrupt for pin 2 and 3 of Port C.

  Serial.println("Initializing Mash monitor...");

  sensors.begin();
  // 9 bits  0.5°C 93.75 ms; 10 bits 0.25°C  187.5 ms; 11 bits 0.125°C 375 ms; 12 bits   0.0625°C  750 ms
  sensors.setResolution(b1Probe, 12); //resolution 9 to 12 bits
  sensors.setResolution(b2Probe, 12);

  
  pinMode(selectPin, INPUT_PULLUP); //INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(selectPin), selectPressed, LOW);
  pinMode(backPin, INPUT_PULLUP);   //INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(backPin), backPressed, LOW);
  pinMode(powerPin, OUTPUT);
  pinMode(boilerSelectPin, OUTPUT);
}


void selectPressed(){
  buttonPressed(MODE, B1_TARGET, currentInput + 1);
}


void backPressed(){
  buttonPressed(B1_TARGET, MODE, currentInput - 1);
}

void buttonPressed(byte endOfList, byte startOfList, byte nextItem){
  if (millis() > (lastSelect + selectDuration)) {
    if(currentInput == endOfList){
      currentInput = startOfList;
    }
    else {
      currentInput = nextItem;
    }

    encoder.setPosition(inputs[currentInput]);
    lastSelect = millis();
    lastPostpone = millis(); // pause monitoring, while input is happening
  }
}


// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}


void loop() {
  monitorMash();
  checkInputs();
}


void checkInputs(){
  //Select button pressed
  if(currentInput != previousInput){
    clearArrows();
    updateDisplay();
    previousInput = currentInput;
  }

  //rotary encoder moved
  int readPos = encoder.getPosition();
  if (inputs[currentInput] != readPos) {
    lastPostpone = millis(); // pause monitoring, while input is happening
    if((readPos <= inputLimit[currentInput])&& (readPos >= 0)){
      inputs[currentInput] = readPos;
    }
    else{
      encoder.setPosition(inputs[currentInput]);
    }

    if((currentInput == POWER) && (inputs[MODE] == OFF)){  //Can't output when off
      encoder.setPosition(0);
      inputs[currentInput] = 0;
    }

    updateDisplay();
  }  
}

void updateDisplay(){
    printInputValues();
    printArrow();
    printTemperatures();
    printHeatIndicator();
}

void senseTemperatures(){
  sensors.requestTemperatures();
  b1Temp = sensors.getTempC(b1Probe);
  b2Temp = sensors.getTempC(b2Probe);
}

void printTemperatures(){
  printValue(0,0, formatValidTemperature(b1Temp));
  printValue(10,0, formatValidTemperature(b2Temp));  
}

String formatValidTemperature(float temp){
  String strRtn = "Error!  ";
  if(temp > -99.0){
    strRtn = formatTemperature(temp);
  }

  return strRtn;
}

void monitorMash() {
  if (millis() > (lastMonitor + monitorDuration) &&
      millis() > (lastPostpone + postponeDuration) ) {
    senseTemperatures();

    switch(inputs[MODE]){
      case OFF:
        setOff();
        break;
      case AUTO1:
        setAuto(false,
                b1Temp,
                assembleFloat(inputs[B1_TARGET], inputs[B1_TARGET_D], 100.0),
                assembleFloat(inputs[B1_TOLERANCE], inputs[B1_TOLERANCE_D], 100.0)
               );
        break;
      case AUTO2:
        setAuto(true,
                b2Temp,
                assembleFloat(inputs[B2_TARGET], inputs[B2_TARGET_D], 100.0),
                assembleFloat(inputs[B2_TOLERANCE], inputs[B2_TOLERANCE_D], 100.0)
               );
        break;
      case MANUAL1:
        setManual(false);
        break;
      case MANUAL2:
        setManual(true);
        break;
      case PRIORITY1:
        priorityMonitor( b1Temp,
                         assembleFloat(inputs[B1_TARGET], inputs[B1_TARGET_D], 100.0),
                         assembleFloat(inputs[B1_TOLERANCE], inputs[B1_TOLERANCE_D], 100.0),
                         false,
                         b2Temp,
                         assembleFloat(inputs[B2_TARGET], inputs[B2_TARGET_D], 100.0),
                         assembleFloat(inputs[B2_TOLERANCE], inputs[B2_TOLERANCE_D], 100.0),
                         true);
        break;
      case PRIORITY2:
        priorityMonitor( b2Temp,
                         assembleFloat(inputs[B2_TARGET], inputs[B2_TARGET_D], 100.0),
                         assembleFloat(inputs[B2_TOLERANCE], inputs[B2_TOLERANCE_D], 100.0),
                         true,
                         b1Temp,
                         assembleFloat(inputs[B1_TARGET], inputs[B1_TARGET_D], 100.0),
                         assembleFloat(inputs[B1_TOLERANCE], inputs[B1_TOLERANCE_D], 100.0),
                         false);
        break;
    }

    updateDisplay();
    lastMonitor = millis();
  }
}

void setOff(){
  analogWrite(powerPin, 0);
  digitalWrite(boilerSelectPin, 0);
  inputs[POWER] = 0.0;
  switchedToBoiler2 = false;
}


/***********************************************
 *  Auto Monitoring
 ***********************************************/
void setAuto(boolean isBoiler2, float boilerTemp, float boilerTarget, float boilerTolerance){

  digitalWrite(boilerSelectPin, isBoiler2);
  switchedToBoiler2 = isBoiler2;

  float diff = max(0.0, boilerTarget - boilerTemp);
  float factor = 0.0;

  if(diff > 0.0){
    factor = 256.0/boilerTolerance; 
  }

  int power = min(255,factor * diff);
  inputs[POWER] = (float)power/2.55;
  analogWrite(powerPin, power);
}


/***********************************************
 *  Manual Setting
 ***********************************************/
void setManual(boolean divert){
  int power = inputs[POWER] * 2.55;
  switchedToBoiler2 = divert;
  analogWrite(powerPin, power);
  digitalWrite(boilerSelectPin, divert);
}


/*(*********************************************
 *  Priority Monotoring
 ***********************************************/

byte priorityPwr = 0;

void priorityMonitor(float primeTemp,  float primeTarget,  float primeTolerance,  boolean primeDiverted,
                     float secondTemp, float secondTarget, float secondTolerance, boolean secondDiverted){


  boolean primeUsingPower = compareTemp(primeTemp, primeTarget, primeTolerance, primeDiverted);

  if (!primeUsingPower) {
    compareTemp(secondTemp, secondTarget, secondTolerance, secondDiverted);
  }

  inputs[POWER] = ((float)priorityPwr)/2.55;
  digitalWrite(boilerSelectPin, switchedToBoiler2);
  analogWrite(powerPin, priorityPwr);
}

boolean compareTemp(float temp, float target, float tolerance, boolean bDiverted){
  if(temp < target - tolerance){ //below tolerance
    switchedToBoiler2 = bDiverted;
    priorityPwr = 255;
    return true;
  }
  else if(temp >= target){   //on or above target
    priorityPwr = 0;
    return false;
  }
  else{  //in tolerance zone
    if((priorityPwr > 0) &&
       (switchedToBoiler2 == bDiverted)){  //still heating
      return true;
    }
  }

  return false;
}


/***********************************************
 *  Display Arrow and Values
 ***********************************************/
void printArrow(){
  String a = arrowType[arrow[currentInput][2]];
  printValue((arrow[currentInput][X]),arrow[currentInput][Y], a);
}

void clearArrows(){ 

  //LHS
  printValue(arrow[B1_TARGET][X], arrow[B1_TARGET][Y], blankArrow);
  printValue(arrow[B1_TOLERANCE][X], arrow[B1_TOLERANCE][Y], blankArrow);
  printValue(arrow[POWER][X], arrow[POWER][Y], blankArrow);

  //RHS
  printValue(arrow[B2_TARGET][X], arrow[B2_TARGET][Y], blankArrow);
  printValue(arrow[B2_TOLERANCE][X], arrow[B2_TOLERANCE][Y], blankArrow);
  printValue(arrow[MODE][X], arrow[MODE][Y], blankArrow);
}


void printInputValues(){
  printValue(inputPosn[B1_TARGET][X],
             inputPosn[B1_TARGET][Y],
             formatInputTemperature(inputs[B1_TARGET], inputs[B1_TARGET_D]));

  printValue(inputPosn[B1_TOLERANCE][X],
             inputPosn[B1_TOLERANCE][Y],
             formatInputTemperature(inputs[B1_TOLERANCE], inputs[B1_TOLERANCE_D]));

  printValue(inputPosn[POWER][X],
             inputPosn[POWER][Y],
             pad((String)inputs[POWER], 3)+"%");

  printValue(inputPosn[B2_TARGET][X],
             inputPosn[B2_TARGET][Y],
             formatInputTemperature(inputs[B2_TARGET], inputs[B2_TARGET_D]));

  printValue(inputPosn[B2_TOLERANCE][X],
             inputPosn[B2_TOLERANCE][Y],
             formatInputTemperature(inputs[B2_TOLERANCE],inputs[B2_TOLERANCE_D]));

  printValue(inputPosn[MODE][X],
             inputPosn[MODE][Y],
             formatMode(inputs[MODE]));
}

void printHeatIndicator(){
  String b1 = " ";
  String b2 = " ";
  if( (inputs[POWER] > 0) && (inputs[MODE] > OFF) ){
    if(switchedToBoiler2){b2 ="*";}
    else{b1="*";}
  }

  printValue(8, 0, b1);
  printValue(18, 0, b2);
}

String formatMode(float m){
  return modes[(int)m];
}

float assembleFloat(int units, int tenths, float maximum){
  float result = (float)units;
  result += (float)tenths/10.0;
  result = min(result, maximum);
  return result;
}

String formatInputTemperature(int u, int t){
  return formatTemperature(assembleFloat(u, t, 100.0));
}

String formatTemperature(float t){
  return pad(oneDecPlace(t), 5)+degree+"C";
}

void printValue(int x, int y, String val){
  lcd.setCursor(x,y);
  lcd.print(val);
}

String pad(String num, int width){
  if(num.length()<width){
    return pad(" " + num, width);
  }
  return num;
}

String oneDecPlace(float reading){
  int a = round(reading * 10);
  float b = a / 10.0;
  String str = (String)b;
  return str.substring(0, str.length()-1);
}