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

#include <EEPROM.h>

/*                               1111111111
                       01234567890123456789
                       Boiler 1   Boiler 2
  Current               68.4˚C *  144.9˚C *    Current
  Target               100.0˚C <<  43.9˚C <-   Target
  Tolerance/Calibrate    0.5˚C <-   0.5˚C <-   Tolerance/Calibrate
  Output               100% <-  Priority1 <-   Mode
*/

#define ONE_WIRE_BUS 5 //
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress b1Probe = { 0x28, 0xFF, 0xFC, 0x69, 0x22, 0x17, 0x03, 0x77 };
DeviceAddress b2Probe = { 0x28, 0xFF, 0x01, 0x6F, 0x22, 0x17, 0x03, 0x34 };

const char degree = (char)223;
const byte backPin = 3;         // back button
const byte selectPin = 2;       // next button
const byte powerPin = 9;        // AC triac | 488 Hz PWM
const byte boilerSelectPin = 4; // relay


const unsigned long selectDuration =  300; //ms between counting button presses
unsigned long lastSelect = 0;

const unsigned long postponeDuration =  1000; //ms after input, before monitoring resumes
unsigned long lastPostpone = 0;

const unsigned long monitorDuration =  2000; // ms between temperature updates
unsigned long lastMonitor = 0;

float b1Temp;
float b2Temp;

int inputs[14] = {
  80,        0,           1,            0,            0,              0,
  67,        0,           1,            0,            0,              0,               0,     0
}; //default values

enum inputMode {
  B1_TARGET, B1_TARGET_D, B1_TOLERANCE, B1_CALIBRATE, B1_TOLERANCE_D, B1_CALIBRATE_D,
  B2_TARGET, B2_TARGET_D, B2_TOLERANCE, B2_CALIBRATE, B2_TOLERANCE_D, B2_CALIBRATE_D,  POWER, MODE
};

int maxLimit[] = {
  100,       9999,        10,           9,            9999,           9999,
  100,       9999,        10,           9,            9999,           9999,            100,   7
};

int minLimit[] = {
  0,         -9999,        0,          -9,            -9999,         -9999,
  0,         -9999,        0,          -9,            -9999,         -9999,              0,   0
};

int currentInput = MODE;
int previousInput = POWER;
boolean switchedToBoiler2 = false;

const byte X = 0;
const byte Y = 1;
int inputPosn[14][2] = { {0, 1}, //B1 Target
  {0, 1},  //B1 Target_D
  {0, 2},  //B1 Tolerance
  {0, 2},  //B1 Calibrate
  {0, 2},  //B1 Tolerace_D
  {0, 2},  //B1 Calibrate_D
  {10, 1}, //B2 Target
  {10, 1}, //B2 Target_D
  {10, 2}, //B2 Tolerance
  {10, 2}, //B2 Calibrate
  {10, 2}, //B2 Tolerance_D
  {10, 2}, //B2 Calibrate_D
  {0, 3},  //Power
  {7, 3}   //Mode
};

#define U_LEFT 0  //unitsLeft
#define U_RIGHT 1 //unitsRight
#define D_LEFT 2  //decimalsLeft
#define D_RIGHT 3 //decimalsRight

String blankArrow = "  ";
const String arrowType[] = {"<-", "<-", "<<", "<<"};

//x,y,arrowType
int arrow[14][3] = { {8, 1, U_LEFT},   //B1 Target
  {8, 1, D_LEFT},   //B1 Target decimal
  {8, 2, U_LEFT},   //B1 Tolerance
  {8, 2, U_LEFT},   //B1 Calibrate
  {8, 2, D_LEFT},   //B1 Tolerance decimal
  {8, 2, D_LEFT},   //B1 Calibrate decimal
  {18, 1, U_RIGHT}, //B2 Target
  {18, 1, D_RIGHT}, //B2 Target decimal
  {18, 2, U_RIGHT}, //B2 Tolerance
  {18, 2, U_RIGHT}, //B2 Calibrate
  {18, 2, D_RIGHT}, //B2 Tolerance decimal
  {18, 2, D_RIGHT}, //B2 Calibrate decimal
  {5, 3, U_LEFT},   // %OUTPUT
  {18, 3, U_RIGHT}  //Mode
};


enum runMode {
  OFF, MANUAL1, MANUAL2, AUTO1, AUTO2, PRIORITY1, PRIORITY2, CALIBRATE
};
const String modes[] = {"       Off",
                        "  Manual 1",
                        "  Manual 2",
                        "    Auto 1",
                        "    Auto 2",
                        "Priority 1",
                        "Priority 2",
                        " Calibrate"
                       };

const int CALIBRATE1_EEPROM_POSN = 0;
const int CALIBRATE1D_EEPROM_POSN = 2;
const int CALIBRATE2_EEPROM_POSN = 4;
const int CALIBRATE2D_EEPROM_POSN = 6;

void setup() {
  Serial.begin(115200); 
  Serial.print("\n\r \n\r Started...");

  // put your setup code here, to run once:
  lcd.begin(20, 4);
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

  inputs[B1_CALIBRATE] =   getEEPROMInt(CALIBRATE1_EEPROM_POSN);
  inputs[B1_CALIBRATE_D] = getEEPROMInt(CALIBRATE1D_EEPROM_POSN);
  inputs[B2_CALIBRATE] =   getEEPROMInt(CALIBRATE2_EEPROM_POSN);
  inputs[B2_CALIBRATE_D] = getEEPROMInt(CALIBRATE2D_EEPROM_POSN);

}

int getEEPROMInt(int posn){
  int _value;
  EEPROM.get(posn, _value);
   if(_value == NAN){
    _value = 0;
  }

  return _value;
}

void updateEEPROMInt(int posn, int _value){
  int val = getEEPROMInt(posn);
  if(val != _value){
    EEPROM.put(posn, _value);
    Serial.println("Written!!");
  }

}

void writeConfig(){
  updateEEPROMInt(CALIBRATE1_EEPROM_POSN, inputs[B1_CALIBRATE]);
  updateEEPROMInt(CALIBRATE1D_EEPROM_POSN, inputs[B1_CALIBRATE_D]);
  updateEEPROMInt(CALIBRATE2_EEPROM_POSN, inputs[B2_CALIBRATE]);
  updateEEPROMInt(CALIBRATE2D_EEPROM_POSN, inputs[B2_CALIBRATE_D]);
}


//Interrupt routine
void selectPressed() {
  buttonPressed(MODE, B1_TARGET, currentInput + 1, 1);
}

//Interrupt routine
void backPressed() {
  buttonPressed(B1_TARGET, MODE, currentInput - 1, -1);
}

void buttonPressed(byte endOfList, byte startOfList, byte nextItem, int incDec) {
  if (millis() > (lastSelect + selectDuration)) {
    if (currentInput == endOfList) {
      currentInput = startOfList;
    }
    else {
      currentInput = nextItem;
    }

    //Skip past Tolerance settings, if we're calibrating (Using one field for two purposes)
    while ( ( (currentInput == B1_TOLERANCE)  || (currentInput == B2_TOLERANCE)   ||
              (currentInput == B1_TOLERANCE_D) || (currentInput == B2_TOLERANCE_D) ||
              (currentInput == B1_TARGET)   || (currentInput == B1_TARGET_D) ||
              (currentInput == B2_TARGET)   || (currentInput == B2_TARGET_D) ||
              (currentInput == POWER)
            )
         && (inputs[MODE] == CALIBRATE)
    ){
      currentInput += incDec;
    }

    //Skip past Calibration settings, if we're not calibrating (Using one field for two purposes)
    if ( ((currentInput == B1_CALIBRATE)   || (currentInput == B2_CALIBRATE) ||
          (currentInput == B1_CALIBRATE_D) || (currentInput == B2_CALIBRATE_D) )
         && (inputs[MODE] != CALIBRATE)
       ) {
      currentInput +=incDec;
    }

    //Skip past Power setting, if we're not in manual mode
    if ( (currentInput == POWER)   &&
         (inputs[MODE] != MANUAL1) && 
         (inputs[MODE] != MANUAL2)
       ) {
      currentInput +=incDec;
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


void checkInputs() {
  //Select button pressed
  if (currentInput != previousInput) {
    clearArrows();
    updateDisplay();
    previousInput = currentInput;
  }

  //rotary encoder moved
  int readPos = encoder.getPosition();
  if (inputs[currentInput] != readPos) {

    if((inputs[currentInput] == CALIBRATE)&&(readPos < CALIBRATE)){
      writeConfig();
    }

    lastPostpone = millis(); // pause monitoring, while input is happening
    if ((readPos <= maxLimit[currentInput]) && (readPos >= minLimit[currentInput])) {
      inputs[currentInput] = readPos;
    }
    else {
      encoder.setPosition(inputs[currentInput]);
    }

    if ((currentInput == POWER) && (inputs[MODE] == OFF)) { //Can't output when off
      encoder.setPosition(0);
      inputs[currentInput] = 0;
    }

    updateDisplay();
  }
}

void updateDisplay() {
  printInputValues();
  printArrow();
  printTemperatures();
  printHeatIndicator();
}

void senseTemperatures() {
  sensors.requestTemperatures();
  float b1Offset = assembleFloat(inputs[B1_CALIBRATE], inputs[B1_CALIBRATE_D], 9.0); // could move this to where calibrate values are updated...
  b1Temp = sensors.getTempC(b1Probe) + b1Offset;

  float b2Offset = assembleFloat(inputs[B2_CALIBRATE], inputs[B2_CALIBRATE_D], 9.0);
  b2Temp = sensors.getTempC(b2Probe) + b2Offset;
}

void printTemperatures() {
  printValue(0, 0, formatValidTemperature(b1Temp));
  printValue(10, 0, formatValidTemperature(b2Temp));
}

String formatValidTemperature(float temp) {
  String strRtn = "Error!  ";
  if (temp > -99.0) {
    strRtn = formatTemperature(temp);
  }

  return strRtn;
}

void monitorMash() {
  if (millis() > (lastMonitor + monitorDuration) &&
      millis() > (lastPostpone + postponeDuration) ) {
    senseTemperatures();

    switch (inputs[MODE]) {
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
        priorityMonitor(false);
        break;
      case PRIORITY2:
        priorityMonitor(true);
        break;
      case CALIBRATE:
        setOff();
        break;
    }

    updateDisplay();
    lastMonitor = millis();
  }
}

void setOff() {
  analogWrite(powerPin, 0);
  digitalWrite(boilerSelectPin, 0);
  inputs[POWER] = 0.0;
  switchedToBoiler2 = false;
}


/***********************************************
    Auto Monitoring
 ***********************************************/
int setAuto(boolean isBoiler2, float boilerTemp, float boilerTarget, float boilerTolerance) {

  digitalWrite(boilerSelectPin, isBoiler2);
  switchedToBoiler2 = isBoiler2;

  float diff = max(0.0, boilerTarget - boilerTemp);
  float factor = 0.0;

  if (diff > 0.0) {
    factor = 256.0 / boilerTolerance;
  }

  int power = min(255, factor * diff);
  inputs[POWER] = (float)power / 2.55;
  analogWrite(powerPin, power);

  return power;
}


/***********************************************
    Manual Setting
 ***********************************************/
void setManual(boolean divert) {
  int power = inputs[POWER] * 2.55;
  switchedToBoiler2 = divert;
  analogWrite(powerPin, power);
  digitalWrite(boilerSelectPin, divert);
}


/*(*********************************************
    Priority Monotoring
 ***********************************************/
void priorityMonitor(boolean primeDiverted){
  int pwr = 0;
  float b1Target = assembleFloat(inputs[B1_TARGET], inputs[B1_TARGET_D], 100.0);
  float b1Tolerance = assembleFloat(inputs[B1_TOLERANCE], inputs[B1_TOLERANCE_D], 100.0);
  float b2Target = assembleFloat(inputs[B2_TARGET], inputs[B2_TARGET_D], 100.0);
  float b2Tolerance = assembleFloat(inputs[B2_TOLERANCE], inputs[B2_TOLERANCE_D], 100.0);
  
  if(primeDiverted){
    pwr = setAuto(primeDiverted, b2Temp, b2Target, b2Tolerance);
    if (pwr == 0){
      setAuto(!primeDiverted, b1Temp, b1Target, b1Tolerance);
    }
  }else{
    pwr = setAuto(primeDiverted, b1Temp, b1Target, b1Tolerance);
    if (pwr == 0){
      setAuto(!primeDiverted, b2Temp, b2Target, b2Tolerance);
    }
  }
}


/***********************************************
    Display Arrow and Values
 ***********************************************/
void printArrow() {
  String a = arrowType[arrow[currentInput][2]];
  printValue((arrow[currentInput][X]), arrow[currentInput][Y], a);
}

void clearArrows() {

  //LHS
  printValue(arrow[B1_TARGET][X], arrow[B1_TARGET][Y], blankArrow);
  printValue(arrow[B1_TOLERANCE][X], arrow[B1_TOLERANCE][Y], blankArrow);
  printValue(arrow[POWER][X], arrow[POWER][Y], blankArrow);

  //RHS
  printValue(arrow[B2_TARGET][X], arrow[B2_TARGET][Y], blankArrow);
  printValue(arrow[B2_TOLERANCE][X], arrow[B2_TOLERANCE][Y], blankArrow);
  printValue(arrow[MODE][X], arrow[MODE][Y], blankArrow);
}


void printInputValues() {
  if (inputs[MODE] == CALIBRATE) {

   String blank = "       ";
   printValue(inputPosn[B1_CALIBRATE][X],
               inputPosn[B1_CALIBRATE][Y],
               formatInputTemperature(inputs[B1_CALIBRATE], inputs[B1_CALIBRATE_D]));

   printValue(inputPosn[B2_CALIBRATE][X],
               inputPosn[B2_CALIBRATE][Y],
               formatInputTemperature(inputs[B2_CALIBRATE], inputs[B2_CALIBRATE_D]));


    printValue(inputPosn[B1_TARGET][X],
               inputPosn[B1_TARGET][Y],
               blank);

    printValue(inputPosn[POWER][X],
               inputPosn[POWER][Y],
               blank);

    printValue(inputPosn[B2_TARGET][X],
               inputPosn[B2_TARGET][Y],
               blank);
  
  }else {
    printValue(inputPosn[B1_TOLERANCE][X],
               inputPosn[B1_TOLERANCE][Y],
               formatInputTemperature(inputs[B1_TOLERANCE], inputs[B1_TOLERANCE_D]));

    printValue(inputPosn[B2_TOLERANCE][X],
               inputPosn[B2_TOLERANCE][Y],
               formatInputTemperature(inputs[B2_TOLERANCE], inputs[B2_TOLERANCE_D]));

    printValue(inputPosn[B1_TARGET][X],
               inputPosn[B1_TARGET][Y],
               formatInputTemperature(inputs[B1_TARGET], inputs[B1_TARGET_D]));

    printValue(inputPosn[POWER][X],
               inputPosn[POWER][Y],
               pad((String)inputs[POWER], 3) + "%");

    printValue(inputPosn[B2_TARGET][X],
               inputPosn[B2_TARGET][Y],
               formatInputTemperature(inputs[B2_TARGET], inputs[B2_TARGET_D]));
  }

  printValue(inputPosn[MODE][X],
             inputPosn[MODE][Y],
             formatMode(inputs[MODE]));
}

void printHeatIndicator() {
  String b1 = " ";
  String b2 = " ";
  if ( (inputs[POWER] > 0) && (inputs[MODE] > OFF) ) {
    if (switchedToBoiler2) {
      b2 = "*";
    }
    else {
      b1 = "*";
    }
  }

  printValue(8, 0, b1);
  printValue(18, 0, b2);
}

String formatMode(float m) {
  return modes[(int)m];
}

float assembleFloat(int units, int tenths, float maximum) {
  float result = (float)units;
  result += (float)tenths / 10.0;
  return min(result, maximum);
}

String formatInputTemperature(int u, int t) {
  return formatTemperature(assembleFloat(u, t, 100.0));
}

String formatTemperature(float t) {
  return pad(oneDecPlace(t), 5) + degree + "C";
}

void printValue(int x, int y, String val) {
  lcd.setCursor(x, y);
  lcd.print(val);
}

String pad(String num, int width) {
  if (num.length() < width) {
    return pad(" " + num, width);
  }
  return num;
}

String oneDecPlace(float reading) {
  int a = round(reading * 10);
  float b = a / 10.0;
  String str = (String)b;
  //if(reading < 0){str = "-"+str;}
  return str.substring(0, str.length() - 1);
}



