#include <Wire.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <WiFi.h>
#include <ezTime.h>
#include <Ticker.h>
#include <Preferences.h>
#include <Adafruit_INA219.h>
#include "secrets.h"

#define DOOR_UNDEFINED 0
#define DOOR_OPEN 1
#define DOOR_CLOSED 2
#define OP_MODE_MANUAL 1
#define OP_MODE_LUX 2
#define OP_MODE_TIME 3


Adafruit_INA219 ina219;
const char ssid[] = WIFI_SSID;
const char password[] = WIFI_PASSWD;
Timezone myTZ;
BH1750 lightMeter;
BME280I2C bme;
unsigned int doorState = 0; // 1 is open, 2 is closed, use the defines above. 
int motorPinForward = 18; 
int motorPinBackward = 19; 
int motorRunningSeconds = 0;
int motorRunDuration = 60;
boolean motorRunning = false;
int doorCylclingState = 0;

int motorNumberPin;
int upButtonPin = 26;
int downButtonPin = 27;
// Instance of the button.
Preferences preferences; 

float lux;
float temperature;
float pressure;
float humidity;
/*
 * If the light stays below minlux for minluxdelay seconds, the door closes. 
 * If the light is above maxlux for maxluxdelay seconds, the door opens.
 */
float minLux = 300; 
float maxLux = 500;
int minLuxDelay = 15;
int maxLuxDelay = 15;
int minLuxSeconds = 0;
int maxLuxSeconds = 0;
int operationMode = OP_MODE_LUX;


double current = 0.0;

double zeroCurrent = 0.0;
int runs = 0; 


void checkMinLuxDuration();
void checkMaxLuxDuration();
void checkMotorRunning();
void readLux();
void checkLux();
void checkButtons();

Ticker minLuxTicker;
Ticker maxLuxTicker;
Ticker readCurrentTicker;
Ticker readLuxTicker;
Ticker checkLuxTicker;
Ticker buttonTicker;
Ticker startMotorTicker;
Ticker motorRunningSecondsCounter;



void setup() {
  Wire.begin(16,17);
  delay(500);
  // sets the two motor pins (open, close) as outputs:
  
  pinMode(motorPinForward, OUTPUT);
  pinMode(motorPinBackward, OUTPUT);
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  waitForSync();
  Serial.print("Wifi connected with IP ");
  Serial.println(WiFi.localIP());
  myTZ.setLocation(F("Europe/Berlin"));
  pinMode(upButtonPin, INPUT_PULLDOWN);
  pinMode(downButtonPin, INPUT_PULLDOWN);

  preferences.begin("chickendoor", false);
 
  unsigned int tempDoorState = preferences.getUInt("doorstate", 0);
  preferences.end(); 
  if (tempDoorState == 0) 
  {
    Serial.println("door state undefined. Cycling door states ...");
   // cycleDoor();
  }
  else doorState = tempDoorState;
  Serial.print("doorState: ");
  Serial.println(doorState);
  
  Serial.print("tempDoorState: ");
  Serial.println(tempDoorState);
  
  //readCurrentTicker.attach(1,readCurrent);
  //readLuxTicker.attach(1,readLux);
  //checkLuxTicker.attach(1,checkLux);
  //buttonTicker.attach(0.5,checkButtons);
 
  lightMeter.begin();
  bme.begin();
  
}

void loop() {
checkButtons();
checkMotorRunning();
if (operationMode == OP_MODE_LUX)
{
  readLux();
  checkLux();
}
delay(200);
}


void openDoor()
{
  Serial.println("call to openDoor");
  if (motorRunning == false)
  {
    moveDoor(motorPinBackward);  
    doorState = DOOR_UNDEFINED;
    preferences.begin("chickendoor", false);
    preferences.putUInt("doorstate", doorState);
    preferences.end(); 
    Serial.println("Starting to open door ...");
    Serial.println("Setting door state to UNDEFINED");
  }
  else Serial.println("Motor already moving");
}

void closeDoor()
{
  Serial.println("call to closeDoor");
  if (motorRunning == false)
  {
    moveDoor(motorPinForward);  
    doorState = DOOR_UNDEFINED;
    preferences.begin("chickendoor", false);
    preferences.putUInt("doorstate", doorState);
    preferences.end();
    Serial.println("Starting to close door ...");
    Serial.println("Setting door state to UNDEFINED");
  }
  else Serial.println("Motor already moving");}



void moveDoor(int thisMotorNumberPin)
{
  motorNumberPin = thisMotorNumberPin;
  digitalWrite(motorNumberPin, HIGH);
  motorRunning = true; 
}

void checkMotorRunning()
{
  if (motorRunning == false) return;
  if (readCurrent() < 10.0) 
  {
    Serial.println("Switching motor off");
    digitalWrite(motorNumberPin, LOW); 
    motorRunning = false;
    if (motorNumberPin == motorPinForward)
    {
    doorState = DOOR_CLOSED;
    preferences.begin("chickendoor", false);
    preferences.putUInt("doorstate", doorState);
    preferences.end(); 
    Serial.println("Written DOOR_CLOSED to settings");
    }
    else
    {
    doorState = DOOR_OPEN;
    preferences.begin("chickendoor", false);
    preferences.putUInt("doorstate", doorState);
    preferences.end(); 
    Serial.println("Written DOOR_OPEN to settings");
    }
  }
}

void readLux()
{
  lux = lightMeter.readLightLevel();  
  //Serial.print("Lux: "); Serial.println(lux);
}

void checkLux()
{
  if (lux < minLux && doorState == DOOR_OPEN) minLuxTicker.attach(1, checkMinLuxDuration);
  if (lux > maxLux && doorState == DOOR_CLOSED) maxLuxTicker.attach(1, checkMaxLuxDuration);
}



void checkMinLuxDuration()
{
  Serial.print("Inside checkMinLuxDuration for ");
  Serial.print(minLuxSeconds );
  Serial.print(" seconds.");
  if (lux > minLux)
  {
    Serial.println("Lux above threshold. Stopping door countdown.");
    minLuxSeconds = 0;
    minLuxTicker.detach();
    return;
  }
  minLuxSeconds++;
  if (minLuxSeconds > minLuxDelay)
  {
    minLuxTicker.detach();
    
    Serial.print("Lux has been below threshold for ");
    Serial.print(minLuxSeconds);
    Serial.println(" seconds. Closing door. ");
    closeDoor();
    minLuxSeconds = 0;
  }
}


void checkMaxLuxDuration()
{
  Serial.print("Inside checkMaxLuxDuration for ");
  Serial.print(maxLuxSeconds );
  Serial.print(" seconds.");
  if (lux < maxLux)
  {
    Serial.println("Lux below threshold. Stopping door countdown.");
    maxLuxSeconds = 0;
    maxLuxTicker.detach();
    return;
  }
  maxLuxSeconds++;
  if (maxLuxSeconds > maxLuxDelay)
  {
    maxLuxTicker.detach();
    
    Serial.print("Lux has been above threshold for ");
    Serial.print(maxLuxSeconds);
    Serial.println(" seconds. Opening door. ");
    openDoor();
    maxLuxSeconds = 0;
    
  }
}

void reportData()
{
  Serial.print(F("Germany:         "));
  Serial.println(myTZ.dateTime());
  
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
}

void reportClimateData()
{
  bme.read(pressure, temperature, humidity); 
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %rH");

  Serial.print("Pressure: ");
  Serial.print(pressure/100);
  Serial.println(" mBar");

}


void checkButtons()
{
  int upButtonState = digitalRead(upButtonPin);
  int downButtonState = digitalRead(downButtonPin);
  
//  Serial.print("upButtonPin: ");
//  Serial.println(upButtonState);
//  Serial.print("downButtonPin: ");
//  Serial.println(downButtonState);

//  if (downButtonState == 1 && doorState == DOOR_OPEN) closeDoor();
//  else if (upButtonState == 1 && doorState == DOOR_CLOSED) openDoor();

if (downButtonState == 1) closeDoor();
  else if (upButtonState == 1) openDoor();
}


boolean checkDoorCycled()
{
  Serial.println("Cycling door to get to defined state");
  if (doorState == DOOR_UNDEFINED)
  closeDoor();
  delay(1000);
  openDoor();  
}

float readCurrent()
{
  float current_mA = 0;
  int counter = 0;
  float tempCurrent = 0;
  for (int i = 0; i < 7; i++)
  {
    tempCurrent = abs(ina219.getCurrent_mA());
    //Serial.print("Temp Current: "); Serial.print(tempCurrent); Serial.println(" mA");
    if (tempCurrent > 0.0)
    {
      current_mA += tempCurrent;
      counter ++;
    }
    delay(10);
  }
  current_mA = current_mA/counter;
  
  //Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
  return current_mA; 
}
