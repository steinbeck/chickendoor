#include <Wire.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <WiFi.h>
#include <ezTime.h>
#include <Ticker.h>
#include <Preferences.h>
#include <Adafruit_INA219.h>

#include <AsyncEventSource.h>
#include <AsyncJson.h>
#include <SPIFFSEditor.h>
#include <WebHandlerImpl.h>
#include <ESPAsyncWebServer.h>
#include <WebAuthentication.h>
#include <AsyncWebSynchronization.h>
#include <AsyncWebSocket.h>
#include <WebResponseImpl.h>
#include <StringArray.h>

#include <AsyncTCP.h>

#include <DNSServer.h>
#include <ESPUI.h>

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif


#include "secrets.h"

#define DOOR_UNDEFINED 0
#define DOOR_OPEN 1
#define DOOR_CLOSED 2
#define OP_MODE_MANUAL 0
#define OP_MODE_LUX 1
#define OP_MODE_TIME 2


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

// Web UI controls
uint16_t luxLabelId;
uint16_t temperatureLabelId;
uint16_t pressureLabelId;
uint16_t humidityLabelId;
uint16_t currentLabelId;
uint16_t doorStateLabelId;
uint16_t openOperationModeSelectorId;
uint16_t closeOperationModeSelectorId;
uint16_t openOperationModeLabelId;
uint16_t closeOperationModeLabelId;
uint16_t dateTimeLabelId;
uint16_t openButtonId;
uint16_t closeButtonId;

uint16_t closingLuxTextId;
uint16_t openingLuxTextId;
uint16_t closingLuxDelayTextId;
uint16_t openingLuxDelayTextId;
uint16_t closingTimeTextId;
uint16_t openingTimeTextId;



char* modeStrings[]={"Manual", "Lux", "Time"};

// Measurement Values
float lux;
float temperature;
float pressure;
float humidity;
/*
 * If the light stays below closingLux for closingLuxdelay seconds, the door closes. 
 * If the light is above openingLux for openingLuxdelay seconds, the door opens.
 */
float closingLux = 300; // if light is smaller than closingLux for closingLuxDelay in minutes, then close the door 
float openingLux = 500; // if light is more than openingLux for openingLuxDelay in minutes, then open the door
int closingLuxDelay = 15;
int openingLuxDelay = 15;
int closingLuxSeconds = 0;
int openingLuxSeconds = 0;
String openingTime = "08:00";
String closingTime = "19:00";
unsigned int openOperationMode = OP_MODE_MANUAL;
unsigned int closeOperationMode = OP_MODE_MANUAL;

static long oldTime = 0;


double current = 0.0;

double zeroCurrent = 0.0;
int runs = 0; 


void checkclosingLuxDuration();
void checkopeningLuxDuration();
void checkMotorRunning();
float readLux();
void checkLux();
void checkButtons();

Ticker closingLuxTicker;
Ticker openingLuxTicker;
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
  WiFi.setHostname("ChickenDoor");
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
  loadPreferences();
  setupWebUI();
  pinMode(upButtonPin, INPUT_PULLDOWN);
  pinMode(downButtonPin, INPUT_PULLDOWN);
  lightMeter.begin();
  bme.begin();
  
}

void loop() {
  checkButtons();
  checkMotorRunning();
  updateWebUI();
//  if (operationMode == OP_MODE_LUX)
//  {
//    checkLux();
//  }
  delay(200);
}

void setupWebUI()
{
  uint16_t dashboardTab = ESPUI.addControl( ControlType::Tab, "Dashboard", "Dashboard" );
  uint16_t settingsTab = ESPUI.addControl( ControlType::Tab, "Settings", "Settings" );
  openOperationModeSelectorId = ESPUI.addControl( ControlType::Select, "Opening Mode", modeStrings[openOperationMode], ControlColor::Alizarin, settingsTab, &operationModeSelector );
  ESPUI.addControl( ControlType::Option, modeStrings[0], modeStrings[0], ControlColor::Alizarin, openOperationModeSelectorId );
  ESPUI.addControl( ControlType::Option, modeStrings[1], modeStrings[1], ControlColor::Alizarin, openOperationModeSelectorId);
  ESPUI.addControl( ControlType::Option, modeStrings[2], modeStrings[2], ControlColor::Alizarin, openOperationModeSelectorId );
  closeOperationModeSelectorId = ESPUI.addControl( ControlType::Select, "Closing Mode", modeStrings[closeOperationMode], ControlColor::Alizarin, settingsTab, &operationModeSelector );
  ESPUI.addControl( ControlType::Option, modeStrings[0], modeStrings[0], ControlColor::Alizarin, closeOperationModeSelectorId );
  ESPUI.addControl( ControlType::Option, modeStrings[1], modeStrings[1], ControlColor::Alizarin, closeOperationModeSelectorId);
  ESPUI.addControl( ControlType::Option, modeStrings[2], modeStrings[2], ControlColor::Alizarin, closeOperationModeSelectorId );


  openingLuxTextId = ESPUI.addControl( ControlType::Text, "Opening Lux", String(openingLux), ControlColor::Alizarin, settingsTab, &textHandler);
  closingLuxTextId = ESPUI.addControl( ControlType::Text, "Closing Lux", String(closingLux), ControlColor::Alizarin, settingsTab, &textHandler);
  openingLuxDelayTextId = ESPUI.addControl( ControlType::Text, "Opening Delay", String(openingLuxDelay), ControlColor::Alizarin, settingsTab, &textHandler);
  closingLuxDelayTextId = ESPUI.addControl( ControlType::Text, "Closing Delay", String(closingLuxDelay), ControlColor::Alizarin, settingsTab, &textHandler);
  openingTimeTextId = ESPUI.addControl( ControlType::Text, "Opening Time", String(openingTime), ControlColor::Alizarin, settingsTab, &textHandler);
  closingTimeTextId = ESPUI.addControl( ControlType::Text, "Closing Time", String(closingTime), ControlColor::Alizarin, settingsTab, &textHandler);

  openButtonId = ESPUI.addControl( ControlType::Button, "Door Control", "Open Door", ControlColor::Emerald,dashboardTab, &buttonHandler);
  closeButtonId = ESPUI.addControl( ControlType::Button, "Door Control", "Close Door", ControlColor::Emerald,dashboardTab, &buttonHandler);
  
  dateTimeLabelId = ESPUI.addControl( ControlType::Label, "Date/Time", "", ControlColor::Peterriver,dashboardTab);
  openOperationModeLabelId = ESPUI.addControl( ControlType::Label, "Opening Mode", "", ControlColor::Peterriver,dashboardTab);
  closeOperationModeLabelId = ESPUI.addControl( ControlType::Label, "Closing Mode", "", ControlColor::Peterriver,dashboardTab);
  doorStateLabelId = ESPUI.addControl( ControlType::Label, "Door State", "", ControlColor::Peterriver,dashboardTab);
  luxLabelId = ESPUI.addControl( ControlType::Label, "Lux", "lux", ControlColor::Peterriver,dashboardTab);
  temperatureLabelId = ESPUI.addControl( ControlType::Label, "Temperature [°C]", "", ControlColor::Peterriver,dashboardTab);
  pressureLabelId = ESPUI.addControl( ControlType::Label, "Pressure [hPa]", "", ControlColor::Peterriver,dashboardTab);
  humidityLabelId = ESPUI.addControl( ControlType::Label, "Humidity [%]", "", ControlColor::Peterriver,dashboardTab);
  currentLabelId = ESPUI.addControl( ControlType::Label, "Current [mA]", "", ControlColor::Peterriver,dashboardTab);
  
  
  ESPUI.begin("ChickenDoor Control");
  
}

void updateWebUI()
{
  if (millis() - oldTime > 5000) {
    readLux();
    readClimateData();
    readCurrent();
    ESPUI.print(dateTimeLabelId, String(myTZ.dateTime()));
    ESPUI.print(luxLabelId, String(readLux()));
    ESPUI.print(temperatureLabelId, String(temperature));
    ESPUI.print(pressureLabelId, String(pressure/100));
    ESPUI.print(humidityLabelId, String(humidity));
    ESPUI.print(currentLabelId, String(readCurrent()));
    switch(doorState)
    {
      case DOOR_OPEN:
        ESPUI.print(doorStateLabelId, "Open");
        break;
      case DOOR_CLOSED:
        ESPUI.print(doorStateLabelId, "Closed");
        break;
      case DOOR_UNDEFINED:
        ESPUI.print(doorStateLabelId, "Undefined");
        break;

    }

    switch(openOperationMode)
    {
      case OP_MODE_MANUAL:
        ESPUI.print(openOperationModeLabelId, modeStrings[0]);
        break;
      case OP_MODE_LUX:
        ESPUI.print(openOperationModeLabelId, modeStrings[1]);
        break;
      case OP_MODE_TIME:
        ESPUI.print(openOperationModeLabelId, modeStrings[2]);
        break;
    }

    switch(closeOperationMode)
    {
      case OP_MODE_MANUAL:
        ESPUI.print(closeOperationModeLabelId, "Manual");
        break;
      case OP_MODE_LUX:
        ESPUI.print(closeOperationModeLabelId, "Lux");
        break;
      case OP_MODE_TIME:
        ESPUI.print(closeOperationModeLabelId, "Time");
        break;
    }    
    oldTime = millis();
  }
}

void loadPreferences()
{
  //Serial.println("Reading Preferences: ");
  preferences.begin("chickendoor", false); 
  unsigned int tempDoorState = preferences.getUInt("doorstate", 0);
  openOperationMode = preferences.getUInt("opopmod", 0);
  //Serial.print("openOperationMode: "); Serial.println(openOperationMode);
  closeOperationMode = preferences.getUInt("clopmod", 0); 
  //Serial.print("closeOperationMode: "); Serial.println(closeOperationMode);
  closingLux = preferences.getFloat("cllux");
  openingLux = preferences.getFloat("oplux");
  closingLuxDelay = preferences.getFloat("clluxdel");
  openingLuxDelay = preferences.getFloat("opluxdel");

  preferences.end(); 

  // postprocessing of preferences
  if (tempDoorState == 0) 
  {
    Serial.println("door state undefined. Cycling door states ...");
   // cycleDoor();
  }
  else doorState = tempDoorState;

  
}

void operationModeSelector(Control *sender, int type)
{
Serial.println(sender->id);
Serial.println(sender->value);
  int tempOpMode;
  
  if (sender->value == "Manual") tempOpMode = OP_MODE_MANUAL;
  else if (sender->value == "Lux") tempOpMode = OP_MODE_LUX;
  else if (sender->value == "Time") tempOpMode = OP_MODE_TIME;  
//  Serial.print("tempOpMode: "); Serial.println(tempOpMode); 
Serial.print("openOperationModeSelectorId: "); Serial.println(openOperationModeSelectorId); 
Serial.print("closeOperationModeSelectorId: "); Serial.println(closeOperationModeSelectorId); 
//  Serial.print("ESPUI.getControl(openOperationModeLabelId)->id: "); Serial.println(ESPUI.getControl(openOperationModeLabelId)->id); 
//  Serial.print("ESPUI.getControl(closeOperationModeLabelId)->id: "); Serial.println(ESPUI.getControl(closeOperationModeLabelId)->id);
  
  if (sender->id == openOperationModeSelectorId) openOperationMode = tempOpMode;
  else if (sender->id == closeOperationModeSelectorId) closeOperationMode = tempOpMode;  

    preferences.begin("chickendoor", false);
    Serial.println(preferences.putUInt("opopmod", openOperationMode));
    Serial.print("Writing setting -> openOperationMode: "); Serial.println(openOperationMode);
    Serial.println(preferences.putUInt("clopmod", closeOperationMode));
    Serial.print("Writing setting -> closeOperationMode: "); Serial.println(closeOperationMode);
    preferences.end();   
}

void buttonHandler(Control *sender, int type)
{
  if (type == B_UP)
  {
    Serial.println(sender->id);
    Serial.println(sender->value);
    
    if (sender->id == openButtonId) 
    {
      openDoor();
      return;
    }
    if (sender->id == closeButtonId) 
    {
      closeDoor();
      return;
    }
  }
}

void textHandler(Control *sender, int type)
{
  Serial.println(sender->id);
  Serial.println(sender->value);
  int tempInt;
  float tempFloat;
  if (sender->id == openingLuxTextId)
  {
    tempInt = sender->value.toInt();
    if (tempInt > 0)
    {
      openingLux = tempInt;
      preferences.begin("chickendoor", false);
      preferences.putFloat("oplux", openingLux);
      preferences.end();   
    }
  }
 if (sender->id == closingLuxTextId)
  {
    tempInt = sender->value.toInt();
    if (tempInt > 0)
    {
      closingLux = tempInt;
      preferences.begin("chickendoor", false);
      preferences.putFloat("cllux", closingLux);
      preferences.end();   
    }
  }
  if (sender->id == closingLuxDelayTextId)
  {
    tempInt = sender->value.toInt();
    if (tempInt > 0)
    {
      closingLuxDelay = tempInt;
      preferences.begin("chickendoor", false);
      preferences.putFloat("clluxdel", closingLuxDelay);
      preferences.end();   
    }
  }
  if (sender->id == openingLuxDelayTextId)
  {
    tempInt = sender->value.toInt();
    if (tempInt > 0)
    {
      openingLuxDelay = tempInt;
      preferences.begin("chickendoor", false);
      preferences.putFloat("opluxdel", openingLuxDelay);
      preferences.end();   
    }
  }

//  openingTimeTextId = ESPUI.addControl( ControlType::Text, "Opening Time", String(openingTime), ControlColor::Alizarin, settingsTab, &textHandler);
//  closingTimeTextId = ESPUI.addControl( ControlType::Text, "Closing Time", String(closingTime), ControlColor::Alizarin, settingsTab, &textHandler);
//

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

float readLux()
{
  lux = lightMeter.readLightLevel();  
  //Serial.print("Lux: "); Serial.println(lux);
  return lux;
}

void checkLux()
{
  if (lux < closingLux && doorState == DOOR_OPEN) closingLuxTicker.attach(1, checkclosingLuxDuration);
  if (lux > openingLux && doorState == DOOR_CLOSED) openingLuxTicker.attach(1, checkopeningLuxDuration);
}



void checkClosingLuxDuration()
{
  Serial.print("Inside checkclosingLuxDuration for ");
  Serial.print(closingLuxSeconds );
  Serial.print(" seconds.");
  if (lux > closingLux)
  {
    Serial.println("Lux above threshold. Stopping door countdown.");
    closingLuxSeconds = 0;
    closingLuxTicker.detach();
    return;
  }
  closingLuxSeconds++;
  if (closingLuxSeconds > closingLuxDelay)
  {
    closingLuxTicker.detach();
    
    Serial.print("Lux has been below threshold for ");
    Serial.print(closingLuxSeconds);
    Serial.println(" seconds. Closing door. ");
    closeDoor();
    closingLuxSeconds = 0;
  }
}


void checkOpeningLuxDuration()
{
  Serial.print("Inside checkopeningLuxDuration for ");
  Serial.print(openingLuxSeconds );
  Serial.print(" seconds.");
  if (lux < openingLux)
  {
    Serial.println("Lux below threshold. Stopping door countdown.");
    openingLuxSeconds = 0;
    openingLuxTicker.detach();
    return;
  }
  openingLuxSeconds++;
  if (openingLuxSeconds > openingLuxDelay)
  {
    openingLuxTicker.detach();
    
    Serial.print("Lux has been above threshold for ");
    Serial.print(openingLuxSeconds);
    Serial.println(" seconds. Opening door. ");
    openDoor();
    openingLuxSeconds = 0;
    
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

void readClimateData()
{
  bme.read(pressure, temperature, humidity); 
//  Serial.print("Temperature: ");
//  Serial.print(temperature);
//  Serial.println("°C");
//
//  Serial.print("Humidity: ");
//  Serial.print(humidity);
//  Serial.println(" %rH");
//
//  Serial.print("Pressure: ");
//  Serial.print(pressure/100);
//  Serial.println(" mBar");

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
