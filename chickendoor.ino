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

// Measurement Values
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
int openOperationMode = OP_MODE_MANUAL;
int closeOperationMode = OP_MODE_MANUAL;

static long oldTime = 0;


double current = 0.0;

double zeroCurrent = 0.0;
int runs = 0; 


void checkMinLuxDuration();
void checkMaxLuxDuration();
void checkMotorRunning();
float readLux();
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

  setupWebUI();

  
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
  openOperationModeSelectorId = ESPUI.addControl( ControlType::Select, "Opening Mode", "", ControlColor::Alizarin, settingsTab, &operationModeSelector );
  ESPUI.addControl( ControlType::Option, "Manual", "Manual", ControlColor::Alizarin, openOperationModeSelectorId );
  ESPUI.addControl( ControlType::Option, "Lux", "Lux", ControlColor::Alizarin, openOperationModeSelectorId);
  ESPUI.addControl( ControlType::Option, "Time", "Time", ControlColor::Alizarin, openOperationModeSelectorId );
  closeOperationModeSelectorId = ESPUI.addControl( ControlType::Select, "Closing Mode", "", ControlColor::Alizarin, settingsTab, &operationModeSelector );
  ESPUI.addControl( ControlType::Option, "Manual", "Manual", ControlColor::Alizarin, closeOperationModeSelectorId );
  ESPUI.addControl( ControlType::Option, "Lux", "Lux", ControlColor::Alizarin, closeOperationModeSelectorId);
  ESPUI.addControl( ControlType::Option, "Time", "Time", ControlColor::Alizarin, closeOperationModeSelectorId );

  
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
        ESPUI.print(openOperationModeLabelId, "Manual");
        break;
      case OP_MODE_LUX:
        ESPUI.print(openOperationModeLabelId, "Lux");
        break;
      case OP_MODE_TIME:
        ESPUI.print(openOperationModeLabelId, "Time");
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
