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
#include <AsyncElegantOTA.h>
#include <WebAuthentication.h>
#include <AsyncWebSynchronization.h>
#include <AsyncWebSocket.h>
#include <WebResponseImpl.h>
#include <StringArray.h>

#include <AsyncTCP.h>

#include <DNSServer.h>
#include <ESPUI.h>
#include <Update.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
//10 seconds WDT ; Less screws up OTA
#define WDT_TIMEOUT 10


const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

//OTA with ESPUI solution thanks to user ENWI (https://githubmemory.com/repo/s00500/ESPUI/issues/116)
const char* OTA_INDEX PROGMEM
  = R"=====(<!DOCTYPE html><html><head><meta charset=utf-8><title>OTA</title></head><body><div class="upload"><form method="POST" action="/ota" enctype="multipart/form-data"><input type="file" name="data" /><input type="submit" name="upload" value="Upload" title="Upload Files"></form></div></body></html>)=====";

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
const char titleVersion[] = "Chicken Door Control V1.2.1";

WiFiClient espClient;
PubSubClient client(espClient);

// Replace with your unique IFTTT URL resource
// Follow https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
const char* resource = GOOGLE_API_KEY;

// Maker Webhooks IFTTT
const char* server = "maker.ifttt.com";
const char* mqtt_server = "192.168.16.42";

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
int networkUnreachableCount = 0;

int motorNumberPin;
int upButtonPin = 26;
int downButtonPin = 27;
// Instance of the button.
Preferences preferences;

// Web UI controls
uint16_t luxLabelId;
uint16_t luxCountdownLabelId;
uint16_t temperatureLabelId;
uint16_t pressureLabelId;
uint16_t humidityLabelId;
uint16_t currentLabelId;
uint16_t doorStateLabelId;
uint16_t logfileLabelId;

uint16_t openOperationModeSelectorId;
uint16_t closeOperationModeSelectorId;
uint16_t openOperationModeLabelId;
uint16_t closeOperationModeLabelId;
uint16_t dateTimeLabelId;
uint16_t openButtonId;
uint16_t closeButtonId;
uint16_t restartButtonId;

uint16_t closingLuxTextId;
uint16_t openingLuxTextId;
uint16_t closingLuxDelayTextId;
uint16_t openingLuxDelayTextId;
uint16_t closingTimeTextId;
uint16_t openingTimeTextId;


boolean luxTickerRunning = false;


char* modeStrings[] = {"Manual", "Lux", "Time"};

// Measurement Values
float lux;
float temperature;
float pressure;
float humidity;
/*
   If the light stays below closingLux for closingLuxdelay seconds, the door closes.
   If the light is above openingLux for openingLuxdelay seconds, the door opens.
*/
float closingLux = 300; // if light is smaller than closingLux for closingLuxDelay in minutes, then close the door
float openingLux = 500; // if light is more than openingLux for openingLuxDelay in minutes, then open the door

int closingLuxDelay;
int openingLuxDelay;

long closingLuxSeconds;
long openingLuxSeconds;
long tempClosingLuxSeconds;
long tempOpeningLuxSeconds;



char openingTime[10] = "08:00";
char closingTime[10] = "19:00";
int openingHour = 8;
int openingMinute = 0;
int closingHour = 19;
int closingMinute = 0;

unsigned int openOperationMode = OP_MODE_MANUAL;
unsigned int closeOperationMode = OP_MODE_MANUAL;

static long oldTime = 0;


double current = 0.0;

double zeroCurrent = 0.0;
int runs = 0;
int last = millis();


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
Ticker networkAliveTicker;
Ticker restartTicker;


void setup() {
  String message;
  Wire.begin(16, 17);
  delay(500);
  // sets the two motor pins (open, close) as outputs:

  pinMode(motorPinForward, OUTPUT);
  pinMode(motorPinBackward, OUTPUT);
  Serial.begin(115200);
  WiFi.setHostname("ChickenDoor");
  WiFi.begin(ssid, password);

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  waitForSync();
  Serial.print("Wifi connected with IP ");
  Serial.println(WiFi.localIP());
  writelog("Chicken Door Control (re)started");
  writelog(String(WiFi.localIP().toString().c_str()));

  myTZ.setLocation(F("Europe/Berlin"));
  loadPreferences();
  message = "Network unreachable count: ";
  message.concat(networkUnreachableCount);
  writelog(message);

  setupWebUI();
  pinMode(upButtonPin, INPUT_PULLDOWN);
  pinMode(downButtonPin, INPUT_PULLDOWN);
  lightMeter.begin();
  bme.begin();
  networkUnreachableCount = 0;

  networkAliveTicker.attach(20, checkNetworkAlive);
  for (int i = 0; i < 10; i++) readLux();
  //restartTicker.attach(3600, restartESP);
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

}

//************************
//This is the central loop
//************************

void loop() {
  checkButtons();
  checkMotorRunning();
  updateWebUI();
  if (openOperationMode == OP_MODE_LUX || closeOperationMode == OP_MODE_LUX)
  {
    checkLux();
  }
  if (openOperationMode == OP_MODE_TIME || closeOperationMode == OP_MODE_TIME)
  {
    checkTime();
  }

  if (millis() - last >= 8000) {
    esp_task_wdt_reset();
    last = millis();
  }

  delay(200);
}

void checkButtons()
{
  int upButtonState = digitalRead(upButtonPin);
  int downButtonState = digitalRead(downButtonPin);

  //  Serial.print("upButtonPin: "); //  Serial.println(upButtonState); //  Serial.print("downButtonPin: "); //  Serial.println(downButtonState); //  if (downButtonState == 1 && doorState == DOOR_OPEN) closeDoor(); //  else if (upButtonState == 1 && doorState == DOOR_CLOSED) openDoor();

  if (downButtonState == 1)
  {
    writelog("The down button was pressed manually");
    closeDoor();
  }
  else if (upButtonState == 1)
  {
    writelog("The up button was pressed manually");
    openDoor();
  }
}

void checkLux()
{
  if (luxTickerRunning)
  {
    //Serial.println("Lux Ticker is already running");
    return;
  }
  if (closeOperationMode == OP_MODE_LUX && lux < closingLux && doorState == DOOR_OPEN)
  {
    String message = "Door closing in " + String(closingLuxDelay - (closingLuxSeconds / 60)) + " minutes";
    writelog(message);
    ESPUI.print(luxCountdownLabelId, message);
    closingLuxSeconds = 0;
    tempClosingLuxSeconds = 0;
    luxTickerRunning = true;
    closingLuxTicker.attach(1, checkClosingLuxDuration);
    return;
  }
  if (openOperationMode == OP_MODE_LUX && lux > openingLux && doorState == DOOR_CLOSED)
  {
    String message = "Door opening in " + String(openingLuxDelay - (openingLuxSeconds / 60)) + " minutes";
    ESPUI.print(luxCountdownLabelId, message);
    writelog(message);
    openingLuxSeconds = 0;
    tempOpeningLuxSeconds = 0;
    luxTickerRunning = true;
    openingLuxTicker.attach(1, checkOpeningLuxDuration);
    return;
  }
}

void checkMotorRunning()
{
  if (motorRunning == false) return;
  if (readCurrent() < 10.0)
  {
    writelog("Switching motor off");
    digitalWrite(motorNumberPin, LOW);
    motorRunning = false;
    if (motorNumberPin == motorPinForward)
    {
      doorState = DOOR_CLOSED;
      preferences.begin("chickendoor", false);
      preferences.putUInt("doorstate", doorState);
      preferences.end();
      writelog("Written DOOR_CLOSED to settings");
    }
    else
    {
      doorState = DOOR_OPEN;
      preferences.begin("chickendoor", false);
      preferences.putUInt("doorstate", doorState);
      preferences.end();
      writelog("Written DOOR_OPEN to settings");
    }
  }
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
  openingLuxDelayTextId = ESPUI.addControl( ControlType::Text, "Opening Delay [min]", String(openingLuxDelay), ControlColor::Alizarin, settingsTab, &textHandler);
  closingLuxDelayTextId = ESPUI.addControl( ControlType::Text, "Closing Delay [min]", String(closingLuxDelay), ControlColor::Alizarin, settingsTab, &textHandler);


  snprintf(openingTime, sizeof(openingTime), "%02i:%02i", openingHour, openingMinute);
  snprintf(closingTime, sizeof(closingTime), "%02i:%02i", closingHour, closingMinute);

  openingTimeTextId = ESPUI.addControl( ControlType::Text, "Opening Time [hh:mm]", String(openingTime), ControlColor::Alizarin, settingsTab, &textHandler);
  closingTimeTextId = ESPUI.addControl( ControlType::Text, "Closing Time [hh:mm]", String(closingTime), ControlColor::Alizarin, settingsTab, &textHandler);

  openButtonId = ESPUI.addControl( ControlType::Button, "Door Control", "Open Door", ControlColor::Emerald, dashboardTab, &buttonHandler);
  closeButtonId = ESPUI.addControl( ControlType::Button, "Door Control", "Close Door", ControlColor::Emerald, dashboardTab, &buttonHandler);

  dateTimeLabelId = ESPUI.addControl( ControlType::Label, "Date/Time", "", ControlColor::Peterriver, dashboardTab);

  luxLabelId = ESPUI.addControl( ControlType::Label, "Lux", "lux", ControlColor::Peterriver, dashboardTab);
  luxCountdownLabelId = ESPUI.addControl( ControlType::Label, "Lux Countdown", "Not ticking", ControlColor::Peterriver, dashboardTab);
  doorStateLabelId = ESPUI.addControl( ControlType::Label, "Door State", "", ControlColor::Peterriver, dashboardTab);
  temperatureLabelId = ESPUI.addControl( ControlType::Label, "Temperature [°C]", "", ControlColor::Peterriver, dashboardTab);
  pressureLabelId = ESPUI.addControl( ControlType::Label, "Pressure [hPa]", "", ControlColor::Peterriver, dashboardTab);
  humidityLabelId = ESPUI.addControl( ControlType::Label, "Humidity [%]", "", ControlColor::Peterriver, dashboardTab);
  currentLabelId = ESPUI.addControl( ControlType::Label, "Current [mA]", "", ControlColor::Peterriver, dashboardTab);
  openOperationModeLabelId = ESPUI.addControl( ControlType::Label, "Opening Mode", "", ControlColor::Peterriver, dashboardTab);
  closeOperationModeLabelId = ESPUI.addControl( ControlType::Label, "Closing Mode", "", ControlColor::Peterriver, dashboardTab);

  restartButtonId = ESPUI.addControl( ControlType::Button, "Chicken Door Control Device", "Restart Device", ControlColor::Alizarin, dashboardTab, &buttonHandler);
  

  //logfileLabelId = ESPUI.addControl( ControlType::Label, "logfile", "", ControlColor::Peterriver,dashboardTab);


  ESPUI.begin(titleVersion);
  ESPUI.server->on("/ota",
                   HTTP_POST,
  [](AsyncWebServerRequest * request) {
    request->send(200);
  },
  handleOTAUpload);

  ESPUI.server->on("/ota",
                   HTTP_GET,
  [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse* response = request->beginResponse_P(200, "text/html", OTA_INDEX);
    request->send(response);
  }
                  );

}

void updateWebUI()
{
  String message;
  if (luxTickerRunning == false) ESPUI.print(luxCountdownLabelId, "Not ticking");
  if (millis() - oldTime > 5000) {
    readLux();
    readClimateData();
    readCurrent();
    ESPUI.print(dateTimeLabelId, String(myTZ.dateTime()));
    ESPUI.print(luxLabelId, String(readLux()));
    ESPUI.print(temperatureLabelId, String(temperature));
    ESPUI.print(pressureLabelId, String(pressure / 100));
    ESPUI.print(humidityLabelId, String(humidity));
    ESPUI.print(currentLabelId, String(readCurrent()));
    switch (doorState)
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

    switch (openOperationMode)
    {
      case OP_MODE_MANUAL:
        ESPUI.print(openOperationModeLabelId, modeStrings[0]);
        break;
      case OP_MODE_LUX:
        message = modeStrings[1];
        message += " > "; message += openingLux; message += ", delay "; message += openingLuxDelay; message += " min";
        ESPUI.print(openOperationModeLabelId, message);
        break;
      case OP_MODE_TIME:
        message = modeStrings[2];
        message += " "; message += openingTime;
        ESPUI.print(openOperationModeLabelId, message);
        break;
    }

    switch (closeOperationMode)
    {
      case OP_MODE_MANUAL:
        ESPUI.print(closeOperationModeLabelId, modeStrings[0]);
        break;
      case OP_MODE_LUX:
        message = modeStrings[1];
        message += " < "; message += closingLux; message += ", delay "; message += closingLuxDelay; message += " min";
        ESPUI.print(closeOperationModeLabelId, message);
        break;
      case OP_MODE_TIME:
        message = modeStrings[2];
        message += " ";  message += closingTime;
        ESPUI.print(closeOperationModeLabelId, message);
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
  openingHour = preferences.getUInt("optimehh");
  openingMinute = preferences.getUInt("optimemm");
  closingHour = preferences.getUInt("cltimehh");
  closingMinute = preferences.getUInt("cltimemm");
  networkUnreachableCount = preferences.getUInt("netuc");

  preferences.end();

  // postprocessing of preferences
  if (tempDoorState == 0)
  {
    writelog("door state undefined. Cycling door states ...");
    checkDoorCycled();
  }
  else doorState = tempDoorState;


}

void operationModeSelector(Control *sender, int type)
{
  int tempOpMode;

  if (sender->value == "Manual") tempOpMode = OP_MODE_MANUAL;
  else if (sender->value == "Lux") tempOpMode = OP_MODE_LUX;
  else if (sender->value == "Time") tempOpMode = OP_MODE_TIME;

  if (sender->id == openOperationModeSelectorId) openOperationMode = tempOpMode;
  else if (sender->id == closeOperationModeSelectorId) closeOperationMode = tempOpMode;

  preferences.begin("chickendoor", false);
  Serial.println(preferences.putUInt("opopmod", openOperationMode));
  Serial.print("Writing setting -> openOperationMode: "); Serial.println(openOperationMode);
  Serial.println(preferences.putUInt("clopmod", closeOperationMode));
  Serial.print("Writing setting -> closeOperationMode: "); Serial.println(closeOperationMode);
  preferences.end();

  if (tempOpMode == OP_MODE_MANUAL || tempOpMode == OP_MODE_TIME)
  {
    if (luxTickerRunning)
    {
      luxTickerRunning = false;
      closingLuxTicker.detach();
      openingLuxTicker.detach();
    }
  }
}

void buttonHandler(Control *sender, int type)
{
  if (type == B_UP)
  {
    Serial.println(sender->id);
    Serial.println(sender->value);

    if (sender->id == openButtonId)
    {
      writelog("The open button was pressed in the web interface");
      openDoor();
      return;
    }
    if (sender->id == closeButtonId)
    {
      writelog("The close button was pressed in the web interface");
      closeDoor();
      return;
    }
    if (sender->id == restartButtonId)
    {
      writelog("The restart button was pressed in the web interface");
      ESP.restart();
      return;
    }

  }
}

void textHandler(Control *sender, int type)
{

  String message;
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

  if (sender->id == openingTimeTextId)
  {
    int tempHour = getHour(sender->value);
    int tempMinute = getMinute(sender->value);

    message = "Opening Time changed to ";
    message.concat(tempHour);
    message.concat(":");
    message.concat(tempMinute);
    writelog(message);


    if (tempHour > -1 && tempMinute > -1)
    {
      openingHour = tempHour;
      openingMinute = tempMinute;

      preferences.begin("chickendoor", false);
      preferences.putUInt("optimehh", openingHour);
      preferences.putUInt("optimemm", openingMinute);
      preferences.end();
    }
  }


  if (sender->id == closingTimeTextId)
  {
    int tempHour = getHour(sender->value);
    int tempMinute = getMinute(sender->value);
    if (tempHour > -1 && tempMinute > -1)
    {
      closingHour = tempHour;
      closingMinute = tempMinute;

      preferences.begin("chickendoor", false);
      preferences.putUInt("cltimehh", closingHour);
      preferences.putUInt("cltimemm", closingMinute);
      preferences.end();
    }
  }

}


int getHour(String timeString)
{
  int hourInt = -1;
  int colonIndex = timeString.indexOf(':');
  if (colonIndex > 0)
  {
    hourInt = timeString.substring(0, colonIndex).toInt();
  }
  return hourInt;
}

int getMinute(String timeString)
{
  int minuteInt = -1;
  int colonIndex = timeString.indexOf(':');
  if (colonIndex > 0)
  {
    minuteInt = timeString.substring(colonIndex + 1, timeString.length()).toInt();
  }
  return minuteInt;
}


void openDoor()
{
  writelog("call to openDoor");
  if (motorRunning == false)
  {
    moveDoor(motorPinBackward);
    doorState = DOOR_UNDEFINED;
    preferences.begin("chickendoor", false);
    preferences.putUInt("doorstate", doorState);
    preferences.end();
    writelog("Starting to open door ...");
    writelog("Setting door state to UNDEFINED");
  }
  else Serial.println("Motor already moving");

  luxTickerRunning = false;
  openingLuxTicker.detach();
}

void closeDoor()
{
  writelog("call to closeDoor");
  if (motorRunning == false)
  {
    moveDoor(motorPinForward);
    doorState = DOOR_UNDEFINED;
    preferences.begin("chickendoor", false);
    preferences.putUInt("doorstate", doorState);
    preferences.end();
    writelog("Starting to close door ...");
    writelog("Setting door state to UNDEFINED");
  }
  else Serial.println("Motor already moving");

  luxTickerRunning = false;
  closingLuxTicker.detach();


}



void moveDoor(int thisMotorNumberPin)
{
  motorNumberPin = thisMotorNumberPin;
  digitalWrite(motorNumberPin, HIGH);
  motorRunning = true;
}



float readLux()
{
  lux = lightMeter.readLightLevel();
  //Serial.print("Lux: "); Serial.println(lux);
  return lux;
}



void checkTime()
{

  if (doorState == DOOR_OPEN && closeOperationMode == OP_MODE_TIME )
  {
    if (myTZ.hour() == closingHour && myTZ.minute() == closingMinute)
      closeDoor();
    return;
  }
  if (doorState == DOOR_CLOSED && openOperationMode == OP_MODE_TIME)
  {
    if (myTZ.hour() == openingHour && myTZ.minute() == openingMinute)
      openDoor();
    return;
  }
}

void checkNetworkAlive()
{
  if ((WiFi.status() != WL_CONNECTED)) {
    networkUnreachableCount++;
    preferences.putUInt("netuc", networkUnreachableCount);
    restartESP();
    WiFi.disconnect();
    WiFi.reconnect();
    writelog("Lost network. Now successfully reconnected to WIFI network");
  }
  else
  {
    Serial.println("Network still alive");
  }
}

void restartESP()
{
  if (motorRunning == false && luxTickerRunning == false)
  {
    writelog("Restarting ESP");
    ESP.restart();
  }
}



void checkClosingLuxDuration()
{
  //  Serial.print("Inside checkclosingLuxDuration for ");
  //  Serial.print(closingLuxSeconds );
  //  Serial.print(" seconds.");
  String message;
  if (motorRunning == false && lux > closingLux) //Running motor can lead to wrong readings of sensor
  {
    writelog("Lux above threshold. Stopping door countdown.");
    closingLuxSeconds = 0;
    luxTickerRunning = false;
    closingLuxTicker.detach();
    ESPUI.print(luxCountdownLabelId, "Not ticking");

    return;
  }
  closingLuxSeconds++;
  if (closingLuxSeconds > tempClosingLuxSeconds + 60)
  {
    tempClosingLuxSeconds = closingLuxSeconds;
    String message = String("Door closing in ") + (closingLuxDelay - (closingLuxSeconds / 60)) + String(" minutes");
    ESPUI.print(luxCountdownLabelId, message);
  }
  if (closingLuxSeconds > closingLuxDelay * 60) // the delay is giving in minutes, the counter counts seconds
  {

    message = "Lux has been below threshold for ";
    message.concat(closingLuxSeconds);
    message.concat(" seconds. Closing door. ");
    writelog(message);

    closeDoor();
    ESPUI.print(luxCountdownLabelId, "Not ticking");
    closingLuxSeconds = 0;
    tempClosingLuxSeconds = 0;
  }

}


void checkOpeningLuxDuration()
{
  //  Serial.print("Inside checkopeningLuxDuration for ");
  //  Serial.print(openingLuxSeconds );
  //  Serial.print(" seconds.");
  String message;
  if (motorRunning == false && lux < openingLux) // running motor can lead to wrong readings of sensors
  {
    writelog("Lux below threshold. Stopping door countdown.");
    openingLuxSeconds = 0;
    luxTickerRunning = false;
    openingLuxTicker.detach();
    ESPUI.print(luxCountdownLabelId, "Not ticking");
    return;
  }
  openingLuxSeconds++;
  if (openingLuxSeconds > tempOpeningLuxSeconds + 60)
  {
    tempOpeningLuxSeconds = openingLuxSeconds;
    String message = "Door opening in " + String(openingLuxDelay - (openingLuxSeconds / 60)) + " minutes";
    ESPUI.print(luxCountdownLabelId, message);
  }
  if (openingLuxSeconds > openingLuxDelay * 60)
  {
    luxTickerRunning = false;
    openingLuxTicker.detach();
    Serial.print("Lux has been above threshold for ");
    Serial.print(openingLuxSeconds);
    Serial.println(" seconds. Opening door. ");

    message = "Lux has been above threshold for ";
    message.concat(openingLuxSeconds);
    message.concat(" seconds. Opening door. ");
    writelog(message);

    openDoor();
    ESPUI.print(luxCountdownLabelId, "Not ticking");
    openingLuxSeconds = 0;
    tempOpeningLuxSeconds = 0;


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



boolean checkDoorCycled()
{
  writelog("Cycling door to get to defined state");
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
  current_mA = current_mA / counter;

  //Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
  return current_mA;
}

// The following methods is from https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
// to log messages to a google sheet using webhooks from IFTTT

void writelog(String message) {
  Serial.print("Connecting to ");
  Serial.print(server);
  Serial.print("About to log message: ");
  Serial.print(message);

  WiFiClient client;
  int retries = 5;
  while (!!!client.connect(server, 80) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if (!!!client.connected()) {
    Serial.println("Failed to connect...");
  }

  Serial.print("Request resource: ");
  Serial.println(resource);

  // Temperature in Celsius
  String jsonObject = String("{\"value1\":\"") + message + "\"}";

  // Comment the previous line and uncomment the next line to publish temperature readings in Fahrenheit
  /*String jsonObject = String("{\"value1\":\"") + (1.8 * bme.readTemperature() + 32) + "\",\"value2\":\""
                      + (bme.readPressure()/100.0F) + "\",\"value3\":\"" + bme.readHumidity() + "\"}";*/

  client.println(String("POST ") + resource + " HTTP/1.1");
  client.println(String("Host: ") + server);
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);

  int timeout = 5 * 10; // 5 seconds
  while (!!!client.available() && (timeout-- > 0)) {
    delay(100);
  }
  if (!!!client.available()) {
    Serial.println("No response...");
  }
  while (client.available()) {
    Serial.write(client.read());
  }

  Serial.println("\nclosing connection");
  client.stop();
}


void handleOTAUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final)
{
  if (!index)
  {
    Serial.printf("UploadStart: %s\n", filename.c_str());
    // calculate sketch space required for the update, for ESP32 use the max constant
#if defined(ESP32)
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
#else
    const uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace))
#endif
    {
      // start with max available size
      Update.printError(Serial);
    }
#if defined(ESP8266)
    Update.runAsync(true);
#endif
  }

  if (len)
  {
    Update.write(data, len);
  }

  // if the final flag is set then this is the last frame of data
  if (final)
  {
    if (Update.end(true))
    {
      // true to set the size to the current progress
      writelog("Successful OTA update. Restarting ...");
      Serial.printf("Update Success: %ub written\nRebooting...\n", index + len);
      ESP.restart();
    }
    else
    {
      Update.printError(Serial);
    }
  }
}
