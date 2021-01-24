#include <Wire.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <WiFi.h>
#include <ezTime.h>
#include <Ticker.h>
#include "secrets.h"

const char ssid[] = WIFI_SSID;
const char password[] = WIFI_PASSWD;
Timezone myTZ;
BH1750 lightMeter;
BME280I2C bme;
int motorPinForward = 18; 
int motorPinBackward = 19; 
int motorRunningSeconds = 0;
int motorRunDuration = 60;
boolean motorRunning = false;
Ticker checkMotorRunningTicker;
int motorNumberPin;
int buttonUpPin = 26;
int buttonDownPin = 27;


int analogPin = 33;
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

double average = 0;
double scaleFactor = 185.0; // for 20A module = 100.0
 // for 30A module = 66.0
double voltage = 0.0;
double current = 0.0;

double zeroCurrent = 0.0;
int runs = 0; 

boolean doorOpen = false;
boolean doorClosed= true; 
void checkMinLuxDuration();
void checkMaxLuxDuration();
void checkMotorRunning(int motorNumber);
Ticker minLuxTicker;
Ticker maxLuxTicker;


struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button button_auf = {buttonUpPin, 0, false};
Button button_zu = {buttonDownPin, 0, false};

void IRAM_ATTR isr1() {
  if (button_auf.pressed == false)
  {
    button_auf.numberKeyPresses += 1;
    button_auf.pressed = true;
  }
  
}

void IRAM_ATTR isr2() {
  if (button_zu.pressed == false) 
  {
    button_zu.numberKeyPresses += 1;
    button_zu.pressed = true;
  }
  
}


void setup() {
  Wire.begin(16,17);
  delay(500);
  // sets the two motor pins (open, close) as outputs:
  pinMode(motorPinForward, OUTPUT);
  pinMode(motorPinBackward, OUTPUT);
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  waitForSync();
  Serial.print("Wifi connected with IP ");
  Serial.println(WiFi.localIP());
  myTZ.setLocation(F("Europe/Berlin"));
  pinMode(button_auf.PIN, INPUT_PULLDOWN);
  attachInterrupt(button_auf.PIN, isr1, FALLING);
  pinMode(button_zu.PIN, INPUT_PULLDOWN);
  attachInterrupt(button_zu.PIN, isr2, FALLING);
  lightMeter.begin();
  bme.begin();
  
}

void loop() {

  readLux();
  checkLux();
  
  //reportData();
  //reportClimateData();

  for(int i = 0; i < 100; i++) {
   average += analogRead(analogPin);
   delay(1);
   }
   average /= 100;
//   Serial.print("Average ADC current read: ");
//   Serial.println(average);
   voltage = ((average-775) / 4095.0) * 3300.0; // in mV
   // for calibration to determine number 2494 at 0A current
   // Serial.println(voltage);
   current = voltage / scaleFactor; // in A
    //prints current in A
//   Serial.print("Current: ");
//   Serial.println(current);

    Serial.println(analogRead(analogPin));
  
//    runs++;
//   zeroCurrent = zeroCurrent + analogRead(analogPin); 
   
   //Serial.print("Average Current: ");
   //Serial.println(zeroCurrent/runs);
  
  handleButtonPressed();
  delay(1000);
}


void openDoor()
{

  if (motorRunning == false)
  {
    moveDoor(motorPinBackward);  
    Serial.println("Starting to open door ...");
  }
  else Serial.println("Motor already moving");
}

void closeDoor()
{
  if (motorRunning == false)
  {
    moveDoor(motorPinForward);  
    Serial.println("Starting to open door ...");
  }
  else Serial.println("Motor already moving");}

void moveDoor(int thisMotorNumberPin)
{
  motorNumberPin = thisMotorNumberPin;
  motorRunningSeconds = 0;
  digitalWrite(motorNumberPin, HIGH);
  motorRunning = true;
  checkMotorRunningTicker.attach(1, checkMotorRunning);
  
}

void checkMotorRunning()
{
  motorRunningSeconds ++; 
  //Serial.println(analogRead(analogPin));
  if (motorRunningSeconds >= motorRunDuration) 
  {
    Serial.println("Switching motor off");
    digitalWrite(motorNumberPin, LOW); 
    motorRunning = false;
    checkMotorRunningTicker.detach();
    
  }
}

void readLux()
{
  lux = lightMeter.readLightLevel();  
}

void checkLux()
{
  if (lux < minLux && doorOpen) minLuxTicker.attach(1, checkMinLuxDuration);
  if (lux > maxLux && doorClosed) maxLuxTicker.attach(1, checkMaxLuxDuration);
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
    doorClosed = true; 
    doorOpen = false;
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
    doorClosed = false; 
    doorOpen = true;
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


void handleButtonPressed()
{
    if (button_auf.pressed) {
      Serial.printf("Button ''Auf'' has been pressed %u times\n", button_auf.numberKeyPresses);
      openDoor();
      button_auf.pressed = false;
  }
    if (button_zu.pressed) {
      Serial.printf("Button ''Zu'' has been pressed %u times\n", button_zu.numberKeyPresses);
      closeDoor();
      button_zu.pressed = false;
  }
}
