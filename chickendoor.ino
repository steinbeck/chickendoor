#include <Wire.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <ESP8266WiFi.h>
#include <ezTime.h>
#include <Ticker.h>
#include "secrets.h"

const char ssid[] = WIFI_SSID;
const char password[] = WIFI_PASSWD;
Timezone myTZ;
BH1750 lightMeter;
BME280I2C bme;
int motorPinForward = D3; 
int motorPinBackward = D6; 
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

boolean doorOpen = true;
boolean doorClosed= false; 
void checkMinLuxDuration();
void checkMaxLuxDuration();
Ticker minLuxTicker;
Ticker maxLuxTicker;

void setup() {
  // sets the pins as outputs:
  pinMode(motorPinForward, OUTPUT);
  pinMode(motorPinBackward, OUTPUT);
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  waitForSync();
  myTZ.setLocation(F("Europe/Berlin"));
  Wire.begin();
  lightMeter.begin();
  bme.begin();
  reportClimateData();
}

void loop() {

  readLux();
  checkLux();
  reportData();
  delay(1000);
}


void openDoor()
{
  moveDoor(motorPinBackward);  
}

void closeDoor()
{
  moveDoor(motorPinForward);
}

void moveDoor(int motornumber)
{
  Serial.print("Starting to open door ...");
  digitalWrite(motornumber, HIGH);
  delay(60000);
  digitalWrite(motornumber, LOW);
  Serial.print("Door should now be open.");
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
