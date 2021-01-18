#include <Wire.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"

const char ssid[] = WIFI_SSID;
const char password[] = WIFI_PASSWD;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

BH1750 lightMeter;
BME280I2C bme;
int motorPinForward = D3; 
int motorPinBackward = D6; 
float lux;
float temperature;
float pressure;
float humidity;

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

  timeClient.begin();
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  // On esp8266 devices you can select SCL and SDA pins using Wire.begin(D1, D2);
  Wire.begin();
  lightMeter.begin();
  bme.begin();
  // testing

  //openDoor();
}

void loop() {

  readLux();
  bme.read(pressure, temperature, humidity);
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

void reportData()
{
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");

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
