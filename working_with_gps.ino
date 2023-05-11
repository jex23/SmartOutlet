#define BLYNK_TEMPLATE_ID "TMPL6fzI-_4I4"
#define BLYNK_DEVICE_NAME "Smart Switch"
#define BLYNK_AUTH_TOKEN "qYWpT0-IW6NpnsCVxUxEh6fu4-80ctrh"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
//#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp32.h>

#include "max6675.h" // max6675.h file is part of the library that you should download from Robojax.com

int soPin = 19;// SO=Serial Out
int csPin = 5;// CS = chip select CS pin
int sckPin = 18;// SCK = Serial Clock pin

MAX6675 james(sckPin, csPin, soPin);// create instance object of MAX6675

static const int RXPin = 2, TXPin = 15;   // GPIO 4=D2(conneect Tx of GPS) and GPIO 5=D1(Connect Rx of GPS
static const uint32_t GPSBaud = 9600; //if Baud rate 9600 didn't work in your case then use 4800
 
TinyGPSPlus gps;      // The TinyGPS++ object
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget
SoftwareSerial mygps(RXPin, TXPin);  // The serial connection to the GPS device


float latitude;     //Storing the Latitude
float longitude;    //Storing the Longitude
float velocity;     //Variable  to store the velocity
float sats;         //Variable to store no. of satellites response
String bearing;     //Variable to store orientation or direction of GPS

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "jex";  // type your wifi name
char pass[] = "12345678";  // type your wifi password

BlynkTimer timer;

int relayPin = 22;

unsigned int move_index = 1;       // fixed location for now


void sendSensor()
{
  float c = james.readCelsius();
  float f = james.readFahrenheit(); // or dht.readTemperature(true) for Fahrenheit
  
  if(c >= 60){
    digitalWrite(relayPin, LOW);
    Blynk.logEvent("device_overheat","WARNING!!!THE DEVICE IS OVERHEATING, OUTLET DISCONNECTED");
    }
    else if(59 >= c > 50){
      Blynk.logEvent("early_warning","WARNING!!!THE DEVICE WILL OVERHEAT");
      }
   
  
    
  if (isnan(c) || isnan(f)) {
    Serial.println("Failed to read from THermocouple sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V0, c);
    Blynk.virtualWrite(V1, f);
    Serial.print("Celcius : ");
    Serial.print(c);
    Serial.print("    Fahrenheit : ");
    Serial.println(f);

     delay(1000);
    
}
void setup()
{   
  
  Serial.begin(115200);
  
  pinMode(relayPin,OUTPUT);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(100L, sendSensor);

 Serial.println();
  mygps.begin(GPSBaud);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
  }
void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    Blynk.virtualWrite(V3, "GPS ERROR");  // Value Display widget  on V3 if GPS not detected
  }
}

void loop()
{
while (mygps.available() > 0)
  {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(mygps.read()))
      displayInfo();
  }
  
  Blynk.run();
  timer.run();
 }
 BLYNK_WRITE(V2) // This function gets called every time there's a value update for virtual pin V0
{
  int value = param.asInt(); // Get the value of V0
  digitalWrite(relayPin, value); // Set the LED pin to the received value (0 or 1)
  }
  void displayInfo()
{
  if (gps.location.isValid() )
  {
    sats = gps.satellites.value();       //get number of satellites
    latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
    velocity = gps.speed.kmph();         //get velocity
    bearing = TinyGPSPlus::cardinal(gps.course.value());     // get the direction

    // Target location (smartphone)
    const double targetLat = 13.1473056;
    const double targetLon = 123.7310278;
 
    Serial.print("SATS:  ");
    Serial.println(sats);  // float to x decimal places
    Serial.print("LATITUDE:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONGITUDE: ");
    Serial.println(longitude, 6);
    Serial.print("SPEED: ");
    Serial.print(velocity);
    Serial.println("kmph");
    Serial.print("DIRECTION: ");
    Serial.println(bearing);

    // Distance between GPS module and smartphone in meters
    double distance = TinyGPSPlus::distanceBetween(latitude, longitude, targetLat, targetLon) * 1000;

    // Check if distance is within 10 meters
    if (distance <= 10.0) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
}
 
    Blynk.virtualWrite(V3, String(latitude, 6));
    Blynk.virtualWrite(V4, String(longitude, 6));
    Blynk.virtualWrite(V5, sats);
    Blynk.virtualWrite(V6, velocity);
    Blynk.virtualWrite(V7, bearing);
    myMap.location(move_index, latitude, longitude, "GPS_Location");
  }
  Serial.println();
}
