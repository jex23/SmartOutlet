#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define the serial port for GPS data input
HardwareSerial SerialGPS(2);

// Define the GPS object
TinyGPSPlus gps;

void setup() {
  // Start the serial communication
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

   // Set virtual pin modes
  Blynk.virtualWrite(V3, "Latitude");
  Blynk.virtualWrite(V4, "Longitude");
}

void loop() {
  // Read GPS data from the serial port
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // Check if we have a valid GPS fix
  if (gps.location.isValid()) {
    // Print latitude and longitude values in degrees
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" degrees, Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Blynk.virtualWrite(V3, gps.location.lat(), 6);
    Blynk.virtualWrite(V4, gps.location.lng(), 6);
  }
 else {
    // Display an error message on Blynk if no valid GPS data is available
    Blynk.virtualWrite(V3, "No GPS data available.");
    Blynk.virtualWrite(V4, "No GPS data available.");
  } 

  // Wait for a moment before reading the next GPS data
  delay(1000);
}
