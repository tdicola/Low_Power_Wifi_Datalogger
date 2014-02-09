/*************************************************** 
  CC3000 Low Power Datalogging
  
  Example 1: No Low Power Optimizations
  
  Created by Tony DiCola (tony@tonydicola.com)

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Based on CC3000 examples written by Limor Fried & Kevin Townsend 
  for Adafruit Industries and released under a BSD license.
  All text above must be included in any redistribution.
  
 ****************************************************/

#include <Adafruit_CC3000.h>
#include <SPI.h>

// CC3000 configuration.
#define ADAFRUIT_CC3000_IRQ    3
#define ADAFRUIT_CC3000_VBAT   5
#define ADAFRUIT_CC3000_CS     10

// Wifi network configuration.
#define WLAN_SSID              "network"
#define WLAN_PASS              "password"
#define WLAN_SECURITY          WLAN_SEC_WPA2

// Data logging configuration.
#define LOGGING_FREQ_SECONDS   60       // Seconds to wait before a new sensor reading is logged.

#define SENSOR_PIN             4        // Analog pin to read sensor values from (for example
                                        // from a photocell or other resistive sensor).

#define SERVER_IP              192, 168, 1, 105    // Logging server IP address.  Note that this
                                                   // should be separated with commas and NOT periods!

#define SERVER_PORT            8000                // Logging server listening port.

// Internal state used by the sketch.
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT);
uint32_t ip;
unsigned long lastSend = 0;

// Take a sensor reading and send it to the server.
void logSensorReading() {
  // Take a sensor reading
  int reading = analogRead(SENSOR_PIN);
  
  // Connect to the server and send the reading.
  Serial.print(F("Sending measurement: ")); Serial.println(reading, DEC);
  Adafruit_CC3000_Client server = cc3000.connectTCP(ip, SERVER_PORT);
  if (server.connected()) {
    server.println(reading);
  }
  else {
    Serial.println(F("Error sending measurement!"));
  }

  // Wait a small period of time before closing the connection
  // so the message is sent to the server.
  delay(100);
  
  // Close the connection to the server.
  server.close();
}

void setup(void)
{  
  Serial.begin(115200);
  
  // Initialize the CC3000.
  Serial.println(F("\nInitializing CC3000..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }

  // Connect to AP.
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
  Serial.println(F("Connected!"));
  
  // Wait for DHCP to be complete.
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100);
  }
  
  // Store the IP of the server.
  ip = cc3000.IP2U32(SERVER_IP);
 
  Serial.println(F("Setup complete."));
}

void loop(void)
{
  unsigned long time = millis();
  if (time - lastSend >= (1000 * (unsigned long)LOGGING_FREQ_SECONDS)) {
    logSensorReading();
    lastSend = time;
  }
}

