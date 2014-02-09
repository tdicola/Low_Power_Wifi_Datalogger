/*************************************************** 
  CC3000 Low Power Datalogging
  
  Example 2: Arduino Power Down Sleep and CC3000 Shutdown
  
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
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

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
                                                   
#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
                                                         // a sensor reading is taken and sent to the server.
                                                         // Don't change this unless you also change the 
                                                         // watchdog timer configuration.

// Internal state used by the sketch.
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT);
int sleepIterations = 0;
uint32_t ip;
volatile bool watchdogActivated = false;

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}

// Enable the CC3000 and connect to the wifi network.
// Return true if enabled and connected, false otherwise.
boolean enableWiFi() {
  Serial.println(F("Turning on CC3000."));
  
  // Turn on the CC3000.
  wlan_start(0);
  
  // Connect to the AP.
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    // Couldn't connect for some reason.  Fail and move on so the hardware goes back to sleep and tries again later.
    Serial.println(F("Failed!"));
    return false;
  }
  Serial.println(F("Connected!"));
  
  // Wait for DHCP to be complete.  Make a best effort with 5 attempts, then fail and move on.
  Serial.println(F("Request DHCP"));
  int attempts = 0;
  while (!cc3000.checkDHCP())
  {
    if (attempts > 5) {
      Serial.println(F("DHCP didn't finish!"));
      return false;
    }
    attempts += 1;
    delay(100);
  }
  
  // Return success, the CC3000 is enabled and connected to the network.
  return true;
}

// Disconnect from wireless network and shut down the CC3000.
void shutdownWiFi() {
  // Disconnect from the AP if connected.
  // This might not be strictly necessary, but I found
  // it was sometimes difficult to quickly reconnect to
  // my AP if I shut down the CC3000 without first
  // disconnecting from the network.
  if (cc3000.checkConnected()) {
    cc3000.disconnect();
  }

  // Wait for the CC3000 to finish disconnecting before
  // continuing.
  while (cc3000.checkConnected()) {
    delay(100);
  }
  
  // Shut down the CC3000.
  wlan_stop();
  
  Serial.println(F("CC3000 shut down.")); 
}

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
  
  // Note that if you're sending a lot of data you
  // might need to tweak the delay here so the CC3000 has
  // time to finish sending all the data before shutdown.
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

  // Turn off the CC3000--it will be turned on later to
  // take sensor readings and send them to the server.
  wlan_stop(); 
  
  // Store the IP of the server.
  ip = cc3000.IP2U32(SERVER_IP);
 
  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  
  Serial.println(F("Setup complete."));
}

void loop(void)
{
  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated)
  {
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      // Log the sensor data (waking the CC3000, etc. as needed)
      if (enableWiFi()) {
        logSensorReading();
      }
      shutdownWiFi();
    }
  }
  
  // Go to sleep!
  sleep();
}

