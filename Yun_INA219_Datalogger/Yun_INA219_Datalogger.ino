/*************************************************** 
  Arduino Yun & INA219 Datalogger
  
  Created by Tony DiCola (tony@tonydicola.com)

  Log the current and voltage read from an INA219 breakout
  using the Arduino Yun's WiFi-accessible console.
  
  See the guide here for more information: 
  
  Load this sketch onto an Arduino Yun with the INA219
  attach to its I2C pins.  Once running, connect to the Yun's
  console and issue the following commands:
  
  S<number> - Start sampling voltage and current, with the
              specified delay (in milliseconds) between samples.
              
              For example to sample twice a second, send the
              command:
              
              S500
              
              Note the command must be followed by a newline character.
              
              Once the device is sampling, a line will be written to the
              console for each sample with the following format:
              
              <shunt voltage in volts>,<bus voltage in volts>,<current in milliamps>
              
              For example if a shunt voltage of 0.01 V, bus voltage of 5V, and
              current of 150 milliamps was measured, the following would be output:
              
              0.010,5.000,150.000
  
  X         - Stop sampling.  Must be followed by a newline character.
  
  Note that this sketch requires the Adafruit INA219 library is installed:
    https://github.com/adafruit/Adafruit_INA219
  
  To adapt this to other Arduinos which use a serial connection, read the comments below
  to change the STREAM #define, and enable the serial port in the setup() function.
 
 ****************************************************/

#include <Wire.h>
#include <Adafruit_INA219.h>

// Comment the below include if you're not running on a Yun:
#include <Console.h>

#define ACCURACY   3    // Number of digits after the decimal to write for sample values.
  
#define STREAM      Console   // Use the Arduino Yun Console as a source of input/output.
                              // Comment this line out if not running on a Yun.

//#define STREAM      Serial    // Use the Arduino hardware serial port as a source of input/output.
                                // Uncomment this line if not running on a Yun.

// Internal state used by the sketch.
Adafruit_INA219 ina219;
unsigned long lastSampleTime = 0;
unsigned long sampleDelayMS = 500;
bool isSampling = false;

void setup(void) 
{
  // Start the Yun bridge and console.
  // Comment out these lines if you're not running on a Yun.
  Bridge.begin();
  Console.begin();
  
  // Uncomment the following line if you're not running on a Yun and want to instead
  // use the Serial output.
  Serial.begin(115200);

  // Initialize the INA219.  
  ina219.begin();
}

void loop(void) 
{
  // Check for any command from the input stream.
  if (Console.available() > 0) {
    // Read command
    char command = STREAM.read();
    // Process start sampling command
    if (command == 'S' || command == 's') {
      // Read the sample delay milliseconds.
      long sd = STREAM.parseInt();
      if (sd > 0) {
        // Start sampling.
        isSampling = true; 
        sampleDelayMS = (unsigned long)sd;
      }
    }
    else if (command == 'X' || command == 'x') {
      isSampling = false;
    } 
  }
  
  // Stop processing if sampling is not happening.
  if (!isSampling) {
    return;
  }
  
  // Wait until it's time to take the next sample.
  unsigned long time = millis();
  while ((time - lastSampleTime) < sampleDelayMS) {
    time = millis();
  }
  lastSampleTime = time;
  
  // Take a sample.
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  
  // Write the sample to the output.
  STREAM.print(shuntvoltage, ACCURACY);
  STREAM.print(",");
  STREAM.print(busvoltage, ACCURACY);
  STREAM.print(",");
  STREAM.print(current_mA, ACCURACY);
  STREAM.println();
}
