#include "Wire.h"
#include "LSM6DS3.h"
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

//*******************************************************************************************************************
// ISR
#define SLEEP_TIME 200    // Sleep time [ms]
bool intFlag = false;

//*******************************************************************************************************************
LSM6DS3 myIMU(I2C_MODE, 0x6A); // Accelerometer

//*******************************************************************************************************************
// Sparkfun sensor board

// Reset pin, MFIO pin
#define resPin  7
#define mfioPin  8

#define BIOSENSOR_WIDTH  411 // Possible widths: 69, 118, 215, 411us
#define BIOSENSOR_SAMPLES  100 // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second

int past_heartrate = 0;

SparkFun_Bio_Sensor_Hub bioHub; 
bioData body; 

//*******************************************************************************************************************
void setup() {

  // Serial port initialization
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  Serial.println("Serial initialized");


  // RTC initialization  
  initRTC(32768 * SLEEP_TIME / 1000);  
  Serial.println("RTC initialized"); 

  //*************************************************************************************
  // Configure acclerometer 
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;        // ±2g
  myIMU.settings.accelSampleRate = 8;   // 104 Hz

  // Disable other sensors
  myIMU.settings.gyroEnabled = 0;       // Disable gyro
  myIMU.settings.tempEnabled = 0;       // Disable temperature

  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  // Configure FIFO
  myIMU.settings.accelFifoEnabled = 1;     //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
  //myIMU.settings.fifoThreshold = 30000;      //
  myIMU.settings.fifoSampleRate = 400;     // Hz 
  myIMU.settings.fifoModeWord = 1;         // Continuous mode

  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("Failed to initalize LSM6DS3");
    while (1);
  }
  Serial.println("LSM6DS3 initialized!");

  // Initialize Pedometer
  if (config_pedometer() != 0) {
    Serial.println("Failed to configure pedometer");
    while (1);
  }
  Serial.println("Pedometer initialized!");

  // Initialize FIFO
  myIMU.fifoBegin();
  myIMU.fifoClear();

  uint8_t tempFIFO_CTRL5 = 0;
  tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
  tempFIFO_CTRL5 |= 1;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

  Serial.println("FIFO initialized!");

  //************************************************************************************************************************************************************************************
  // Biosensor setup
  if (bioHub.begin(Wire, resPin, mfioPin) == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!");
 
  Serial.println("Configuring Sensor...."); 
  int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings. 
  if(error == 0){ // Zero errors!
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  // Set pulse width.
  error = bioHub.setPulseWidth(BIOSENSOR_WIDTH);
  if (error == 0){// Zero errors.
    Serial.println("Pulse Width Set.");
  }
  else {
    Serial.println("Could not set Pulse Width.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  // Check that the pulse width was set. 
  int pulseWidthVal = bioHub.readPulseWidth();
  Serial.print("Pulse Width: ");
  Serial.println(pulseWidthVal);

  // Set sample rate per second. Remember that not every sample rate is
  // available with every pulse width. Check hookup guide for more information.  
  error = bioHub.setSampleRate(BIOSENSOR_SAMPLES);
  if (error == 0){// Zero errors.
    Serial.println("Sample Rate Set.");
  }
  else {
    Serial.println("Could not set Sample Rate!");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  // Check sample rate.
  int sampleVal = bioHub.readSampleRate();
  Serial.print("Sample rate is set to: ");
  Serial.println(sampleVal); 

  // Enable host side accelerometer
  if (!enableHostSideAccelerometer()) {
    Serial.println("Failed to enable host side accelerometer");
    while (1); // Halt if accelerometer enable fails
  }

  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  Serial.println("Loading up the buffer with data....");
  delay(4000); 

  Serial.println("Setup finished!");
}

void loop() {
  // Sleep
  __WFI();
  __SEV();
  __WFI();

  if (intFlag) {
    intFlag = false;

    // Send accelerometer data to biohub
    sendAccelerometerToBioHub();

    // Read step counter (2 bytes)
    /*uint8_t dataByte = 0;
    uint16_t stepCount = 0;

    myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
    stepCount = (dataByte << 8) & 0xFFFF;

    myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
    stepCount |=  dataByte;

    Serial.print("Steps: ");
    Serial.println(stepCount);*/

    
    // Read sensor hub
    body = bioHub.readBpm();
    Serial.print("Heartrate: ");
    Serial.println(body.heartRate); 
    Serial.print("Confidence: ");
    Serial.println(body.confidence); 
    Serial.print("Oxygen: ");  
    Serial.println(body.oxygen); 
    Serial.print("Status: ");
    Serial.println(body.status);  
  }
}

//**************************************************************************************************************
// configure accelerometer
int config_pedometer(void) {
    uint8_t errorAccumulator = 0;

    // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);

    // Step 3:	Enable pedometer algorithm
    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

    //Step 4:	Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x80);

    return errorAccumulator;
}

//**************************************************************************************************************
// Function to enable the biohub to receive accelerometer data
bool enableHostSideAccelerometer() {
  Serial.println("Enabling host side accelerometer...");
  
  // Send command: 44 04 01 01
  Wire.beginTransmission(0x55);
  Wire.write(0x44); // Command family
  Wire.write(0x04); // Command index
  Wire.write(0x01); // Enable
  Wire.write(0x01); // Parameter
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("I2C transmission error enabling accelerometer: ");
    Serial.println(error);
    return false;
  }
  
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(0x55, 1);
  
  if (Wire.available() >= 1) {
    uint8_t response = Wire.read();
    
    if (response == 0x00) {
      Serial.println("Host side accelerometer enabled successfully");
      return true;
    } else {
      Serial.print("Failed to enable host side accelerometer, response: ");
      Serial.println(response, HEX);
      return false;
    }
  } else {
    Serial.println("No response from biohub when enabling accelerometer");
    return false;
  }
}

//**************************************************************************************************************
// Function to send accelerometer data to biohub
bool sendAccelerometerToBioHub() {
  int16_t accelData[40][3]; // Array to store 20 samples of X, Y, Z data
  
  // Variables for min/max debugging
  int16_t minX = INT16_MAX, maxX = INT16_MIN;
  int16_t minY = INT16_MAX, maxY = INT16_MIN;
  int16_t minZ = INT16_MAX, maxZ = INT16_MIN;
  
  // Collect 19 accelerometer samples from FIFO (raw data only)
  int16_t rawData[40][3]; // Store raw accelerometer data temporarily
  
  int i = 0;
  unsigned long startTime = millis();
  while( (( myIMU.fifoGetStatus() & 0x1000 ) == 0 && i < 40) ) {
    myIMU.fifoRead(); myIMU.fifoRead(); myIMU.fifoRead(); // Read (and ignore) three gyrometer readings
    rawData[i][0] = (int16_t) myIMU.fifoRead(); // Raw X
    rawData[i][1] = (int16_t) myIMU.fifoRead(); // Raw Y
    rawData[i][2] = (int16_t) myIMU.fifoRead(); // Raw Z
    myIMU.fifoRead(); myIMU.fifoRead(); myIMU.fifoRead(); myIMU.fifoRead(); myIMU.fifoRead(); myIMU.fifoRead(); // Read (and ignore) external sensor data and timestamp data
    i += 1;
  }
  unsigned long endTime = millis();
  unsigned long duration = endTime - startTime;
  Serial.print("Duration: ");
  Serial.println(duration);
  Serial.println(myIMU.fifoGetStatus() & 0x0FFF);
  Serial.println(i);
  
  // Reset FIFO immediately after reading raw data
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x00);

  uint8_t tempFIFO_CTRL5 = 0;
  tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
  tempFIFO_CTRL5 |= 1;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);
  
  // Now process the raw data - track min/max and convert to milli-g
  for (int j = 0; j < i; j++) {
    int16_t ax = rawData[j][0];
    int16_t ay = rawData[j][1];
    int16_t az = rawData[j][2];
    
    // Track min/max for debugging
    if (ax < minX) minX = ax;
    if (ax > maxX) maxX = ax;
    if (ay < minY) minY = ay;
    if (ay > maxY) maxY = ay;
    if (az < minZ) minZ = az;
    if (az > maxZ) maxZ = az;
    
    // Convert to milli-g (LSB = 0.061 mg for ±2g range)
    accelData[j][0] = ax * 61 / 1000; // X in milli-g
    accelData[j][1] = ay * 61 / 1000; // Y in milli-g  
    accelData[j][2] = az * 61 / 1000; // Z in milli-g
  }
  
  // Print min/max values for debugging
  int16_t minX_mg = minX * 61 / 1000;
  int16_t maxX_mg = maxX * 61 / 1000;
  int16_t minY_mg = minY * 61 / 1000;
  int16_t maxY_mg = maxY * 61 / 1000;
  int16_t minZ_mg = minZ * 61 / 1000;
  int16_t maxZ_mg = maxZ * 61 / 1000;

  Serial.print("Accel Min [X Y Z] (mg): ");
  Serial.print(minX_mg); Serial.print(" ");
  Serial.print(minY_mg); Serial.print(" ");
  Serial.println(minZ_mg);

  Serial.print("Accel Max [X Y Z] (mg): ");
  Serial.print(maxX_mg); Serial.print(" ");
  Serial.print(maxY_mg); Serial.print(" ");
  Serial.println(maxZ_mg); 
  
  // Send command header: AA 14 00
  Wire.beginTransmission(0x55); // 7-bit address
  Wire.write(0x14); // Command family
  Wire.write(0x00); // Command index
  
  // Send 20 samples (each sample is 6 bytes: 2 bytes each for X, Y, Z)
  for (int i = 0; i < 5; i++) {
    // Send X value (MSB first)
    Wire.write((accelData[i][0] >> 8) & 0xFF);
    Wire.write(accelData[i][0] & 0xFF);
    
    // Send Y value (MSB first)
    Wire.write((accelData[i][1] >> 8) & 0xFF);
    Wire.write(accelData[i][1] & 0xFF);
    
    // Send Z value (MSB first)
    Wire.write((accelData[i][2] >> 8) & 0xFF);
    Wire.write(accelData[i][2] & 0xFF);
  }
  
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C transmission error: ");
    Serial.println(error);
    return false;
  }

  delay(5);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(0x55, static_cast<uint8_t>(1)); // Request 1 byte from biohub
  
  if (Wire.available() >= 1) {
    uint8_t response = Wire.read(); // Should be 0x00
    
    if (response == 0x00) {
      Serial.println("Accelerometer data sent successfully to biohub");
      return true;
    } else {
      Serial.print("Unexpected response from biohub: ");
      Serial.println(response, HEX);
      return false;
    }
  } else {
    Serial.println("No response from biohub");
    return false;
  }
}

//**************************************************************************************************************
// RTC initialization
void initRTC(unsigned long count30) 
{
  // See "6.22 RTC - Real-time counter 6.22.10 Registers"
  NRF_CLOCK->TASKS_LFCLKSTOP = 1;
  NRF_CLOCK->LFCLKSRC = 1;      // select LFXO
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
  while(NRF_CLOCK->LFCLKSTAT != 0x10001)
  
  NRF_RTC2->TASKS_STOP = 1;      // stop counter 
  NRF_RTC2->TASKS_CLEAR = 1;     // clear counter
  NRF_RTC2->PRESCALER = 0;       // set counter prescaler, fout=32768/(1+PRESCALER)　32768Hz
  NRF_RTC2->CC[0] = count30;     // set value for TRC compare register 0
  NRF_RTC2->INTENSET = 0x10000;  // enable interrupt CC[0] compare match event
  NRF_RTC2->EVTENCLR = 0x10000;  // clear counter when CC[0] event
  NVIC_SetPriority(RTC2_IRQn,3); // Higher priority than SoftDeviceAPI
  NVIC_EnableIRQ(RTC2_IRQn);     // enable interrupt  
  NRF_RTC2->TASKS_START = 1;     // start Timer
}

//**************************************************************************************************************
// RTC interrupt handler
extern "C" void RTC2_IRQHandler(void)
{
  if ((NRF_RTC2->EVENTS_COMPARE[0] != 0) && ((NRF_RTC2->INTENSET & 0x10000) != 0)) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;  // clear compare register 0 event
    NRF_RTC2->TASKS_CLEAR = 1;        // clear counter

    intFlag = true;   
  }
}
