#include <bluefruit.h>
#include "LSM6DS3.h"
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>


//*******************************************************************************************************************
// Instantiate Bluetooth services

BLEDis bledis; // Device information service
BLEBas blebas; // Battery service

// Built-in heart rate monitor service
BLEService hrmService = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrmMeasurement = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLECharacteristic bodySensorLocation = BLECharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);

// Built-in pulse oximeter service 
BLEService pulseOxService = BLEService(0x1822);
BLECharacteristic spO2Char = BLECharacteristic(0x2A5E);

// Custom step counter service 
BLEService stepService = BLEService("230af0e3-9c7a-4be2-81cc-dcbed3383dd5");
BLECharacteristic stepCountChar = BLECharacteristic("7f2319c9-3920-4091-ba22-6f447001a64e");

//*******************************************************************************************************************
// Sparkfun sensor board

// Reset pin, MFIO pin
int resPin = 7;
int mfioPin = 8;

int width = 411; // Possible widths: 69, 118, 215, 411us
int samples = 100; // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second

int past_heartrate = 0;

SparkFun_Bio_Sensor_Hub bioHub; 
bioData body;  

//*******************************************************************************************************************
// ISR
#define SLEEP_TIME 200    // Sleep time [ms]
bool intFlag = false;

//*******************************************************************************************************************
// Accelerometer
LSM6DS3 myIMU(I2C_MODE, 0x6A); // Accelerometer

//*******************************************************************************************************************
void setup() {

  // Initialize serial communication
  Serial.begin(115200);  while(!Serial) delay(10);
  Wire.begin();
  Serial.println("Serial and Wire initialized");

  // RTC initialization  
  initRTC(32768 * SLEEP_TIME / 1000);  
  Serial.println("RTC initialized"); 

  //*******************************************************************************************************************
  // Bluetooth setup

  // Initialize Bluefruit 
  Bluefruit.begin(2, 0);
  Serial.println("BLE initialized");
  Bluefruit.setTxPower(4);
  //Bluefruit.setConnLedInterval(50);
  Bluefruit.setName("XIAO_HealthMonitor"); 
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);


  // ---- Device Information Service ---- 
  Serial.println("Configuring the device information service");
  bledis.setManufacturer("Seeed_Studio");
  bledis.setModel("XIAO_nRF52840");
  bledis.begin(); 


  // ---- Battery Service ----
  Serial.println("Configuring the battery service");
  blebas.begin();
  //blebas.write(100); // Start at 100% battery


  // ---- Heart Rate Monitor Service ---- 
  Serial.println("Configuring the heart rate monitor service");
  hrmService.begin();
  hrmMeasurement.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ); // Bluetooth SIG defines characteristic as Notify
  hrmMeasurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // Default security for this characteristic
  hrmMeasurement.setFixedLen(2);  // Configure 1 byte for flags, 1 byte for heart rate 
  //hrmMeasurement.setCccdWriteCallback(cccd_callback);
  hrmMeasurement.begin();
  //uint8_t hrmData[2] = {0b00000110, 0x40}; 
  //hrmMeasurement.notify(hrmData, 2);

  bodySensorLocation.setProperties(CHR_PROPS_READ);
  bodySensorLocation.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  bodySensorLocation.setFixedLen(1);
  bodySensorLocation.begin();
  //bodySensorLocation.write8(0); // Other


  // ---- Pulse Oximeter Service ---- 
  Serial.println("Configuring the pulse oximeter service");
  pulseOxService.begin(); 
  spO2Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  spO2Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  spO2Char.setFixedLen(2);
  spO2Char.begin();


  // ---- Step Count Service ---- 
  Serial.println("Configuring the step count service");
  stepService.begin();
  stepCountChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  stepCountChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  stepCountChar.setFixedLen(2);  // 2 bytes for steps
  stepCountChar.begin();

  // Advertising
  Serial.println("Configuring the advertising payload");
  startAdv();
  Serial.print("Advertising at device address:");
  ble_gap_addr_t addr;
  addr = Bluefruit.getAddr();

  Serial.print("Device Address: ");
  for (int i = 5; i >= 0; i--) {
    if (addr.addr[i] < 16) Serial.print("0");
    Serial.print(addr.addr[i], HEX);
    if (i > 0) Serial.print(":");
  }

  //*************************************************************************************
  // Accelerometer Setup 
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;        // ±2g
  myIMU.settings.accelSampleRate = 4;   // 104 Hz

  // Disable other sensors
  myIMU.settings.gyroEnabled = 0;       // Disable gyro
  myIMU.settings.tempEnabled = 1;       // Disable temperature

  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  // Configure FIFO
  myIMU.settings.accelFifoEnabled = 1;     //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
  myIMU.settings.fifoThreshold = 120;      //
  myIMU.settings.fifoSampleRate = 100;     // Hz 
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
  tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
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
  error = bioHub.setPulseWidth(width);
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
  error = bioHub.setSampleRate(samples);
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

  //************************************************************************************************************************************************************************************
  Serial.println("Setup finished!");
}

//************************************************************************************************************************************************************************************
void loop() {
  // Sleep
  __WFI();
  __SEV();
  __WFI();

  if (intFlag) {
    intFlag = false;

    // Read step counter
    uint8_t dataByte = 0;
    uint16_t stepCount = 0;

    myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
    stepCount = (dataByte << 8) & 0xFFFF;

    myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
    stepCount |=  dataByte;

    Serial.print("Steps: ");
    Serial.println(stepCount);

    // Read sensor hub
    body = bioHub.readBpm();

    // Send heart rate
    uint8_t hrm_flags = 0x00;
    uint8_t hrm_val;
    if (body.confidence == 0) {
      hrm_val = (uint8_t) past_heartrate;
    }
    else {
      hrm_val = (uint8_t) body.heartRate;
      past_heartrate = body.heartRate;
    }
    uint8_t hrmData[2] = { hrm_flags, hrm_val };
    hrmMeasurement.notify(hrmData, 2);

    // Send SpO2
    uint8_t spo2_flags = 0x00;
    uint8_t spo2_val = (uint8_t) body.oxygen;
    uint8_t spo2_data[2] = { spo2_flags, spo2_val };
    spO2Char.notify(spo2_data, 2);

    // Send Step Count
    stepCountChar.write16(stepCount);

    // Send Simulated Battery
    blebas.notify(93 + random(-3, 3));  // 93% battery

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

//************************************************************************************************************************************************************************************
// Bluetooth helper functions
void startAdv() {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Add services
  Bluefruit.Advertising.addService(hrmService);
  Bluefruit.Advertising.addService(pulseOxService);
  Bluefruit.Advertising.addService(stepService);
  Bluefruit.Advertising.addService(blebas);

  // Include name
  //Bluefruit.ScanResponse.addName();

  // Start advertising 
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setIntervalMS(32, 244);    // Fast advertising
  Bluefruit.Advertising.setFastTimeout(30); // Number of seconds in fast mode
  Bluefruit.Advertising.start(0);  // 0 = advertise forever
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name); 
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising");
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
  int16_t accelData[20][3]; // Array to store 20 samples of X, Y, Z data
  
  // Variables for min/max debugging
  int16_t minX = INT16_MAX, maxX = INT16_MIN;
  int16_t minY = INT16_MAX, maxY = INT16_MIN;
  int16_t minZ = INT16_MAX, maxZ = INT16_MIN;
  
  // Collect 19 accelerometer samples from FIFO (raw data only)
  int16_t rawData[19][3]; // Store raw accelerometer data temporarily
  
  for (int i = 0; i < 19; i++) {
    myIMU.fifoRead(); myIMU.fifoRead(); myIMU.fifoRead(); // Read (and ignore) three gyrometer readings
    rawData[i][0] = (int16_t) myIMU.fifoRead(); // Raw X
    rawData[i][1] = (int16_t) myIMU.fifoRead(); // Raw Y
    rawData[i][2] = (int16_t) myIMU.fifoRead(); // Raw Z
    myIMU.fifoTimestamp(); myIMU.fifoTimestamp(); // Read (and ignore) external sensor data and timestamp data
  }
  
  // Reset FIFO immediately after reading raw data
  uint8_t tempFIFO_CTRL5 = 0;
  tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
  tempFIFO_CTRL5 |= 0;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

  tempFIFO_CTRL5 = 0;
  tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
  tempFIFO_CTRL5 |= 1;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);
  
  // Now process the raw data - track min/max and convert to milli-g
  for (int i = 0; i < 19; i++) {
    int16_t ax = rawData[i][0];
    int16_t ay = rawData[i][1];
    int16_t az = rawData[i][2];
    
    // Track min/max for debugging
    if (ax < minX) minX = ax;
    if (ax > maxX) maxX = ax;
    if (ay < minY) minY = ay;
    if (ay > maxY) maxY = ay;
    if (az < minZ) minZ = az;
    if (az > maxZ) maxZ = az;
    
    // Convert to milli-g (LSB = 0.061 mg for ±2g range)
    accelData[i][0] = ax * 61 / 1000; // X in milli-g
    accelData[i][1] = ay * 61 / 1000; // Y in milli-g  
    accelData[i][2] = az * 61 / 1000; // Z in milli-g
  }
  
  // Use 19th sample for 20th sample to handle timing discrepancies
  accelData[19][0] = accelData[18][0];
  accelData[19][1] = accelData[18][1];
  accelData[19][2] = accelData[18][2];
  
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