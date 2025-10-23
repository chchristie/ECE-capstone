#include <Arduino.h>
#include "LSM6DS3.h"
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>
#include <bluefruit.h>

//*******************************************************************************************************************
// Watchdog Timer
#define WATCHDOG_TIMEOUT_SECONDS 10  // Reset if frozen for 10 seconds

//*******************************************************************************************************************
// ISR
volatile bool intFlag = false;
volatile int intCounter = 0;
#define SLEEP_TIME 10  

//*******************************************************************************************************************
// IMU
#define IMU_ADDRESS 0x6A
LSM6DS3 myIMU(I2C_MODE, IMU_ADDRESS);
#define IMU_BUFFER_SIZE  20
#define IMU_SAMPLING_RATE 100

int16_t accelBuffX1[10];
int16_t accelBuffX2[10];
int16_t accelBuffY1[10];
int16_t accelBuffY2[10];
int16_t accelBuffZ1[10];
int16_t accelBuffZ2[10];

//*******************************************************************************************************************
// Sparkfun sensor board
#define BIOSENSOR_ADDRESS 0x55

// Reset pin, MFIO pin
int resPin = 7;
int mfioPin = 8;

#define BIOSENSOR_WIDTH 411 // Possible widths: 69, 118, 215, 411us
#define BIOSENSOR_SAMPLES 100 // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second

int past_heartrate = 0;
uint8_t biosensorErrorCount = 0;  // Track consecutive errors
#define MAX_BIOSENSOR_ERRORS 5    // Reset biosensor after 5 consecutive errors

SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);
bioData biohubData;
uint8_t biohubFifoData[6];

//*******************************************************************************************************************
// Battery monitoring
#define VBAT_DIVIDER 2.0
#define VBAT_MIN 3.0
#define VBAT_MAX 3.7

//*******************************************************************************************************************
// Bluetooth Services and Characteristics
BLEDis bledis;

// Battery Service (0x180F) - Custom implementation instead of BLEBas
BLEService batteryService = BLEService(UUID16_SVC_BATTERY);
BLECharacteristic batteryChar = BLECharacteristic(UUID16_CHR_BATTERY_LEVEL);

// Heart Rate Service (0x180D)
BLEService hrmService = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrmMeasurement = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLECharacteristic bodySensorLocation = BLECharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);

// Pulse Oximeter Service (0x1822)
BLEService pulseOxService = BLEService(0x1822);
BLECharacteristic pulseOxChar = BLECharacteristic(0x2A5E);

// Custom accelerometer service 
#define accelService_UUID(val) (const uint8_t[]) { \
    0x85, 0xB4, 0x93, 0x81, 0x58, 0x40, 0x0C, 0xBE, \
    0xD1, 0x4D, (uint8_t)(val & 0xff), (uint8_t)(val >> 8), 0x00, 0x00, 0xC4, 0x52 }

// Motion Service (0x1819) for Accelerometer
BLEService motionService = BLEService(0x1819);
BLECharacteristic accelX1Char = BLECharacteristic(accelService_UUID(0x0010));
BLECharacteristic accelX2Char = BLECharacteristic(accelService_UUID(0x0020));
BLECharacteristic accelY1Char = BLECharacteristic(accelService_UUID(0x0030));
BLECharacteristic accelY2Char = BLECharacteristic(accelService_UUID(0x0040));
BLECharacteristic accelZ1Char = BLECharacteristic(accelService_UUID(0x0050));
BLECharacteristic accelZ2Char = BLECharacteristic(accelService_UUID(0x0060));
BLECharacteristic accelTimestampChar = BLECharacteristic(accelService_UUID(0x0070));

// Time Sync Characteristic (custom UUID for receiving timestamp from phone)
BLECharacteristic timeSyncChar = BLECharacteristic("6E400010-B5A3-F393-E0A9-E50E24DCCA9E");

// Timestamp tracking
uint64_t baseTimestampMs = 0;  // Unix milliseconds from phone at sync time
uint32_t baseMillis = 0;       // millis() value when time was synced

bool connectedFlag = false;

//*******************************************************************************************************************
void setup() {

  // Serial port initialization
  // Serial.begin(115200);
  // while (!Serial && millis() < 5000);  // Wait max 5 seconds for serial
  // Serial.println("Serial initialized");
  // Serial.println("===========================================");
  // Serial.println("IMPROVED Arduino Code for Heart Rate App");
  // Serial.println("Outputs: Heart Rate, SpO2, Battery, Accelerometer");
  // Serial.println("Version: 1.1 - Stability Fixes Applied");
  // Serial.println("===========================================");

  // Initialize Watchdog Timer for stability
  initWatchdog();
  // Serial.println("Watchdog timer initialized (10 second timeout)");

  // I2C initialization
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C speed for reliability
  // Serial.println("I2C initialized at 400kHz");

  // RTC initialization
  initRTC(32768 * SLEEP_TIME / 1000);
  // Serial.println("RTC initialized");

  //*************************************************************************************
  // Configure accelerometer
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;        // ±2g
  myIMU.settings.accelSampleRate = 104; // 104 Hz

  // Disable other sensors
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.tempEnabled = 0;

  // Initialize IMU
  if (myIMU.begin() != 0) {
    // Serial.println("Failed to initialize IMU - continuing anyway");
    // Don't halt - accelerometer optional
  } else {
    // Serial.println("IMU initialized successfully!");
  }

  //************************************************************************************************************************************************************************************
  // Biosensor setup with error recovery
  // Serial.println("Initializing MAX30101 biosensor...");

  if (bioHub.begin() == 0) {
    // Serial.println("Biosensor started!");
  } else {
    // Serial.println("Warning: Could not communicate with biosensor - will retry");
    // Don't halt - will retry later
  }

  // Serial.println("Configuring Biosensor....");
  int error = bioHub.configBpm(MODE_ONE);
  if(error == 0){
    // Serial.println("Biosensor configured successfully.");
  } else {
    // Serial.print("Warning: Error configuring sensor: ");
    // Serial.println(error);
    // Continue anyway - sensor may recover
  }

  // Enable host side accelerometer
  if (!enableHostSideAccelerometer()) {
    // Serial.println("Failed to configure biosensor for host side accelerometer");
  }
  else {
    // Serial.println("Biosensor configured for host side accelerometer");
  }

  // Set sample rate per second
  error = bioHub.setSampleRate(BIOSENSOR_SAMPLES);
  if (error != 0) {
    // Serial.print("Warning: Could not set biosensor sample rate: ");
    // Serial.println(error);
  } else {
    // Serial.print("Biosensor sample rate set to ");
    // Serial.println(bioHub.readSampleRate());
  }

  // Set pulse width
  error = bioHub.setPulseWidth(BIOSENSOR_WIDTH);
  if (error != 0) {
    // Serial.print("Warning: Could not set biosensor pulse width: ");
    // Serial.println(error);
  } else {
    // Serial.print("Biosensor pulse width set to ");
    // Serial.println(bioHub.readPulseWidth());
  }

  //************************************************************************************************************************************************************************************
  // Bluetooth Initialization

  // Serial.println("Initializing Bluetooth...");

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("HeartRate_SpO2_Accel");

  // Set low power mode to prevent deep sleep that kills BLE connection
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Device Information Service
  bledis.setManufacturer("SeedStudio");
  bledis.setModel("XIAO_nRF52840");
  bledis.begin();
  // Serial.println("Device Information Service started");

  // Battery Service (0x180F) - Custom implementation
  batteryService.begin();

  batteryChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  batteryChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryChar.setFixedLen(1);
  batteryChar.setCccdWriteCallback(cccd_callback);
  batteryChar.begin();
  batteryChar.write8(93); // Initial value
  // Serial.println("Battery Service (0x180F) started");

  // Heart Rate Service (0x180D)
  hrmService.begin();

  hrmMeasurement.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  hrmMeasurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrmMeasurement.setFixedLen(2);
  hrmMeasurement.setCccdWriteCallback(cccd_callback);
  hrmMeasurement.begin();

  bodySensorLocation.setProperties(CHR_PROPS_READ);
  bodySensorLocation.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bodySensorLocation.setFixedLen(1);
  bodySensorLocation.begin();
  bodySensorLocation.write8(2); // Wrist location
  // Serial.println("Heart Rate Service (0x180D) started");

  // Pulse Oximeter Service (0x1822)
  pulseOxService.begin();

  pulseOxChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  pulseOxChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pulseOxChar.setFixedLen(2);
  pulseOxChar.setCccdWriteCallback(cccd_callback);
  pulseOxChar.begin();
  // Serial.println("Pulse Oximeter Service (0x1822) started");

  // Motion Service (0x1819)
  motionService.begin();

  accelX1Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelX2Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelY1Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelY2Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelZ1Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelZ2Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);

  accelX1Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelX2Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelY1Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelY2Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelZ1Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelZ2Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  
  accelX1Char.setFixedLen(20);  
  accelX2Char.setFixedLen(20);  
  accelY1Char.setFixedLen(20);  
  accelY2Char.setFixedLen(20);  
  accelZ1Char.setFixedLen(20);  
  accelZ2Char.setFixedLen(20);  
  
  accelX1Char.setCccdWriteCallback(cccd_callback);
  accelX2Char.setCccdWriteCallback(cccd_callback);
  accelY1Char.setCccdWriteCallback(cccd_callback);
  accelY2Char.setCccdWriteCallback(cccd_callback);
  accelZ1Char.setCccdWriteCallback(cccd_callback);
  accelZ2Char.setCccdWriteCallback(cccd_callback);

  accelX1Char.begin();
  accelX2Char.begin();
  accelY1Char.begin();
  accelY2Char.begin();
  accelZ1Char.begin();
  accelZ2Char.begin();

  accelTimestampChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelTimestampChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelTimestampChar.setFixedLen(8);
  accelTimestampChar.setCccdWriteCallback(cccd_callback);
  accelTimestampChar.begin();

  // Serial.println("Motion Service (0x1819) started");

  // Time Sync Characteristic (writable - phone sends timestamp here)
  timeSyncChar.setProperties(CHR_PROPS_WRITE);
  timeSyncChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  timeSyncChar.setFixedLen(8);  // 8 bytes for 64-bit timestamp
  timeSyncChar.setWriteCallback(timesync_callback);
  timeSyncChar.begin();
  // Serial.println("ime Sync Characteristic started");

  // Advertising Setup
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Add ALL services to advertising
  Bluefruit.Advertising.addService(hrmService);       // 0x180D
  Bluefruit.Advertising.addService(batteryService);   // 0x180F
  Bluefruit.Advertising.addService(pulseOxService);   // 0x1822
  Bluefruit.Advertising.addService(motionService);    // 0x1819

  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  // Serial.println("Advertising started with all services");
  // Serial.println("   - Heart Rate (0x180D)");
  // Serial.println("   - Battery (0x180F)");
  // Serial.println("   - Pulse Oximeter (0x1822)");
  // Serial.println("   - Motion (0x1819)");

  // Print address
  // Serial.print("Device address: ");
  uint8_t addr[6];
  Bluefruit.getAddr(addr);
  // for (int i = 5; i >= 0; i--) {
  //   if (addr[i] < 16) // Serial.print("0");
  //   // Serial.print(addr[i], HEX);
  //   if (i > 0) // Serial.print(":");
  // }
  // Serial.println("");

  //************************************************************************************************************************************************************************************
  // Serial.println("Loading biosensor buffer with initial data...");
  delay(4000);

  // Serial.println("===========================================");
  // Serial.println("Setup complete! Ready for connections.");
  // Serial.println("All sensors operational:");
  // Serial.println("âœ… Heart Rate sensor");
  // Serial.println("âœ… SpO2 sensor");
  // Serial.println("âœ… Battery monitor");
  // Serial.println("âœ… Accelerometer");
  // Serial.println("âœ… Timestamp support");
  // Serial.println("Waiting for React Native app connection...");
  // Serial.println("===========================================");
}

void loop() {
  // Pet the watchdog to prevent reset
  feedWatchdog();

  // Sleep
  __WFI();
  __SEV();
  __WFI();

  if (intFlag) {
    intFlag = false;
    intCounter += 1;

    if (intCounter <= 10) {
      accelBuffX1[intCounter - 1] = myIMU.readRawAccelX();
      accelBuffY1[intCounter - 1] = myIMU.readRawAccelY();
      accelBuffZ1[intCounter - 1] = myIMU.readRawAccelZ();
    }
    else {
      accelBuffX2[intCounter - 11] = myIMU.readRawAccelX();
      accelBuffY2[intCounter - 11] = myIMU.readRawAccelY();
      accelBuffZ2[intCounter - 11] = myIMU.readRawAccelZ();
    }

    uint8_t biohubStatus = 0;
    switch (intCounter) {
      case 2:
        // Skip - no host-side accelerometer
        break;
      case 3:
        requestBiohubStatus();
        break;
      case 4:
        biohubStatus = readBiohubStatus();
        if (biohubStatus == 1) {
          biosensorErrorCount++;
          // Serial.print("âš ï¸ Biosensor I2C error (");
          // Serial.print(biosensorErrorCount);
          // Serial.println("/5)");

          // Reset biosensor if too many errors
          if (biosensorErrorCount >= MAX_BIOSENSOR_ERRORS) {
            // Serial.println("ðŸ”„ Resetting biosensor due to repeated errors...");
            resetBiosensor();
            biosensorErrorCount = 0;
          }
        } else {
          biosensorErrorCount = 0;  // Reset error counter on success
        }
        break;
      case 5:
        requestBiohubNumFifoSamples();
        break;
      case 6:
        readBiohubNumFifoSamples();
        break;
      case 7:
        requestBiohubData();
        break;
      case 8:
        biohubStatus = readBiohubData();
        if (biohubStatus == 1) {
          // Serial.println("âš ï¸ Failed to read biosensor data - will retry");
        } else {
          // Successfully read data
          // Serial.print("HR: ");
          // Serial.print(biohubData.heartRate);
          // Serial.print(" | SpO2: ");
          // Serial.print(biohubData.oxygen);
          // Serial.print(" | Status: ");
          // Serial.println(biohubData.status);
        }
        break;
      case 10:
        if (Bluefruit.connected()) {
          // Send ALL sensor data
          sendHrmBLE();         // Heart rate
          sendPulseOxBLE();     // SpO2
          sendBatteryBLE();     // Battery level
          sendAccelerometerBLE(); // Accelerometer with timestamp
        }
        break;
      case 20:
        intCounter = 0;
        sendAccelerometerDataToBiohub();
        break;
    }

    if (intFlag == true) {
      // Serial.print("âš ï¸ Processing took too long at step: ");
      // Serial.println(intCounter);
    }
  }
}

//**************************************************************************************************************
// Watchdog functions
void initWatchdog() {
  // Configure WDT
  NRF_WDT->CONFIG = 0x01;  // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = WATCHDOG_TIMEOUT_SECONDS * 32768;  // Timeout in seconds (32768 Hz clock)
  NRF_WDT->RREN |= 0x01;  // Enable reload register 0

  // Enable WDT
  NRF_WDT->TASKS_START = 1;
}

void feedWatchdog() {
  NRF_WDT->RR[0] = 0x6E524635;  // Reload value
}

//**************************************************************************************************************
// Reset biosensor after errors
void resetBiosensor() {
  // Try to reinitialize the biosensor
  delay(100);

  if (bioHub.begin() == 0) {
    // Serial.println("âœ… Biosensor reinitialized successfully");

    // Reconfigure
    int error = bioHub.configBpm(MODE_ONE);
    if (error == 0) {
      // Serial.println("âœ… Biosensor reconfigured");
    }

    bioHub.setSampleRate(BIOSENSOR_SAMPLES);
    bioHub.setPulseWidth(BIOSENSOR_WIDTH);
  } else {
    // Serial.println("âŒ Failed to reinitialize biosensor - will retry later");
  }
}

//**************************************************************************************************************
// CCCD callback - called when client subscribes/unsubscribes to notifications
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
  if (chr->uuid == hrmMeasurement.uuid) {
    // Serial.print("Heart Rate CCCD updated: ");
  }
  else if (chr->uuid == pulseOxChar.uuid) {
    // Serial.print("Heart Rate CCCD updated: ");
  }
  else if (chr->uuid == batteryChar.uuid) {
    // Serial.print("Battery CCCD updated: ");
  }
  else if (chr->uuid == accelX1Char.uuid) {
    // Serial.print("Accel X1 CCCD updated: ");
  }
  else if (chr->uuid == accelX2Char.uuid) {
    // Serial.print("Accel X2 CCCD updated: ");
  }
  else if (chr->uuid == accelY1Char.uuid) {
    // Serial.print("Accel Y1 CCCD updated: ");
  }
  else if (chr->uuid == accelY2Char.uuid) {
    // Serial.print("Accel Y2 CCCD updated: ");
  }
  else if (chr->uuid == accelZ1Char.uuid) {
    // Serial.print("Accel Z1 CCCD updated: ");
  }
  else if (chr->uuid == accelZ2Char.uuid) {
    // Serial.print("Accel Z2 CCCD updated: ");
  }

  if (cccd_value & BLE_GATT_HVX_NOTIFICATION) {
    // Serial.println("Notifications ENABLED");
  } else {
    // Serial.println("Notifications DISABLED");
  }
}

//**************************************************************************************************************
// RTC initialization
void initRTC(unsigned long count30)
{
  NRF_CLOCK->TASKS_LFCLKSTOP = 1;
  NRF_CLOCK->LFCLKSRC = 1;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
  while(NRF_CLOCK->LFCLKSTAT != 0x10001);

  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->CC[0] = count30;
  NRF_RTC2->INTENSET = 0x10000;
  NRF_RTC2->EVTENCLR = 0x10000;
  NVIC_SetPriority(RTC2_IRQn,3);
  NVIC_EnableIRQ(RTC2_IRQn);
  NRF_RTC2->TASKS_START = 1;
}

//**************************************************************************************************************
// RTC interrupt handler
extern "C" void RTC2_IRQHandler(void)
{
  if ((NRF_RTC2->EVENTS_COMPARE[0] != 0) && ((NRF_RTC2->INTENSET & 0x10000) != 0)) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->TASKS_CLEAR = 1;
    intFlag = true;
  }
}

//**************************************************************************************************************
// Function to enable the biohub to receive accelerometer data
bool enableHostSideAccelerometer() {
  // Serial.println("Configuring biosensor for host side accelerometer");
  
  uint8_t response;

  // Set FIFO threshold: Send command 10 01 0F
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x10); // Command family
  Wire.write(0x01); // Command index
  Wire.write(0x0F); // Threshold
  response = Wire.endTransmission();
  
  if (response != 0) {
    // Serial.print("I2C transmission setting FIFO threshold: ");
    // Serial.println(response);
    return false;
  }
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    // Serial.print("Failed to enable input FIFO for host accelerometer , response: ");
    // Serial.println(response, HEX);
    return false;
  }

  // Enable MAX30101: Send command 44 03 01
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x44); // Command family
  Wire.write(0x03); // Command index
  Wire.write(0x01); // Enable
  response = Wire.endTransmission();
  
  if (response != 0) {
    // Serial.print("I2C transmission enablinb MAX30101 threshold: ");
    // Serial.println(response);
    return false;
  }
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    // Serial.print("Failed to enable MAX30101 , response: ");
    // Serial.println(response, HEX);
    return false;
  }

  // Enable input FIFO: Send command 44 04 01 01
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x44); // Command family
  Wire.write(0x04); // Command index
  Wire.write(0x01); // Enable
  Wire.write(0x01); // Parameter
  response = Wire.endTransmission();
  
  if (response != 0) {
    // Serial.print("I2C transmission error enabling host accelerometer FIFO: ");
    // Serial.println(response);
    return false;
  }
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    // Serial.print("Failed to enable input FIFO for host accelerometer , response: ");
    // Serial.println(response, HEX);
    return false;
  }
  
  // Enable MaximFast algorithm: Send command 0xAA 0x52 0x02 0x01
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x52); // Command family
  Wire.write(0x02); // Command index
  Wire.write(0x01); // Enable
  response = Wire.endTransmission();

  if (response != 0) {
    // Serial.print("I2C transmission error enabling MaximFast algorithm: ");
    // Serial.println(response);
    return false;
  }
  delay(45);

  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    // Serial.print("Failed to enable MaximFast algorithm mode , response: ");
    // Serial.println(response, HEX);
    return false;
  }
  
  return true;
}

//**************************************************************************************************************
// Function to send accelerometer data to biohub
uint8_t sendAccelerometerDataToBiohub() {
  // Send command header: AA 14 00
  Wire.beginTransmission(BIOSENSOR_ADDRESS); // 7-bit address
  Wire.write(0x14); // Command family
  Wire.write(0x00); // Command index
  
  // Send 20 samples (each sample is 6 bytes: 2 bytes each for X, Y, Z)
  for (size_t i = 0; i < 20; i++) {
    int16_t scaledX, scaledY, scaledZ;
    
    if (i < 10) {
      // Use first buffer for samples 0-9, scale by 61/1000
      scaledX = (accelBuffX1[i] * 61) / 1000;
      scaledY = (accelBuffY1[i] * 61) / 1000;
      scaledZ = (accelBuffZ1[i] * 61) / 1000;
    }
    else {
      // Use second buffer for samples 10-19, scale by 61/1000
      size_t idx = i - 10;
      scaledX = (accelBuffX2[idx] * 61) / 1000;
      scaledY = (accelBuffY2[idx] * 61) / 1000;
      scaledZ = (accelBuffZ2[idx] * 61) / 1000;
    }
    
    // Send X value (MSB first)
    Wire.write((scaledX >> 8) & 0xFF);
    Wire.write(scaledX & 0xFF);
    
    // Send Y value (MSB first)
    Wire.write((scaledY >> 8) & 0xFF);
    Wire.write(scaledY & 0xFF);
    
    // Send Z value (MSB first)
    Wire.write((scaledZ >> 8) & 0xFF);
    Wire.write(scaledZ & 0xFF);
  }
  
  uint8_t response = Wire.endTransmission();
  if (response != 0) {
    // Serial.print("I2C transmission error: ");
    // Serial.println(response);
  }
  return response;
}

// Function to check biohub status after having received accelerometer data
bool checkBiohubStatusAfterAccelerometer() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(1)); // Request 1 byte from biohub
  
  if (Wire.available() >= 1) {
    uint8_t response = Wire.read(); 
    
    if (response == 0x00) { // Should be 0x00
      return true;
    } 
    else {
      // Serial.print("Unexpected response from biohub: ");
      // Serial.println(response, HEX);
      return false;
    }
  } 
  else {
    // Serial.println("No response from biohub");
    return false;
  }
}

// Biosensor communication functions with error handling
void requestBiohubStatus() {
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    // Serial.print("I2C error requesting status: ");
    // Serial.println(error);
  }
}

uint8_t readBiohubStatus() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(2));
  if (Wire.available() >= 2) {
    uint8_t statusByte = Wire.read();
    uint8_t responseByte = Wire.read();
    return statusByte;
  }
  return 1;  // Error
}

void requestBiohubNumFifoSamples() {
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x12);
  Wire.write(0x00);
  Wire.endTransmission();
}

uint8_t readBiohubNumFifoSamples() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(2));
  if (Wire.available() >= 2) {
    uint8_t statusByte = Wire.read();
    uint8_t responseByte = Wire.read();
    return responseByte;
  }
  return 0;
}

void requestBiohubData() {
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();
}

uint8_t readBiohubData() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(7));
  if (Wire.available() >= 7) {
    uint8_t statusByte = Wire.read();
    for (size_t i = 0; i < 6; i++) {
      biohubFifoData[i] = Wire.read();
    }
    if (statusByte == 0) {
      formatBiohubData();
      return 0;  // Success
    }
  }
  return 1;  // Error
}

void formatBiohubData() {
    biohubData.heartRate = (uint16_t(biohubFifoData[0]) << 8);
    biohubData.heartRate |= (biohubFifoData[1]);
    biohubData.heartRate /= 10;

    biohubData.confidence = biohubFifoData[2];

    biohubData.oxygen = uint16_t(biohubFifoData[3]) << 8;
    biohubData.oxygen |= biohubFifoData[4];
    biohubData.oxygen /= 10;

    biohubData.status = biohubFifoData[5];
}

//**************************************************************************************************************
// Bluetooth callback functions
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  // Serial.print("âœ… Connected to ");
  // Serial.println(central_name);
  // Serial.println("All sensors now transmitting data:");
  // Serial.println("  â€¢ Heart Rate");
  // Serial.println("  â€¢ SpO2");
  // Serial.println("  â€¢ Battery Level");
  // Serial.println("  â€¢ Accelerometer");

  connectedFlag = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  // Serial.print("âŒ Disconnected, reason = 0x");
  // Serial.println(reason, HEX);
  // Serial.println("Advertising restarted - ready for new connection");

  connectedFlag = false;
  biosensorErrorCount = 0;  // Reset error counter on disconnect
}

//**************************************************************************************************************
// Time sync callback - called when phone writes timestamp
void timesync_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  if (len == 8) {
    // Read 8-byte timestamp (little-endian format)
    baseTimestampMs = 0;
    for (int i = 0; i < 8; i++) {
      baseTimestampMs |= ((uint64_t)data[i]) << (i * 8);
    }
    baseMillis = millis();

    // Serial.print("âœ… Time synced! Unix timestamp: ");
    // Serial.print((uint32_t)(baseTimestampMs / 1000));
    // Serial.println(" seconds");
  } else {
    // Serial.print("âš ï¸ Invalid time sync data length: ");
    // Serial.println(len);
  }
}

//**************************************************************************************************************
// Get current timestamp in milliseconds (handles millis() overflow)
uint64_t getCurrentTimestamp() {
  if (baseTimestampMs == 0) {
    // Not synced yet - return millis as fallback
    return (uint64_t)millis();
  }

  // Handle millis() overflow safely
  uint32_t currentMillis = millis();
  uint32_t elapsed;

  if (currentMillis >= baseMillis) {
    // Normal case: no overflow
    elapsed = currentMillis - baseMillis;
  } else {
    // Overflow occurred
    elapsed = (0xFFFFFFFF - baseMillis) + currentMillis + 1;
  }

  return baseTimestampMs + elapsed;
}

//**************************************************************************************************************
// BLE DATA SENDING FUNCTIONS - All sensors working

void sendHrmBLE() {
  // Send heart rate with contact detection flag
  uint8_t hrm_flags = (biohubData.status == 3) ? 0b00000110 : 0b00000000;
  uint8_t hrm_val = (uint8_t) biohubData.heartRate;

  uint8_t hrmData[2] = { hrm_flags, hrm_val };
  hrmMeasurement.notify(hrmData, 2);

  // Serial.print("ðŸ’“ HR: ");
  // Serial.print(hrm_val);
  // Serial.print(" BPM (Contact: ");
  // Serial.print((biohubData.status == 3) ? "YES" : "NO");
  // Serial.println(")");
}

void sendPulseOxBLE() {
  // Send SpO2 level
  uint8_t pulseOx_flags = 0x00;
  uint8_t pulseOx_val = (uint8_t) biohubData.oxygen;

  uint8_t pulseOx_data[2] = { pulseOx_flags, pulseOx_val };
  pulseOxChar.notify(pulseOx_data, 2);

  // Serial.print("ðŸ©¸ SpO2: ");
  // Serial.print(pulseOx_val);
  // Serial.println("%");
}

void sendBatteryBLE() {
  // Calculate battery level
  #ifdef VBAT_PIN
    float vbat = analogRead(VBAT_PIN) * 3.3 / 1024.0 * VBAT_DIVIDER;
    int batteryLevel = (int)((vbat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0);
    batteryLevel = constrain(batteryLevel, 0, 100);
  #else
    int batteryLevel = 93;  // Default if no battery pin
  #endif

  uint8_t batteryData[1] = { (uint8_t)batteryLevel };
  batteryChar.notify(batteryData, 1);

  // Serial.print("ðŸ”‹ Battery: ");
  // Serial.print(batteryLevel);
  // Serial.println("%");
}

void sendAccelerometerBLE() {
  // Get current timestamp
  uint64_t timestamp = getCurrentTimestamp();
  // Pack into array
  uint8_t timestampData[8];
  for (int i = 0; i < 8; i++) {
    timestampData[i] = (timestamp >> (i * 8)) & 0xFF;
  }
  accelTimestampChar.notify(timestampData, 8);

  // Send accelerometer data
  accelX1Char.notify(accelBuffX1, 20);
  accelX2Char.notify(accelBuffX2, 20);
  accelY1Char.notify(accelBuffY1, 20);
  accelY2Char.notify(accelBuffY2, 20);
  accelZ1Char.notify(accelBuffZ1, 20);
  accelZ2Char.notify(accelBuffZ2, 20);

  // Print reading from buffer for debugging
  int16_t x = accelBuffX2[intCounter - 1];
  int16_t y = accelBuffY2[intCounter - 1];
  int16_t z = accelBuffZ2[intCounter - 1];

  // Convert to g units for display (assuming Â±2g range)
  float xG = (float)x / 16384.0;
  float yG = (float)y / 16384.0;
  float zG = (float)z / 16384.0;
  float magnitude = sqrt(xG*xG + yG*yG + zG*zG);

  // Serial.print("ðŸ“Š Accel: X=");
  // Serial.print(xG, 2);
  // Serial.print("g Y=");
  // Serial.print(yG, 2);
  // Serial.print("g Z=");
  // Serial.print(zG, 2);
  // Serial.print("g Mag=");
  // Serial.print(magnitude, 2);
  // Serial.println("g");
}