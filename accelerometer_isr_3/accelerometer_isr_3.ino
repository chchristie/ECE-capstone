#include <Arduino.h>
#include "LSM6DS3.h"
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>
#include <bluefruit.h>

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

uint16_t accelBuffX[20];
uint16_t accelBuffY[20];
uint16_t accelBuffZ[20];

//*************x******************************************************************************************************
// Sparkfun sensor board
#define BIOSENSOR_ADDRESS 0x55

// Reset pin, MFIO pin
int resPin = 7;
int mfioPin = 8;

#define BIOSENSOR_WIDTH 411 // Possible widths: 69, 118, 215, 411us
#define BIOSENSOR_SAMPLES 100 // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second

int past_heartrate = 0;

SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 
bioData biohubData; 
uint8_t biohubFifoData[6];

//*******************************************************************************************************************
// Bluetooth
BLEDis bledis; // Device information service
BLEBas blebas; // Battery service

// Built-in heart rate monitor service
BLEService hrmService = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrmMeasurement = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLECharacteristic bodySensorLocation = BLECharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);

// Built-in pulse oximeter service 
BLEService pulseOxService = BLEService(0x1822);
BLECharacteristic pulseOxChar = BLECharacteristic(0x2A5E);

// Custom accelerometer service 
#define accelService_UUID(val) (const uint8_t[]) { \
    0x85, 0xB4, 0x93, 0x81, 0x58, 0x40, 0x0C, 0xBE, \
    0xD1, 0x4D, (uint8_t)(val & 0xff), (uint8_t)(val >> 8), 0x00, 0x00, 0xC4, 0x52 }

BLEService accelService = BLEService(accelService_UUID(0x0000));
BLECharacteristic accelXChar = BLECharacteristic(accelService_UUID(0x0030));
BLECharacteristic accelYChar = BLECharacteristic(accelService_UUID(0x0060));
BLECharacteristic accelZChar = BLECharacteristic(accelService_UUID(0x0090));

bool connectedFlag = false;

//*******************************************************************************************************************
void setup() {

  // Serial port initialization
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial initialized");

  // I2C initialization
  Wire.begin();
  Serial.println("I2C initialized");

  // RTC initialization  
  initRTC(32768 * SLEEP_TIME / 1000);  
  Serial.println("RTC initialized"); 

  //*************************************************************************************
  // Configure acclerometer 
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;        // ±2g
  myIMU.settings.accelSampleRate = 100;   // 104 Hz

  // Disable other sensors
  myIMU.settings.gyroEnabled = 0;       // Disable gyro
  myIMU.settings.tempEnabled = 0;       // Disable temperature

  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("Failed to initalize IMU");
    while (1);
  }
  Serial.println("IMU initialized!");


  //************************************************************************************************************************************************************************************
  // Biosensor setup
  if (bioHub.begin(Wire, resPin, mfioPin) == 0) // Zero errors!
    Serial.println("Biosensor started!");
  else
    Serial.println("Could not communicate with the biosensor!");
 
  Serial.println("Configuring Biosensor...."); 
  int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings. 
  if(error == 0){ // Zero errors!
    Serial.println("Biosensor configured.");
  }
  else {
    Serial.print("Error configuring sensor: ");
    Serial.println(error); 
  }

  // Enable host side accelerometer
  if (!enableHostSideAccelerometer()) {
    Serial.println("Failed to configure biosensor for host side accelerometer");
  }
  else {
    Serial.println("Biosensor configured for host side accelerometer");
  }

  // Set sample rate per second. Remember that not every sample rate is
  // available with every pulse width. Check hookup guide for more information.  
  error = bioHub.setSampleRate(BIOSENSOR_SAMPLES);
  if (error != 0) {// Zero errors.
    Serial.print("Could not set biosensor sample rate! Error: ");
    Serial.println(error); 
  }
  Serial.print("Biosensor sample rate set to ");
  Serial.println(bioHub.readSampleRate()); 

  // Set pulse width.
  error = bioHub.setPulseWidth(BIOSENSOR_WIDTH);
  if (error != 0) {
    Serial.print("Could not set biosensor pulse width! Error: ");
    Serial.println(error);  
  }
  Serial.print("Biosensor pulse width set to ");
  Serial.println(bioHub.readPulseWidth());

  //************************************************************************************************************************************************************************************
  // Bluetooth

  // Bluefruit class
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configUuid128Count(15);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);        
  Bluefruit.setConnLedInterval(50);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Device information service
  bledis.setManufacturer("Seeed_Studio");
  bledis.setModel("XIAO_nRF52840");
  bledis.begin(); 

  // Battery service
  blebas.begin();

  // Heart rate monitoring service
  hrmService.begin();

  hrmMeasurement.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  hrmMeasurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrmMeasurement.setFixedLen(2);
  hrmMeasurement.begin();

  bodySensorLocation.setProperties(CHR_PROPS_READ);
  bodySensorLocation.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  bodySensorLocation.setFixedLen(1);
  bodySensorLocation.begin();
  bodySensorLocation.write8(0); // Other

  // Pulse oximeter service
  pulseOxService.begin();

  pulseOxChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  pulseOxChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pulseOxChar.setFixedLen(2);
  pulseOxChar.begin();

  // Accelerometer data service
  accelService.begin();

  accelXChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelXChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelXChar.setFixedLen(40);  // 20 samples of 2 bytes
  accelXChar.begin();

  accelYChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelYChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelYChar.setFixedLen(40);  // 20 samples of 2 bytes
  accelYChar.begin();

  accelZChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelZChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelZChar.setFixedLen(40);  // 20 samples of 2 bytes
  accelZChar.begin();

  // Advertisement Settings
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(hrmService);
  Bluefruit.Advertising.addService(pulseOxService);
  Bluefruit.Advertising.addService(blebas);
  Bluefruit.Advertising.addService(accelService);
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setIntervalMS(32, 244);     // fast mode 32 ms, slow mode 244 ms
  Bluefruit.Advertising.setFastTimeout(30);         // fast mode 30 sec
  Bluefruit.Advertising.start(0);                   // 0 = Don't stop advertising after n seconds

  // Print address
  Serial.print("Advertising at device address ");
  ble_gap_addr_t addr;
  addr = Bluefruit.getAddr();

  for (int i = 5; i >= 0; i--) {
    if (addr.addr[i] < 16) Serial.print("0");
    Serial.print(addr.addr[i], HEX);
    if (i > 0) Serial.print(":");
  }
  Serial.println("");

  //************************************************************************************************************************************************************************************
  Serial.println("Loading up the biohub buffer with data....");
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
    intCounter += 1;

    accelBuffX[intCounter - 1] = myIMU.readRawAccelX() * 61 / 1000;
    accelBuffY[intCounter - 1] = myIMU.readRawAccelY() * 61 / 1000;
    accelBuffZ[intCounter - 1] = myIMU.readRawAccelZ() * 61 / 1000;

    uint8_t biohubStatus = 0;
    switch (intCounter) {
      case 2:
        checkBiohubStatusAfterAccelerometer();
        break;
      case 3: 
        requestBiohubStatus();
        break;
      case 4:
        biohubStatus = readBiohubStatus();
        if (biohubStatus == 1) {
          Serial.println("Biohub status error in 4");
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
          Serial.println("Biohub status error in 8");
        }
        Serial.print("Heartrate: ");
        Serial.println(biohubData.heartRate); 
        Serial.print("Confidence: ");
        Serial.println(biohubData.confidence); 
        Serial.print("Oxygen: ");  
        Serial.println(biohubData.oxygen); 
        Serial.print("Status: ");
        Serial.println(biohubData.status); 
        break;
      case 10:
        if (Bluefruit.connected()) {
          sendHrmBLE();
          sendPulseOxBLE();
          sendBatteryBLE();
          sendAccelXBLE();
          sendAccelYBLE();
          sendAccelZBLE();
        }
        break;
      case 20:
        intCounter = 0;
        sendAccelerometerDataToBiohub();
        break;
    }
    
    if (intFlag == true) {
      Serial.print("Took too long: "); Serial.println(intCounter);
    }
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
  Serial.println("Configuring biosensor for host side accelerometer");
  
  uint8_t response;

  // Set FIFO threshold: Send command 10 01 0F
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x10); // Command family
  Wire.write(0x01); // Command index
  Wire.write(0x0F); // Threshold
  response = Wire.endTransmission();
  
  if (response != 0) {
    Serial.print("I2C transmission setting FIFO threshold: ");
    Serial.println(response);
    return false;
  }
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    Serial.print("Failed to enable input FIFO for host accelerometer , response: ");
    Serial.println(response, HEX);
    return false;
  }

  // Enable MAX30101: Send command 44 03 01
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x44); // Command family
  Wire.write(0x03); // Command index
  Wire.write(0x01); // Enable
  response = Wire.endTransmission();
  
  if (response != 0) {
    Serial.print("I2C transmission enablinb MAX30101 threshold: ");
    Serial.println(response);
    return false;
  }
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    Serial.print("Failed to enable MAX30101 , response: ");
    Serial.println(response, HEX);
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
    Serial.print("I2C transmission error enabling host accelerometer FIFO: ");
    Serial.println(response);
    return false;
  }
  delay(45);
  
  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    Serial.print("Failed to enable input FIFO for host accelerometer , response: ");
    Serial.println(response, HEX);
    return false;
  }
  
  // Enable MaximFast algorithm: Send command 0xAA 0x52 0x02 0x01
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x52); // Command family
  Wire.write(0x02); // Command index
  Wire.write(0x01); // Enable
  response = Wire.endTransmission();

  if (response != 0) {
    Serial.print("I2C transmission error enabling MaximFast algorithm: ");
    Serial.println(response);
    return false;
  }
  delay(45);

  // Read response: should be 0x00 for success
  Wire.requestFrom(BIOSENSOR_ADDRESS, 1);
  response = Wire.read();
  if (response != 0) {
    Serial.print("Failed to enable MaximFast algorithm mode , response: ");
    Serial.println(response, HEX);
    return false;
  }
  
  return true;
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

//**************************************************************************************************************
// Function to send accelerometer data to biohub
uint8_t sendAccelerometerDataToBiohub() {
  // Send command header: AA 14 00
  Wire.beginTransmission(BIOSENSOR_ADDRESS); // 7-bit address
  Wire.write(0x14); // Command family
  Wire.write(0x00); // Command index
  
  // Send 20 samples (each sample is 6 bytes: 2 bytes each for X, Y, Z)
  for (size_t i = 0; i < 20; i++) {
    // Send X value (MSB first)
    Wire.write((accelBuffX[i] >> 8) & 0xFF);
    Wire.write(accelBuffX[i] & 0xFF);
    
    // Send Y value (MSB first)
    Wire.write((accelBuffY[i] >> 8) & 0xFF);
    Wire.write(accelBuffY[i] & 0xFF);
    
    // Send Z value (MSB first)
    Wire.write((accelBuffZ[i] >> 8) & 0xFF);
    Wire.write(accelBuffZ[i] & 0xFF);
  }
  
  uint8_t response = Wire.endTransmission();
  if (response != 0) {
    Serial.print("I2C transmission error: ");
    Serial.println(response);
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
      Serial.print("Unexpected response from biohub: ");
      Serial.println(response, HEX);
      return false;
    }
  } 
  else {
    Serial.println("No response from biohub");
    return false;
  }
}

// Function to prep biohub for reading status register
void requestBiohubStatus() {
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

// Function to read biohub status
uint8_t readBiohubStatus() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(2));
  uint8_t statusByte = Wire.read();
  if (statusByte) return statusByte;
  uint8_t responseByte = Wire.read();
  return responseByte;
}

void requestBiohubNumFifoSamples() {
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x12);
  Wire.write(0x00);
  Wire.endTransmission();
}

uint8_t readBiohubNumFifoSamples() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(2));
  uint8_t statusByte = Wire.read();
  if (statusByte) return statusByte;
  uint8_t responseByte = Wire.read();
  return responseByte;
}

// Function to prep biohub for reading data FIFO
void requestBiohubData() {
  Wire.beginTransmission(BIOSENSOR_ADDRESS);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();
}

// Function to read biohub data
uint8_t readBiohubData() {
  Wire.requestFrom(BIOSENSOR_ADDRESS, static_cast<uint8_t>(7));
  uint8_t statusByte = Wire.read();
  for (size_t i = 0; i < 6; i++) {
    biohubFifoData[i] = statusByte ? 0 : Wire.read();
  }
  formatBiohubData();
  return statusByte;
}

// Format returned biohub data from FIFO
void formatBiohubData() {
    // Heart Rate formatting
    biohubData.heartRate = (uint16_t(biohubFifoData[0]) << 8);
    biohubData.heartRate |= (biohubFifoData[1]);
    biohubData.heartRate /= 10;

    // Confidence formatting
    biohubData.confidence = biohubFifoData[2];

    // Blood oxygen level formatting
    biohubData.oxygen = uint16_t(biohubFifoData[3]) << 8;
    biohubData.oxygen |= biohubFifoData[4];
    biohubData.oxygen /= 10;

    //"Machine State" - has a finger been detected?
    biohubData.status = biohubFifoData[5];
}

//**************************************************************************************************************
// Bluetooth callback functions
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("【connect_callback】 Connected to ");
  Serial.println(central_name);

  connectedFlag = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("【disconnect_callback】 Disconnected, reason = 0x");
  Serial.println(reason, HEX);

  connectedFlag = false;
}

//**************************************************************************************************************

void sendHrmBLE() {
  uint8_t hrm_flags = (biohubData.status == 3) ? 0b00000100 : 0b00000010;   // bits 1-2 of flag indicate skin contact. 01: contact not detected. 10: contact detected
  uint8_t hrm_val = (uint8_t) biohubData.heartRate;
  uint8_t hrmData[2] = { hrm_flags, hrm_val };
  hrmMeasurement.notify(hrmData, 2);
}

void sendPulseOxBLE() {
  uint8_t pulseOx_flags = 0x00;
  uint8_t pulseOx_val = (uint8_t) biohubData.oxygen;
  uint8_t pulseOx_data[2] = { pulseOx_flags, pulseOx_val };
  pulseOxChar.notify(pulseOx_data, 2);
}

void sendBatteryBLE() {
  blebas.notify(93);
}

void sendAccelXBLE() {
  accelXChar.notify(accelBuffX, 40);
}

void sendAccelYBLE() {
  accelYChar.notify(accelBuffY, 40);
}

void sendAccelZBLE() {
  accelZChar.notify(accelBuffZ, 40);
}



