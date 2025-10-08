#include "Wire.h"
#include "LSM6DS3.h"
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

//*******************************************************************************************************************
// ISR
volatile bool intFlag = false;
#define IMU_INT_PIN 1;

//*******************************************************************************************************************
#define IMU_Address 0x6A
LSM6DS3 myIMU(I2C_MODE, IMU_Address); // Accelerometer
#define IMU_BUFFER_SIZE  20
#define IMU_SAMPLING_RATE 100
#define int2Pin PIN_LSM6DS3TR_C_INT1

//*******************************************************************************************************************
// Sparkfun sensor board

// Reset pin, MFIO pin
#define resPin 7
#define mfioPin 8

#define BIOSENSOR_WIDTH 411 // Possible widths: 69, 118, 215, 411us
#define BIOSENSOR_SAMPLES 100 // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second

int past_heartrate = 0;

SparkFun_Bio_Sensor_Hub bioHub; 
bioData body; 

unsigned long previous_time = 0;
uint8_t dataBuff[120];
uint8_t dataBuff2[120];
//*******************************************************************************************************************
void setup() {

  // Serial port initialization
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial initialized");

  // I2C initialization
  Wire.begin();
  Serial.println("I2C initialized");

  // Interrupt pin configuration
  pinMode(int2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(int2Pin), imuInterruptHandler, RISING);

  //*************************************************************************************
  // Configure acclerometer 
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;        // ±2g
  myIMU.settings.accelSampleRate = 100;   // 104 Hz

  // Disable other sensors
  myIMU.settings.gyroEnabled = 0;       // Disable gyro
  myIMU.settings.tempEnabled = 0;       // Disable temperature

  //Non-basic mode settings
  //myIMU.settings.commMode = 1;

  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("Failed to initalize IMU");
    while (1);
  }
  Serial.println("IMU initialized!");

  // Configure FIFO
  myIMU.settings.accelFifoEnabled = 1;     //Set to include accelerometer in the FIFO
  myIMU.settings.gyroFifoEnabled = 0;
  myIMU.settings.timestampFifoEnabled = 0;
  myIMU.settings.fifoThreshold = 240;

  // Initialize FIFO
  myIMU.fifoBegin();

  // Manually set mode and sampling rate
  uint8_t tempFIFO_CTRL5 = 0;
  tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
  tempFIFO_CTRL5 |= 1;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

  // Enable FIFO threshold interrupt
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x08); 

  // Limit FIFO size to threshold
  uint8_t tempCTRL4_C = 0;
  myIMU.readRegister(&tempCTRL4_C, LSM6DS3_ACC_GYRO_CTRL4_C);
  tempCTRL4_C |= 1;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, tempCTRL4_C);

  myIMU.fifoClear();

  Serial.println("FIFO initialized!");

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

  // Set pulse width.
  error = bioHub.setPulseWidth(BIOSENSOR_WIDTH);
  if (error != 0) {
    Serial.print("Could not set biosensor pulse width! Error: ");
    Serial.println(error);  
  }
  Serial.print("Biosensor pulse width set to ");
  Serial.println(bioHub.readPulseWidth());

  // Set sample rate per second. Remember that not every sample rate is
  // available with every pulse width. Check hookup guide for more information.  
  error = bioHub.setSampleRate(BIOSENSOR_SAMPLES);
  if (error != 0) {// Zero errors.
    Serial.print("Could not set biosensor sample rate! Error: ");
    Serial.println(error); 
  }
  Serial.print("Biosensor sample rate set to ");
  Serial.println(bioHub.readSampleRate()); 

  // Enable host side accelerometer
  if (!enableHostSideAccelerometer()) {
    Serial.println("Failed to configure biosensor for host side accelerometer");
  }
  else {
    Serial.println("Biosensor configured for host side accelerometer");
  }

  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  //Serial.println("Loading up the buffer with data....");
  //delay(4000); 

  Serial.println("Setup finished!");
}

void loop() {
  // Sleep
  __WFI();
  __SEV();
  __WFI();

  if (intFlag) {
    intFlag = false;
    Serial.println(millis() - previous_time);
    previous_time = millis();

    /*uint8_t i = 0;

    Wire.beginTransmission(IMU_Address);
    Wire.write(LSM6DS3_ACC_GYRO_OUTX_L_XL);
    if (Wire.endTransmission() != 0) {
        Serial.println("Error");
    } 
    else {
      Wire.requestFrom(IMU_Address, 20);
      while ((Wire.available()) && (i < 20)) { // slave may send less than requested
          dataBuff[i] = Wire.read();
          i++;
      }
    } 
    Serial.println(i);*/

    status_t returnError;
    returnError = myIMU.readRegisterRegion(dataBuff, LSM6DS3_ACC_GYRO_OUTX_L_XL, 120);
    //returnError = myIMU.readRegisterRegion(dataBuff2, LSM6DS3_ACC_GYRO_OUTX_L_XL, 120);
    //returnError = myIMU.readRegisterRegion(dataBuff + 240, LSM6DS3_ACC_GYRO_OUTX_L_XL, 120);
    //returnError = myIMU.readRegisterRegion(dataBuff + 360, LSM6DS3_ACC_GYRO_OUTX_L_XL, 120);
    Serial.println(returnError == IMU_SUCCESS);

    // Reset FIFO 
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x00);
    uint8_t tempFIFO_CTRL5 = 0;
    tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
    tempFIFO_CTRL5 |= 1;
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);
    
    // Read sensor hub
    /*body = bioHub.readBpm();
    Serial.print("Heartrate: ");
    Serial.println(body.heartRate); 
    Serial.print("Confidence: ");
    Serial.println(body.confidence); 
    Serial.print("Oxygen: ");  
    Serial.println(body.oxygen); 
    Serial.print("Status: ");
    Serial.println(body.status);  */
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
void imuInterruptHandler() {
  intFlag = true;
}

