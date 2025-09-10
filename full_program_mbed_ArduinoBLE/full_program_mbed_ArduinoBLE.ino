#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

// ---- Define pins and constants

// ---- Define BLE Services ----
// Heart rate monitor service
BLEService heartRateService("180D");  // Built-in heart rate service
BLECharacteristic heartRateMeasurementCharacteristic("2A37", BLERead | BLENotify, 2);
BLEByteCharacteristic bodyLocationCharacteristic("2A38", BLERead);

// Pulse oximeter service
BLEService pulseOxService("1822");    // Built-in pulse oximeter service
BLECharacteristic spo2MeasurementCharacteristic("2A5E", BLERead | BLENotify, 2);

// Step count service
BLEService stepService("230af0e3-9c7a-4be2-81cc-dcbed3383dd5");       // Custom service for step count
BLEUnsignedShortCharacteristic stepCountCharacteristic("7f2319c9-3920-4091-ba22-6f447001a64e", BLERead | BLENotify);

// Battery service
BLEService batteryService("180F");    // Built-in battery service
BLEByteCharacteristic batteryLevelCharacteristic("2A19", BLERead | BLENotify);


// ---- Define Accelerometer ----
LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("XIAO_HealthMonitor");
  BLE.setDeviceName("XIAO nRF52840 Sense");
  BLE.setAdvertisedService(heartRateService);  // Main advertised service

  // Set connect/disconnect callbacks
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // ---- Heart Rate Service ----
  heartRateService.addCharacteristic(heartRateMeasurementCharacteristic);
  heartRateService.addCharacteristic(bodyLocationCharacteristic);
  BLE.addService(heartRateService);
  bodyLocationCharacteristic.writeValue((byte)0);  // 0 = other

  // ---- Pulse Oximeter ----
  pulseOxService.addCharacteristic(spo2MeasurementCharacteristic);
  BLE.addService(pulseOxService);

  // ---- Step Counter ----
  stepService.addCharacteristic(stepCountCharacteristic);
  BLE.addService(stepService);

  // ---- Battery Service ----
  batteryService.addCharacteristic(batteryLevelCharacteristic);
  BLE.addService(batteryService);
  batteryLevelCharacteristic.writeValue((byte)100); // Start at 100% battery


  // Start advertising
  BLE.advertise();
  Serial.println("Advertising");
}

void loop() {
  BLE.poll();

  // Simulated values
  static uint16_t steps = 1000;
  static byte heartRate = 75;
  static byte spo2 = 97;
  static byte battery = 95;

  // === Update Heart Rate ===
  byte hrmPacket[] = {0x00, heartRate};  // flags = 0x00, uint8 bpm
  heartRateMeasurementCharacteristic.writeValue(hrmPacket, 2);

  // === Update SpO2 ===
  byte spo2Packet[] = {0x00, spo2};  // flags = 0x00
  spo2MeasurementCharacteristic.writeValue(spo2Packet, 2);

  // === Update Step Count ===
  stepCountCharacteristic.writeValue(steps);

  // === Update Battery ===
  batteryLevelCharacteristic.writeValue(battery);

  // Simulate changing values
  heartRate++;
  steps += 5;
  battery = max(battery - 1, 75);

  delay(2000);
}

// === BLE Connection Events ===
void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("Connected to central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, HIGH);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, LOW);
}