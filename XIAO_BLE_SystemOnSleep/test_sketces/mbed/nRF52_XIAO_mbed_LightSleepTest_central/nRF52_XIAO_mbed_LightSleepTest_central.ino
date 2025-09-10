//----------------------------------------------------------------------------------------------
// Board Library : Seeed nRF52 mbed-enable Borads 2.9.1
// Board Select  : Seeed nRF52 mbed-enable Borads / Seeed XIAO BLE - nRF52840
//----------------------------------------------------------------------------------------------

// 2023/05/21
// Peripheral sketch :  nRF52_XIAO_mbed_LightSleepTest_peripheral.ino

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSerif9pt7b.h"
#include <ArduinoBLE.h>

#define LOOP_TIME    1            // Loop time [sec]

#define dataNum 8                 // mS data 4 byte, uS data 4 byte (2 data 8 byte)
union unionData {                 // Union for data type conversion
  uint32_t  dataBuff32[dataNum/4];
  uint16_t  dataBuff16[dataNum/2];
  uint8_t   dataBuff8[dataNum];
};
union unionData ud;

const char* versionNumber = "0.99";  // Central sketch version number
uint32_t mS, uS;
int err;                             // scan_connect() error code

// Instances of the class
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Custum Service and Characteristic
#define myUUID(val) ("526539fb-" val "-4889-b224-ace499220efa") //UUID派生のためのマクロ
BLEService        secondService(myUUID("0000"));
BLECharacteristic dataCharacteristic(myUUID("0010"), BLEWrite | BLERead, dataNum);

// Instances of BLEDevice
BLEDevice peripheral;

void setup()
{
  // Serial port initialization
  Serial.begin(115200);
//  while (!Serial);
  delay(2000);

  // LED intialization
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  // Initialization of display SSD1306
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))       //表示用電圧発生
  {
    Serial.println("Initialization failure of SSD1306");
  }
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 15);
  display.println("Light Sleep");
  display.setCursor(0, 31);
  display.println("BLE Central");
  display.setCursor(0, 47);
  display.print("Ver   : ");
  display.println(versionNumber);
  display.display();
  delay(5000);
  display.clearDisplay();

  // Initialization of BLE
  BLE.begin();

  Serial.println("End of initialization");

}

//******************************************************************************************
void loop()
{
  // Scanning and connecting Peripheral
  err = scan_connect();
    Serial.print("ERROR ");
    Serial.println(err);

  // if connected
  if(err == 0)
  {
    while (peripheral.connected())
    {
      digitalWrite(LED_GREEN, LOW);
      uint32_t timestamp = millis();
 
      // Restore and display incoming data     
      mS = ud.dataBuff32[0];
      uS = ud.dataBuff32[1];
  
      Serial.print("【loop】 ");
      Serial.print(mS);
      Serial.print(" ");
      Serial.println(uS);  

      display.clearDisplay();
      display.setCursor(0, 15);
      display.print("mS "); display.print(mS);
      display.setCursor(0, 31);
      display.print("uS "); display.print(uS);
      display.display();

      digitalWrite(LED_GREEN, HIGH);
      
      while(millis() - timestamp < 1000 * LOOP_TIME);
    }    
  }  
}


//**************************************************************************************************
// Scan for peripheral and connect when found
int scan_connect(void)
{
  BLE.scanForUuid(myUUID("0000"));
  Serial.println("1.SCANNING ................");

  peripheral = BLE.available();
  if (!peripheral)
  {
    Serial.println("2x.Peripheral unabailable");
    return 2;
  }
    
  if (peripheral.localName() != "XIAO_BLE_peripheral")
  {
    Serial.println("3x.Peripheral local name miss match");
    return 3;
  }

  BLE.stopScan();
  Serial.println("4.Stop scanning");
  
  Serial.println("5.CONNECTING ................");
  if (!peripheral.connect()) 
  {
    Serial.println("5x.Can't connect");
    return 5;
  }

  if (!peripheral.discoverAttributes()) 
  {
    Serial.println("6x.Didn't discover attributes");
    peripheral.disconnect();
    return 6;
  }

  dataCharacteristic = peripheral.characteristic(myUUID("0010"));
  dataCharacteristic.setEventHandler(BLEWritten, characteristicWritten);  
  
  if (!dataCharacteristic.canSubscribe())
  {
    Serial.println("8x.Can't subscribe");
    peripheral.disconnect();
    return 8;
  }

  if (!dataCharacteristic.subscribe())
  {
    Serial.println("9x.Can't Subscribe");
    peripheral.disconnect();
    return 9;
  }

  Serial.println("10.Success scanning and connecting");
  return 0;
}

//****************************************************************************************************
// Characteristic written event handler
void characteristicWritten(BLEDevice peripheral, BLECharacteristic thisChar) 
{
    digitalWrite(LED_RED, LOW);

    dataCharacteristic.readValue(ud.dataBuff8, dataNum);

    delayMicroseconds(10000); // for LED
    
    digitalWrite(LED_RED, HIGH);
}
