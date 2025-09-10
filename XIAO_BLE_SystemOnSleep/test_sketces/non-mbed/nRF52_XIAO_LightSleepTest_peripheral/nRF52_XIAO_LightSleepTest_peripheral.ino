//----------------------------------------------------------------------------------------------
// BSP : Seeed nRF52 Borads 1.1.1
// Board : Seeed nRF52 Borads / Seeed XIAO nRF52840 Sense
// important : need for Serial.print() <Arduino.h> <Adafruit_TinyUSB.h>
// \Arduino15\packages\Seeeduino\hardware\nrf52\1.1.1\libraries\Bluefruit52Lib\src
//----------------------------------------------------------------------------------------------

// 2023/05/21
// Central sketch : nRF52_XIAO_LightSleepTEST_central.ino
// Important : NVIC_SetPriority(RTC2_IRQn,3);

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // For Serial.print()
#include <bluefruit.h>        // \Arduino15\packages\Seeeduino\hardware\nrf52\1.1.1\libraries\Bluefruit52Lib\src

#define SLEEP_TIME  10        // Sleep time [sec]

#define dataNum 8             // mS data 4 byte, uS data 4 byte (2 data 8 byte)
union unionData {             // Union for data type conversion
  uint32_t  dataBuff32[dataNum/4];
  uint16_t  dataBuff16[dataNum/2];
  uint8_t   dataBuff8[dataNum];
};
union unionData ud;
uint32_t mS, uS;
bool intFlag = true;          // Interrupu flag
bool subscribedFlag = false;  // Subscribed flag
bool connectedFlag = false;   // Connected flag
int16_t conn;                 // Connect handler

// Custum Service and Characteristic
// 52c40000-8682-4dd1-be0c-40588193b485 for example
#define secondService_UUID(val) (const uint8_t[]) { \
    0x85, 0xB4, 0x93, 0x81, 0x58, 0x40, 0x0C, 0xBE, \
    0xD1, 0x4D, (uint8_t)(val & 0xff), (uint8_t)(val >> 8), 0x00, 0x00, 0xC4, 0x52 }

BLEService        secondService        (secondService_UUID(0x0000));
BLECharacteristic dataCharacteristic   (secondService_UUID(0x0030));

void setup()
{
  // Serial port initialization
  Serial.begin(115200);
//  while (!Serial);
  delay(2000);
  
  // LED initialization   
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT); 
   
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  // RTC initialization  
  initRTC(32768 * SLEEP_TIME);    // SLEEP_MTIME [sec]  

  // Initialization of Bruefruit class
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configUuid128Count(15);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);        // Check bluefruit.h for supported values
  Bluefruit.setConnLedInterval(50);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Custom Service Settings
  secondService.begin();

  dataCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  dataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dataCharacteristic.setFixedLen(dataNum);
  dataCharacteristic.begin();

  // Advertisement Settings
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(secondService);
  Bluefruit.ScanResponse.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setIntervalMS(20, 153);     // fast mode 20mS, slow mode 153mS
  Bluefruit.Advertising.setFastTimeout(30);         // fast mode 30 sec
  Bluefruit.Advertising.start(0);                   // 0 = Don't stop advertising after n seconds

  Serial.println("End of initialization");
}

//*************************************************************************************************************
void loop()
{
  if(intFlag == true)     // If wake up interrupt
  {       
    Serial.println("Connecting to Central ....");
    while(!Bluefruit.connected()) {
    }
    Serial.println("Connected to Central");  
    
    if (Bluefruit.connected())      // If connected to the central
    {
      digitalWrite(LED_GREEN, LOW);

      mS = millis();
      uS = micros();
      ud.dataBuff32[0] = mS;
      ud.dataBuff32[1] = uS;

      Serial.print("【LOOP】 ");
      Serial.print(mS);
      Serial.print(" ");
      Serial.println(uS);

      dataCharacteristic.notify(ud.dataBuff8, dataNum);   // Data transfer
      delayMicroseconds(100000);      // Delay 100mS

      digitalWrite(LED_GREEN, HIGH);
    }    
    intFlag = false;
    connectedFlag = false;        
  }
  // Sleep
  __WFI();
  __SEV();
  __WFI(); 
}

//**************************************************************************************************************
// callback function
void connect_callback(uint16_t conn_handle)
{
  conn = conn_handle;
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
    digitalWrite(LED_RED, LOW);
    delayMicroseconds(10000);      // for LED
    digitalWrite(LED_RED, HIGH);   
  }
}
