//----------------------------------------------------------------------------------------------
// Board Library : Seeed nRF52 mbed-enable Borads 2.9.1
// Board Select  : Seeed nRF52 mbed-enable Borads / Seeed XIAO BLE Sense - nRF52840
//----------------------------------------------------------------------------------------------

// 2023/05/21
// Central sketch :  nRF52_XIAO_mbed_LightSleepTest_central.ino 

#include <ArduinoBLE.h>


#define SLEEP_TIME  10        // Sleep time [sec]c

#define dataNum 8             // mS data 4 byte, uS data 4 byte (2 data 8 byte)
union unionData {             // Union for data type conversion
  uint32_t    dataBuff32[dataNum/4];
  uint16_t    dataBuff16[dataNum/2];
  uint8_t     dataBuff8[dataNum];
};
union unionData ud;
uint32_t mS, uS;
bool intFlag = true;          // Interrupu flag
bool subscribedFlag = false;  // Subscribed flag
bool connectedFlag = false;   // Connected flag
int16_t conn;                 // Connect handler


// Custum Service and Characteristic
#define myUUID(val) ("526539fb-" val "-4889-b224-ace499220efa")
BLEService        secondService(myUUID("0000"));
BLECharacteristic dataCharacteristic(myUUID("0010"), BLENotify, dataNum);
BLEDevice mycentral;

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
  
  // Initialization of BLE class
  if (!BLE.begin())
  {
    Serial.println("starting BLE module failed!");
    while (1);
  }
  Serial.println(BLE.address());

  // set the local name peripheral advertises
  BLE.setLocalName("XIAO_BLE_peripheral");
  // set the device name peripheral advertises  (Default "Arduino")
  BLE.setDeviceName("XIAO BLE Sence");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(secondService);
  // add the characteristic to the service  
  secondService.addCharacteristic(dataCharacteristic);
  // add service  
  BLE.addService(secondService);

  // assign event handlers for connected, disconnected
  BLE.setEventHandler(BLEConnected, bleConnectedHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectedHandler);

  // assign event handlers for characteristic
  dataCharacteristic.setEventHandler(BLESubscribed, bleSubscribedHandler);  
  dataCharacteristic.setEventHandler(BLEUnsubscribed, bleSubscribedHandler); 

  // start advertising
  BLE.setAdvertisingInterval(160);    //0.625mS*160=100mS
  BLE.setConnectionInterval(6, 3200);  //1.25mS*6=7.5mS, 1.25mS*3200=4S
  BLE.advertise();

  Serial.println("End of initialization");
}

//*************************************************************************************************************
void loop()
{
  if(intFlag == true)     // If wake up interrupt
  {
    if(!connectedFlag) 
    {
      Serial.println("Connecting to Central ....");
      do {
        mycentral = BLE.central();
      } while(!mycentral || !subscribedFlag);
      Serial.println("Connected to Central");
    }
    
    if(mycentral.connected())
    {      
      digitalWrite(LED_GREEN, LOW);

      mS = millis();
      uS = micros();
      ud.dataBuff32[0] = mS;
      ud.dataBuff32[1] = uS;

      Serial.print("【loop】 ");
      Serial.print(mS);
      Serial.print(" ");
      Serial.print(uS);
      Serial.println();
            
      dataCharacteristic.writeValue(ud.dataBuff8, dataNum);   // Data transfer
      delayMicroseconds(100000);      // Delay 100mS

      digitalWrite(LED_GREEN, HIGH);      
    }   
    intFlag = false;   
  }
  // Sleep
  __WFI();
  __SEV();
  __WFI();
}

//**************************************************************************************************************
// callback function
void bleConnectedHandler(BLEDevice mycentral)
{
  Serial.println("Connected");
  connectedFlag = true;
}

void bleDisconnectedHandler(BLEDevice mycentral)
{
  Serial.println("Disconnected");
  connectedFlag = false;  
}

void bleSubscribedHandler(BLEDevice mycentral, BLECharacteristic dataCharacteristic)
{
  Serial.println("Subscribed");
  subscribedFlag = true;  
}

void bleUnsubscribedHandler(BLEDevice mycentral, BLECharacteristic dataCharacteristic)
{
  Serial.println("Unsubscribed");
  subscribedFlag = false;  
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
//  NVIC_SetPriority(RTC2_IRQn,3); // Higher priority than SoftDeviceAPI
  NVIC_EnableIRQ(RTC2_IRQn);     // enable interrupt  
  NRF_RTC2->TASKS_START = 1;     // start Timer
}

//**************************************************************************************************************
// RTC interrupt handler
// Important need "_v"
extern "C" void RTC2_IRQHandler_v(void) 
{
  if ((NRF_RTC2->EVENTS_COMPARE[0] != 0) && ((NRF_RTC2->INTENSET & 0x10000) != 0)) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;  // clear compare register 0 event
    NRF_RTC2->TASKS_CLEAR = 1;        // clear counter

    intFlag = true;   
    digitalWrite(LED_RED, LOW);
    delayMicroseconds(5000);      // for LED
    digitalWrite(LED_RED, HIGH);   
  }
}
