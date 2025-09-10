#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // for Serial.print()

// ---- Definition of pins and constants ----
#define VBAT          (32)    //P0_31   // Battery voltage monitoring pin
#define VBAT_ENABLE   (14)    //P0_14   // Enable battery voltage monitoring pin
#define HICHG         (22)    //P0_13   // Charge current setting pin LOW: 100mA HIGH: 50mA
#define CHG           (23)    //P0_17   // Charge indicator LOW:charge HIGH:no charge
#define VBAT_LOWER    3.5     // Battery voltage lower limit


// ---- Variable definitions ----
bool LED_OnOff;

void setup() 
{
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  pinMode(VBAT, INPUT);
  pinMode(HICHG, OUTPUT);
  digitalWrite(HICHG, LOW); // Charge current 100mA
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW); // Enable battery voltage monitoring

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH); // LOW = On, HIGH = Off 
  digitalWrite(LED_GREEN, HIGH); 
  digitalWrite(LED_BLUE, HIGH);

  analogReference(AR_DEFAULT);
  analogReadResolution(12);
 
}

void loop() 
{
  LED_OnOff = !LED_OnOff; 
  digitalWrite(LED_GREEN, (LED_OnOff ? LOW : HIGH));

  // Average battery voltage 32 times
  int isCharging = digitalRead(CHG);
  int Vadc = 0;
  for (int i = 0; i < 32; i++) Vadc = Vadc + analogRead(VBAT);
  float Vbatt = ((510e3 + 1000e3) / 510e3) * 2.4 * Vadc / 32 / 4096;

  Serial.println(Vbatt, 3);
  Serial.println(isCharging? "Not charging" : "Charging");

  if(Vbatt < VBAT_LOWER) digitalWrite(LED_RED, LOW);
  else digitalWrite(LED_RED, HIGH);

  delay(1000);
}