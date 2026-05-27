#include <avr/sleep.h>
#include <avr/wdt.h>
#include <TinyWireM.h>
#include <Tiny4kOLED.h>
#include <TinyBME280.h>

#define AHT20_ADDR 0x38

void setup() {
  TinyWireM.begin();
  
  // 1. AHT20 sensor initialization
  TinyWireM.beginTransmission(AHT20_ADDR);
  TinyWireM.send(0xBE);
  TinyWireM.send(0x08);
  TinyWireM.send(0x00);
  TinyWireM.endTransmission();
  delay(20); 

  // 2. Initializing the BMP280 sensor (via TinyBME280)
  BME280setI2Caddress(0x77); // Standard I2C address for hybrid boards
  BME280setup();             // Loading calibration data

  // 3. OLED display settings
  oled.begin(128, 32, sizeof(tiny4koled_init_128x32r), tiny4koled_init_128x32r);
  oled.setFont(FONT6X8);     // Small font to fit 3 lines
  oled.clear();
  oled.on();
  
  setupWatchdog(9);          // 8 second sleep timer
}

void loop() {
  float humi = 0.0;
  
  // === HUMIDITY READING (with AHT20) ===
  TinyWireM.beginTransmission(AHT20_ADDR);
  TinyWireM.send(0xAC);
  TinyWireM.send(0x33);
  TinyWireM.send(0x00);
  TinyWireM.endTransmission();
  
  delay(80); // We are waiting for the completion of the measurement.

  TinyWireM.requestFrom(AHT20_ADDR, 6);
  uint8_t buf[6];
  for(int i = 0; i < 6; i++) {
    buf[i] = TinyWireM.receive();
  }

  // Humidity calculation
  if((buf[0] & 0x80) == 0) { 
    uint32_t rawHumi = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (buf[3]);
    rawHumi = rawHumi >> 4; // Offset for leveling humidity bits
    humi = (rawHumi * 100.0) / 1048576.0;
  }

  // === TEMPERATURE AND PRESSURE READING (with BMP280) ===
  // The library returns temperature * 100 (e.g. 2512 = 25.12 C)
  float tempC = BME280temperature() / 100.0; 
  
  // Returns pressure in Pascals. Divide by 100 to get hectopascals (hPa)
  int32_t pressureHPa = BME280pressure() / 100;

  // === SCREEN OUTPUT ===
  oled.clear();
  
  oled.setCursor(0, 0);
  oled.print("Temp: "); oled.print(tempC, 1); oled.print(" C");

  oled.setCursor(0, 1);
  oled.print("Humi: "); oled.print(humi, 1); oled.print(" %");

  oled.setCursor(0, 2);
  oled.print("Pres: "); oled.print(pressureHPa); oled.print(" hPa");

  delay(100); 
  
  // === SLEEP ===
  systemSleep(); 
}

// Configuring Watchdog Interrupts
void setupWatchdog(int ii) {
  byte bb;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  
  MCUSR &= ~(1 << WDRF);
  WDTCR |= (1 << WDCE) | (1 << WDE);
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

ISR(WDT_vect) {} // Awakening

void systemSleep() {
  ADCSRA &= ~(1<<ADEN); // Disabling the ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();         // The controller falls asleep here.
  sleep_disable(); 
  ADCSRA |= (1<<ADEN);  // Enabling the ADC
}
