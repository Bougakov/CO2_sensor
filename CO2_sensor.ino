#include "MegunoLink.h"
TimePlot MyPlot; 

/*
 * Global status variable:
   0 - init
   1 - green
   2 - yellow
   3 - red
   4 - violet
 
 */

int status = 0;

/*
 * CO2 sensor
 */

#define ventPin 7
#include <SoftwareSerial.h>
SoftwareSerial swSerial(12, 11);  // ->RX, ->TX

// Any Arduino pins labeled:  SDA  SCL
// Uno, Redboard, Pro:        A4   A5 

/*
 * Air pressure sensor
 */


#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
#define ALTITUDE 200.0  // Altitude in meters

/*
 * Light intensity sensor
 */

#include <Wire.h>
int BH1750_address = 0x23; // i2c Address of sensor
byte buff[2];

/*
 * RGB strip
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN 4
#define NUM_LEDS 6
#define BRIGHTNESS 255
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

/*
 * Dust sensor
 */

int ledPower = 6; 
int dustPin = A0;  
int delayTime = 280;
int delayTime2 = 40;
float offTime = 9680;

/*
 * DS18B20 temp sensor
 */

#include <OneWire.h> 
#include <DallasTemperature.h>
// Data wire is plugged into pin 3 on the Arduino 
OneWire oneWire(3); 
DallasTemperature sensors(&oneWire);
#define power_DS18B20 2 // power on pin for temp sensor

/*
 * SETUP
 */

void setup() {
  Serial.begin(9600);
  swSerial.begin(9600);
  setup_rgb();
  setup_CO2();
  setup_lux();
  setup_pressuretemp();
  setup_dust();
//  setup_DS18B20();
}

void loop() {
  loop_CO2();
  loop_pressuretemp();
  loop_dust();
  loop_lux();
//  loop_DS18B20();
  loop_rgb();
  delay(5000);
}

/**********************************************************************************************************/

/*
 * DS18B20
 */


void setup_DS18B20(void) {
  // Serial.begin(9600);
  pinMode(power_DS18B20, OUTPUT);
  sensors.begin(); 
}

void loop_DS18B20(void) {
 digitalWrite(power_DS18B20, HIGH);
 delay(100);
 sensors.requestTemperatures(); // Send the command to get temperature readings 
 MyPlot.SendData("Temp, C (DS18B20)", sensors.getTempCByIndex(0) );
 digitalWrite(power_DS18B20, LOW);
}

/*
 * CO2 sensor
 */

void setup_CO2() {
  pinMode(ventPin, OUTPUT);

  //           bytes:                         3     4           6     7
  byte setrangeA_cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00,
                           0x00, 0x13, 0x88, 0xCB};
  unsigned char setrangeA_response[9];
  swSerial.write(setrangeA_cmd, 9);
  swSerial.readBytes(setrangeA_response, 9);
  int setrangeA_i;
  byte setrangeA_crc = 0;
  for (setrangeA_i = 1; setrangeA_i < 8; setrangeA_i++)
    setrangeA_crc += setrangeA_response[setrangeA_i];
  setrangeA_crc = 255 - setrangeA_crc;
  setrangeA_crc += 1;
  if (!(setrangeA_response[0] == 0xFF && setrangeA_response[1] == 0x99 &&
        setrangeA_response[8] == setrangeA_crc)) {
    Serial.println("Range CRC error: " + String(setrangeA_crc) + " / " +
                   String(setrangeA_response[8]) + " (bytes 6 and 7)");
  } else {
    // Serial.println("Range was set! (bytes 6 and 7)");
  }
  delay(1000);

  /*
  //        bytes:       0     1     2     3     4     5     6     7     8
  byte setABC_cmd[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
  unsigned char setABC_response[9];
  swSerial.write(setABC_cmd, 9);
  swSerial.readBytes(setABC_response, 9);
  int setABC_i;
  byte setABC_crc = 0;
  for (setABC_i = 1; setABC_i < 8; setABC_i++)
    setABC_crc += setABC_response[setABC_i];
  setABC_crc = 255 - setABC_crc;
  setABC_crc += 1;
  if (!(setABC_response[0] == 0xFF && setABC_response[1] == 0x99 &&
        setABC_response[8] == setABC_crc)) {
    Serial.println("Error turning ABC off: " + String(setABC_crc) + " / " +
                   String(setABC_response[8]) + " (bytes 6 and 7)");
  } else {
    // Serial.println("ABC was turned off!");
  }
  delay(1000);
  */
}

void loop_CO2() {

  digitalWrite(ventPin, HIGH);
  delay(3000);
  
  byte measure_cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  unsigned char measure_response[9];
  unsigned long th, tl, ppm = 0, ppm2 = 0, ppm3 = 0, ppm4 = 0;
  unsigned int responseHigh, responseLow;

  // CO2 via UART:
  swSerial.write(measure_cmd, 9);
  swSerial.readBytes(measure_response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc += measure_response[i];
  crc = 255 - crc;
  crc += 1;
  if (!(measure_response[0] == 0xFF && measure_response[1] == 0x86 &&
        measure_response[8] == crc)) {
    Serial.println("CRC error: " + String(crc) + " / " +   String(measure_response[8]));
  }
  responseHigh = (unsigned int)measure_response[2];
  responseLow = (unsigned int)measure_response[3];
  ppm = (256 * responseHigh) + responseLow;

  // CO2 via PWM:
  /*
  do {
    th = pulseIn(pwmPin, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm2 = 2000 * (th - 2) / (th + tl - 4);
    ppm3 = 5000 * (th - 2) / (th + tl - 4);
    ppm4 = 10000 * (th - 2) / (th + tl - 4);
  } while (th == 0);
  */

  /*
— 350 — 450 ppm: Нормальный уровень на открытом воздухе.
— < 600 ppm: Приемлемые уровни. Уровень. рекомендованный для спален, детских
садов и школ.
— 600 — 1000 ppm: Жалобы на несвежий воздух, возможно снижение концентрации
внимания.
— 1000 ppm: Максимальный уровень стандартов ASHRAE (American Society of Heating,
Refrigerating and Air-Conditioning Engineers) и OSHA (Occupational Safety &
Health Administration).
— 1000 — 2500 ppm: Общая вялость, снижение концентрации внимания, возможна
головная боль.
— 2500 — 5000 ppm: Возможны нежелательные эффекты на здоровье.
*/

  // Serial.print((ppm * 2.0) / 5.0);
  // Serial.print(" ");
  unsigned long trueppm = (ppm * 2.0) / 5.0;
  MyPlot.SendData("CO2 in air, ppm", trueppm);
  digitalWrite(ventPin, LOW);

  if (trueppm >= 2000) {
//    Serial.println("blue");
    status = 0; // blue
  }

  if (trueppm <= 450) {
//    Serial.println("green");
    status = 1; // green    
  }

  if (trueppm > 450 && trueppm <= 600) {
//    Serial.println("yellow");
    status = 2; // yellow    
  }
  
  if (trueppm > 600 && trueppm <= 1000) {
//    Serial.println("red");
    status = 3; // red    
  }

  if (trueppm > 1000 && trueppm <= 2000) {
//    Serial.println("magenta");
    status = 4; // magenta    
  }

}

/*
 * BMP180 air pressure and temperature
 */


void setup_pressuretemp() {
  if (!pressure.begin()) {
    Serial.println("BMP180 init fail\n\n");
    while (1)
      ;  // Pause forever.
  }
}

void loop_pressuretemp() {
  char status;
  double T, P, p0, a;
  status = pressure.startTemperature();
  if (status != 0) {
    // Wait for the measurement to complete:
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      // Serial.print(T, 2);
      MyPlot.SendData("Temp, C (BMP180)", T );
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0) {
          // Serial.print(" ");
          // Serial.print(P * 0.75006157584566, 2);  
          MyPlot.SendData("Air pressure, mmHg", P * 0.75006157584566 );
          //          Serial.println(" mmHg");
          //          Serial.println("");
        } else
          Serial.println("error retrieving pressure measurement\n");
      } else
        Serial.println("error starting pressure measurement\n");
    } else
      Serial.println("error retrieving temperature measurement\n");
  } else
    Serial.println("error starting temperature measurement\n");
    delay(3000);  // Pause for 3 seconds.
}

/*
 * Light intensity
 */


void setup_lux(){
  Wire.begin();
  BH1750_Init(BH1750_address);
  delay(200);
}

void loop_lux(){
  float valf=0;
  if(BH1750_Read(BH1750_address)==2){
    valf=((buff[0]<<8)|buff[1])/1.2;
    if(valf<0) {
      Serial.println("Got negative value from lux sensor!");
    } else {
      // Serial.println((int)valf,DEC); 
      MyPlot.SendData("Light intensity, lux", valf);
    }
  }
  delay(1000);
}

void BH1750_Init(int address){
  
  Wire.beginTransmission(address);
  Wire.write(0x10); // 1 [lux] aufloesung
  Wire.endTransmission();
}

byte BH1750_Read(int address){
  byte i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();  
  return i;
}

/*
 * Dust sensor
 */


void setup_dust() {
  pinMode(ledPower, OUTPUT);
  pinMode(dustPin, INPUT);
  digitalWrite(ledPower, HIGH); // turn the LED off
}
 
void loop_dust() {
  digitalWrite(ledPower, LOW); // power on the LED
  delayMicroseconds(delayTime);
  float dustVal = analogRead(dustPin); 
  delayMicroseconds(delayTime2);
  digitalWrite(ledPower, HIGH); // turn the LED off
  delayMicroseconds(offTime);
  // calculate dust density
  // http://www.howmuchsnow.com/arduino/airquality/
  // y (dust density mg/m3) = 0.172*x - 0.0999
  float dustDensity = (0.172*(dustVal*(5/1024.0))-0.0999)*1000; // ug/M3
  /* Air Quality Chart - Small Count Reading (0.5 micron)+
  3000 +     = VERY POOR
  1050-3000  = POOR
  300-1050   = FAIR
  150-300    = GOOD
  75-150     = VERY GOOD
  0-75       = EXCELLENT */
  MyPlot.SendData("Dust density, mg/M3", dustDensity);
}

/*
 * RGB strip
 */

void setup_rgb() {
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop_rgb() {

  if (status == 0) {
    for(uint16_t r=1; r<6; r++) {
      led_blue();
    }
  }

  if (status == 1) {
    for(uint16_t r=1; r<6; r++) {
      led_green();
    }
  }

  if (status == 2) {
    for(uint16_t r=1; r<6; r++) {
      led_yellow();
    }
  }

  if (status == 3) {
    for(uint16_t r=1; r<6; r++) {
      led_red();
    }
  }

  if (status == 4) {
    for(uint16_t r=1; r<6; r++) {
      led_magenta();
    }
  }
}  

void led_red() {
  //Serial.println("red");
  for(uint16_t col=0; col<256; col++) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(col,0,0,0) );
    }
    strip.show();
    delay(1);
  }
  //Serial.println("/red");
  for(uint16_t col=255; col>8; col -= 1) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(col,0,0,0) );
    }
    Serial.println(col);
    strip.show();
    delay(3);
  }
  led_clear();
  delay(500);
}

void led_yellow() {
  //Serial.println("yellow");
  for(uint16_t col=0; col<256; col++) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(col,col,0,0) );
    }
    strip.show();
    delay(4);
  }
  //Serial.println("/yellow");
  for(uint16_t col=255; col>8; col -= 1) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(col,col,0,0) );
    }
    strip.show();
    delay(9);
  }
  led_clear();
  delay(500);
}

void led_blue() {
  //Serial.println("blue");
  for(uint16_t col=0; col<256; col++) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(0,0,col,0) );
    }
    strip.show();
    delay(16);
  }
  //Serial.println("/blue");
  led_clear();
  for(uint16_t col=255; col>8; col -= 1) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(0,0,col,0) );
    }
    strip.show();
    delay(32);
  }
  led_clear();
  delay(500);
}

void led_green() {
  //Serial.println("green");
  for(uint16_t col=0; col<256; col++) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(0,col,0,0) );
    }
    strip.show();
    delay(8);
  }
  //Serial.println("/green");
  for(uint16_t col=255; col>8; col -= 1) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(0,col,0,0) );
    }
    strip.show();
    delay(18);
  }
  led_clear();
  delay(500);
}

void led_magenta() {
  //Serial.println("magenta");
  for(uint16_t col=0; col<256; col++) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(col,0,col,0) );
    }
    strip.show();
    delay(1);
  }
  //Serial.println("/magenta");
  for(uint16_t col=255; col>1; col -= 1) {
    for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(col,0,col,0) );
    }
    strip.show();
    delay(3);
  }
  led_clear();
  delay(500);
}

void led_clear() {
  for(uint16_t id=0; id<strip.numPixels(); id++) {
      strip.setPixelColor(id, strip.Color(0,0,0,0) );
    }
    strip.show();
}

