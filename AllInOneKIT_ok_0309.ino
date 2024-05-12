// Diagnastic program of ESP32 and PICO All In One KIT 
#include "DHT.h"
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_BMP3XX.h"
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <BH1750.h>
#include "paj7620.h"
#include <Adafruit_NeoPixel.h>
#include "MAX30105.h"
#include <TinyGPS++.h>
#include <SPI.h>
#include "FS.h"
#include "SPI.h"
#include "Arduino.h"
#include "Audio.h"
#include "SD.h"
#include "FS.h"

#define ADCPIN                2        // 트리머 센서에 연결된 GPIO 핀
#define DHTPIN                21       // DHT22 센서에 연결된 GPIO 핀
#define DHTTYPE               DHT22   
#define SD_CS                 10       // SD Card 
#define SPI_MOSI              11    
#define SPI_MISO              13
#define SPI_SCK               12
#define SOUND_SPEED           0.034    //소리 속도 정의
#define I2S_DOUT              14       // I2S 센서에 연결된 GPIO 핀
#define I2S_BCLK              45        
#define I2S_LRC               46
#define LED_PIN               48       // WS2812B에 연결된 GPIO 핀
#define LED_COUNT             10       // WS2812B LED개수
#define SCREEN_WIDTH          128      // OLED 디스플레이 너비, 픽셀 단위
#define SCREEN_HEIGHT         64       // OLED 디스플레이 높이, 픽셀 단위
#define OLED_RESET            -1       // 리셋 핀 번호(또는 아두이노 리셋 핀을 공유하는 경우 -1)
#define SCREEN_ADDRESS        0x3C     // OLED I2C 주소
#define RXD2                  5        // GPS센서에 연결된 GPIO 핀
#define TXD2                  4
#define PIR                   37       // PIR센서에 연결된 GPIO 핀
#define LDR                   3        //포토레지스트용 아날로그 입력
#define oneWireBus           16        // DS18B20 GPIO번호 셑
#define trigPin              40        // HC SR04센서에 연결된 GPIO 핀
#define echoPin              36            


int LDR_Val =  0;            //포토레지스트 값을 저장할 변수
long duration;  //unsigned long delayTime;
float distanceCmCurrent = 0;
const int buttonPin1 = 47;  // GPIO번호 47
const int buttonPin2 = 35;  // GPIO번호 35
const int buttonPin3 = 39;  // GPIO번호 39
const int buttonPin4 = 38;  // GPIO번호 38
const int ledPin1 =  45;    // GPIO번호 45
const int ledPin2 =  46;    // GPIO번호 46
const int ledPin3 =  48;    // GPIO번호 48
const int ledPin4 =  40;    // GPIO번호 40
const int buzzer_relay = 42;      // GPIO번호 42
const int motionSensor = 37;// 센서 GPIO번호
int i = 0;                  // index value 
int error = 0;              // error flag clear
int dutyCycle = 0;
boolean newData = false;    // GPS check   

Adafruit_BME280 bme;        // I2C 인터페이스 방식으로 사용
Adafruit_BMP3XX bmp;
Adafruit_MPU6050 mpu;       // MPU 6050
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // I2C에 연결된 SSD1306 디스플레이에 대한 선언
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);         // Setting up the NeoPixel library
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(oneWireBus);         //  OneWire장치와 통신을 위해 인스턴스를 셑
DallasTemperature sensors(&oneWire); // oneWire reference를 Dallas Temperature sensor에 전달
MAX30105 particleSensor;
BH1750 lightMeter(0x23);
Audio audio;
TinyGPSPlus gps;
HardwareSerial neogps(1);


void IRAM_ATTR toggleLED1() // 인터랍트 서비스 루틴
{
  //detachInterrupt(buttonPin1);
  digitalWrite(ledPin1, !digitalRead(ledPin1));
  digitalWrite(buzzer_relay, !digitalRead(buzzer_relay));
}
void IRAM_ATTR toggleLED2()
{
  //detachInterrupt(buttonPin2);
  digitalWrite(ledPin2, !digitalRead(ledPin2));
  digitalWrite(buzzer_relay, !digitalRead(buzzer_relay));
}
void IRAM_ATTR toggleLED3()
{
  //detachInterrupt(buttonPin3);
  digitalWrite(ledPin3, !digitalRead(ledPin3));
  digitalWrite(buzzer_relay, !digitalRead(buzzer_relay));
}
void IRAM_ATTR toggleLED4()
{
  //detachInterrupt(buttonPin4);
  digitalWrite(ledPin4, !digitalRead(ledPin4));
  digitalWrite(buzzer_relay, !digitalRead(buzzer_relay));
}

void setup() 
{
  Wire.begin();
  paj7620Init();
  Serial.begin(9600);         // 시리얼전송 속도를 9600보로 셑
  Serial.println();
  strip.begin(); // 네오픽셀 초기화                
  strip.show();  // 모든 픽셀 OFF


  pinMode(buttonPin1, INPUT_PULLDOWN); // buttonPin을 INPUT으로 선언
  pinMode(buttonPin2, INPUT_PULLDOWN); // buttonPin을 INPUT으로 선언
  pinMode(buttonPin3, INPUT_PULLDOWN); // buttonPin을 INPUT으로 선언
  pinMode(buttonPin4, INPUT_PULLDOWN); // buttonPin을 INPUT으로 선언
  pinMode(ledPin1, OUTPUT);   // ledPin을 OUTPUT으로 선언
  pinMode(ledPin2, OUTPUT);   // ledPin을 OUTPUT으로 선언
  pinMode(ledPin3, OUTPUT);   // ledPin을 OUTPUT으로 선언
  pinMode(ledPin4, OUTPUT);   // ledPin을 OUTPUT으로 선언
  pinMode(buzzer_relay, OUTPUT);    // buzzer를 OUTPUT으로 선언
  pinMode(motionSensor, INPUT);     // PIR
  pinMode(trigPin, OUTPUT); // HC-SR04 출력모드로 선언
  pinMode(echoPin, INPUT);  // HC-SR04 입력모드로 선언

  strip.begin();           // NeoPixel object 초기화
  strip.setBrightness(10); // 밝기값 세팅 (max = 255)
  
// Button, LED, Buzzer, Relay
  attachInterrupt(buttonPin1, toggleLED1, RISING);  // RED
  attachInterrupt(buttonPin2, toggleLED2, RISING);  // GREEN
  attachInterrupt(buttonPin3, toggleLED3, RISING);  // BLUE
  attachInterrupt(buttonPin4, toggleLED4, RISING);  // YELLOW

  Serial.println();
  Serial.println();  
  Serial.println("01. check switchs and leds, Press any key to escape");
  while (Serial.available() == 0);    // 어떤키와 리턴을 기다림
  
  detachInterrupt(buttonPin1);
  detachInterrupt(buttonPin2);
  detachInterrupt(buttonPin3);
  detachInterrupt(buttonPin4);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);

// BME280
  Wire.beginTransmission(0x76);
  error = Wire.endTransmission();
    if (error == 0)   {
      Serial.println("02. BME280 is ok"); // 정상경우
  }
    else    {
    Serial.println("02. BME280 - check soldering ???");
  }
  delay(200);

// BH1750
  Wire.beginTransmission(0x23);
  error = Wire.endTransmission();
    if (error == 0)   {
      Serial.println("03. BH1750 is ok"); // 정상경우
  }
    else    {
    Serial.println("03. BH1750 - check soldering ???");
  }
  delay(200);

// MAX30102
  Wire.beginTransmission(0x57);
  error = Wire.endTransmission();
    if (error == 0)   {
      Serial.println("04. MAX30102 is ok"); // 정상경우
  }
    else    {
    Serial.println("04. MAX30102 - check soldering ???");
  }
  delay(200);

// OLED
  Wire.beginTransmission(0x3c);
  error = Wire.endTransmission();
    if (error == 0)   {
      Serial.println("05. OLED is ok"); // 정상경우
  }
    else    {
    Serial.println("05. OLED - check soldering ???");
  }
  delay(200);

// MPU6050
  Wire.beginTransmission(0x68);
  error = Wire.endTransmission();
    if (error == 0)   {
      Serial.println("06. MPU6050 is ok"); // 정상경우
  }
    else    {
    Serial.println("06. MPU6050 - check soldering ???");
  }
  delay(200);

// BMP388
  Wire.beginTransmission(0x77);
  error = Wire.endTransmission();
    if (error == 0)   {
      Serial.println("07. BMP388 is ok"); // 정상경우
  }
    else    {
    Serial.println("07. BMP388 - check soldering ???");
  }
  delay(200);

// GPS
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);  
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  if(newData == true) {  // newData가 참이면
  Serial.println("08. GPS is ok");
  }
  else    {
  Serial.println("08. GPS - check soldering ???");
  } 
  delay(200);

// DHT
  dht.begin();
  delay(2000);
  // 약250ms동안 습도, 온도를 읽는다 (최대 2초)
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("09. DHT22 - check soldering ???"));
    return;
  }
  else    {
  Serial.println("09. DHT22 is ok");
  }
  delay(200);

// PAJ7620-제스처센서
  uint8_t data = 0, error;
	error = paj7620ReadReg(0x43, 1, &data);	 // 제스처 결과에 대해 Bank_0_Reg_0x43을 읽습니다.
	if (!error) 
  {
      Serial.println("10. PAJ7620 is ok"); // 정상경우
  }
    else    {
    Serial.println("10. PAJ7620 - check soldering ???");
  }

  delay(200);

// DS18B20
  sensors.begin();      // 센서 시작
  delay(100);
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  if ((temperatureC > 5) & (temperatureC < 40) ) {
    Serial.println("11. DS18B20 is ok");
  }
    else {
    Serial.println("11. DS18B20 - check wiring ???");
  }
  delay(200);

// PIR
  Serial.print("12. PIR   test -----> ");
  for (int i = 0; i <= 100; i++) {
  if ((i > 10)  && (digitalRead(motionSensor)))  {   // read new state
      Serial.println("ok");
      break; // exit
  } 
  delay(500);
  if (i >= 100) {   // check end of for
    Serial.println("not ok !!!");
  }   
  }
  delay(500);

// HC-SR04
  Serial.print("13. HC-SR04  distance test ----->  ");
  for (int i = 0; i <= 10000; i++) {
  digitalWrite(trigPin, LOW);  // trigPin 초기화 및 초음파 방출과장
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH); // trigPin에 HIGH출력
  delayMicroseconds(10);       // 10ms 딜레이
  digitalWrite(trigPin, LOW);  // trigPin에 LOW출력
  duration = pulseIn(echoPin, HIGH);  // 마이크로 세컨단위로 펄스길이를 읽어낸다
  distanceCmCurrent = duration * SOUND_SPEED/2;  // 펄스길이를 거리로 변환
//  Serial.println(distanceCmCurrent);
  if (distanceCmCurrent < 1) { 
    Serial.println("not ok");
    break;
  }
  else if ((distanceCmCurrent > 10) && (distanceCmCurrent < 12) ) { // 10-12cm 이면 escape
    Serial.println("ok");
    break;
  }
  delay(100);
   if (i >= 10000) {    // check end of for
    Serial.println("not ok !!!");
  }   
  }
  delay(200);

// LDR 
  Serial.print("14. LDR  bright test ----->  ");
  for (int i = 0; i <= 500; i++) {
    LDR_Val = analogRead(LDR);   //LDR값 아날로그 읽기
//    Serial.println(LDR_Val);     // 값을 출력하여 확인 후에 상위값과 하위값을 조정하세요'

    if (LDR_Val > 2500) {        //광도가 어두운  경우
        digitalWrite(ledPin3,HIGH); /*LED Remains OFF*/
    }
    else if  (LDR_Val < 1000) {
        digitalWrite(ledPin4,HIGH);  // LED 켜짐 - LDR 값이 낮을때
    }
    if (digitalRead(ledPin3) && digitalRead(ledPin4)) {
      Serial.println("ok");
      break; // exit
    }
    delay(500);      //0.5초마다 값 읽기 딜레이
    if (i >= 500) {    // check end of for
    Serial.println("not ok !!!");
  }
  }
  delay(200);

 // Trimmer
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  Serial.print("15. Rotate Trimmer  ----->");
  for (int i = 0; i <= 200; i++) {
    dutyCycle = analogRead(ADCPIN);
    if (dutyCycle < 500) {        //광도가 HIGH인 경우
        digitalWrite(ledPin1,HIGH); /*LED Remains OFF*/
    }
    else if (dutyCycle > 4000) {
        digitalWrite(ledPin2,HIGH);  // LED 켜짐 - LDR 값이 100 미만
    }
    if (digitalRead(ledPin1) && digitalRead(ledPin2)) {
      Serial.println("ok");
      break; // exit
    }
    delay(100);        //0.1초마다 값 읽기 딜레이
    if (i >= 200) {    // check end of for
    Serial.println("not ok !!!");
  }
  }
    delay(200);

  // ADDRESSABLE LED
  strip.begin();      // Initialize NeoPixel object
  Serial.begin(9600); // 시리얼버퍼 클리어
                        
  for(int i=0; i<LED_COUNT; i++) {   
     strip.setPixelColor(i, 0, 0, 255);  // 파란색을 세팅
     strip.show();                       // 색깔출력
     delay(200); 
  }
  Serial.println("16. check Addressable LEDs, Press any key to escape, ");
  while (Serial.available() == 0);    // 어떤키와 리턴을 기다림

  pinMode(SD_CS, OUTPUT);      
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);    
  if(!SD.begin(SD_CS)){
        Serial.println("17. SD Card Mount Failed");
        return;  // 중지 SD카드 불량
    }
  Serial.println("17. SD Card ------> ok");
  SD.begin(SD_CS);
   
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21); // 0...21
  audio.connecttoFS(SD, "1.mp3");// 1.mp3 파일음악이 저장되어 있어야합니다  
  Serial.println("18. if no sound, check MAX98357 !!!");

} // end of setup

void loop() {
  audio.loop();   
}
