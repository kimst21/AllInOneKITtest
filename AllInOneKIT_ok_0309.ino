#include "DHT.h"           // DHT22 온습도 센서용 라이브러리
#include <Wire.h>           // I2C 통신을 위한 라이브러리
#include <OneWire.h>        // DS18B20 온도센서용 OneWire 프로토콜 라이브러리
#include <DallasTemperature.h> // DS18B20 센서 라이브러리
#include <Adafruit_NeoPixel.h> // WS2812B 네오픽셀 제어용 라이브러리
#include <TinyGPS++.h>      // GPS 데이터를 파싱하기 위한 라이브러리
#include <SPI.h>            // SPI 통신용 라이브러리
#include "FS.h"            // 파일 시스템 관련 라이브러리
#include "Arduino.h"       // 기본 Arduino 함수 정의
#include "Audio.h"         // MAX98357 I2S 오디오 출력 라이브러리
#include "SD.h"            // SD 카드 사용을 위한 라이브러리

// ----------- 핀 및 상수 정의 -----------
#define ADCPIN                2       // 트리머 입력 아날로그 핀
#define DHTPIN                21      // DHT22 센서 핀
#define DHTTYPE               DHT22   // DHT 센서 타입
#define SD_CS                 10      // SD카드 CS 핀
#define SPI_MOSI              11      // SPI MOSI 핀
#define SPI_MISO              13      // SPI MISO 핀
#define SPI_SCK               12      // SPI 클럭 핀
#define SOUND_SPEED           0.034   // 초음파 센서 거리 계산용 속도(cm/us)
#define I2S_DOUT              14      // I2S 오디오 출력용 데이터 핀
#define I2S_BCLK              41      // I2S 비트 클럭 핀
#define I2S_LRC               42      // I2S LRC 클럭 핀
#define LED_PIN               4       // WS2812 네오픽셀 데이터 핀
#define LED_COUNT             10      // 네오픽셀 개수
#define RXD2                  5       // GPS 수신용 UART RX 핀
#define TXD2                  4       // GPS 송신용 UART TX 핀
#define PIR                   47      // PIR 모션 센서 핀
#define LDR                   2       // 조도 센서 핀
#define oneWireBus            7       // DS18B20 연결용 핀
#define relay                 37      // 릴레이 출력 핀
#define BUZZER_PIN            48      // 부저 핀
#define CHANNEL 0                    // PWM 채널 번호
#define FREQ 2000                    // PWM 주파수 (Hz)
#define RESOLUTION 8                // PWM 해상도 (8비트)

// ----------- 버튼 및 LED 핀 -----------
const int buttonPin1 = 6;    // 버튼1 입력 핀
const int buttonPin2 = 1;    // 버튼2 입력 핀
const int buttonPin3 = 15;   // 버튼3 입력 핀
const int buttonPin4 = 7;    // 버튼4 입력 핀
const int ledPin1 = 41;      // LED1 출력 핀
const int ledPin2 = 42;      // LED2 출력 핀
const int ledPin3 = 17;      // LED3 출력 핀
const int ledPin4 = 4;       // LED4 출력 핀
const int trigPin = 41;      // 초음파 센서 트리거 핀
const int echoPin = 35;      // 초음파 센서 에코 핀

// ----------- 전역 변수 -----------
bool newData;                            // GPS 데이터 수신 여부 플래그
int i = 0;                               // 루프용 인덱스
int LDR_Val =  0;                        // LDR 센서 값 저장 변수
int ldr_h = 0, ldr_l = 0;                // 밝기 판별용 플래그
long duration;                           // 초음파 펄스 시간 저장
float distanceCmCurrent = 0;            // 현재 측정된 거리 값 (cm)
bool led1On = false, led2On = false, led3On = false, led4On = false; // 각 버튼 상태
int dutyCycle = 0;                       // 아날로그 입력값 저장용

// ----------- 객체 선언 -----------
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // 네오픽셀 객체 생성
DHT dht(DHTPIN, DHTTYPE);                                           // DHT22 객체 생성
OneWire oneWire(oneWireBus);                                       // OneWire 객체 생성
DallasTemperature sensors(&oneWire);                               // DS18B20 센서 객체 생성
Audio audio;                                                       // 오디오 객체 생성
TinyGPSPlus gps;                                                   // GPS 파서 객체 생성
HardwareSerial neogps(1);                                          // UART1 객체 생성 (GPS용)

// ----------- 함수: 4개 버튼 모두 눌릴 때까지 대기 -----------
void waitForAllButtons() {
  // 버튼을 풀업 입력으로 설정
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);

  // LED 핀 출력 설정 및 초기화
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);

  // 각 버튼이 눌릴 때까지 대기하며 LED 점등
  while (true) {
    if (!led1On && digitalRead(buttonPin1) == LOW) {
      digitalWrite(ledPin1, HIGH);
      led1On = true;
      delay(200); // 디바운싱
    }
    if (!led2On && digitalRead(buttonPin2) == LOW) {
      digitalWrite(ledPin2, HIGH);
      led2On = true;
      delay(200);
    }
    if (!led3On && digitalRead(buttonPin3) == LOW) {
      digitalWrite(ledPin3, HIGH);
      led3On = true;
      delay(200);
    }
    if (!led4On && digitalRead(buttonPin4) == LOW) {
      digitalWrite(ledPin4, HIGH);
      led4On = true;
      delay(200);
    }
    if (led1On && led2On && led3On && led4On) return; // 모두 눌렸으면 함수 종료
  }
}

// ----------- 함수: 딩동 부저음 재생 -----------
void playDingDong() {
  ledcSetup(CHANNEL, FREQ, RESOLUTION);         // PWM 채널 설정
  ledcAttachPin(BUZZER_PIN, CHANNEL);           // 부저 핀을 PWM 채널에 연결

  ledcWrite(CHANNEL, 128); delay(300);          // 딩 소리
  ledcWrite(CHANNEL, 0);   delay(100);
  ledcWriteTone(CHANNEL, 800); delay(400);      // 동 소리
  ledcWrite(CHANNEL, 0);
}

// ----------- 초기 설정 -----------
void setup() {
  Wire.begin();                        // I2C 시작
  Serial.begin(9600);                  // 시리얼 모니터 시작
  for (int i = 0; i < 30; i++) Serial.println("");
  Serial.println("ESR TEST START !!!");

  strip.begin();                       // 네오픽셀 초기화
  strip.setBrightness(10);            // 밝기 설정
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, 0, 0, 0);
  strip.show();

  pinMode(trigPin, OUTPUT);           // 초음파 트리거 핀 설정
  pinMode(echoPin, INPUT);            // 초음파 에코 핀 설정
  pinMode(relay, OUTPUT);             // 릴레이 핀 설정
  pinMode(BUZZER_PIN, OUTPUT);        // 부저 핀 설정

  waitForAllButtons();                // 모든 버튼 입력 대기
  Serial.println("모든 버튼이 눌렸습니다!");
  Serial.println("릴레이 소리가 2번 납니다!");
  // 릴레이 2회 클릭 (테스트용)
  digitalWrite(relay, HIGH); delay(500); digitalWrite(relay, LOW); delay(500);
  digitalWrite(relay, HIGH); delay(500); digitalWrite(relay, LOW);

  // I2C 디바이스 스캔
  int error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("0x"); if (address < 16) Serial.print("0");
      Serial.print(address, HEX); Serial.print(". "); nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x"); if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found\n");
  Serial.println(nDevices);
  delay(3000);
  Serial.println("딩동 소리가 납니다!");
  playDingDong();                     // 부저 테스트

  // GPS 데이터 수신 테스트
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) newData = true;
    }
  }
  if (newData) Serial.println("1. GPS is ok");
  else         Serial.println("1. GPS - check soldering ???");
  delay(200);

  // DHT22 센서 테스트
  dht.begin();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) Serial.println("2. DHT22 - check soldering ???");
  else  Serial.println("2. DHT22 is ok");
  delay(200);

  // DS18B20 온도센서 테스트
  sensors.begin(); delay(100);
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  if ((temperatureC > 5) && (temperatureC < 40)) Serial.println("3. DS18B20 is ok");
  else Serial.println("3. DS18B20 - check wiring ???");
  delay(200);

  // PIR 센서 테스트
  pinMode(PIR, INPUT);
  Serial.print("4. PIR   test -----> "); delay(5000);
  for (int i = 0; i <= 10000; i++) {
    if (digitalRead(PIR)) { Serial.println("ok"); break; }
    delay(100);
    if (i >= 10000) { Serial.println("4. PIR not ok !!!"); break; }
  }
  delay(500);

  // 초음파 센서 거리 측정
  Serial.print("5. HC-SR04  distance test ----->  ");
  for (int i = 0; i <= 10000; i++) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distanceCmCurrent = duration * SOUND_SPEED / 2;
    if (distanceCmCurrent < 1) { Serial.println("not ok"); break; }
    else if ((distanceCmCurrent > 10) && (distanceCmCurrent < 12)) {
      Serial.println("ok"); break;
    }
    delay(100);
    if (i >= 10000) Serial.println("not ok !!!");
  }
  delay(200);

  // 조도센서(LDR) 테스트
  Serial.println("Pls. set the T-L switch to on LDR and enter any key");
  while (Serial.available() > 0) Serial.read();
  while (Serial.available() == 0);
  Serial.print("6. LDR  bright test ----->  ");
  delay(200);
  for (int i = 0; i <= 500; i++) {
    LDR_Val = analogRead(LDR);
    if (LDR_Val > 3500) ldr_h = 1;
    else if (LDR_Val < 1300) ldr_l = 1;
    if ((ldr_h + ldr_l) == 2) { Serial.println("ok"); break; }
    delay(500);
    if (i >= 500) Serial.println("not ok !!!");
  }
  delay(200);

  // 트리머(가변저항) 테스트
  Serial.println("Pls. set the T-L switch to on TRINNER and enter any key");
  while (Serial.available() > 0) Serial.read();
  while (Serial.available() == 0);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  Serial.print("7. Rotate Trimmer  ----->");
  for (int i = 0; i <= 200; i++) {
    dutyCycle = analogRead(ADCPIN);
    if (dutyCycle < 500) digitalWrite(ledPin1, HIGH);
    else if (dutyCycle > 4000) digitalWrite(ledPin2, HIGH);
    if (digitalRead(ledPin1) && digitalRead(ledPin2)) {
      Serial.println("ok"); break;
    }
    delay(100);
    if (i >= 200) Serial.println("7. TRIMER not ok !!!");
  }
  delay(200);

  // 네오픽셀 점등 테스트
  strip.begin();
  Serial.println("8. check Addressable LEDs, Press any key to escape, ");
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, 0, 0, 255);
    strip.show(); delay(300);
  }
  while (Serial.available() > 0) Serial.read();
  while (Serial.available() == 0);

  // SD카드 및 오디오 재생 테스트
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  for (int i = 0; i < 10; i++) {
    if (!SD.begin(SD_CS)) SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  }
  if (i >= 10) Serial.println("9.  check MAX98357, SD Card, speaker wiring ???");
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);
  audio.connecttoFS(SD, "1.mp3");
  Serial.println("9. if no sound, check speaker or SD Card !!!");
}

// ----------- 메인 루프 -----------
void loop() {
  audio.loop(); // 오디오 스트리밍 유지
}
