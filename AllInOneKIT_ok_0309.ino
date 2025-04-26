// ---------- 라이브러리 포함 ----------
#include "DHT.h"               // DHT22 온습도 센서용 라이브러리
#include <Wire.h>              // I2C 통신용 라이브러리
#include <OneWire.h>           // DS18B20 온도 센서 OneWire 통신용
#include <DallasTemperature.h> // DS18B20 온도 센서 라이브러리
#include <Adafruit_NeoPixel.h> // WS2812B 네오픽셀 제어용 라이브러리
#include <TinyGPS++.h>         // GPS 파싱 라이브러리
#include <SPI.h>               // SPI 통신용 라이브러리
#include "FS.h"                // 파일 시스템 라이브러리 (SD 카드용)
#include "Arduino.h"           // Arduino 기본 API
#include "Audio.h"             // MAX98357 오디오 출력용 라이브러리
#include "SD.h"                // SD 카드 라이브러리

// ---------- 핀 번호 및 상수 정의 ----------
#define ADCPIN                2    // 아날로그 입력 핀 (트리머 연결)
#define DHTPIN                21   // DHT22 데이터 핀
#define DHTTYPE               DHT22 // DHT 센서 타입 설정
#define SD_CS                 10   // SD 카드 CS 핀
#define SPI_MOSI              11   // SPI MOSI 핀
#define SPI_MISO              13   // SPI MISO 핀
#define SPI_SCK               12   // SPI 클럭 핀
#define SOUND_SPEED           0.034 // 초음파 속도 (cm/us)
#define I2S_DOUT              14   // I2S 오디오 데이터 출력 핀
#define I2S_BCLK              41   // I2S 비트 클럭 핀
#define I2S_LRC               42   // I2S 채널 클럭 핀
#define LED_PIN               4    // 네오픽셀 LED 제어 핀
#define LED_COUNT             10   // 네오픽셀 LED 개수
#define RXD2                  5    // GPS 수신 핀
#define TXD2                  4    // GPS 송신 핀
#define PIR                   47   // PIR 인체 감지 센서 핀
#define LDR                   2    // 광센서 (LDR) 핀
#define oneWireBus            7    // DS18B20 데이터 통신용 핀
#define relay                 37   // 릴레이 제어 핀
#define BUZZER_PIN            48   // 부저 출력 핀
#define CHANNEL               0    // PWM 채널 번호
#define FREQ                  2000 // PWM 주파수 (2kHz)
#define RESOLUTION            8    // PWM 해상도 (8비트)

// ---------- 버튼과 LED 핀 정의 ----------
const int buttonPin1 = 6;
const int buttonPin2 = 1;
const int buttonPin3 = 15;
const int buttonPin4 = 7;
const int ledPin1 = 41;
const int ledPin2 = 42;
const int ledPin3 = 17;
const int ledPin4 = 4;

// ---------- 초음파 센서 핀 정의 ----------
const int trigPin = 41;
const int echoPin = 35;

// ---------- 전역 변수 선언 ----------
bool newData;
int i = 0;
int LDR_Val = 0;
int ldr_h = 0;
int ldr_l = 0;
long duration;
float distanceCmCurrent = 0;
bool led1On = false, led2On = false, led3On = false, led4On = false;
int dutyCycle = 0;

// ---------- 객체 생성 ----------
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // 네오픽셀 객체
DHT dht(DHTPIN, DHTTYPE);                                          // DHT22 객체
OneWire oneWire(oneWireBus);                                       // OneWire 객체
DallasTemperature sensors(&oneWire);                               // DS18B20 센서 객체
Audio audio;                                                       // 오디오 객체
TinyGPSPlus gps;                                                   // GPS 객체
HardwareSerial neogps(1);                                          // 하드웨어 UART1 사용 GPS용

// ---------- 모든 버튼이 눌릴 때까지 대기 ----------
void waitForAllButtons() {
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);

  while (true) {
    if (!led1On && digitalRead(buttonPin1) == LOW) {
      digitalWrite(ledPin1, HIGH);
      led1On = true;
      delay(200);
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
    if (led1On && led2On && led3On && led4On) {
      return;
    }
  }
}

// ---------- 부저로 딩동 소리 재생 ----------
void playDingDong() {
  ledcSetup(CHANNEL, FREQ, RESOLUTION); // PWM 채널 설정
  ledcAttachPin(BUZZER_PIN, CHANNEL);    // 부저 핀을 PWM 채널에 연결
  ledcWrite(CHANNEL, 128);               // 50% duty로 소리 출력

  tone(BUZZER_PIN, 1000); // 1kHz 소리
  delay(300);             // 0.3초 동안 재생
  noTone(BUZZER_PIN);     // 소리 끄기
  delay(100);

  tone(BUZZER_PIN, 800);  // 800Hz 소리
  delay(400);             // 0.4초 동안 재생
  noTone(BUZZER_PIN);     // 소리 끄기
}

// ---------- 초기화 코드 ----------
void setup() {
  Wire.begin();                 // I2C 초기화
  Serial.begin(9600);            // 시리얼 통신 시작
  for (int i = 0; i < 30; i++) Serial.println(""); // 화면 클리어용
  Serial.println("ESR TEST START !!!");

  strip.begin();                 // 네오픽셀 초기화
  strip.setBrightness(10);        // LED 밝기 설정
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, 0, 0, 0); // LED 끄기
  strip.show();

  pinMode(trigPin, OUTPUT);       // 초음파 트리거 핀 출력 설정
  pinMode(echoPin, INPUT);        // 초음파 에코 핀 입력 설정
  pinMode(relay, OUTPUT);         // 릴레이 출력 설정
  pinMode(BUZZER_PIN, OUTPUT);    // 부저 출력 설정

  waitForAllButtons();            // 모든 버튼 눌릴 때까지 대기
  Serial.println("모든 버튼이 눌렸습니다.");
  Serial.println("릴레이 소리가 2번 들려 합니다.");
  digitalWrite(relay, HIGH); delay(500); digitalWrite(relay, LOW); delay(500);
  digitalWrite(relay, HIGH); delay(500); digitalWrite(relay, LOW);

  // I2C 디바이스 스캔
  int error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(". ");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found\n");
  Serial.println(nDevices);
  delay(3000);

  playDingDong(); // 부저로 딩동 소리
  Serial.println("부저에서 딩동 소리가 들려야 합니다.");

  // GPS 초기화 및 데이터 수신 확인
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) newData = true;
    }
  }
  if (newData) {
    Serial.println("1. GPS is ok");
  } else {
    Serial.println("1. GPS - check soldering ???");
  }
  delay(200);

  // DHT22 온습도 센서 초기화
  dht.begin();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("2. DHT22 - check soldering ???");
  } else {
    Serial.println("2. DHT22 is ok");
  }
  delay(200);

  // DS18B20 온도 센서 확인
  sensors.begin();
  delay(100);
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  if ((temperatureC > 5) && (temperatureC < 40)) {
    Serial.println("3. DS18B20 is ok");
  } else {
    Serial.println("3. DS18B20 - check wiring ???");
  }
  delay(200);

  // PIR 인체 감지 테스트
  pinMode(PIR, INPUT);
  Serial.print("4. PIR   test -----> ");
  delay(5000);
  for (int i = 0; i <= 10000; i++) {
    if (digitalRead(PIR)) {
      Serial.println("ok");
      break;
    }
    delay(100);
    if (i >= 10000) {
      Serial.println("4. PIR not ok !!!");
      break;
    }
  }
  delay(500);

  // 초음파 거리 측정 테스트
  Serial.print("5. HC-SR04  distance test ----->  ");
  for (int i = 0; i <= 10000; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distanceCmCurrent = duration * SOUND_SPEED / 2;
    if (distanceCmCurrent < 1) {
      Serial.println("not ok");
      break;
    }
    else if ((distanceCmCurrent > 10) && (distanceCmCurrent < 12)) {
      Serial.println("ok");
      break;
    }
    delay(100);
    if (i >= 10000) Serial.println("not ok !!!");
  }
  delay(200);

  // LDR 센서 테스트
  Serial.println("Pls. set the T-L switch to on LDR and enter any key");
  while (Serial.available() > 0) { Serial.read(); }
  while (Serial.available() == 0);

  Serial.print("6. LDR  bright test ----->  ");
  delay(200);
  for (int i = 0; i <= 500; i++) {
    LDR_Val = analogRead(LDR);
//    Serial.print(LDR_Val); // 환경에 따라 조정하세요
    if (LDR_Val > 3000) ldr_h = 1;
    else if (LDR_Val < 2500) ldr_l = 1;
    if ((ldr_h + ldr_l) == 2) {
      Serial.println("ok");
      break;
    }
    delay(500);
    if (i >= 500) Serial.println("not ok !!!");
  }
  delay(200);

  // 트리머 테스트
  Serial.println("Pls. set the T-L switch to on TRIMMER and enter any key");
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
      Serial.println("ok");
      break;
    }
    delay(100);
    if (i >= 200) Serial.println("7. TRIMMER not ok !!!");
  }
  delay(200);

  // 네오픽셀 LED 점등 테스트
  strip.begin();
  Serial.begin(9600);

  Serial.println("8. check Addressable LEDs, Press any key to escape, ");
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, 0, 0, 255);
    strip.show();
    delay(300);
  }
  while (Serial.available() > 0) Serial.read();
  while (Serial.available() == 0);

  // SD 카드 및 오디오 설정
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  for (int i = 0; i < 10; i++) {
    if (!SD.begin(SD_CS)) SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  }

  if (i >= 10) Serial.println("9. check MAX98357, SD Card, speaker wiring ???");
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);
  audio.connecttoFS(SD, "1.mp3");
  Serial.println("9. if no sound, check speaker or SD Card !!!");
}

// ---------- 메인 루프 ----------
void loop() {
  audio.loop(); // 오디오 재생 처리
}
