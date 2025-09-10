#include <Wire.h>                // I2C 통신을 위한 Wire 라이브러리 포함
#include "DHT.h"                 // DHT22 온습도 센서 제어용 라이브러리 포함
#include <OneWire.h>             // DS18B20 온도센서의 1-Wire 통신용 라이브러리 포함
#include <Adafruit_Sensor.h>     // Adafruit 통합 센서 라이브러리 포함
#include <Adafruit_BME280.h>     // BME280 온습도/기압 센서 드라이버 포함
#include <DallasTemperature.h>   // DS18B20 온도센서 제어용 라이브러리 포함
#include <Adafruit_MPU6050.h>    // MPU6050 6축 관성센서 드라이버 포함
#include <Adafruit_SSD1306.h>    // SSD1306 OLED 디스플레이 드라이버 포함
#include <BH1750.h>              // BH1750 디지털 조도센서 라이브러리 포함
#include <Adafruit_NeoPixel.h>   // WS2812B 주소지정 LED 제어용 라이브러리 포함
#include "MAX30105.h"            // MAX30102 심박수/SpO2 센서 라이브러리 포함
#include <TinyGPS++.h>           // GPS 모듈 NMEA 데이터 파싱용 라이브러리 포함
#include <SPI.h>                 // SPI 통신을 위한 라이브러리 포함
#include "FS.h"                  // 파일시스템 접근을 위한 라이브러리 포함
#include "Arduino.h"             // Arduino 기본 함수들을 위한 라이브러리 포함
#include "Audio.h"               // ESP32 I2S 오디오 재생용 라이브러리 포함
#include "SD.h"                  // SD카드 파일시스템 접근용 라이브러리 포함
#include "driver/i2s.h"          // ESP32 I2S 하드웨어 직접 제어용 드라이버 포함

// ==================== 핀 정의 및 설정 ====================
#define ADCPIN 2                 // 아날로그 입력 핀 번호 (트리머 연결용)
#define DHTPIN 21                // DHT22 센서 데이터 핀 번호
#define DHTTYPE DHT22            // DHT 센서 타입 지정 (DHT22 사용)
#define SEALEVELPRESSURE_HPA (1013.25)  // 해수면 기준 기압값 (hPa 단위)

// SPI 및 SD카드 핀 설정
#define SD_CS 10                 // SD카드 칩 셀렉트 핀 번호
#define SPI_MOSI 11              // SPI 마스터 출력, 슬레이브 입력 핀 번호
#define SPI_MISO 13              // SPI 마스터 입력, 슬레이브 출력 핀 번호
#define SPI_SCK 12               // SPI 클럭 신호 핀 번호

// 초음파 센서 및 오디오 설정
#define SOUND_SPEED 0.034        // 음속 상수 (cm/μs 단위, 거리 계산용)
#define I2S_DOUT 14              // I2S 데이터 출력 핀 (스피커 연결용)
#define I2S_BCLK 36              // I2S 비트 클럭 핀 (오디오 동기화용)
#define I2S_LRC 6                // I2S 좌우 채널 선택 핀 (워드 클럭)

// LED 설정
#define LED_PIN 4                // 내장 WS2812B LED 스트립 데이터 핀 번호
#define LED_COUNT 10             // 내장 LED 개수
#define LED_PIN_ex 4             // 외부 WS2812B LED 스트립 데이터 핀 번호
#define LED_COUNT_ex 10          // 외부 LED 개수

// OLED 디스플레이 설정
#define SCREEN_WIDTH 128         // OLED 디스플레이 가로 픽셀 수
#define SCREEN_HEIGHT 64         // OLED 디스플레이 세로 픽셀 수
#define OLED_RESET -1            // OLED 리셋 핀 사용 안함 (-1로 설정)
#define SCREEN_ADDRESS 0x3C      // OLED I2C 주소 (0x3C 또는 0x3D)

// UART 및 센서 핀 설정
#define RXD2 5                   // GPS 모듈 수신용 UART RX 핀 번호
#define TXD2 4                   // GPS 모듈 송신용 UART TX 핀 번호  
#define PIR 47                   // PIR 동작감지 센서 신호 핀 번호
#define LDR 2                    // 조도센서(LDR) 아날로그 입력 핀 번호
#define oneWireBus 7             // DS18B20 온도센서 1-Wire 데이터 핀 번호
#define trigPin 41               // 초음파 센서 트리거 핀 번호
#define echoPin 35               // 초음파 센서 에코 핀 번호

// ==================== 버튼 및 LED 핀 정의 ====================
const int buttonPin1 = 6;       // 첫 번째 푸시버튼 입력 핀 번호
const int buttonPin2 = 1;       // 두 번째 푸시버튼 입력 핀 번호
const int buttonPin3 = 15;      // 세 번째 푸시버튼 입력 핀 번호
const int buttonPin4 = 7;       // 네 번째 푸시버튼 입력 핀 번호
const int ledPin1 = 41;         // 첫 번째 상태표시 LED 출력 핀 번호
const int ledPin2 = 42;         // 두 번째 상태표시 LED 출력 핀 번호
const int ledPin3 = 17;         // 세 번째 상태표시 LED 출력 핀 번호
const int ledPin4 = 4;          // 네 번째 상태표시 LED 출력 핀 번호

// ==================== PWM 및 기타 설정 ====================
const int PWMResolution = 12;   // PWM 해상도 설정 (12비트 = 4096 레벨)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);  // 최대 듀티 사이클 계산
const int motionSensor = 47;    // PIR 동작감지 센서 핀 번호 (PIR과 동일)
const int buzzerPin = 48;       // 부저 출력 핀 번호

// ==================== 전역 변수 및 멜로디 배열 ====================
int dutyCycle = 0;              // PWM 듀티 사이클 저장 변수

// 부저용 멜로디 설정 - 정확한 도레미파솔라시도 주파수 (Hz 단위)
int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};      // 각 음계의 주파수 배열
int noteDurations[] = {4, 4, 4, 4, 4, 4, 4, 4};              // 각 음표의 길이 (1/4박자)

// ==================== 센서 객체 생성 ====================
Adafruit_BME280 bme;           // BME280 온습도/기압 센서 제어 객체 생성
Adafruit_MPU6050 mpu;          // MPU6050 6축 관성센서 제어 객체 생성
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // OLED 디스플레이 객체 생성
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);        // WS2812B LED 스트립 객체 생성
DHT dht(DHTPIN, DHTTYPE);      // DHT22 온습도 센서 객체 생성
OneWire oneWire(oneWireBus);   // DS18B20용 1-Wire 통신 객체 생성
DallasTemperature sensors(&oneWire);  // DS18B20 온도센서 제어 객체 생성
MAX30105 particleSensor;       // MAX30102 심박수/SpO2 센서 객체 생성
BH1750 lightMeter(0x23);       // BH1750 조도센서 객체 생성 (I2C 주소 0x23)
Audio audio;                   // ESP32 I2S 오디오 재생 객체 생성
TinyGPSPlus gps;              // GPS NMEA 데이터 파싱 객체 생성
HardwareSerial neogps(1);     // GPS 모듈용 하드웨어 시리얼 객체 생성 (UART1 사용)

void setup() {
  // ==================== 기본 초기화 ====================
  Wire.begin(8,9);             // I2C 통신 초기화 (SDA=GPIO8, SCL=GPIO9)
  Serial.begin(9600);          // 시리얼 통신 초기화 (9600bps 속도로 USB 연결)

  // 시리얼 모니터 화면 정리를 위한 빈 줄 출력
  for (int i = 0; i < 30; i++) {
    Serial.println("");        // 30줄의 빈 줄을 출력하여 화면 정리
  }
  Serial.println("ESR TEST START !!!");  // 테스트 시작 메시지 출력

  // ==================== WS2812B LED 초기화 ====================
  strip.begin();               // WS2812B LED 스트립 하드웨어 초기화
  strip.show();                // 모든 LED를 꺼진 상태로 업데이트

  // ==================== GPIO 핀모드 설정 ====================
  pinMode(buttonPin1, INPUT_PULLUP);    // 버튼1을 풀업 저항이 활성화된 입력으로 설정
  pinMode(buttonPin2, INPUT_PULLUP);    // 버튼2을 풀업 저항이 활성화된 입력으로 설정
  pinMode(buttonPin3, INPUT_PULLUP);    // 버튼3을 풀업 저항이 활성화된 입력으로 설정
  pinMode(buttonPin4, INPUT_PULLUP);    // 버튼4을 풀업 저항이 활성화된 입력으로 설정
  pinMode(ledPin1, OUTPUT);             // LED1을 디지털 출력으로 설정
  pinMode(ledPin2, OUTPUT);             // LED2를 디지털 출력으로 설정
  pinMode(ledPin3, OUTPUT);             // LED3를 디지털 출력으로 설정
  pinMode(ledPin4, OUTPUT);             // LED4를 디지털 출력으로 설정
  pinMode(buzzerPin, OUTPUT);           // 부저를 디지털 출력으로 설정
  pinMode(motionSensor, INPUT);         // PIR 센서를 디지털 입력으로 설정
  pinMode(trigPin, OUTPUT);             // 초음파 센서 트리거를 출력으로 설정
  pinMode(echoPin, INPUT);              // 초음파 센서 에코를 입력으로 설정

  strip.setBrightness(10);     // WS2812B LED 밝기를 10/255 레벨로 설정 (눈 보호)

  // 모든 상태 LED를 초기 OFF 상태로 설정
  digitalWrite(ledPin1, LOW);  // LED1 끄기
  digitalWrite(ledPin2, LOW);  // LED2 끄기
  digitalWrite(ledPin3, LOW);  // LED3 끄기
  digitalWrite(ledPin4, LOW);  // LED4 끄기

  // ==================== 시작 멜로디 재생 ====================
  const int BUZZER_CHANNEL = 0;        // LEDC 채널 0번 사용
  const int BUZZER_RESOLUTION = 8;     // PWM 해상도 8비트 (256레벨)
  
  // LEDC 하드웨어 초기화 및 부저핀 연결
  ledcSetup(BUZZER_CHANNEL, 1000, BUZZER_RESOLUTION);  // LEDC 채널 설정 (1kHz 기본 주파수)
  ledcAttachPin(buzzerPin, BUZZER_CHANNEL);            // 부저 핀을 LEDC 채널에 연결
  
  // 8개 음표를 순서대로 재생 (도=262Hz ~ 높은도=523Hz)
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote]; // 음표 길이 계산 (ms 단위)
    
    if (melody[thisNote] > 0) {        // 유효한 주파수인 경우
      ledcWriteTone(BUZZER_CHANNEL, melody[thisNote]);  // 해당 주파수로 톤 출력
      delay(noteDuration);             // 음표 길이만큼 대기
      ledcWriteTone(BUZZER_CHANNEL, 0); // 톤 출력 중지
    } else {
      delay(noteDuration);             // 무음 처리 (쉼표)
    }
    
    int pauseBetweenNotes = noteDuration * 0.30;  // 음표 간 간격 계산 (30%)
    delay(pauseBetweenNotes);          // 음표 간 간격 대기
  }
  
  // LEDC 하드웨어 정리 및 부저 완전 정지
  ledcWrite(BUZZER_CHANNEL, 0);        // PWM 출력을 0으로 설정
  ledcDetachPin(buzzerPin);            // 부저 핀을 LEDC 채널에서 분리
  delay(200);                          // 200ms 대기

  // ==================== 01. 버튼 입력 테스트 ====================
  // 버튼1 테스트: 버튼이 눌릴 때까지 무한 대기
  while (1) {
    if (digitalRead(buttonPin1) == LOW) {    // 버튼1이 눌렸는지 확인 (풀업이므로 LOW가 눌림)
      digitalWrite(ledPin1, HIGH);           // 버튼1이 눌리면 LED1 켜기
      break;                                 // while 루프 탈출
    }
  }
  
  // 버튼2 테스트: 버튼이 눌릴 때까지 무한 대기
  while (1) {
    if (digitalRead(buttonPin2) == LOW) {    // 버튼2가 눌렸는지 확인
      digitalWrite(ledPin2, HIGH);           // 버튼2가 눌리면 LED2 켜기
      break;                                 // while 루프 탈출
    }
  }
  
  // 버튼3 테스트: 버튼이 눌릴 때까지 무한 대기
  while (1) {
    if (digitalRead(buttonPin3) == LOW) {    // 버튼3이 눌렸는지 확인
      digitalWrite(ledPin3, HIGH);           // 버튼3이 눌리면 LED3 켜기
      break;                                 // while 루프 탈출
    }
  }
  
  // 버튼4 테스트: 버튼이 눌릴 때까지 무한 대기
  while (1) {
    if (digitalRead(buttonPin4) == LOW) {    // 버튼4가 눌렸는지 확인
      digitalWrite(ledPin4, HIGH);           // 버튼4가 눌리면 LED4 켜기
      break;                                 // while 루프 탈출
    }
  }
  
  Serial.println("01. Buttons are ok");     // 버튼 테스트 완료 메시지 출력
  Serial.println("----------------------------------------------------------------------.");

  // ==================== 02. I2C 주소 스캔 ====================
  int error, address;          // I2C 통신 오류 코드와 주소 저장 변수
  int nDevices = 0;           // 발견된 I2C 디바이스 개수 카운터
  Serial.print("02. I2C address : ");     // I2C 주소 스캔 시작 메시지
  
  // 0x01부터 0x7E까지 모든 I2C 주소에 대해 통신 시도
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);       // 해당 주소로 I2C 통신 시작
    error = Wire.endTransmission();        // 통신 종료 및 오류 코드 확인
    
    if (error == 0) {                      // 통신 성공 (ACK 수신)
      Serial.print("0x");                  // 16진수 표시를 위한 접두사
      if (address < 16) {
        Serial.print("0");                 // 한 자리 수인 경우 앞에 0 추가
      }
      Serial.print(address, HEX);          // 주소를 16진수로 출력
      Serial.print(". ");                  // 구분자 출력
      nDevices++;                          // 발견된 디바이스 개수 증가
    } else if (error == 4) {               // 알 수 없는 오류 발생
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");                 // 한 자리 수인 경우 앞에 0 추가
      }
      Serial.println(address, HEX);        // 오류가 발생한 주소 출력
    }
  }
  
  if (nDevices == 0) {                     // 발견된 디바이스가 없는 경우
    Serial.println("No I2C devices found\n");
  }
  Serial.println();                        // 줄바꿈

  // ==================== 03. BME280 온습도/기압 센서 테스트 ====================
  Wire.beginTransmission(0x76);            // BME280 기본 I2C 주소로 통신 시도
  error = Wire.endTransmission();          // 통신 결과 확인
  if (error == 0) {                        // 통신 성공
    Serial.println("03. BME280 is ok");    // 성공 메시지 출력
  } else {                                 // 통신 실패
    Serial.println("03. BME280 - check soldering ???");  // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 04. BH1750 디지털 조도센서 테스트 ====================
  Wire.beginTransmission(0x23);            // BH1750 기본 I2C 주소로 통신 시도
  error = Wire.endTransmission();          // 통신 결과 확인
  if (error == 0) {                        // 통신 성공
    Serial.println("04. BH1750 is ok");    // 성공 메시지 출력
  } else {                                 // 통신 실패
    Serial.println("04. BH1750 - check soldering ???");  // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 05. MAX30102 심박수/SpO2 센서 테스트 ====================
  Wire.beginTransmission(0x57);            // MAX30102 I2C 주소로 통신 시도
  error = Wire.endTransmission();          // 통신 결과 확인
  if (error == 0) {                        // 통신 성공
    Serial.println("05. MAX30102 is ok");  // 성공 메시지 출력
  } else {                                 // 통신 실패
    Serial.println("05. MAX30102 - check soldering ???");  // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 06. OLED 디스플레이 테스트 ====================
  Wire.beginTransmission(0x3c);            // OLED 기본 I2C 주소로 통신 시도
  error = Wire.endTransmission();          // 통신 결과 확인
  if (error == 0) {                        // 통신 성공
    Serial.println("06. OLED is ok");      // 성공 메시지 출력
  } else {                                 // 통신 실패
    Serial.println("06. OLED - check soldering ???");     // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 07. MPU6050 6축 관성센서 테스트 ====================
  Wire.beginTransmission(0x69);            // MPU6050 대체 I2C 주소로 통신 시도
  error = Wire.endTransmission();          // 통신 결과 확인
  if (error == 0) {                        // 통신 성공
    Serial.println("07. MPU6050 is ok");   // 성공 메시지 출력
  } else {                                 // 통신 실패
    Serial.println("07. MPU6050 - check soldering ???");  // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 08. GPS 모듈 테스트 ====================
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);  // GPS 모듈용 UART 초기화 (9600bps, 8데이터비트, 패리티없음, 1스톱비트)
  bool newData = false;                    // 새로운 GPS 데이터 수신 플래그
  
  // 1초 동안 GPS 데이터 수신 시도
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (neogps.available()) {           // GPS 모듈에서 데이터가 있는지 확인
      if (gps.encode(neogps.read())) {     // NMEA 데이터를 읽고 파싱
        newData = true;                    // 유효한 데이터 수신됨
      }
    }
  }
  
  if (newData) {                           // 유효한 GPS 데이터를 받은 경우
    Serial.println("09. GPS is ok");       // 성공 메시지 출력
  } else {                                 // GPS 데이터를 받지 못한 경우
    Serial.println("09. GPS - check soldering ???");  // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 10. DHT22 온습도센서 테스트 ====================
  dht.begin();                             // DHT22 센서 초기화
  
  float h = dht.readHumidity();            // 습도 값 읽기 (%)
  float t = dht.readTemperature();         // 온도 값 읽기 (°C)
  float f = dht.readTemperature(true);     // 온도 값 읽기 (°F)
  
  // 읽은 값들이 유효한지 확인 (NaN은 Not a Number, 읽기 실패를 의미)
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("10. DHT22 - check soldering ???"));  // 납땜 확인 메시지
  } else {                                 // 모든 값이 유효한 경우
    Serial.println("10. DHT22 is ok");     // 성공 메시지 출력
  }
  delay(200);                              // 200ms 대기
  
  // ==================== 11. PAJ7620 제스처인식 센서 테스트 ====================
  Wire.beginTransmission(0x73);            // PAJ7620 I2C 주소로 통신 시도
  error = Wire.endTransmission();          // 통신 결과 확인
  if (error == 0) {                        // 통신 성공
    Serial.println("11. PAJ7620 is ok");   // 성공 메시지 출력
  } else {                                 // 통신 실패
    Serial.println("11. PAJ7620 - check soldering ???");  // 납땜 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 12. DS18B20 디지털 온도센서 테스트 ====================
  sensors.begin();                         // DS18B20 센서 초기화
  delay(100);                              // 센서 안정화를 위한 100ms 대기
  sensors.requestTemperatures();           // 온도 측정 요청
  float temperatureC = sensors.getTempCByIndex(0);  // 첫 번째 센서의 온도 값 읽기 (°C)
  
  // 온도 값이 합리적인 범위 내에 있는지 확인 (5°C ~ 40°C)
  if (temperatureC > 5 && temperatureC < 40) {
    Serial.println("12. DS18B20 is ok");   // 성공 메시지 출력
  } else {                                 // 온도 값이 비정상적인 경우
    Serial.println("12. DS18B20 - check wiring ???");  // 배선 확인 메시지
  }
  delay(200);                              // 200ms 대기

  // ==================== 13. PIR 수동적외선 동작감지센서 테스트 ====================
  Serial.print("13. PIR   test -----> "); // PIR 테스트 시작 메시지
  delay(3000);                             // PIR 센서 안정화를 위한 3초 대기
  
  // 10초 동안 동작 감지 시도 (100번 * 100ms = 10초)
  for (int i = 0; i <= 100; i++) {
    bool isDetected = digitalRead(motionSensor);  // PIR 센서 상태 읽기
    if (isDetected) {                      // 동작이 감지된 경우
      Serial.println("ok");                // 성공 메시지 출력
      break;                               // for 루프 탈출
    }
    delay(100);                            // 100ms 대기
    if (i >= 100) {                        // 10초 동안 감지되지 않은 경우
      Serial.println("not ok !!!");        // 실패 메시지 출력
      break;                               // for 루프 탈출
    }
  }
  delay(500);                              // 500ms 대기

  // ==================== 14. HC-SR04 초음파 거리센서 테스트 ====================
  Serial.print("14. HC-SR04  distance test ----->  ");  // 초음파 센서 테스트 시작 메시지
  
  // 최대 1000초 동안 거리 측정 시도
  for (int i = 0; i <= 10000; i++) {
    digitalWrite(trigPin, LOW);            // 트리거 핀을 LOW로 설정
    delayMicroseconds(3);                  // 3마이크로초 대기 (신호 안정화)
    digitalWrite(trigPin, HIGH);           // 트리거 핀을 HIGH로 설정 (초음파 발생)
    delayMicroseconds(10);                 // 10마이크로초 대기 (초음파 발생 시간)
    digitalWrite(trigPin, LOW);            // 트리거 핀을 LOW로 설정 (초음파 발생 종료)
    
    long duration = pulseIn(echoPin, HIGH); // 에코 핀에서 HIGH 신호 지속 시간 측정 (마이크로초)
    float distanceCmCurrent = duration * SOUND_SPEED / 2;  // 거리 계산 (왕복 거리를 반으로 나누기)
    
    if (distanceCmCurrent < 1) {           // 거리가 1cm 미만인 경우 (센서 오류)
      Serial.println("change SR04 !!!");   // 센서 교체 메시지
      break;                               // for 루프 탈출
    } else if (distanceCmCurrent > 10 && distanceCmCurrent < 15) {  // 10~15cm 범위의 물체 감지
      Serial.println("ok");                // 성공 메시지 출력
      break;                               // for 루프 탈출
    }
    delay(100);                            // 100ms 대기
    if (i >= 10000) {                      // 최대 시도 횟수 도달
      Serial.println("14. HC-SR04 is not ok");  // 실패 메시지 출력
    }
  }
  delay(200);                              // 200ms 대기

  // ==================== 15. 트리머(가변저항) 테스트 ====================
  digitalWrite(ledPin1, LOW);              // LED1 끄기 (테스트 준비)
  digitalWrite(ledPin2, LOW);              // LED2 끄기 (테스트 준비)
  Serial.print("15. Rotate Trimmer  ----->");  // 트리머 테스트 시작 메시지
  
  // 20초 동안 트리머 회전 테스트 (200번 * 100ms = 20초)
  for (int i = 0; i <= 200; i++) {
    dutyCycle = analogRead(ADCPIN);        // 트리머 값 읽기 (0~4095 범위)
    
    if (dutyCycle < 1000) {                // 트리머가 왼쪽으로 돌려진 경우
      digitalWrite(ledPin1, HIGH);         // LED1 켜기
    } else if (dutyCycle > 3500) {         // 트리머가 오른쪽으로 돌려진 경우
      digitalWrite(ledPin2, HIGH);         // LED2 켜기
    }
    
    // 양쪽 끝까지 모두 돌려서 두 LED가 모두 켜진 경우
    if (digitalRead(ledPin1) && digitalRead(ledPin2)) {
      Serial.println("ok");                // 성공 메시지 출력
      break;                               // for 루프 탈출
    }
    delay(100);                            // 100ms 대기
    if (i >= 200) {                        // 20초 동안 완료되지 않은 경우
      Serial.println(" not ok !!!");       // 실패 메시지 출력
    }
  }
  delay(200);                              // 200ms 대기

  // ==================== 16. WS2812B 주소지정 LED 테스트 ====================
  strip.begin();                           // WS2812B LED 스트립 재초기화
  Serial.println("16. check Addressable LEDs, Press any key to escape, ");  // LED 테스트 메시지

  // 외부 WS2812B 스트립 객체 생성 및 초기화
  Adafruit_NeoPixel strip_ex(LED_COUNT_ex, LED_PIN_ex, NEO_GRB + NEO_KHZ800);
  strip_ex.begin();                        // 외부 LED 스트립 초기화
  strip_ex.setBrightness(40);              // 외부 LED 밝기 설정 (40/255)

  // 외부 LED 스트립의 각 LED를 순차적으로 녹색으로 켜기
  for (int i=0; i<10; i++) {
    strip_ex.setPixelColor(i, 0, 255, 0);  // i번째 LED를 녹색으로 설정 (R=0, G=255, B=0)
    strip_ex.show();                       // LED 업데이트 실행
    delay(100);                            // 100ms 대기 (시각적 효과)
  }

  // 육안 확인을 위한 사용자 입력 대기
  while (Serial.available() > 0) {        // 시리얼 버퍼에 남아있는 데이터 제거
    char t = Serial.read();                // 한 문자씩 읽어서 버퍼 비우기
  }
  while (Serial.available() == 0);        // 사용자가 키를 누를 때까지 대기

  // 내장 LED를 순차적으로 끄기
  for(int i=0; i<LED_COUNT; i++) {
    strip.setPixelColor(i, 0, 0, 0);       // i번째 LED를 검은색으로 설정 (꺼짐)
    strip.show();                          // LED 업데이트 실행
    delay(200);                            // 200ms 대기
  }

  // ==================== 17. INMP441 I2S 마이크 테스트 ====================
  Serial.print("17. INMP441 Microphone test -----> ");  // 마이크 테스트 시작 메시지
  
  // I2S1 포트 설정 구조체
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),    // 마스터 모드, 수신 전용
    .sample_rate = 16000,                  // 샘플링 주파수 16kHz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,         // 32비트 샘플
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // 왼쪽 채널만 사용
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,    // 표준 I2S 포맷
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // 인터럽트 우선순위 레벨1
    .dma_buf_count = 4,                    // DMA 버퍼 개수 4개
    .dma_buf_len = 1024,                   // 각 DMA 버퍼 길이 1024 샘플
    .use_apll = false,                     // APLL 사용하지 않음
    .tx_desc_auto_clear = false,           // 송신 디스크립터 자동 클리어 비활성화
    .fixed_mclk = 0                        // 고정 마스터 클럭 사용하지 않음
  };
  
  // I2S1 핀 설정 구조체 (하드코딩된 핀 번호 사용)
  i2s_pin_config_t pin_config = {
    .bck_io_num = 42,                      // INMP441 CK 핀 (비트 클럭)
    .ws_io_num = 41,                       // INMP441 WS 핀 (워드 선택) 
    .data_out_num = -1,                    // 데이터 출력 사용 안함
    .data_in_num = 38                      // INMP441 SD 핀 (마이크 데이터 입력)
  };
  
  // I2S1 드라이버 설치 및 초기화
  esp_err_t result = i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  if (result != ESP_OK) {                  // 드라이버 설치 실패
    Serial.println("I2S driver install failed");
    Serial.println(" check wiring ???");   // 배선 확인 메시지
  } else {                                 // 드라이버 설치 성공
    // 핀 설정 적용
    i2s_set_pin(I2S_NUM_1, &pin_config);  // I2S 핀 설정 적용
    i2s_zero_dma_buffer(I2S_NUM_1);        // DMA 버퍼 초기화
    
    delay(500);                            // 마이크 안정화를 위한 500ms 대기
    
    int32_t samples[512];                  // 오디오 샘플 저장 배열
    size_t bytes_read;                     // 읽은 바이트 수 저장 변수
    long background_sum = 0;               // 배경 노이즈 합계
    int background_count = 0;              // 배경 노이즈 샘플 개수
    
    // 배경 노이즈 측정 (40번 * 50ms = 2초)
    for (int i = 0; i < 40; i++) {
      i2s_read(I2S_NUM_1, samples, sizeof(samples), &bytes_read, portMAX_DELAY);  // I2S로부터 오디오 데이터 읽기
      
      // 읽은 샘플들의 절댓값 합계 계산
      for (int j = 0; j < bytes_read/4; j++) {
        int32_t sample = samples[j] >> 8;  // 32비트에서 24비트로 변환
        background_sum += abs(sample);     // 절댓값을 합계에 더하기
        background_count++;                // 샘플 개수 증가
      }
      delay(50);                           // 50ms 대기
    }
    
    // 배경 노이즈 평균 레벨 계산 및 임계값 설정
    long background_level = background_sum / background_count;  // 평균 배경 노이즈 레벨
    long threshold = background_level * 3; // 임계값을 배경 노이즈의 3배로 설정
    
    // 음성 감지 테스트 (200번 * 50ms = 10초)
    bool voice_detected = false;           // 음성 감지 플래그
    
    for (int attempt = 0; attempt < 200; attempt++) {
      i2s_read(I2S_NUM_1, samples, sizeof(samples), &bytes_read, portMAX_DELAY);  // 오디오 데이터 읽기
      
      long current_sum = 0;                // 현재 샘플들의 합계
      int current_count = 0;               // 현재 샘플 개수
      
      // 현재 읽은 샘플들의 평균 레벨 계산
      for (int j = 0; j < bytes_read/4; j++) {
        int32_t sample = samples[j] >> 8;  // 32비트에서 24비트로 변환
        current_sum += abs(sample);        // 절댓값을 합계에 더하기
        current_count++;                   // 샘플 개수 증가
      }
      
      long current_level = current_sum / current_count;  // 현재 평균 레벨
      
      if (current_level > threshold) {     // 현재 레벨이 임계값을 초과하는 경우
        voice_detected = true;             // 음성 감지됨
        break;                             // for 루프 탈출
      }
      
      delay(50);                           // 50ms 대기
      
      if (attempt >= 199) {                // 10초 동안 음성이 감지되지 않은 경우
        Serial.println("not ok !!!");      // 실패 메시지 출력
        break;                             // for 루프 탈출
      }
    }
    // I2S 하드웨어 정리
    i2s_driver_uninstall(I2S_NUM_1);      // I2S 드라이버 제거
    // 결과 출력
    if (voice_detected) {                  // 음성이 감지된 경우
      Serial.println("ok");                // 성공 메시지 출력
    } 
  }
  delay(200);                              // 200ms 대기

  // ==================== 18. SD카드 및 오디오 시스템 테스트 ====================
  SPIClass spi = SPIClass(HSPI);           // HSPI(SPI2) 객체 생성
  pinMode(SD_CS, OUTPUT);                  // SD카드 칩 셀렉트 핀을 출력으로 설정
  digitalWrite(SD_CS, HIGH);               // SD카드 칩 셀렉트를 HIGH로 설정 (비활성화)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // SPI 통신 초기화 (클럭, MISO, MOSI 핀 설정)

  int sd_retry = 0;                        // SD카드 초기화 재시도 카운터
  // SD카드 초기화를 최대 10번 시도
  for (sd_retry = 0; sd_retry < 10; sd_retry++) {
    if (SD.begin(SD_CS)) {                 // SD카드 초기화 시도
      break;                               // 성공시 for 루프 탈출
    }
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // SPI 재초기화
    delay(100);                            // 100ms 대기 후 재시도
  }

  if (sd_retry >= 10) {                    // 10번 시도 후에도 실패한 경우
    Serial.println("18.  check MAX98357, SD Card or speaker wiring ???");  // 배선 확인 메시지
  } else {                                 // SD카드 초기화 성공
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);  // 오디오 출력 핀 설정
    audio.setVolume(21);                   // 오디오 볼륨 설정 (0~21 범위)
    audio.connecttoFS(SD, "1.mp3");        // SD카드의 "1.mp3" 파일 재생 시작
    Serial.println("18. if no sound, check speaker or SD Card !!!");  // 사운드 확인 메시지
  }

} // setup() 함수 끝

// ==================== 메인 루프 ====================
void loop() {
  audio.loop();                            // I2S 오디오 스트리밍 지속적 처리 (논블로킹)
}
