#include <Wire.h>                    // Library for I2C communication
#include <Adafruit_MPU6050.h>       // Library for MPU6050 accelerometer/gyroscope
#include <Adafruit_Sensor.h>        // Unified sensor library
#include <math.h>                   // Library for mathematical operations
#include <SPI.h>                    // Library for SPI communication
#include <MFRC522.h>                // Library for RFID module
#include <Servo.h>                  // Library to control servo motors
#include <LiquidCrystal_I2C.h>      // Library for I2C LCD
#include <WiFi.h>                   // Library for WiFi functions
#include <ArduinoHttpClient.h>      // Library for HTTP client

// Pin definitions
#define RST_PIN 6
#define SS_PIN 5
#define SERVO_PIN 10
#define BUTTON_PIN 2
#define BUZZER_PIN 7
#define MOTION_SENSOR_PIN 3
#define ULTRASONIC_TRIGGER_PIN 8
#define ULTRASONIC_ECHO_PIN 9
#define LED_PIN A0

// Initialize components
Adafruit_MPU6050 mpu;
MFRC522 rfid(SS_PIN, RST_PIN);
Servo servo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Threshold for motion detection
#define MOTION_THRESHOLD 2.0
bool buzzerOn = false;
bool isServoAt128 = false;
bool buttonWasOff = false;

// Predefined RFID UIDs (authorized cards)
const byte verifiedUIDs[3][4] = {
    {0x17, 0xEE, 0x44, 0xB5},
    {0xF3, 0x17, 0xAF, 0x2A},
    {0xCA, 0x02, 0x01, 0x81}
};

// Variables for ultrasonic sensor
long duration;
int distance;

// Motion timeout duration
unsigned long motionDetectedTime = 0;
unsigned long motionTimeout = 30000;

// WiFi and ThingSpeak configurations
const char* ssid = "The_Myriad_Dubai";
const char* password = "123$$123";
const char* serverAddress = "api.thingspeak.com";
int port = 80;
const char* writeAPIKey = "GDDU44K5TK644FKM";
WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize RFID module
  SPI.begin();
  rfid.PCD_Init();

  // Initialize servo and set initial position
  servo.attach(SERVO_PIN);
  servo.write(90);

  // Configure pins
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initialize LCD
  lcd.init();
  lcd.setBacklight(1);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);
  int servoPosition = servo.read();

  // Check for new RFID card and verify it
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    if (isCardVerified(rfid.uid.uidByte, rfid.uid.size)) {
      buzzerOn = false;
      digitalWrite(BUZZER_PIN, LOW);
      moveServoTo128();
      sendDataToThingSpeak(0, 0);
    } else {
      Serial.println("Unverified Card");
    }
    rfid.PICC_HaltA();
  }

  // Handle servo reset based on button state
  if (isServoAt128) {
    if (buttonState == LOW) {
      buttonWasOff = true;
    } else if (buttonWasOff && buttonState == HIGH) {
      delay(2000);
      servo.write(90);
      isServoAt128 = false;
      buttonWasOff = false;
      sendDataToThingSpeak(0, 0);
    }
  }

  // Activate buzzer if button is pressed and servo is at default position
  if (buttonState == LOW && servoPosition == 90) {
    digitalWrite(BUZZER_PIN, HIGH);
    while (true) {
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        if (isCardVerified(rfid.uid.uidByte, rfid.uid.size)) {
          digitalWrite(BUZZER_PIN, LOW);
          moveServoTo128();
          sendDataToThingSpeak(0, 1);
          break;
        }
        rfid.PICC_HaltA();
      }
    }
  }

  // Detect motion using MPU6050
  if (buttonState == HIGH && servoPosition == 90) {
    detectMotion();
  }

  // Handle motion sensor LED timeout
  if (digitalRead(MOTION_SENSOR_PIN) == HIGH) {
    digitalWrite(LED_PIN, HIGH);
    motionDetectedTime = millis();
  }
  if (millis() - motionDetectedTime > motionTimeout) {
    digitalWrite(LED_PIN, LOW);
  }

  // Measure distance using ultrasonic sensor
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  handleUltrasonicSensor(distance);
}

// Detect motion with MPU6050 and trigger buzzer if needed
void detectMotion() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float correctedZ = accel.acceleration.z - 9.8;
  float totalAccel = sqrt(
      accel.acceleration.x * accel.acceleration.x +
      accel.acceleration.y * accel.acceleration.y +
      correctedZ * correctedZ
  );

  if (totalAccel > MOTION_THRESHOLD && !buzzerOn) {
    buzzerOn = true;
    digitalWrite(BUZZER_PIN, HIGH);

    unsigned long startTime = millis();
    while (millis() - startTime < 10000 && buzzerOn) {
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        if (isCardVerified(rfid.uid.uidByte, rfid.uid.size)) {
          buzzerOn = false;
          digitalWrite(BUZZER_PIN, LOW);
          sendDataToThingSpeak(1, 0);
        }
        rfid.PICC_HaltA();
      }
    }

    if (buzzerOn) {
      buzzerOn = false;
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

// Verify RFID card UID
bool isCardVerified(byte *uid, byte uidSize) {
  for (int i = 0; i < 3; i++) {
    bool match = true;
    for (int j = 0; j < uidSize; j++) {
      if (verifiedUIDs[i][j] != uid[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      return true;
    }
  }
  return false;
}

// Move servo to position 128
void moveServoTo128() {
  servo.write(128);
  isServoAt128 = true;
  lcd.clear();
  lcd.print("Access Granted");
  delay(2000);
  lcd.clear();
}

// Handle ultrasonic sensor logic
void handleUltrasonicSensor(int distance) {
   if (distance <= 100 && distance > 40) {
    lcd.clear();
    lcd.print("WELCOME");
    delay(1000);
  } else if (distance <= 40 && distance > 20) {
    lcd.clear();
    lcd.print("WARNING");
    lcd.setCursor(0, 1);
    lcd.print("TOO CLOSE");
  } else if (distance <= 20) {
    lcd.clear();
    lcd.print("WARNING");
    lcd.setCursor(0, 1);
    lcd.print("TOO CLOSE");
    tone(BUZZER_PIN, 1000);
    delay(1000);
    noTone(BUZZER_PIN);
  } else {
    lcd.clear();
    lcd.print("GOODBYE");
    delay(1000);
  }
}

// Send data to ThingSpeak
void sendDataToThingSpeak(int motionStatus, int buttonPressStatus) {
  if (WiFi.status() == WL_CONNECTED) {
    client.beginRequest();
    client.post("/update");
    client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
    client.sendHeader("Content-Length", 40);
    client.beginBody();
    client.print("api_key=");
    client.print(writeAPIKey);
    client.print("&field1=");
    client.print(motionStatus);
    client.print("&field2=");
    client.print(buttonPressStatus);
    client.endRequest();

    int statusCode = client.responseStatusCode();
    String response = client.responseBody();
    Serial.println("Status Code: " + String(statusCode));
    Serial.println("Response: " + response);
  } else {
    Serial.println("Wi-Fi not connected.");
  }
}
