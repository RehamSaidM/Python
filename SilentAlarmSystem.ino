// RehamS. 16s18106


#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <WiFi101.h>
#include <Keypad.h>
#include <DueFlashStorage.h>
#include <Crypto.h>

#define FIRMWARE_VERSION 3

LiquidCrystal_I2C lcd(0x27, 16, 2);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
DueFlashStorage dueFlash;

char daysOfTheWeek[7][12] = {
  "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
};

float volt, voltage;
int percent;
int lastEntS = 0;
int lastEntM = 0;
int lastEntH = 0;
unsigned long previousMillis = 0;
unsigned long blinkMillis = 0;
String password = "1A4B7C";
bool authenticated = false;
const String AUTH_KEY = "mySecretKey123";
String number = "+96894822755";

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {44, 45, 46, 47};
byte colPins[COLS] = {48, 49, 50, 51};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const int pirPin = 12;
const int ledPin = 2;
const int echoPin = 3;
const int trigPin = 4;
const int usledPin = 7;
const int doorPin = 8;

#define mywifi Serial2
#define myGSM Serial3

Servo myServo;
DateTime now;
RTC_DS1307 rtc;
unsigned long sms_delay;


void setup() {
  
  Serial.begin(9600);
  myGSM.begin(9600);
  mywifi.begin(115200);
  while (!Serial);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");

  int attempts = 0;
  String enteredPassword = "";
  String resetCombo = "";

  while (!authenticated && attempts < 3) {
    char key = keypad.getKey();
    if (key) {
      if (key == '#') {
        if (enteredPassword == password) {
          lcd.clear();
          lcd.print("Access Granted");
          Serial.println("Access Granted.");
          authenticated = true;
          delay(2000);
          lcd.clear();
        } else {
          attempts++;
          lcd.clear();
          lcd.print("Wrong Password");
          Serial.println("Access Denied. Try again:");
          delay(2000);
          if (attempts < 3) {
            lcd.clear();
            lcd.print("Enter Password:");
            enteredPassword = "";
          }
        }
      }
      else if (key == '*') {
        enteredPassword = "";
        lcd.clear();
        lcd.print("Enter Password:");
      }
      else {
        enteredPassword += key;
        lcd.setCursor(0,1);
        lcd.print(enteredPassword);
      }
    }
  }

  if (!authenticated) {
    lcd.clear();
    lcd.print("System Locked!");
    Serial.println("System Locked! Too many wrong attempts.");

    SendLockAlert();
    resetCombo = "";

    while (true) {
      char resetKey = keypad.getKey();
      if (resetKey) {
        resetCombo += resetKey;
        if (resetCombo.length() > 5) {
          resetCombo = resetCombo.substring(resetCombo.length() - 5);
        }

        if (resetCombo == "A6B9C") {
          Serial.println("Reset Combo Entered! Resetting...");
          lcd.clear();
          lcd.print("Resetting...");
          delay(1000);
          NVIC_SystemReset();
        }
      }
    }
  }

  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(usledPin, OUTPUT);

  myServo.attach(doorPin);
  myServo.write(0);
  checkRollbackProtection();
  if (!rtc.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("RTC Error!");
    while (1);
  }

  u8g2.begin();
}

long measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  long distance = duration * 0.034 / 2;
  return distance;
}

void loop() {
  int motionDetected = digitalRead(pirPin);
  long distance = measureDistance();

  digitalWrite(usledPin, distance < 50 ? HIGH : LOW);
  digitalWrite(ledPin, motionDetected ? HIGH : LOW);

  now = rtc.now();
  show_data();

  if (distance < 50 || motionDetected) {
    myServo.write(90);
    SendMessage();
    if (millis() - sms_delay > 10000) {
      SendMessage();
      sms_delay = millis();
    }
    lastEntH = now.hour();
    lastEntM = now.minute();
    lastEntS = now.second();
  }

  read_volt();

  Serial.print("Voltage = "); Serial.print(voltage);
  Serial.print(" V  Percent = "); Serial.print(percent);
  Serial.print(" %  Distance: "); Serial.print(distance);
  Serial.print("  motion: "); Serial.println(motionDetected);

  delay(1000);
}

void read_volt() {
  volt = analogRead(A0);
  voltage = volt * (3.3 / 1023.0) * 5.1;
  percent = map(voltage * 100, 0, 833, 0, 100);

  u8g2.firstPage();
  do {
    displayVoltage();
  } while (u8g2.nextPage());
}

void show_data() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  lcd.setCursor(6, 0);
  lcd.print(now.hour()); lcd.print(":");
  lcd.print(now.minute()); lcd.print(":");
  lcd.print(now.second());

  lcd.setCursor(0, 1);
  lcd.print("LstEnt:");
  lcd.setCursor(7, 1);
  lcd.print(lastEntH); lcd.print(":");
  lcd.print(lastEntM); lcd.print(":");
  lcd.print(lastEntS);
}

void displayVoltage() {
  u8g2.setFont(u8g2_font_6x13_tf);
  String voltageStr = String(voltage, 2);
  u8g2.drawStr(5, 20, "Voltage:");
  u8g2.drawStr(80, 20, voltageStr.c_str());
  u8g2.drawStr(110, 20, "V");

  char percentStr[5];
  itoa(percent, percentStr, 10);
  u8g2.drawStr(5, 40, "Percent:");
  u8g2.drawStr(80, 40, percentStr);
  u8g2.drawStr(110, 40, "%");
}

void SendMessage() {
  myGSM.println("AT+CMGF=1");
  delay(1000);
  myGSM.println("AT+CMGS=\"" + number + "\"\r");
  delay(1000);
  Serial.println(" MOTION DETECTED! ");
  myGSM.print("Motion Detected at ");
  myGSM.print(lastEntH); myGSM.print(":");
  myGSM.print(lastEntM); myGSM.print(":");
  myGSM.print(lastEntS);
  delay(100);
  myGSM.println((char)26);
  delay(1000);
}

void SendLockAlert() {
  myGSM.println("AT+CMGF=1");
  delay(1000);
  myGSM.println("AT+CMGS=\"" + number + "\"\r");
  delay(1000);
  myGSM.print("Unauthorized access, system locked.");
  delay(100);
  myGSM.println((char)26);
  delay(1000);
  Serial.println("Lock alert SMS sent.");
}


void checkRollbackProtection() {
  byte storedVersion = dueFlash.read(0);

  Serial.print("Stored Firmware Version: ");
  Serial.println(storedVersion);
  Serial.print("Current Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);

  if (FIRMWARE_VERSION < storedVersion) {
    Serial.println("\u26a0\ufe0f ROLLBACK ATTEMPT DETECTED. SYSTEM HALTED.");
    while (true);
  }

  if (FIRMWARE_VERSION > storedVersion) {
    Serial.println("New version detected. Writing to flash.");
    dueFlash.write(0, (byte)FIRMWARE_VERSION);
  }
}
