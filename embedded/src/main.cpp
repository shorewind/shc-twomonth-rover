#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>

#define WIRE_SDA 0  // Use GP0 as I2C0 SDA
#define WIRE_SCL 1  // Use GP1 as I2C0 SCL
arduino::MbedI2C Wire_(WIRE_SDA, WIRE_SCL);

#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

#define PWM_LF 4
#define PWM_RF 5
#define PWM_LB 6
#define PWM_RB 7

#define LED_PIN 25
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;

void setup() {
  // Initialize outputs
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_LF, OUTPUT);
  pinMode(PWM_RF, OUTPUT);
  pinMode(PWM_LB, OUTPUT);
  pinMode(PWM_RB, OUTPUT);
  // Turn LED on for initialization
  digitalWrite(LED_PIN, HIGH);

  // Configure serial transport
  Serial.begin(115200);
  delay(100);

  analogWrite(PWM_LF, 0);
  analogWrite(PWM_RF, 0);
  analogWrite(PWM_LB, 0);
  analogWrite(PWM_RB, 0);

  // Turn LED off after serial initialization
  // digitalWrite(LED_PIN, LOW);

  if (!bmp.begin_I2C(0x77, &Wire_)) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }

  if (!sox.begin_I2C(0x6A, &Wire_)) {
    Serial.println("Failed to find LSM6DSOX chip");
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
}

int last_read = 0;  // declare and initialize time of last read

void loop() {
  if (millis() - last_read > 500) {  // automated data collection
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      delay(100);
      return;
    }

    Serial.print(millis() / 1000.0);
    Serial.print(",");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print(",");
    Serial.print(bmp.temperature);
    Serial.print(",");
    Serial.print(bmp.pressure / 100.0);
    Serial.print(",");

    // Get a new normalized sensor event
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);

    Serial.print(accel.acceleration.x);
    Serial.print(","); Serial.print(accel.acceleration.y);
    Serial.print(","); Serial.print(accel.acceleration.z);
    Serial.println();

    last_read = millis();
  }
  
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "ping") {
      Serial.println("pong");
    }
    else if (command == "forward") {
      Serial.println("move forward");
      Serial.println("motors on");
      analogWrite(PWM_LF, 255);
      analogWrite(PWM_RF, 255);
      // delay(1000);
    }
    else if (command == "off") {
      Serial.println("motors off");
      analogWrite(PWM_LF, 0);
      analogWrite(PWM_RF, 0);
    }
    else if (command == "backward") {
      Serial.println("move backward");
    }
    else if (command == "left") {
      Serial.println("turn left");
      Serial.println("motors on");
      analogWrite(PWM_LF, 255);
      delay(1000);
      Serial.println("motors off");
      analogWrite(PWM_LF, 0);
    }
    else if (command == "right") {
      Serial.println("turn right");
      Serial.println("motors on");
      analogWrite(PWM_RF, 255);
      delay(1000);
      Serial.println("motors off");
      analogWrite(PWM_RF, 0);
    }
    else if (command == "extend") {
      Serial.println("extend arm");
    }
    else if (command == "retract") {
      Serial.println("retract arm");
    }
  }
}
