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

#define PWM_1L 2  // PWM_1 = forward
#define PWM_1R 3
#define PWM_2L 4  // PWM_2 = backward
#define PWM_2R 5

#define LED_PIN 25
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;

void setup() {
  // Initialize outputs
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_1L, OUTPUT);
  pinMode(PWM_1R, OUTPUT);
  pinMode(PWM_2L, OUTPUT);
  pinMode(PWM_2R, OUTPUT);
  // Turn LED on for initialization
  digitalWrite(LED_PIN, HIGH);

  // Configure serial transport
  Serial.begin(115200);
  delay(100);

  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, LOW);

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
      digitalWrite(PWM_1L, HIGH);
      digitalWrite(PWM_1R, HIGH);
      digitalWrite(PWM_2L, LOW);
      digitalWrite(PWM_2R, LOW);
    }
    else if (command == "backward") {
      Serial.println("move backward");
      Serial.println("motors on");
      digitalWrite(PWM_1L, LOW);
      digitalWrite(PWM_1R, LOW);
      digitalWrite(PWM_2L, HIGH);
      digitalWrite(PWM_2R, HIGH);
    }
    else if (command == "left") {
      Serial.println("turn left");
      Serial.println("motors on");
      digitalWrite(PWM_1L, LOW);
      digitalWrite(PWM_1R, HIGH);
      digitalWrite(PWM_2L, HIGH);
      digitalWrite(PWM_2R, LOW);
    }
    else if (command == "right") {
      Serial.println("turn right");
      Serial.println("motors on");
      digitalWrite(PWM_1L, HIGH);
      digitalWrite(PWM_1R, LOW);
      digitalWrite(PWM_2L, LOW);
      digitalWrite(PWM_2R, HIGH);
    }
    else if (command == "extend") {
      Serial.println("extend arm");
    }
    else if (command == "retract") {
      Serial.println("retract arm");
    }
    else if (command == "halt") {
      Serial.println("halt");
      Serial.println("motors off");
      digitalWrite(PWM_1L, LOW);
      digitalWrite(PWM_1R, LOW);
      digitalWrite(PWM_2L, LOW);
      digitalWrite(PWM_2R, LOW);
    }
    else if (command == "auto") {
      Serial.println("begin autonomous mode");
    }
  }
}
