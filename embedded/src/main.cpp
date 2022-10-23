#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
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

#define BUILTIN_LED 25
#define AUTO_LED 16
#define MANUAL_LED 17
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;
Servo armServo;

void setup() {
  // Initialize outputs
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(AUTO_LED, OUTPUT);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(PWM_1L, OUTPUT);
  pinMode(PWM_1R, OUTPUT);
  pinMode(PWM_2L, OUTPUT);
  pinMode(PWM_2R, OUTPUT);

  armServo.attach(22);

  // Turn LED on for initialization
  digitalWrite(BUILTIN_LED, HIGH);

  // Configure serial transport
  Serial.begin(115200);
  delay(100);

  // Manual mode LED on
  digitalWrite(MANUAL_LED, HIGH);

  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, LOW);

  // Turn LED off after serial initialization
  digitalWrite(BUILTIN_LED, LOW);

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

void forward() {
  Serial.println("move forward");
  Serial.println("motors on");
  digitalWrite(PWM_1L, HIGH);
  digitalWrite(PWM_1R, HIGH);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, LOW);
}

void backward() {
  Serial.println("move backward");
  Serial.println("motors on");
  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, HIGH);
  digitalWrite(PWM_2R, HIGH);
}

void left() {
  Serial.println("turn left");
  Serial.println("motors on");
  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, HIGH);
  digitalWrite(PWM_2L, HIGH);
  digitalWrite(PWM_2R, LOW);
}

void right() {
  Serial.println("turn right");
  Serial.println("motors on");
  digitalWrite(PWM_1L, HIGH);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, HIGH);
}

void halt() {
  Serial.println("halt");
  Serial.println("motors off");
  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, LOW);
  armServo.write(90);
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
      forward();
    }
    else if (command == "backward") {
      backward();
    }
    else if (command == "left") {
      left();
    }
    else if (command == "right") {
      right();
    }
    else if (command == "extend") {
      Serial.println("extend arm");
      armServo.write(95);
    }
    else if (command == "retract") {
      Serial.println("retract arm");
      armServo.write(85);
    }
    else if (command == "halt") {
      halt();
    }
    else if (command == "auto") {
      Serial.println("begin autonomous mode");

      digitalWrite(AUTO_LED, HIGH);

      delay(10000);
      /*
      Forward 24 inches:
      forward();
      delay(time needed);
      halt();

      delay(2000);

      Turn 90° counterclockwise:
      left();
      delay(time needed);
      halt();

      delay(2000);

      Forward 6 inches:
      forward();
      delay(time needed);
      halt();

      delay(2000);

      Turn 90° counterclockwise:
      left();
      delay(time needed);
      halt();

      delay(2000);

      Forward 18 inches:
      forward();
      delay(time needed);
      halt();

      delay(2000);

      Backwards 12 inches:
      backward();
      delay(time needed);
      halt();

      delay(2000);

      Turn 270° clockwise:
      right();
      delay(time needed);
      halt();

      delay(2000);

      Forward 9 inches:
      forward();
      delay(time needed);
      halt();

      delay(2000);

      90° clockwise:
      right();
      delay(time needed);
      halt();

      delay(2000);

      Forward 18 inches:
      forward();
      delay(time needed);
      halt();*/
      
      digitalWrite(AUTO_LED, LOW); 

      Serial.println("end autonomous mode");
    } 
  }
}