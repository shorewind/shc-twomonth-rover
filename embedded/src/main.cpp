#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>  // barometric pressure and temperature sensor library
#include <Adafruit_LSM6DSOX.h>  // acceleration sensor library

#define WIRE_SDA 0  // Use GP0 as I2C0 SDA
#define WIRE_SCL 1  // Use GP1 as I2C0 SCL
arduino::MbedI2C Wire_(WIRE_SDA, WIRE_SCL);

// accelerometer pin definitions
#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

// motor drivers pulse-width modulation GPIO pins
#define PWM_1L 2  // PWM_1 = forward
#define PWM_1R 3
#define PWM_2L 4  // PWM_2 = backward
#define PWM_2R 5

#define BUILTIN_LED 25
#define AUTO_LED 16
#define MANUAL_LED 17
#define SEALEVELPRESSURE_HPA (1013.25)

// declare hardware instances
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;
Servo armServo;

void setup() {
  // initialize outputs
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(AUTO_LED, OUTPUT);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(PWM_1L, OUTPUT);
  pinMode(PWM_1R, OUTPUT);
  pinMode(PWM_2L, OUTPUT);
  pinMode(PWM_2R, OUTPUT);

  armServo.attach(22);  // assign servo pin

  // turn LED on for initialization
  digitalWrite(BUILTIN_LED, HIGH);

  // configure serial transport
  Serial.begin(115200);  // baud rate
  delay(100);

  // manual mode LED on
  digitalWrite(MANUAL_LED, HIGH);

  // initialize motor drivers
  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, LOW);

  // initialize servoPos
  armServo.write(5);

  // turn LED off after serial initialization
  digitalWrite(BUILTIN_LED, LOW);

  if (!bmp.begin_I2C(0x77, &Wire_)) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }

  if (!sox.begin_I2C(0x6A, &Wire_)) {
    Serial.println("Failed to find LSM6DSOX chip");
  }

  // set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
}

// movement controls functions
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

void halt() {  // turn all motors off
  Serial.println("halt");
  Serial.println("motors off");
  digitalWrite(PWM_1L, LOW);
  digitalWrite(PWM_1R, LOW);
  digitalWrite(PWM_2L, LOW);
  digitalWrite(PWM_2R, LOW);
}

int last_read = 0;  // declare and initialize time of last read
int last_read_servo = 0;
int servoPos = 5;  // declare and initialize arm servo position

void loop() {
  // automated data collection approximately every 500 ms
  if (millis() - last_read > 500) {
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

    // get a new normalized sensor event
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);

    Serial.print(accel.acceleration.x);
    Serial.print(","); Serial.print(accel.acceleration.y);
    Serial.print(","); Serial.print(accel.acceleration.z);
    Serial.println();

    last_read = millis();  // reassign time of last read
  }
  
  if (Serial.available()) {
    // read command input from serial monitor
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
    else if (command == "extend") {  // move to set position
      Serial.println("extending arm");
      servoPos = 165;
      armServo.write(servoPos);
    }
    else if (command == "retract") {  // move to set position
      Serial.println("retracting arm");
      servoPos = 135;
      armServo.write(servoPos);
    }
    else if (command == "extend_one") {  // sets new position
      Serial.println("extending arm one degree");
      if(servoPos <= 174) {
        servoPos++;
      }
    }
    else if (command == "retract_one") {  // sets new position
      Serial.println("retracting arm one degree");
      if(6 <= servoPos) {
        servoPos--;
      }
    }
    else if (command == "set_pos") {  // move to new position
      Serial.println("going to position");
      armServo.write(servoPos);
    }
    else if (command == "halt") {
      halt();
    }
    else if (command == "auto") {
      // execute "follow the instructions" challenge
      Serial.println("begin autonomous mode");
      // turn autonomous mode LED on, manual mode LED off
      digitalWrite(AUTO_LED, HIGH);
      digitalWrite(MANUAL_LED, LOW);

      // note: delays prevent data readings
      delay(2000);  // delay 2 seconds

      // forward 24 inches
      forward();
      delay(3072);  // manually timed how long it took for the rover to complete action
      halt();
      delay(2000);
      
      // turn 90 degrees counterclockwise
      left();
      delay(1400);
      halt();
      delay(2000);

      // forward 6 inches
      forward();
      delay(831);
      halt();
      delay(2000);

      // turn 90 degrees counterclockwise
      left();
      delay(1400);
      halt();
      delay(2000);

      // forward 18 inches
      forward();
      delay(2274);
      halt();
      delay(2000);

      // backwards 12 inches
      backward();
      delay(1475);
      halt();
      delay(2000);

      // turn 270 degrees clockwise
      right();
      delay(4200);
      halt();
      delay(2000);

      // forward 9 inches
      forward();
      delay(1080);
      halt();
      delay(2000);

      // turn 90 degrees clockwise
      right();
      delay(1400);
      halt();
      delay(2000);

      // forward 18 inches
      forward();
      delay(2274);
      halt();

      // after instructions are executed, turn off autonomous mode LED and turn on manual mode LED
      digitalWrite(AUTO_LED, LOW); 
      digitalWrite(MANUAL_LED, HIGH);

      Serial.println("end autonomous mode");
    } 
  }
}