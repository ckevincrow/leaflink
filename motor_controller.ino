#include <SoftwareSerial.h>
SoftwareSerial hc12(10, 11); // RX, TX

const int MOTOR_IN1 = 5;
const int MOTOR_IN2 = 6;
const int MOTOR_EN  = 9;
const int MOTOR_SPEED = 220;

void moveForward() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_EN, MOTOR_SPEED);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, 0);
}

void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  Serial.begin(9600);
  hc12.begin(9600);

  stopMotor();
}

void loop() {
  if (hc12.available()) {
    char cmd = hc12.read();

    Serial.print("Received: ");
    Serial.println(cmd);

    if (cmd == 'F') moveForward();
    if (cmd == 'B') moveBackward();
    if (cmd == 'S') stopMotor();
  }
}
