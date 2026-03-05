#include <SoftwareSerial.h>
SoftwareSerial hc12(10, 11); // RX, TX

// Ultrasonic Pins
const int FRONT_TRIG = 2;
const int FRONT_ECHO = 3;
const int REAR_TRIG  = 4;
const int REAR_ECHO  = 7;

const int STOP_DISTANCE = 30;
const unsigned long STOP_DELAY = 2000;

char direction = 'F';   // Start moving forward

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void setup() {
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(REAR_TRIG, OUTPUT);
  pinMode(REAR_ECHO, INPUT);

  Serial.begin(9600);
  hc12.begin(9600);

  hc12.write('F');   // Start forward
}

void loop() {

  if (direction == 'F') {

    long frontDistance = readDistanceCM(FRONT_TRIG, FRONT_ECHO);
    Serial.print("Front: ");
    Serial.println(frontDistance);

    if (frontDistance <= STOP_DISTANCE) {

      hc12.write('S');       // Stop motor
      delay(STOP_DELAY);     // Wait 2 seconds

      // Re-check front sensor
      frontDistance = readDistanceCM(FRONT_TRIG, FRONT_ECHO);

      if (frontDistance <= STOP_DISTANCE) {
        direction = 'B';
        hc12.write('B');     // Reverse
      } else {
        hc12.write('F');     // Continue forward
      }
    }
  }

  else if (direction == 'B') {

    long rearDistance = readDistanceCM(REAR_TRIG, REAR_ECHO);
    Serial.print("Rear: ");
    Serial.println(rearDistance);

    if (rearDistance <= STOP_DISTANCE) {

      hc12.write('S');       // Stop motor
      delay(STOP_DELAY);     // Wait 2 seconds

      // Re-check rear sensor
      rearDistance = readDistanceCM(REAR_TRIG, REAR_ECHO);

      if (rearDistance <= STOP_DISTANCE) {
        direction = 'F';
        hc12.write('F');     // Reverse direction
      } else {
        hc12.write('B');     // Continue backward
      }
    }
  }

  delay(100);
}
