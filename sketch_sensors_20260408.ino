// Select correct serial for HC-12
#if defined(HAVE_HWSERIAL1)
  #define hc12 Serial1
#else
  #define hc12 Serial   // fallback (not ideal, but works)
#endif

// Ultrasonic Pins
const int FRONT_TRIG = 2;
const int FRONT_ECHO = 3;
const int REAR_TRIG  = 4;
const int REAR_ECHO  = 7;

const int STOP_DISTANCE = 30;
const unsigned long STOP_DELAY = 4000;

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

  // Ensure clean trigger signals
  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(REAR_TRIG, LOW);

  Serial.begin(9600);   // Debug
  hc12.begin(9600);     // HC-12
}

void loop() {

  if (direction == 'F') {

    long frontDistance = readDistanceCM(FRONT_TRIG, FRONT_ECHO);
    Serial.print("Front: ");
    Serial.println(frontDistance);

    if (frontDistance <= STOP_DISTANCE) {

      hc12.write('S');  // Explicit STOP
      delay(STOP_DELAY);

      frontDistance = readDistanceCM(FRONT_TRIG, FRONT_ECHO);

      if (frontDistance <= STOP_DISTANCE) {
        direction = 'B';
      }
    }
  }

  else if (direction == 'B') {

    long rearDistance = readDistanceCM(REAR_TRIG, REAR_ECHO);
    Serial.print("Rear: ");
    Serial.println(rearDistance);

    if (rearDistance <= STOP_DISTANCE) {

      hc12.write('S');  // Explicit STOP
      delay(STOP_DELAY);

      rearDistance = readDistanceCM(REAR_TRIG, REAR_ECHO);

      if (rearDistance <= STOP_DISTANCE) {
        direction = 'F';
      }
    }
  }

  // Send movement command
  hc12.write(direction);

  delay(100);
}