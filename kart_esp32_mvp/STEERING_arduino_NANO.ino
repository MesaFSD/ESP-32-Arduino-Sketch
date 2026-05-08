#include <Servo.h>

Servo steering;

#define SPEED_LIMIT 20

int currentOut = 1500;

void setup() {
  steering.attach(6);
  steering.writeMicroseconds(1500);
  pinMode(4, INPUT);
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  int pwmIn = pulseIn(4, HIGH, 25000);

  if (pwmIn == 0) {
    steering.writeMicroseconds(1500);
    currentOut = 1500;
    Serial.println("No signal — neutral");
    delay(20);
    return;
  }

  // flipped direction: right = right
  int command = map(pwmIn, 1000, 2000, 100, -100);
  command = constrain(command, -SPEED_LIMIT, SPEED_LIMIT);

  int targetOut = map(command, -100, 100, 1000, 2000);

  // smooth stepping
  if (currentOut < targetOut) {
    currentOut = min(currentOut + 15, targetOut);
  } else if (currentOut > targetOut) {
    currentOut = max(currentOut - 15, targetOut);
  }

  steering.writeMicroseconds(currentOut);

  Serial.print("PWM in: ");
  Serial.print(pwmIn);
  Serial.print(" us  |  Command: ");
  Serial.print(command);
  Serial.print("  |  Output: ");
  Serial.print(currentOut);
  Serial.println(" us");

  delay(10);
}
