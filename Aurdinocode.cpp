// Define motor pins
int motorA_EN = 5;  // PWM pin for speed control (Motor A)
int motorA_IN1 = 2; // Motor A forward
int motorA_IN2 = 3; // Motor A backward
int motorB_EN = 6;  // PWM pin for speed control (Motor B)
int motorB_IN3 = 4; // Motor B forward
int motorB_IN4 = 7; // Motor B backward

// Ultrasonic sensor pins
const int trigPin = 9;  
const int echoPin = 10;

// Define constants for distance thresholds
const int safeDistance = 20; // Safe distance from obstacles (in cm)

void setup() {
  // Set motor pins as output
  pinMode(motorA_EN, OUTPUT);
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

// Function to get distance from ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);       
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH); 
  long distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}

// Function to move forward
void moveForward() {
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, 255); // Full speed

  digitalWrite(motorB_IN3, HIGH);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_EN, 255); // Full speed
}

// Function to stop
void stopMoving() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, 0);

  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_EN, 0);
}

// Function to turn left
void turnLeft() {
  digitalWrite(motorA_IN1, LOW);  // Stop left motor
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, 0);
  
  digitalWrite(motorB_IN3, HIGH); // Right motor moves forward
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_EN, 200);    // Slow down turning speed
}

// Function to turn right
void turnRight() {
  digitalWrite(motorA_IN1, HIGH); // Left motor moves forward
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, 200);    // Slow down turning speed

  digitalWrite(motorB_IN3, LOW);  // Stop right motor
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_EN, 0);
}

void loop() {
  // Get the distance to the nearest obstacle
  long distance = getDistance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > safeDistance) {
    moveForward();  // Move forward if no obstacle
  } else if (distance <= safeDistance) {
    stopMoving();   // Stop if obstacle detected
    delay(500);     // Short pause

    // Randomly turn left or right to avoid obstacle
    if (random(2) == 0) {
      turnLeft();
      delay(1000); // Turn for 1 second
    } else {
      turnRight();
      delay(1000); // Turn for 1 second
    }
    
    stopMoving();   // Stop after turning
    delay(500);
  }
}
