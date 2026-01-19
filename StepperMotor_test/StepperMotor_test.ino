#include <AccelStepper.h>

// A4988 Stepper Motor Driver Test using AccelStepper
// Pin Definitions
const int dirPin = 2;  // Connect to A4988 DIRECTION
const int stepPin = 3; // Connect to A4988 STEP

// Motor configuration
const int stepsPerRevolution = 200; 
const int microstepResolution = 16;
const int revolutions = 5;
const long targetPosition = (long)stepsPerRevolution * microstepResolution * revolutions;

// Define a stepper and the pins it will use
// AccelStepper::DRIVER means a driver that takes Step and Direction signals
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Setup serial output
  Serial.begin(9600);
  Serial.println("Stepper Motor Test with AccelStepper");
  
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(1000);
  // Set the acceleration to be faster than default:
  stepper.setAcceleration(10000);
}

void loop() {
  // Read potentiometer to adjust speed dynamically
  // Map analog reading (0-1023) to a speed range (e.g., 100 to 2000 steps/sec)
  int val = analogRead(A0);
  float speed = map(val, 0, 1023, 20000, 100); // Inverted logic from original: Low val (low delay) -> High speed
  
  stepper.setMaxSpeed(speed);

  // Move to the target position
  if (stepper.distanceToGo() == 0) {
    // Delay between moves
    delay(1000);
    
    // Toggle direction by swapping target position
    if (stepper.currentPosition() != 0) {
      stepper.moveTo(0);
    } else {
      stepper.moveTo(targetPosition);
    }
  }

  // Run the motor
  stepper.run();
}