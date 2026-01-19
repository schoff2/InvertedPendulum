#include <FastAccelStepper.h>
#include <Wire.h>

// =================================================================================
// 2-Stage PID Controller for Inverted Pendulum
// Stage 1 (Inner Loop): Balances the pendulum (Angle -> Motor Speed)
// Stage 2 (Outer Loop): Keeps the cart centered (Position -> Target Angle)
// =================================================================================

// --- Hardware Settings ---
const int AS5048_ADDR = 0x40;
const int ANGLE_MSB_REG = 0xFE;
const int ANGLE_LSB_REG = 0xFF;

const int dirPin = 2;
const int stepPin = 9;
const int homeSwitchPin = 4;

// Motor Config
const int stepsPerRevolution = 200;
const int microstepResolution = 16;
// const int revolutions = 3; // Not used in balancing directly, but good for limits
const float homingSpeed = 3200.0;
const int32_t maxPosition = 48000; // Used for homing/soft limits
const float maxMotorSpeed = 100000;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- Control Loop Variables ---
unsigned long lastLoopTime = 0;
unsigned long lastOuterLoopTime = 0;
const unsigned long LOOP_INTERVAL_US = 5000; // 5ms loop (200Hz)
const unsigned long OUTER_LOOP_INTERVAL_US = 25000;

// --- Angle Variables ---
float initialAngle = 0.0;
float currentAngle = 0.0;
float angleOffset = -180.0; // Adjust if zero is not pointing up

// --- PID 1: Angle Controller (Inner Loop) ---
// Goal: Maintain Target Angle (usually 0, unless modified by Position PID)
// Output: Motor Speed (steps/s)
float angle_Kp = 14000.0;   // Proportional Gain - STRENGTH of reaction
float angle_Ki = 0.0;     // Integral Gain - Corrects steady-state error (use sparingly)
float angle_Kd = 2000.0;    // Derivative Gain - DAMPING (resists sudden changes)

float angle_lastError = 0.0;
float angle_integral = 0.0;

// --- PID 2: Position Controller (Outer Loop) ---
// Goal: Maintain Position 0 (Center of track)
// Output: Target Angle for inner loop
float pos_Kp = 0.00020;      // Proportional Gain - How much to lean to correct position
float pos_Ki = 0.0;       // Integral Gain
float pos_Kd = 0.0002; //1.5;       // Derivative Gain

float pos_lastError = 0.0;
float pos_integral = 0.0;
float pid_pos_out = 0.0;

long targetPosition = 0;  // Center

float pos_derivative = 0.0;
float pos_derivative_alpha = 0.7;



// --- Safety Limits ---
const float MAX_TARGET_ANGLE = 12.0; // Max lean angle (degrees) requested by position loop
const float FALL_LIMIT = 30.0;       // Stop if pendulum falls beyond this

void setup() {
  // --- Pin Setup ---
  pinMode(homeSwitchPin, INPUT_PULLUP);

  // --- Motor Setup ---
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    stepper->setAcceleration(100000); // High accel for quick reaction
    stepper->setSpeedInHz(1000);
  }

  // --- Comms Setup ---
  Wire.begin();
  Wire.setClock(400000); // Fast I2C
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing Inverted Pendulum Controller...");


  // --- Homing Sequence ---
  Serial.println("Homing...");
  stepper->setSpeedInHz(homingSpeed);
  stepper->runForward();
  
  // Wait for switch
  while (digitalRead(homeSwitchPin) == LOW);
  
  stepper->forceStop();
  // Assume homing switch is at one end. 
  // If track length is 'maxPosition', and switch is at 0 or max?
  // User code set 'maxPosition' after homing, implying switch is at the end.
  stepper->setCurrentPosition(maxPosition); 
  
  // Move to center
  stepper->setSpeedInHz(homingSpeed * 2);
  stepper->moveTo(maxPosition / 2);
  while(stepper->isRunning()); // Block until centered
  
  // Reset center to 0 for easier PID math
  stepper->setCurrentPosition(0); 
  
  // Give user a moment to stabilize pendulum by hand
  delay(5000);
  lastLoopTime = micros();
  lastOuterLoopTime = micros();
  Serial.println("Centered. Ready to balance.");

  // --- Angle Sensor Initialization ---
  uint16_t raw = read14BitAngle();
  initialAngle = raw * 0.02197265625;
  Serial.print("Initial Sensor Angle: "); Serial.println(initialAngle);
}

void loop() {
  unsigned long now = micros();
  
  // Run loop at fixed interval
  if (now - lastLoopTime >= LOOP_INTERVAL_US) {
    float dt = (now - lastLoopTime) / 1000000.0; // dt in seconds
    lastLoopTime = now;

    // 1. Read Sensors
    currentAngle = getPhysicalAngle();
    long currentPos = stepper->getCurrentPosition();

    // 2. Safety Check
    if (abs(currentAngle) > FALL_LIMIT) {
      stepper->forceStop();
      pos_integral = 0; // Reset integrals
      angle_integral = 0;
      return; // Stop here, don't run PIDs
    }

    // 3. Outer Loop: Position PID
    // Calculate error: Target - Current
    // If we are at +1000 (right), error is -1000. 
    // We want to lean LEFT (negative angle). 
    // So simple P-control: TargetAngle = Kp * error = Kp * (-1000) = negative angle.
    if (now - lastOuterLoopTime >= OUTER_LOOP_INTERVAL_US) {
      float dt = (now - lastOuterLoopTime) / 1000000.0;
      lastOuterLoopTime = now;
      float pos_error = targetPosition - currentPos;

      // Add a deadband
      if (abs(pos_error) < 150) {
        pos_error = 0;
        pos_integral = 0; 
      }
      
      pos_integral += pos_error * dt;
      // Clamp integral to prevent windup? (Optional)
      
      float pos_derivative_raw = (pos_error - pos_lastError) / dt;
      // Low-pass filter
      pos_derivative = (pos_derivative_alpha * pos_derivative_raw) + ((1.0 - pos_derivative_alpha) * pos_derivative);


      pos_lastError = pos_error;

      pid_pos_out = (pos_Kp * pos_error) + (pos_Ki * pos_integral) + (pos_Kd * pos_derivative);
    }

    // This output is the TARGET ANGLE for the inner loop
    // Clamp it to prevent requesting dangerous leans
    float targetAngle = constrain(pid_pos_out, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);


    // 4. Inner Loop: Angle PID
    // Error = Target - Current
    // If Target is -5 (lean left) and Current is 0 (upright), Error is -5.
    // Motor needs to move LEFT (negative speed) to create left lean?
    // Wait. To create a LEFT lean, the cart must accelerate LEFT (move feet under head).
    // NO. If we are upright (0) and want to lean left (-5), we must move LEFT?
    // If we move LEFT, the top falls RIGHT.
    // If we move RIGHT, the top falls LEFT.
    // So to achieve -5 deg (left lean), we must move RIGHT momentarily?
    // Let's rely on the standard balancing logic:
    // If Angle Error is positive (leaning right relative to target), we move RIGHT (positive speed) to catch it.
    float angle_error = currentAngle - targetAngle; // Normal error
    
    angle_integral += angle_error * dt;
    angle_integral = constrain(angle_integral, -1000, 1000); // Anti-windup

    float angle_derivative = (angle_error - angle_lastError) / dt;
    angle_lastError = angle_error;

    float motorSpeed = (angle_Kp * angle_error) + (angle_Ki * angle_integral) + (angle_Kd * angle_derivative);
    if(motorSpeed > maxMotorSpeed)
    {
        motorSpeed = maxMotorSpeed;
    }
    else if(motorSpeed < -maxMotorSpeed)
    {
        motorSpeed = -maxMotorSpeed;
    }
    

    // 5. Apply to Motor
    if (abs(motorSpeed) < 1.0) {
      stepper->stopMove(); // Stop jitter
    } else {
      stepper->setSpeedInHz(abs(motorSpeed));
      if (motorSpeed > 0) {
        stepper->runForward();
      } else {
        stepper->runBackward();
      }
    }

    // 6. Telemetry (Optional - slows down loop if too verbose)
    // Print every 20th loop (approx 10Hz)
    static int printCount = 0;
    if (printCount++ > 20) {
      printCount = 0;

      Serial.print("Pos:"); Serial.print(currentPos);
      // Serial.print(" pos_Kp_component:"); Serial.print(pos_Kp * pos_error);
      // Serial.print(" pos_Ki_component:"); Serial.print(pos_Ki * pos_integral);
      // Serial.print(" pos_Kd_component:"); Serial.print(pos_Kd * pos_derivative);
      Serial.print(" AngleOut:"); Serial.print(pid_pos_out);
      Serial.print(" TgtAng:"); Serial.print(targetAngle);

      
      Serial.print(" Ang:"); Serial.print(currentAngle);
      Serial.print(" angle_Kp_component:"); Serial.print(angle_Kp * angle_error);
      Serial.print(" angle_Ki_component:"); Serial.print(angle_Ki * angle_integral);
      Serial.print(" angle_Kd_component:"); Serial.print(angle_Kd * angle_derivative);

      Serial.print(" Speed:"); Serial.print(motorSpeed);
      Serial.print(" dt:"); Serial.println(dt*1000.0);
    }
  }
}

float filteredAngle = 0.0;
float alpha = 0.7; // Smoothing factor (0.1 = heavy smoothing, 1.0 = no smoothing)

// Helper: Read and process angle
float getPhysicalAngle() {
  uint16_t rawAngle = read14BitAngle();
  float degrees = rawAngle * 0.02197265625;
  float currentRaw = degrees - initialAngle - 180.0;

  // Normalize -180 to 180
  if (currentRaw > 180.0) currentRaw -= 360.0;
  if (currentRaw < -180.0) currentRaw += 360.0;

  // Low-pass filter
  filteredAngle = (alpha * currentRaw) + ((1.0 - alpha) * filteredAngle);
  return filteredAngle;
}

uint16_t read14BitAngle() {
  uint8_t msb, lsb;
  Wire.beginTransmission(AS5048_ADDR);
  Wire.write(ANGLE_MSB_REG); 
  Wire.endTransmission(false); 
  Wire.requestFrom(AS5048_ADDR, 2);
  if (Wire.available() >= 2) {
    msb = Wire.read(); 
    lsb = Wire.read(); 
  } else {
    return 0; // Error handling could be better
  }
  return ((uint16_t)msb << 6) | (lsb & 0x3F);
}
