#include <Wire.h>

// Default I2C address for AS5048B
const int AS5048_ADDR = 0x40; 

// Register addresses for Angle 
const int ANGLE_MSB_REG = 0xFE;
const int ANGLE_LSB_REG = 0xFF;

float initialAngle = 0.0;

uint16_t read14BitAngle();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor

  delay(100); // Allow sensor to stabilize
  uint16_t raw = read14BitAngle();
  initialAngle = raw * 0.02197265625;

  Serial.println("AS5048B Angle Reader Initialized");
  Serial.print("Initial Offset: ");
  Serial.println(initialAngle);
}

void loop() {
  uint16_t rawAngle = read14BitAngle();
  
  // Convert 14-bit raw value (0-16383) to degrees [cite: 19, 218]
  // 360 / 16384 = 0.02197 degrees per LSB 
  float degrees = rawAngle * 0.02197265625;
  
  float displayedAngle = degrees - initialAngle;

  // Normalize to -180 to 180
  if (displayedAngle > 180.0) displayedAngle -= 360.0;
  if (displayedAngle < -180.0) displayedAngle += 360.0;

  // Serial Plotter format: "Label:Value"
  Serial.print("Angle:");
  Serial.println(displayedAngle, 2);

  delay(50); // Faster update rate for smoother plotting
}

uint16_t read14BitAngle() {
  uint8_t msb, lsb;

  // Request the angle registers 
  Wire.beginTransmission(AS5048_ADDR);
  Wire.write(ANGLE_MSB_REG); 
  Wire.endTransmission(false); // Restart for read sequence [cite: 705]

  Wire.requestFrom(AS5048_ADDR, 2);
  if (Wire.available() >= 2) {
    msb = Wire.read(); // 0xFE contains bits <13:6> 
    lsb = Wire.read(); // 0xFF contains bits <5:0> 
  }
  else {
    // Avoid printing error to Serial if plotting to keep graph clean, 
    // or print it in a way that doesn't break the plotter (e.g. standard error)
    // For now, just return 0 or last known. 
    // We will keep the error print but it might show as a spike or ignored text in plotter.
    // Serial.println("Error: Unable to read angle registers"); 
    return 0;
  }

  // Combine into a 14-bit value
  // The MSB register contains the top 8 bits, LSB register bottom 6 bits 
  uint16_t angle = (uint16_t)msb << 6;
  angle |= (lsb & 0x3F);

  return angle;
}
