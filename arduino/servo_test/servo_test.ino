// MINIMAL DIAGNOSTIC SKETCH
// Upload this to find exactly where the servo_controller hangs

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // Long delay first - ensures Serial Monitor is connected before ANY output
  delay(3000);

  Serial.begin(115200);
  delay(500);

  Serial.println("=== STEP 1: Serial OK ===");
  Serial.flush();

  // Test I2C init
  Wire.begin();  // use default pins
  Serial.println("=== STEP 2: Wire.begin() OK ===");
  Serial.flush();

  // Scan for I2C devices
  Serial.println("Scanning I2C...");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found: 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("  NO I2C DEVICES FOUND - check SDA/SCL wiring");
  Serial.flush();

  Serial.println("=== STEP 3: I2C scan done ===");

  // Test PCA9685 init
  pwm.begin();
  Serial.println("=== STEP 4: pwm.begin() OK ===");
  Serial.flush();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Serial.println("=== STEP 5: PCA9685 configured ===");
  Serial.flush();

  // Move servos
  pwm.setPWM(0, 0, 300);  // pan center
  pwm.setPWM(1, 0, 410);  // tilt center
  Serial.println("=== STEP 6: Servos moved to center ===");
  Serial.flush();

  Serial.println("=== ALL DONE - type C to center, D,x,y to move ===");
}

void loop() {
  // Echo anything received
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.print("Got: "); Serial.println(cmd);

    if (cmd == "C" || cmd == "c") {
      pwm.setPWM(0, 0, 300);
      pwm.setPWM(1, 0, 410);
      Serial.println("Centered!");
    }
  }
  delay(10);
}
