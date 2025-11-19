// =====================================================================
// EROBOTA16 - FINAL RACING CONTROL CODE
// =====================================================================
// Optimized for Python Racing UI - Upload this to your Arduino
// Tuned for responsive, smooth control with racing-style interface
// =====================================================================

// --- PIN DEFINITIONS (TB6612FNG Motor Driver) ---
#define PWMA 2  // Motor A PWM (Left Wheel)
#define AIN2 3  // Motor A Direction 2
#define AIN1 4  // Motor A Direction 1
#define STBY 5  // Standby (HIGH = enabled)
#define BIN1 6  // Motor B Direction 1
#define BIN2 7  // Motor B Direction 2
#define PWMB 8  // Motor B PWM (Right Wheel)

// --- CONFIGURATION ---
const long BAUD_RATE = 9600;       // Serial speed (match with Python)
#define MAX_CMD_LEN 64             // Command buffer size
#define COMMAND_TIMEOUT 500        // Stop after 500ms no command

// --- MOTOR TUNING ---
#define USE_SMOOTHING true         // Smooth acceleration (recommended)
#define SMOOTHING_FACTOR 0.3       // 0.0=instant, 1.0=very smooth

// --- GLOBAL VARIABLES ---
char serialCmd[MAX_CMD_LEN];
unsigned long lastCmdTime = 0;
int lastLeftSpeed = 1;
int lastRightSpeed = 1;
float smoothLeftSpeed = 0;
float smoothRightSpeed = 0;

void setup() {
  // Configure motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);
  
  // Safety: Start with motors stopped
  stopMotors();

  // Initialize serial
  Serial.begin(BAUD_RATE);
  lastCmdTime = millis();
  
  // Signal ready to Python UI
  Serial.println("EROBOTA16_READY");
}

void loop() {
  // Process incoming commands
  if (readSerialCommand()) {
    parseCommand(serialCmd);
    lastCmdTime = millis();
  }

  // Safety timeout
  if (millis() - lastCmdTime > COMMAND_TIMEOUT) {
    stopMotors();
  }
}

/**
 * Non-blocking serial command reader
 */
bool readSerialCommand() {
  static unsigned int cmdPos = 0;
  
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (cmdPos > 0) {
        serialCmd[cmdPos] = '\0';
        cmdPos = 0;
        return true;
      }
    } 
    else if (inChar >= 32 && inChar <= 126) {
      if (cmdPos < MAX_CMD_LEN - 1) {
        serialCmd[cmdPos++] = inChar;
      } else {
        cmdPos = 0;
      }
    }
  }
  return false;
}

/**
 * Parse "SPD:X;TURN:Y" commands
 */
void parseCommand(char* cmd) {
  int baseSpeed = 0;
  int turnValue = 0;

  char* token = strtok(cmd, ";");
  while (token != NULL) {
    char* separator = strchr(token, ':');
    if (separator != NULL) {
      *separator = '\0';
      char* key = token;
      int val = atoi(separator + 1);
      
      if (strcmp(key, "SPD") == 0) {
        baseSpeed = val;
      } else if (strcmp(key, "TURN") == 0) {
        turnValue = val;
      }
    }
    token = strtok(NULL, ";");
  }

  // Differential drive mixing (tank-style steering)
  // Turn by varying left/right wheel speeds
  int leftSpeed  = constrain(baseSpeed + turnValue, -255, 255);
  int rightSpeed = constrain(baseSpeed - turnValue, -255, 255);

  // Apply smoothing
  if (USE_SMOOTHING) {
    smoothLeftSpeed += SMOOTHING_FACTOR * (leftSpeed - smoothLeftSpeed);
    smoothRightSpeed += SMOOTHING_FACTOR * (rightSpeed - smoothRightSpeed);
    leftSpeed = (int)smoothLeftSpeed;
    rightSpeed = (int)smoothRightSpeed;
  }

  // Update motors only if changed
  if (leftSpeed != lastLeftSpeed) {
    setMotorSpeed(AIN1, AIN2, PWMA, leftSpeed);
    lastLeftSpeed = leftSpeed;
  }
  
  if (rightSpeed != lastRightSpeed) {
    setMotorSpeed(BIN1, BIN2, PWMB, rightSpeed);
    lastRightSpeed = rightSpeed;
  }
}

/**
 * Control individual motor
 */
void setMotorSpeed(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } 
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  } 
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

/**
 * Emergency stop
 */
void stopMotors() {
  if (lastLeftSpeed != 0) {
    setMotorSpeed(AIN1, AIN2, PWMA, 0);
    lastLeftSpeed = 0;
    smoothLeftSpeed = 0;
  }
  
  if (lastRightSpeed != 0) {
    setMotorSpeed(BIN1, BIN2, PWMB, 0);
    lastRightSpeed = 0;
    smoothRightSpeed = 0;
  }
}
