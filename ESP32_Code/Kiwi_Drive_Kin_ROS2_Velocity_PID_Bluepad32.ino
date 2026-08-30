#include <Bluepad32.h>
#include <esp_arduino_version.h>
#include <math.h>



// ---------------------------------------------------------------------------------------------------------------------------
// Bluepad32 controllers
// ---------------------------------------------------------------------------------------------------------------------------

ControllerPtr myControllers[BP32_MAX_GAMEPADS];





// ---------------------------------------------------------------------------------------------------------------------------
// ESP32 pin assignments
// ---------------------------------------------------------------------------------------------------------------------------

// Shared TB6612FNG standby pin
const uint8_t STBY = 19;

// ---------------- Motor 1 ----------------
const uint8_t M1_PWM = 25;
const uint8_t M1_IN1 = 26;
const uint8_t M1_IN2 = 27;
    //------------- Motor 1 Encoder --------
const uint8_t M1_ENCA = 34;
const uint8_t M1_ENCB = 35;

// ---------------- Motor 2 ----------------
const uint8_t M2_PWM = 13;
const uint8_t M2_IN1 = 12;
const uint8_t M2_IN2 = 14;

    //------------- Motor 1 Encoder --------
    // VP = GPIO36
    // VN = GPIO39
const uint8_t M2_ENCA = 36;
const uint8_t M2_ENCB = 39;

// ---------------- Motor 3 ----------------

const uint8_t M3_PWM = 32;
const uint8_t M3_IN1 = 33;
const uint8_t M3_IN2 = 23;

    //------------- Motor 1 Encoder --------
    // RX2 = GPIO16
    // TX2 = GPIO17
const uint8_t M3_ENCA = 16;
const uint8_t M3_ENCB = 17;





// ---------------------------------------------------------------------------------------------------------------------------
// PWM configuration
// ---------------------------------------------------------------------------------------------------------------------------

const uint32_t PWM_FREQUENCY_HZ = 20000;
const uint8_t PWM_RESOLUTION_BITS = 8;

// Used by ESP32 Arduino Core 2.x
const uint8_t M1_PWM_CHANNEL = 0;
const uint8_t M2_PWM_CHANNEL = 1;
const uint8_t M3_PWM_CHANNEL = 2;





// ---------------------------------------------------------------------------------------------------------------------------
// Controller configuration
// ---------------------------------------------------------------------------------------------------------------------------

// Bluepad32 joystick values are approximately -512 to +512.
const int AXIS_MAX = 512;

const int LEFT_STICK_DEADZONE = 80;
const int RIGHT_STICK_DEADZONE = 30;

// Maximum PWM the speed controller may command.
const int MAX_PWM = 240;

// Smallest PWM that reliably moves the motors.
const int MIN_PWM = 45;

// Overall joystick sensitivity.
const float TRANSLATION_SCALE = 1.00f;
const float ROTATION_SCALE = 0.80f;

// Positive logical wheel command to physical motor direction.
const int M1_DIRECTION = 1;
const int M2_DIRECTION = 1;
const int M3_DIRECTION = 1;

// Serial diagnostic interval. (Set to 100ms so it wont spam the serial).
const unsigned long PRINT_INTERVAL_MS = 100;
unsigned long lastPrintTime = 0;





// ---------------------------------------------------------------------------------------------------------------------------
// Encoder and velocity-control configuration
// ---------------------------------------------------------------------------------------------------------------------------

// Confirmed using full x4 quadrature decoding.
const float COUNTS_PER_REVOLUTION = 1976.1f;

// ROS / odometry telemetry interval (20 Hz or ever 50ms).
const unsigned long ODOM_TELEMETRY_INTERVAL_MS = 50;
unsigned long lastOdomTelemetryTime = 0;

// Leave control headroom below the nominal 130 RPM no-load speed.
// (after watching video this is important for autonomous control because it leaves headroom to allow the robot to speed up if detecting an error if needed to).
const float MAX_TARGET_RPM = 100.0f;

// Velocity loop at 50 Hz. (20ms) (50x per sec).
const uint32_t PID_PERIOD_US = 20000;

// Positive motor command produced negative raw encoder counts in
// the Motor 3 position test. Since all three encoders are wired
// identically I simply inverted all 3 for ideal operation.
const int ENCODER_DIRECTION[3] = {-1, -1, -1};

// Starting PID gains.
float velocityKp[3] = {0.80f, 0.80f, 0.80f};
float velocityKi[3] = {0.30f, 0.30f, 0.30f};
float velocityKd[3] = {0.05f, 0.05f, 0.05f};

// RPM first order low-pass filter: 0 = no new data, 1 = no filtering. ( RPMfilt = alpha * RPMnew + (1 - alpha)RPMold )
const float RPM_FILTER_ALPHA = 0.30f;

// Stop tiny target commands.
const float TARGET_RPM_DEADBAND = 1.0f;

// Integral anti-windup limit. (I == I + e * delta_t) this is very important for making if a wheel is stuck
// and commands to that wheel want a specific rpm this will prevent the error integral from shooting up (limits to -100 <-> 100)
const float INTEGRAL_LIMIT = 100.0f;





// ---------------------------------------------------------------------------------------------------------------------------
// Manual / autonomous mode configuration
// ---------------------------------------------------------------------------------------------------------------------------


// Manual is for using the bluetooth controller (xbox) and autonomous is for ROS2 and Nav2 control thru the pi
enum class ControlMode {
  MANUAL,
  AUTONOMOUS
};

// Starts the controller in manual, will update to autonomous if wanting to start off in self nav
ControlMode controlMode = ControlMode::MANUAL;

// Xbox Start button misc bit.
const uint8_t START_MISC_MASK = 0x04;

// Stop the robot if the Pi stops refreshing autonomous commands. So if ROS2 crashes which did happen a couple times this will stop
// the robot rather than continuing indefinetly 
const unsigned long AUTO_COMMAND_TIMEOUT_MS = 300;

String serialCommandBuffer;
unsigned long lastAutoCommandTimeMs = 0; // storing the last autonomous command
bool autoTimeoutActive = false;          // if timeout occurs this will flip so rather than repeating the timeout error indefinetly it does it once
bool startButtonWasPressed = false;      





// ---------------------------------------------------------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------------------------------------------------------

//  actual pwm output calculated by controller (not the targeted RPM or measured RPM)
int motor1Command = 0;
int motor2Command = 0;
int motor3Command = 0;

//  Linked this to A so pressing A will cause the software motor switch, to switch back pressing B can be pressed
bool emergencyStopActive = false;

// Encoder state.
volatile int32_t encoderCount[3] = {0, 0, 0};
volatile uint8_t previousEncoderState[3] = {0, 0, 0};

// Quadrature lookup table in RAM for ISR use.
DRAM_ATTR const int8_t QUADRATURE_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Velocity controller state.
int32_t previousPIDEncoderCount[3] = {0, 0, 0};

float targetRPM[3] = {0.0f, 0.0f, 0.0f};
float measuredRPM[3] = {0.0f, 0.0f, 0.0f};
float rawRPM[3] = {0.0f, 0.0f, 0.0f};

float velocityIntegral[3] = {0.0f, 0.0f, 0.0f};
float previousVelocityError[3] = {0.0f, 0.0f, 0.0f};

uint32_t previousPIDTimeUs = 0;





// ---------------------------------------------------------------------------------------------------------------------------
// Function declarations
// ---------------------------------------------------------------------------------------------------------------------------

void stopAllMotors();
void enableMotorDrivers();
void disableMotorDrivers();

void configurePWM();
void writeMotorPWM(uint8_t pwmPin, uint8_t channel, int duty);

void setMotor(
  uint8_t pwmPin,
  uint8_t pwmChannel,
  uint8_t in1Pin,
  uint8_t in2Pin,
  int speed
);

void resetVelocityController();
void driveKiwi(float x, float y, float rotation);
void runVelocityPID();

void processSerialCommands();
void processCompleteSerialCommand(String command);
void setControlMode(ControlMode newMode, const char* source);
void checkAutonomousTimeout();
const char* controlModeName();

void processControllers();
void processGamepad(ControllerPtr ctl);
void printControllerData(
  ControllerPtr ctl,
  int rawLX,
  int rawLY,
  int rawRX,
  float x,
  float y,
  float rotation
);

void sendOdometryTelemetry();

void IRAM_ATTR motor1EncoderISR();
void IRAM_ATTR motor2EncoderISR();
void IRAM_ATTR motor3EncoderISR();





// ---------------------------------------------------------------------------------------------------------------------------
// Bluepad32 callbacks
// ---------------------------------------------------------------------------------------------------------------------------

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;

      ControllerProperties properties = ctl->getProperties();

      Serial.println();
      Serial.println("=================================");
      Serial.printf("CONTROLLER CONNECTED: slot %d\n", i);
      Serial.printf("Model: %s\n", ctl->getModelName().c_str());
      Serial.printf(
        "VID: 0x%04X  PID: 0x%04X\n",
        properties.vendor_id,
        properties.product_id
      );
      Serial.println("=================================");

      emergencyStopActive = false;
      stopAllMotors();
      enableMotorDrivers();
      return;
    }
  }

  Serial.println(
    "Controller connected, but no controller slot was available."
  );
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;

      Serial.println();
      Serial.printf(
        "CONTROLLER DISCONNECTED: slot %d\n",
        i
      );

      startButtonWasPressed = false;
      stopAllMotors();
      disableMotorDrivers();
      return;
    }
  }
}





// ---------------------------------------------------------------------------------------------------------------------------
// PWM functions
// ---------------------------------------------------------------------------------------------------------------------------

void configurePWM() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3

  bool motor1Attached = ledcAttach(
    M1_PWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  bool motor2Attached = ledcAttach(
    M2_PWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  bool motor3Attached = ledcAttach(
    M3_PWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

#else

  ledcSetup(
    M1_PWM_CHANNEL,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  ledcSetup(
    M2_PWM_CHANNEL,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  ledcSetup(
    M3_PWM_CHANNEL,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  ledcAttachPin(M1_PWM, M1_PWM_CHANNEL);
  ledcAttachPin(M2_PWM, M2_PWM_CHANNEL);
  ledcAttachPin(M3_PWM, M3_PWM_CHANNEL);

  bool motor1Attached = true;
  bool motor2Attached = true;
  bool motor3Attached = true;

#endif

  if (!motor1Attached ||
      !motor2Attached ||
      !motor3Attached) {
    Serial.println(
      "ERROR: One or more PWM pins failed to attach."
    );
  } else {
    Serial.printf(
      "PWM configured: %lu Hz, %u-bit\n",
      PWM_FREQUENCY_HZ,
      PWM_RESOLUTION_BITS
    );
  }
}

void writeMotorPWM(
  uint8_t pwmPin,
  uint8_t channel,
  int duty
) {
  duty = constrain(duty, 0, 255);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwmPin, duty);
#else
  ledcWrite(channel, duty);
#endif
}





// ---------------------------------------------------------------------------------------------------------------------------
// Motor-driver control
// ---------------------------------------------------------------------------------------------------------------------------

void enableMotorDrivers() {
  digitalWrite(STBY, HIGH);
}

void disableMotorDrivers() {
  digitalWrite(STBY, LOW);
}

void setMotor(
  uint8_t pwmPin,
  uint8_t pwmChannel,
  uint8_t in1Pin,
  uint8_t in2Pin,
  int speed
) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    writeMotorPWM(pwmPin, pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    writeMotorPWM(pwmPin, pwmChannel, -speed);
  } else {
    writeMotorPWM(pwmPin, pwmChannel, 0);

    // Coast mode.
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void resetVelocityController() {
  portENTER_CRITICAL(&encoderMux);
  previousPIDEncoderCount[0] = encoderCount[0];
  previousPIDEncoderCount[1] = encoderCount[1];
  previousPIDEncoderCount[2] = encoderCount[2];
  portEXIT_CRITICAL(&encoderMux);

  for (int i = 0; i < 3; i++) {
    targetRPM[i] = 0.0f;
    rawRPM[i] = 0.0f;
    measuredRPM[i] = 0.0f;
    velocityIntegral[i] = 0.0f;
    previousVelocityError[i] = 0.0f;
  }

  previousPIDTimeUs = micros();
}

void stopAllMotors() {
  targetRPM[0] = 0.0f;
  targetRPM[1] = 0.0f;
  targetRPM[2] = 0.0f;

  velocityIntegral[0] = 0.0f;
  velocityIntegral[1] = 0.0f;
  velocityIntegral[2] = 0.0f;

  previousVelocityError[0] = 0.0f;
  previousVelocityError[1] = 0.0f;
  previousVelocityError[2] = 0.0f;

  motor1Command = 0;
  motor2Command = 0;
  motor3Command = 0;

  setMotor(
    M1_PWM,
    M1_PWM_CHANNEL,
    M1_IN1,
    M1_IN2,
    0
  );

  setMotor(
    M2_PWM,
    M2_PWM_CHANNEL,
    M2_IN1,
    M2_IN2,
    0
  );

  setMotor(
    M3_PWM,
    M3_PWM_CHANNEL,
    M3_IN1,
    M3_IN2,
    0
  );
}





// ---------------------------------------------------------------------------------------------------------------------------
// Encoder interrupts
// ---------------------------------------------------------------------------------------------------------------------------

void IRAM_ATTR updateEncoderFromISR(
  uint8_t motorIndex,
  uint8_t pinA,
  uint8_t pinB
) {
  const uint8_t a = digitalRead(pinA);
  const uint8_t b = digitalRead(pinB);
  const uint8_t currentState = (a << 1) | b;

  portENTER_CRITICAL_ISR(&encoderMux);

  const uint8_t tableIndex =
    (previousEncoderState[motorIndex] << 2) |
    currentState;

  encoderCount[motorIndex] +=
    ENCODER_DIRECTION[motorIndex] *
    QUADRATURE_TABLE[tableIndex];

  previousEncoderState[motorIndex] = currentState;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR motor1EncoderISR() {
  updateEncoderFromISR(0, M1_ENCA, M1_ENCB);
}

void IRAM_ATTR motor2EncoderISR() {
  updateEncoderFromISR(1, M2_ENCA, M2_ENCB);
}

void IRAM_ATTR motor3EncoderISR() {
  updateEncoderFromISR(2, M3_ENCA, M3_ENCB);
}





// ---------------------------------------------------------------------------------------------------------------------------
// Input-processing functions
// ---------------------------------------------------------------------------------------------------------------------------

float normalizeAxisWithDeadzone(
  int rawValue,
  int deadzone
) {
  int magnitude = abs(rawValue);

  if (magnitude <= deadzone) {
    return 0.0f;
  }

  float normalizedMagnitude =
    static_cast<float>(magnitude - deadzone) /
    static_cast<float>(AXIS_MAX - deadzone);

  normalizedMagnitude = constrain(
    normalizedMagnitude,
    0.0f,
    1.0f
  );

  return rawValue < 0
           ? -normalizedMagnitude
           : normalizedMagnitude;
}





// ---------------------------------------------------------------------------------------------------------------------------
// Kiwi-drive calculations
// ---------------------------------------------------------------------------------------------------------------------------

void driveKiwi(
  float x,
  float y,
  float rotation
) {
  const float SIN_60 = 0.8660254f;

  x *= TRANSLATION_SCALE;
  y *= TRANSLATION_SCALE;
  rotation *= ROTATION_SCALE;

  /*
            FRONT OF ROBOT
                ^

        Motor 1       Motor 3
             \         /
              \       /
                Motor 2

                  BACK

    x: positive = robot right
    y: positive = robot forward
  */

  float wheel1 =
    (-0.5f * x) +
    (SIN_60 * y) +
    rotation;

  float wheel2 =
      x +
      rotation;

  float wheel3 =
      (-0.5f * x) -
      (SIN_60 * y) +
      rotation;

  // Preserve wheel ratios while limiting every command to +/-1.
  float largest = fabsf(wheel1);

  if (fabsf(wheel2) > largest) {
    largest = fabsf(wheel2);
  }

  if (fabsf(wheel3) > largest) {
    largest = fabsf(wheel3);
  }

  if (largest > 1.0f) {
    wheel1 /= largest;
    wheel2 /= largest;
    wheel3 /= largest;
  }

  // Create target RPM instead of direct PWM.
  targetRPM[0] =
    wheel1 * MAX_TARGET_RPM * M1_DIRECTION;

  targetRPM[1] =
    wheel2 * MAX_TARGET_RPM * M2_DIRECTION;

  targetRPM[2] =
    wheel3 * MAX_TARGET_RPM * M3_DIRECTION;

  for (int i = 0; i < 3; i++) {
    if (fabsf(targetRPM[i]) < TARGET_RPM_DEADBAND) {
      targetRPM[i] = 0.0f;
    }
  }
}





// ---------------------------------------------------------------------------------------------------------------------------
// Velocity PID
// ---------------------------------------------------------------------------------------------------------------------------

void applyMotorCommand(uint8_t motorIndex, int command) {
  switch (motorIndex) {
    case 0:
      motor1Command = command;
      setMotor(
        M1_PWM,
        M1_PWM_CHANNEL,
        M1_IN1,
        M1_IN2,
        motor1Command
      );
      break;

    case 1:
      motor2Command = command;
      setMotor(
        M2_PWM,
        M2_PWM_CHANNEL,
        M2_IN1,
        M2_IN2,
        motor2Command
      );
      break;

    case 2:
      motor3Command = command;
      setMotor(
        M3_PWM,
        M3_PWM_CHANNEL,
        M3_IN1,
        M3_IN2,
        motor3Command
      );
      break;
  }
}

void runVelocityPID() {
  const uint32_t currentTimeUs = micros();

  if (
    static_cast<uint32_t>(
      currentTimeUs - previousPIDTimeUs
    ) < PID_PERIOD_US
  ) {
    return;
  }

  const float dt =
    static_cast<float>(
      static_cast<uint32_t>(
        currentTimeUs - previousPIDTimeUs
      )
    ) / 1000000.0f;

  previousPIDTimeUs = currentTimeUs;

  // Reject unreasonable timing intervals.
  if (dt <= 0.0f || dt > 0.25f) {
    resetVelocityController();
    return;
  }

  int32_t currentCounts[3];

  portENTER_CRITICAL(&encoderMux);
  currentCounts[0] = encoderCount[0];
  currentCounts[1] = encoderCount[1];
  currentCounts[2] = encoderCount[2];
  portEXIT_CRITICAL(&encoderMux);

  for (uint8_t i = 0; i < 3; i++) {
    const int32_t deltaCounts =
      currentCounts[i] -
      previousPIDEncoderCount[i];

    previousPIDEncoderCount[i] =
      currentCounts[i];

    rawRPM[i] =
      (
        static_cast<float>(deltaCounts) /
        COUNTS_PER_REVOLUTION
      ) *
      (60.0f / dt);

    measuredRPM[i] =
      RPM_FILTER_ALPHA * rawRPM[i] +
      (1.0f - RPM_FILTER_ALPHA) * measuredRPM[i];

    if (
      emergencyStopActive ||
      fabsf(targetRPM[i]) <
        TARGET_RPM_DEADBAND
    ) {
      velocityIntegral[i] = 0.0f;
      previousVelocityError[i] = 0.0f;
      applyMotorCommand(i, 0);
      continue;
    }

    const float error =
      targetRPM[i] - measuredRPM[i];

    velocityIntegral[i] += error * dt;

    velocityIntegral[i] = constrain(
      velocityIntegral[i],
      -INTEGRAL_LIMIT,
      INTEGRAL_LIMIT
    );

    const float derivative =
      (error - previousVelocityError[i]) / dt;

    previousVelocityError[i] = error;

    /*
      Feed-forward supplies most of the expected PWM.
      PID corrects motor mismatch, load, friction, and
      battery-voltage changes.
    */
    const float feedForward =
      (targetRPM[i] / MAX_TARGET_RPM) *
      static_cast<float>(MAX_PWM);

    const float correction =
      velocityKp[i] * error +
      velocityKi[i] * velocityIntegral[i] +
      velocityKd[i] * derivative;

    float pwmOutput =
      feedForward + correction;

    pwmOutput = constrain(
      pwmOutput,
      -static_cast<float>(MAX_PWM),
      static_cast<float>(MAX_PWM)
    );

    // Overcome static friction without changing direction.
    if (
      pwmOutput > 0.0f &&
      pwmOutput < static_cast<float>(MIN_PWM)
    ) {
      pwmOutput = static_cast<float>(MIN_PWM);
    } else if (
      pwmOutput < 0.0f &&
      pwmOutput > -static_cast<float>(MIN_PWM)
    ) {
      pwmOutput = -static_cast<float>(MIN_PWM);
    }

    applyMotorCommand(
      i,
      static_cast<int>(roundf(pwmOutput))
    );
  }
}





// ---------------------------------------------------------------------------------------------------------------------------
// Manual / autonomous mode and serial commands
// ---------------------------------------------------------------------------------------------------------------------------

const char* controlModeName() {
  return controlMode == ControlMode::MANUAL
           ? "MANUAL"
           : "AUTO";
}

void setControlMode(
  ControlMode newMode,
  const char* source
) {
  if (controlMode == newMode) {
    return;
  }

  stopAllMotors();
  resetVelocityController();

  controlMode = newMode;
  autoTimeoutActive = false;

  if (
    controlMode == ControlMode::AUTONOMOUS
  ) {
    lastAutoCommandTimeMs = millis();
  }

  if (!emergencyStopActive) {
    enableMotorDrivers();
  }

  Serial.printf(
    "MODE,%s,SOURCE,%s\n",
    controlModeName(),
    source
  );
}

void processCompleteSerialCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command.equalsIgnoreCase("MODE,AUTO")) {
    setControlMode(
      ControlMode::AUTONOMOUS,
      "SERIAL"
    );

    Serial.println("ACK,MODE,AUTO");
    lastAutoCommandTimeMs = millis();
    autoTimeoutActive = false;
    return;
  }

  if (command.equalsIgnoreCase("MODE,MANUAL")) {
    setControlMode(
      ControlMode::MANUAL,
      "SERIAL"
    );

    Serial.println("ACK,MODE,MANUAL");
    return;
  }

  if (command.equalsIgnoreCase("STOP")) {
    stopAllMotors();
    Serial.println("ACK,STOP");
    return;
  }

  if (command.equalsIgnoreCase("PING")) {
    Serial.println("ACK,PONG");
    return;
  }

  float x = 0.0f;
  float y = 0.0f;
  float rotation = 0.0f;

  const int parsed = sscanf(
    command.c_str(),
    "V,%f,%f,%f",
    &x,
    &y,
    &rotation
  );

  if (parsed == 3) {
    if (
      controlMode !=
      ControlMode::AUTONOMOUS
    ) {
      Serial.println("ERR,NOT_IN_AUTO_MODE");
      return;
    }

    if (emergencyStopActive) {
      stopAllMotors();
      Serial.println("ERR,ESTOP_ACTIVE");
      return;
    }

    x = constrain(x, -1.0f, 1.0f);
    y = constrain(y, -1.0f, 1.0f);
    rotation = constrain(
      rotation,
      -1.0f,
      1.0f
    );

    enableMotorDrivers();
    driveKiwi(x, y, rotation);

    lastAutoCommandTimeMs = millis();
    autoTimeoutActive = false;

    Serial.printf(
      "ACK,V,%.4f,%.4f,%.4f\n",
      x,
      y,
      rotation
    );
    return;
  }

  Serial.print("ERR,UNKNOWN_COMMAND,");
  Serial.println(command);
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming =
      static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      processCompleteSerialCommand(
        serialCommandBuffer
      );

      serialCommandBuffer = "";
      continue;
    }

    if (serialCommandBuffer.length() < 96) {
      serialCommandBuffer += incoming;
    } else {
      serialCommandBuffer = "";
      Serial.println("ERR,COMMAND_TOO_LONG");
    }
  }
}

void checkAutonomousTimeout() {
  if (
    controlMode != ControlMode::AUTONOMOUS
  ) {
    return;
  }

  if (
    millis() - lastAutoCommandTimeMs <=
    AUTO_COMMAND_TIMEOUT_MS
  ) {
    return;
  }

  stopAllMotors();

  if (!autoTimeoutActive) {
    autoTimeoutActive = true;
    Serial.println("WARN,AUTO_COMMAND_TIMEOUT");
  }
}





// ---------------------------------------------------------------------------------------------------------------------------
// Serial diagnostics
// ---------------------------------------------------------------------------------------------------------------------------

void printControllerData(
  ControllerPtr ctl,
  int rawLX,
  int rawLY,
  int rawRX,
  float x,
  float y,
  float rotation
) {
  const unsigned long currentTime = millis();

  if (
    currentTime - lastPrintTime <
    PRINT_INTERVAL_MS
  ) {
    return;
  }

  lastPrintTime = currentTime;

  Serial.printf(
    "LX:%4d LY:%4d RX:%4d  "
    "X:%+.2f Y:%+.2f ROT:%+.2f  "
    "T_RPM[%+.1f,%+.1f,%+.1f]  "
    "M_RPM[%+.1f,%+.1f,%+.1f]  "
    "PWM[%4d,%4d,%4d]  "
    "ENC[%ld,%ld,%ld]  "
    "MODE:%s  ESTOP:%s\n",

    rawLX,
    rawLY,
    rawRX,

    x,
    y,
    rotation,

    targetRPM[0],
    targetRPM[1],
    targetRPM[2],

    measuredRPM[0],
    measuredRPM[1],
    measuredRPM[2],

    motor1Command,
    motor2Command,
    motor3Command,

    static_cast<long>(encoderCount[0]),
    static_cast<long>(encoderCount[1]),
    static_cast<long>(encoderCount[2]),

    controlModeName(),
    emergencyStopActive ? "YES" : "NO"
  );
}





// ---------------------------------------------------------------------------------------------------------------------------
// Gamepad processing
// ---------------------------------------------------------------------------------------------------------------------------

void processGamepad(ControllerPtr ctl) {
  const int rawLeftX = ctl->axisX();
  const int rawLeftY = ctl->axisY();
  const int rawRightX = ctl->axisRX();

  const float leftX =
    normalizeAxisWithDeadzone(
      rawLeftX,
      LEFT_STICK_DEADZONE
    );

  const float leftY =
    normalizeAxisWithDeadzone(
      rawLeftY,
      LEFT_STICK_DEADZONE
    );

  const float rightX =
    normalizeAxisWithDeadzone(
      rawRightX,
      RIGHT_STICK_DEADZONE
    );

  const float x = -leftX;
  const float y = -leftY;
  const float rotation = rightX;

  // Start toggles MANUAL / AUTO on the press edge.
  const bool startPressed =
    (ctl->miscButtons() & START_MISC_MASK) != 0;

  if (
    startPressed &&
    !startButtonWasPressed
  ) {
    if (
      controlMode == ControlMode::MANUAL
    ) {
      setControlMode(
        ControlMode::AUTONOMOUS,
        "START_BUTTON"
      );
    } else {
      setControlMode(
        ControlMode::MANUAL,
        "START_BUTTON"
      );
    }
  }

  startButtonWasPressed = startPressed;

  // A button latches the emergency stop in either mode.
  if (ctl->a()) {
    emergencyStopActive = true;
    stopAllMotors();
    disableMotorDrivers();
  }

  // B clears emergency stop in either mode.
  if (ctl->b() && emergencyStopActive) {
    emergencyStopActive = false;

    stopAllMotors();
    resetVelocityController();
    enableMotorDrivers();

    if (
      controlMode == ControlMode::AUTONOMOUS
    ) {
      lastAutoCommandTimeMs = millis();
    }

    Serial.println("ACK,ESTOP,CLEARED");
  }

  if (emergencyStopActive) {
    stopAllMotors();

    printControllerData(
      ctl,
      rawLeftX,
      rawLeftY,
      rawRightX,
      x,
      y,
      rotation
    );
    return;
  }

  // In AUTO, controller buttons still work, but joystick
  // motion is ignored. The Pi supplies V,x,y,rotation.
  if (
    controlMode == ControlMode::AUTONOMOUS
  ) {
    printControllerData(
      ctl,
      rawLeftX,
      rawLeftY,
      rawRightX,
      0.0f,
      0.0f,
      0.0f
    );
    return;
  }

  // Manual joystick mode.
  if (
    x == 0.0f &&
    y == 0.0f &&
    rotation == 0.0f
  ) {
    stopAllMotors();

    printControllerData(
      ctl,
      rawLeftX,
      rawLeftY,
      rawRightX,
      0.0f,
      0.0f,
      0.0f
    );
    return;
  }

  enableMotorDrivers();
  driveKiwi(x, y, rotation);

  printControllerData(
    ctl,
    rawLeftX,
    rawLeftY,
    rawRightX,
    x,
    y,
    rotation
  );
}

void processControllers() {
  for (ControllerPtr ctl : myControllers) {
    if (
      ctl != nullptr &&
      ctl->isConnected() &&
      ctl->hasData() &&
      ctl->isGamepad()
    ) {
      processGamepad(ctl);
    }
  }
}





// ---------------------------------------------------------------------------------------------------------------------------
// ROS / odometry telemetry
// ---------------------------------------------------------------------------------------------------------------------------

void sendOdometryTelemetry() {
  const unsigned long now = millis();

  if (
    now - lastOdomTelemetryTime <
    ODOM_TELEMETRY_INTERVAL_MS
  ) {
    return;
  }

  lastOdomTelemetryTime = now;

  int32_t count1;
  int32_t count2;
  int32_t count3;

  portENTER_CRITICAL(&encoderMux);

  count1 = encoderCount[0];
  count2 = encoderCount[1];
  count3 = encoderCount[2];

  portEXIT_CRITICAL(&encoderMux);

  Serial.printf(
    "ODOM,%.4f,%.4f,%.4f,%ld,%ld,%ld\n",
    measuredRPM[0],
    measuredRPM[1],
    measuredRPM[2],
    static_cast<long>(count1),
    static_cast<long>(count2),
    static_cast<long>(count3)
  );
}





// ---------------------------------------------------------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("Starting Kiwi Drive Velocity PID");
  Serial.println("======================================");

  pinMode(STBY, OUTPUT);
  disableMotorDrivers();

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  /*
    GPIO34, GPIO35, GPIO36, and GPIO39 do not have
    internal pull-up or pull-down resistors.
  */
  pinMode(M1_ENCA, INPUT);
  pinMode(M1_ENCB, INPUT);

  pinMode(M2_ENCA, INPUT);
  pinMode(M2_ENCB, INPUT);

  pinMode(M3_ENCA, INPUT);
  pinMode(M3_ENCB, INPUT);

  previousEncoderState[0] =
    (digitalRead(M1_ENCA) << 1) |
    digitalRead(M1_ENCB);

  previousEncoderState[1] =
    (digitalRead(M2_ENCA) << 1) |
    digitalRead(M2_ENCB);

  previousEncoderState[2] =
    (digitalRead(M3_ENCA) << 1) |
    digitalRead(M3_ENCB);

  attachInterrupt(
    digitalPinToInterrupt(M1_ENCA),
    motor1EncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(M1_ENCB),
    motor1EncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(M2_ENCA),
    motor2EncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(M2_ENCB),
    motor2EncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(M3_ENCA),
    motor3EncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(M3_ENCB),
    motor3EncoderISR,
    CHANGE
  );

  configurePWM();

  stopAllMotors();
  resetVelocityController();
  enableMotorDrivers();

  BP32.setup(
    &onConnectedController,
    &onDisconnectedController
  );

  BP32.enableVirtualDevice(false);

  // Keep commented unless intentionally clearing pairings.
  // BP32.forgetBluetoothKeys();

  Serial.printf(
    "Bluepad32 firmware: %s\n",
    BP32.firmwareVersion()
  );

  Serial.println();
  Serial.println("Pin assignments:");

  Serial.printf(
    "M1: PWM=%u IN1=%u IN2=%u ENCA=%u ENCB=%u\n",
    M1_PWM,
    M1_IN1,
    M1_IN2,
    M1_ENCA,
    M1_ENCB
  );

  Serial.printf(
    "M2: PWM=%u IN1=%u IN2=%u ENCA=%u ENCB=%u\n",
    M2_PWM,
    M2_IN1,
    M2_IN2,
    M2_ENCA,
    M2_ENCB
  );

  Serial.printf(
    "M3: PWM=%u IN1=%u IN2=%u ENCA=%u ENCB=%u\n",
    M3_PWM,
    M3_IN1,
    M3_IN2,
    M3_ENCA,
    M3_ENCB
  );

  Serial.printf(
    "CPR: %.1f | Max target: %.1f RPM | PID: %lu us\n",
    COUNTS_PER_REVOLUTION,
    MAX_TARGET_RPM,
    PID_PERIOD_US
  );

  Serial.println();
  Serial.println("Controls:");
  Serial.println("Left stick: translate");
  Serial.println("Right stick X: rotate");
  Serial.println("A button: emergency stop");
  Serial.println("B button: clear emergency stop");
  Serial.println("Start (misc 0x04): toggle MANUAL / AUTO");
  Serial.println("Serial: MODE,AUTO | MODE,MANUAL");
  Serial.println("Serial: V,x,y,rotation | STOP | PING");
  Serial.println();
  Serial.println("Waiting for Bluetooth controller...");
}





// ---------------------------------------------------------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------------------------------------------------------

void loop() {
  BP32.update();

  // USB commands from Raspberry Pi / ROS 2.
  processSerialCommands();

  // Keep controller buttons active in both modes.
  processControllers();

  // Safety watchdog for AUTO mode.
  checkAutonomousTimeout();

  // Per-wheel velocity PID at 50 Hz.
  runVelocityPID();

  // Send wheel RPM + encoder counts for ROS odometry.
  sendOdometryTelemetry();

  delay(1);
}
