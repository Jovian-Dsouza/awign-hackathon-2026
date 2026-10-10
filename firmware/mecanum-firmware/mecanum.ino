// Four-wheel Mecanum drive for an Arduino Uno with a 4-channel L293D
// motor shield (Adafruit Motor Shield v1 / common "L293D Motor Drive
// Shield" clones).
//
// The shield latches motor direction through a 74HC595 and PWMs speed
// on the timer pins below. That is four independent H-bridges, which a
// Mecanum chassis needs. The official Arduino Motor Shield Rev3 only
// has two DC channels and will not run this sketch.
//
//   Shield terminal    Wheel              PWM pin
//   M1                 front left         11
//   M2                 front right         3
//   M3                 rear left           6
//   M4                 rear right          5
//
// Rollers, viewed from above, form an X:
//
//     FL \\    // FR
//     RL //    \\ RR
//
// Prefer the I command to flip a channel in software. Swapping the two
// motor wires also works if you want a permanent hardware fix.
//
// Serial protocol, 115200 8N1, one command per line:
//   V <vx> <vy> <wz>   body velocity, each axis -255..255
//                      +vx forward, +vy strafe left, +wz rotate CCW
//   P <0-255>          scale applied to letter commands
//   F B L R C A        forward, back, strafe left, strafe right,
//                      rotate clockwise, rotate counter-clockwise
//   M <1-4> <speed>    one motor, speed -255..255 (0 = coast that motor)
//   W <s1> <s2> <s3> <s4>  all four motors at once
//   I <i1> <i2> <i3> <i4>  per-motor invert flags, each 0 or 1
//   H <ms>             command watchdog timeout in ms (0 = off)
//   Z                  ping (reply Z) — used by the Pi to detect drops
//   S                  coast (all bridges released)
//   ?                  print this command set
//
// Example:  V 180 0 0     drive forward at about 70%
//           V 0 120 40    strafe left while yawing CCW
//           M 1 120       run motor 1 forward at ~47%
//           I 0 1 0 0     invert motor 2

const uint8_t MOTOR_LATCH = 12;
const uint8_t MOTOR_CLOCK = 4;
const uint8_t MOTOR_ENABLE = 7;  // 74HC595 output enable, active low
const uint8_t MOTOR_DATA = 8;

const uint8_t MOTOR1_PWM = 11;
const uint8_t MOTOR2_PWM = 3;
const uint8_t MOTOR3_PWM = 6;
const uint8_t MOTOR4_PWM = 5;

// Bit positions in the 74HC595 latch. A/B are the two direction inputs
// of each L293D half-bridge.
const uint8_t MOTOR1_A = 2;
const uint8_t MOTOR1_B = 3;
const uint8_t MOTOR2_A = 1;
const uint8_t MOTOR2_B = 4;
const uint8_t MOTOR3_A = 5;
const uint8_t MOTOR3_B = 7;
const uint8_t MOTOR4_A = 0;
const uint8_t MOTOR4_B = 6;

const int8_t RELEASED = 0;
const int8_t FORWARD = 1;
const int8_t BACKWARD = -1;

uint8_t latchState = 0;
int commandScale = 200;
uint8_t motorInvert[4] = {0, 0, 0, 0};
int motorSpeed[4] = {0, 0, 0, 0};
unsigned long watchdogMs = 0;
unsigned long lastCommandMs = 0;

void latchWrite() {
  digitalWrite(MOTOR_LATCH, LOW);
  // The shield clocks bit 7 first.
  for (int8_t bit = 7; bit >= 0; bit--) {
    digitalWrite(MOTOR_CLOCK, LOW);
    digitalWrite(MOTOR_DATA, (latchState & (1 << bit)) ? HIGH : LOW);
    digitalWrite(MOTOR_CLOCK, HIGH);
  }
  digitalWrite(MOTOR_LATCH, HIGH);
}

void setBridge(uint8_t bitA, uint8_t bitB, int8_t direction) {
  latchState &= ~((1 << bitA) | (1 << bitB));
  if (direction > 0) {
    latchState |= (1 << bitA);
  } else if (direction < 0) {
    latchState |= (1 << bitB);
  }
  latchWrite();
}

void setMotorRaw(uint8_t pwmPin, uint8_t bitA, uint8_t bitB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    analogWrite(pwmPin, 0);
    setBridge(bitA, bitB, RELEASED);
    return;
  }
  setBridge(bitA, bitB, speed > 0 ? FORWARD : BACKWARD);
  analogWrite(pwmPin, abs(speed));
}

void applyMotor(uint8_t index, int speed) {
  speed = constrain(speed, -255, 255);
  motorSpeed[index] = speed;
  if (motorInvert[index]) {
    speed = -speed;
  }
  switch (index) {
    case 0:
      setMotorRaw(MOTOR1_PWM, MOTOR1_A, MOTOR1_B, speed);
      break;
    case 1:
      setMotorRaw(MOTOR2_PWM, MOTOR2_A, MOTOR2_B, speed);
      break;
    case 2:
      setMotorRaw(MOTOR3_PWM, MOTOR3_A, MOTOR3_B, speed);
      break;
    case 3:
      setMotorRaw(MOTOR4_PWM, MOTOR4_A, MOTOR4_B, speed);
      break;
  }
}

// Positive speed drives the wheel so the robot moves forward when all
// four speeds are equal and positive.
void driveWheels(int frontLeft, int frontRight, int rearLeft, int rearRight) {
  applyMotor(0, frontLeft);
  applyMotor(1, frontRight);
  applyMotor(2, rearLeft);
  applyMotor(3, rearRight);
}

// X-configuration Mecanum inverse kinematics.
//   +vx  forward
//   +vy  strafe left
//   +wz  yaw counter-clockwise
void driveBody(int vx, int vy, int wz) {
  long fl = (long)vx - vy - wz;
  long fr = (long)vx + vy + wz;
  long rl = (long)vx + vy - wz;
  long rr = (long)vx - vy + wz;

  long peak = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
  if (peak > 255) {
    fl = fl * 255 / peak;
    fr = fr * 255 / peak;
    rl = rl * 255 / peak;
    rr = rr * 255 / peak;
  }

  driveWheels((int)fl, (int)fr, (int)rl, (int)rr);
}

void stopAll() {
  driveWheels(0, 0, 0, 0);
}

void printHelp() {
  // First line is a stable marker the Pi watches for after USB brownouts.
  Serial.println(F("READY mecanum"));
  Serial.println(F("Mecanum ready"));
  Serial.println(F("V <vx> <vy> <wz>   -255..255"));
  Serial.println(F("P <0-255>          letter-command scale"));
  Serial.println(F("M <1-4> <speed>    one motor -255..255"));
  Serial.println(F("W <s1> <s2> <s3> <s4>"));
  Serial.println(F("I <i1> <i2> <i3> <i4>  invert 0/1"));
  Serial.println(F("H <ms>             watchdog (0=off)"));
  Serial.println(F("Z                  ping"));
  Serial.println(F("F B L R C A S"));
}

void markCommand() {
  lastCommandMs = millis();
}

void handleLine(char *line) {
  while (*line == ' ') {
    line++;
  }
  if (*line == '\0') {
    return;
  }

  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') {
    cmd = cmd - 'a' + 'A';
  }

  int vx = 0;
  int vy = 0;
  int wz = 0;
  int scale = commandScale;
  int motor = 0;
  int speed = 0;
  int s1 = 0;
  int s2 = 0;
  int s3 = 0;
  int s4 = 0;
  int i1 = 0;
  int i2 = 0;
  int i3 = 0;
  int i4 = 0;
  long timeout = 0;

  switch (cmd) {
    case 'V':
      if (sscanf(line + 1, "%d %d %d", &vx, &vy, &wz) == 3) {
        driveBody(vx, vy, wz);
        markCommand();
        Serial.print(F("V "));
        Serial.print(vx);
        Serial.print(' ');
        Serial.print(vy);
        Serial.print(' ');
        Serial.println(wz);
      } else {
        Serial.println(F("ERR V vx vy wz"));
      }
      break;
    case 'P':
      if (sscanf(line + 1, "%d", &scale) == 1) {
        commandScale = constrain(scale, 0, 255);
        markCommand();
        Serial.print(F("P "));
        Serial.println(commandScale);
      } else {
        Serial.println(F("ERR P 0-255"));
      }
      break;
    case 'M':
      if (sscanf(line + 1, "%d %d", &motor, &speed) == 2 &&
          motor >= 1 && motor <= 4) {
        applyMotor((uint8_t)(motor - 1), speed);
        markCommand();
        Serial.print(F("M "));
        Serial.print(motor);
        Serial.print(' ');
        Serial.println(motorSpeed[motor - 1]);
      } else {
        Serial.println(F("ERR M 1-4 speed"));
      }
      break;
    case 'W':
      if (sscanf(line + 1, "%d %d %d %d", &s1, &s2, &s3, &s4) == 4) {
        driveWheels(s1, s2, s3, s4);
        markCommand();
        Serial.print(F("W "));
        Serial.print(motorSpeed[0]);
        Serial.print(' ');
        Serial.print(motorSpeed[1]);
        Serial.print(' ');
        Serial.print(motorSpeed[2]);
        Serial.print(' ');
        Serial.println(motorSpeed[3]);
      } else {
        Serial.println(F("ERR W s1 s2 s3 s4"));
      }
      break;
    case 'I':
      if (sscanf(line + 1, "%d %d %d %d", &i1, &i2, &i3, &i4) == 4) {
        motorInvert[0] = i1 ? 1 : 0;
        motorInvert[1] = i2 ? 1 : 0;
        motorInvert[2] = i3 ? 1 : 0;
        motorInvert[3] = i4 ? 1 : 0;
        // Re-apply current logical speeds so a spinning wheel reverses now.
        for (uint8_t i = 0; i < 4; i++) {
          applyMotor(i, motorSpeed[i]);
        }
        markCommand();
        Serial.print(F("I "));
        Serial.print(motorInvert[0]);
        Serial.print(' ');
        Serial.print(motorInvert[1]);
        Serial.print(' ');
        Serial.print(motorInvert[2]);
        Serial.print(' ');
        Serial.println(motorInvert[3]);
      } else {
        Serial.println(F("ERR I i1 i2 i3 i4"));
      }
      break;
    case 'H':
      if (sscanf(line + 1, "%ld", &timeout) == 1 && timeout >= 0) {
        watchdogMs = (unsigned long)timeout;
        markCommand();
        Serial.print(F("H "));
        Serial.println(watchdogMs);
      } else {
        Serial.println(F("ERR H ms"));
      }
      break;
    case 'F':
      driveBody(commandScale, 0, 0);
      markCommand();
      Serial.println(F("F"));
      break;
    case 'B':
      driveBody(-commandScale, 0, 0);
      markCommand();
      Serial.println(F("B"));
      break;
    case 'L':
      driveBody(0, commandScale, 0);
      markCommand();
      Serial.println(F("L"));
      break;
    case 'R':
      driveBody(0, -commandScale, 0);
      markCommand();
      Serial.println(F("R"));
      break;
    case 'C':
      driveBody(0, 0, -commandScale);
      markCommand();
      Serial.println(F("C"));
      break;
    case 'A':
      driveBody(0, 0, commandScale);
      markCommand();
      Serial.println(F("A"));
      break;
    case 'S':
      stopAll();
      markCommand();
      Serial.println(F("S"));
      break;
    case 'Z':
      // Ping only — do not refresh the motion watchdog.
      Serial.println(F("Z"));
      break;
    case '?':
      printHelp();
      markCommand();
      break;
    default:
      Serial.println(F("ERR unknown"));
      break;
  }
}

void setup() {
  pinMode(MOTOR_LATCH, OUTPUT);
  pinMode(MOTOR_CLOCK, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_DATA, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);

  latchState = 0;
  latchWrite();
  digitalWrite(MOTOR_ENABLE, LOW);  // enable the latch outputs
  stopAll();

  Serial.begin(115200);
  lastCommandMs = millis();
  printHelp();
}

void loop() {
  static char buffer[48];
  static uint8_t length = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buffer[length] = '\0';
      handleLine(buffer);
      length = 0;
      continue;
    }
    if (length < sizeof(buffer) - 1) {
      buffer[length++] = c;
    }
  }

  if (watchdogMs > 0) {
    unsigned long now = millis();
    if ((now - lastCommandMs) >= watchdogMs) {
      stopAll();
      lastCommandMs = now;
    }
  }
}
