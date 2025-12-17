#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse range for ~0° to ~180°  (servo dependent)
// -------------------------------------------------
#define SERVOMIN 100
#define SERVOMAX 600

// RING channels
// -------------------------------------------------
const int R_BASE  = 0;
const int R_TIP   = 1;
const int R_SPLAY = 2;

// PINKY channels
// -------------------------------------------------
const int P_BASE  = 3;
const int P_TIP   = 4;
const int P_SPLAY = 5;

// MIDDLE channels
// -------------------------------------------------
const int M_BASE  = 6;
const int M_TIP   = 7;
const int M_SPLAY = 8;

// INDEX channels
// -------------------------------------------------
const int I_BASE  = 9;
const int I_TIP   = 10;
const int I_SPLAY = 11;

// THUMB channels
// -------------------------------------------------
const int T_BASE  = 12;
const int T_TIP   = 15; //Roll
const int T_SPLAY = 14;

// Store last commanded angle to avoid hammering
int lastAngle[16] = { -1 };

// Safe servo update (prevents repeated signals)
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);

  // Ignore duplicate commands
  if (lastAngle[channel] == angle)
    return;

  lastAngle[channel] = angle;

  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulselen);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);

  // Initial positions for all fingers
  // -------------------------------------------------
  setServoAngle(R_BASE, 10);
  setServoAngle(R_TIP, 10);
  setServoAngle(R_SPLAY, 180);

  setServoAngle(P_BASE, 20);
  setServoAngle(P_TIP, 20);
  setServoAngle(P_SPLAY, 10);

  setServoAngle(M_BASE, 20);
  setServoAngle(M_TIP, 20);
  setServoAngle(M_SPLAY, 60);

  setServoAngle(I_BASE, 20);
  setServoAngle(I_TIP, 20);
  setServoAngle(I_SPLAY, 20);

  setServoAngle(T_BASE, 180);
  setServoAngle(T_TIP, 20);
  setServoAngle(T_SPLAY, 50);

  Serial.println("Ready for packets: P:x,y,z;R:x,y,z;M:x,y,z;I:x,y,z;T:x,y,z");
}

void processFingerCommand(char finger, int base, int tip, int splay) {

  if (finger == 'P') {
    setServoAngle(P_BASE, base);
    setServoAngle(P_TIP, tip);
    setServoAngle(P_SPLAY, splay);
  }

  else if (finger == 'R') {
    setServoAngle(R_BASE, base);
    setServoAngle(R_TIP, tip);
    setServoAngle(R_SPLAY, splay);
  }

  else if (finger == 'M') {
    setServoAngle(M_BASE, base);
    setServoAngle(M_TIP, tip);
    setServoAngle(M_SPLAY, splay);
  }

  else if (finger == 'I') {
    setServoAngle(I_BASE, base);
    setServoAngle(I_TIP, tip);
    setServoAngle(I_SPLAY, splay);
  }

  else if (finger == 'T') {
    setServoAngle(T_BASE, base);
    setServoAngle(T_TIP, tip);
    setServoAngle(T_SPLAY, splay);
  }
}

void processPacketPart(String part) {
  int colon = part.indexOf(':');
  int c1 = part.indexOf(',', colon + 1);
  int c2 = part.indexOf(',', c1 + 1);

  if (colon == -1 || c1 == -1 || c2 == -1) return;

  char finger = part.charAt(0);
  int base  = part.substring(colon + 1, c1).toInt();
  int tip   = part.substring(c1 + 1, c2).toInt();
  int splay = part.substring(c2 + 1).toInt();

  processFingerCommand(finger, base, tip, splay);
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  // Packets should be like:
  // P:x,y,z;R:x,y,z;M:x,y,z;I:x,y,z;T:x,y,z
  // Order does not matter
  // -------------------------------------------------
  int start = 0;

  while (true) {
    int nextSemi = line.indexOf(';', start);
    if (nextSemi == -1) break;

    String part = line.substring(start, nextSemi);
    processPacketPart(part);

    start = nextSemi + 1;
  }
}
