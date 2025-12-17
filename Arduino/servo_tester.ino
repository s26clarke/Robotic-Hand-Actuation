#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100  // Min pulse length count (~0°)
#define SERVOMAX  600  // Max pulse length count (~180°)

void setup() {
  Serial.begin(9600);
  Serial.println("PCA9685 Servo Angle Control");
  Serial.println("Enter as: channel,angle  (e.g., 0,90)");

  pwm.begin();
  pwm.setPWMFreq(60);   // 50–60 Hz for most analog servos
  delay(10);

  //INIT TO 0 pos


  //THUMB
  setServoAngle(12, 180); //BASE (180-20)
  delay(100);
  setServoAngle(15, 20); //ROLL (20-80)
  delay(100);
  setServoAngle(14, 50); //SPLAY (50-160)
  delay(100);

  //INDEX
  setServoAngle(9, 20); //BASE (20-90)
  delay(100);
  setServoAngle(10, 20); //TIP (20-180)
  delay(100);
  setServoAngle(11, 20); //SPLAY (20-40)
  delay(100);

  //MIDDLE 
  setServoAngle(6, 20); //BASE (10-90)
  delay(100);
  setServoAngle(7, 20); //TIP (20-150)
  delay(100);
  setServoAngle(8, 60); //SPLAY (60-80)
  delay(100);


  //RING 
  setServoAngle(0, 10); //BASE (10-180)
  delay(100);
  setServoAngle(1, 10); //TIP
  delay(100);
  setServoAngle(2, 180); //SPLAY (180-110)
  delay(100);

  //PINKY
  setServoAngle(3, 20); //BASE (20,90)
  delay(100);
  setServoAngle(4, 20); //TIP (20-150)
  delay(100);
  setServoAngle(5, 10); //SPLAY (10-60)
}

// Function to move a servo on a specific channel to a specific angle
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);  // keep within valid range
  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulselen);

  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" → Angle: ");
  Serial.print(angle);
  Serial.print("° (Pulse ");
  Serial.print(pulselen);
  Serial.println(")");


  
}

void loop() {
  // Check if serial data is available
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // read a line
    input.trim();  // remove whitespace/newline

    // Find the comma position
    int commaIndex = input.indexOf(',');

    if (commaIndex > 0) {
      // Extract channel and angle
      int channel = input.substring(0, commaIndex).toInt();
      int angle = input.substring(commaIndex + 1).toInt();

      // Set the servo angle
      setServoAngle(channel, angle);
    } else {
      Serial.println("Invalid input. Use format: channel,angle (e.g., 2,120)");
    }
  }
}
