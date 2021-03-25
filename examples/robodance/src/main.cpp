#include <Arduino.h>
#include <SoftwareSerial.h>

#include "../../../linkspider.h"

SoftwareSerial serialSSC(19, 18);  // RX, TX

int pressed = 0;
double pitch = 0.0;
double roll = 0.0;
double yaw = 0.0;

LinkSpider_ConnectorSSC32 connectorSSC;
LinkSpider_Leg leg[6];
LinkSpider_Posture posture;

const double anchorRotDegs[6] = {
    135, 45,  // L1, R1
    180, 0,   // L2, R2
    225, 315  // L3, R3
};

void setup() {
  // Setup serial connection
  Serial.begin(9600);  // Serial Monitor
  serialSSC.begin(9600);     // Serial connection to SSC 32

  // Setup anchor positions
  leg[L1].setAnchorPos(-3.75, 7.5, 0);
  leg[R1].setAnchorPos(3.75, 7.5, 0);
  leg[L2].setAnchorPos(-3.75, 0, 0);
  leg[R2].setAnchorPos(3.75, 0, 0);
  leg[L3].setAnchorPos(-3.75, -7.5, 0);
  leg[R3].setAnchorPos(3.75, -7.5, 0);

  // Config from calibration example by pressing 'c' in serial monitor
  // BEGIN =============================
  leg[0].setAnchorRotDeg(135.00);
  leg[0].setNormalPosPWM(0, 1480.00);
  leg[0].setRatioDegPWM(0, 0.09);
  leg[0].setNormalPosPWM(1, 1440.00);
  leg[0].setRatioDegPWM(1, 0.10);
  leg[0].setNormalPosPWM(2, 2440.00);
  leg[0].setRatioDegPWM(2, -0.10);

  leg[1].setAnchorRotDeg(45.00);
  leg[1].setNormalPosPWM(0, 1650.00);
  leg[1].setRatioDegPWM(0, 0.09);
  leg[1].setNormalPosPWM(1, 1540.00);
  leg[1].setRatioDegPWM(1, -0.12);
  leg[1].setNormalPosPWM(2, 640.00);
  leg[1].setRatioDegPWM(2, 0.11);

  leg[2].setAnchorRotDeg(180.00);
  leg[2].setNormalPosPWM(0, 1450.00);
  leg[2].setRatioDegPWM(0, 0.09);
  leg[2].setNormalPosPWM(1, 1590.00);
  leg[2].setRatioDegPWM(1, 0.10);
  leg[2].setNormalPosPWM(2, 2430.00);
  leg[2].setRatioDegPWM(2, -0.10);

  leg[3].setAnchorRotDeg(0.00);
  leg[3].setNormalPosPWM(0, 1470.00);
  leg[3].setRatioDegPWM(0, 0.10);
  leg[3].setNormalPosPWM(1, 1590.00);
  leg[3].setRatioDegPWM(1, -0.11);
  leg[3].setNormalPosPWM(2, 440.00);
  leg[3].setRatioDegPWM(2, 0.12);

  leg[4].setAnchorRotDeg(225.00);
  leg[4].setNormalPosPWM(0, 1380.00);
  leg[4].setRatioDegPWM(0, 0.09);
  leg[4].setNormalPosPWM(1, 1160.00);
  leg[4].setRatioDegPWM(1, 0.11);
  leg[4].setNormalPosPWM(2, 2470.00);
  leg[4].setRatioDegPWM(2, -0.12);

  leg[5].setAnchorRotDeg(315.00);
  leg[5].setNormalPosPWM(0, 1210.00);
  leg[5].setRatioDegPWM(0, 0.08);
  leg[5].setNormalPosPWM(1, 1450.00);
  leg[5].setRatioDegPWM(1, -0.11);
  leg[5].setNormalPosPWM(2, 600.00);
  leg[5].setRatioDegPWM(2, 0.12);
  // END ===============================

  // Setup SSC 32 config
  connectorSSC.setInterval(200);
  connectorSSC.setServoPin(L1, SERVO_1, 4);
  connectorSSC.setServoPin(L1, SERVO_2, 5);
  connectorSSC.setServoPin(L1, SERVO_3, 6);
  connectorSSC.setServoPin(L2, SERVO_1, 8);
  connectorSSC.setServoPin(L2, SERVO_2, 9);
  connectorSSC.setServoPin(L2, SERVO_3, 10);
  connectorSSC.setServoPin(L3, SERVO_1, 12);
  connectorSSC.setServoPin(L3, SERVO_2, 13);
  connectorSSC.setServoPin(L3, SERVO_3, 14);
  connectorSSC.setServoPin(R1, SERVO_1, 20);
  connectorSSC.setServoPin(R1, SERVO_2, 21);
  connectorSSC.setServoPin(R1, SERVO_3, 22);
  connectorSSC.setServoPin(R2, SERVO_1, 24);
  connectorSSC.setServoPin(R2, SERVO_2, 25);
  connectorSSC.setServoPin(R2, SERVO_3, 26);
  connectorSSC.setServoPin(R3, SERVO_1, 28);
  connectorSSC.setServoPin(R3, SERVO_2, 29);
  connectorSSC.setServoPin(R3, SERVO_3, 30);

  // Set normal position (FRS, MS, L, H)
  posture.setNormalPos(20, 30, 30, 7);

  // Set frame length on each legs (coxa, femur, tibia)
  for (size_t i = 0; i < 6; i++) {
    leg[i].setFrameLength(3, 5, 9);
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Get pressed button
    while (Serial.available() > 0) {
      pressed = Serial.read();
    }

    // Config pitch/roll/yaw
    if (pressed == 'q') pitch -= 1;
    if (pressed == 'w') pitch += 1;
    if (pressed == 'a') roll -= 1;
    if (pressed == 's') roll += 1;
    if (pressed == 'z') yaw -= 1;
    if (pressed == 'x') yaw += 1;

    // Set rx, ry, rz and compute
    posture.setRotationRad(pitch * M_PI / 180, roll * M_PI / 180, yaw * M_PI / 180);
    posture.compute();

    // Set tip position of all legs, compute and set it to SSC servo PWM
    for (size_t i = 0; i < 6; i++) {
      leg[i].setTipPos(posture.getCoordinate(i, 0), posture.getCoordinate(i, 1), posture.getCoordinate(i, 2));
      leg[i].compute();

      for (size_t j = 0; j < 3; j++) {
        connectorSSC.setServoValue(i, j, leg[i].getAnglePWM(j));
      }
    }

    // Compute the SSC serial command and send to SSC serial connection
    connectorSSC.compute();
    serialSSC.println(connectorSSC.getPrintable());
  }

  // Debug
  Serial.println(String("Pitch: ") + pitch + " Roll: " + roll + " Yaw: " + yaw);
  delay(50);
}
