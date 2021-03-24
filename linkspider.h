#ifndef LINK_SPIDER
#define LINK_SPIDER

#include <math.h>
#include <stdio.h>

enum LinkSpider_LegIndex_t {
  L1 = 0, R1 = 1,
  L2 = 2, R2 = 3,
  L3 = 4, R3 = 5
};

enum LinkSpider_ServoIndex_t {
  SERVO_1 = 0,
  SERVO_2 = 1,
  SERVO_3 = 2
};

enum LinkSpider_ComboIndex_t {
  COMBO_A = 0,
  COMBO_B = 1
};

class LinkSpider_Leg
{
protected:
  double state_anchor[4]; // x0, y0, z0, r0 (rad)
  double state_frame[3]; // coxa, femur, tibia (cm)
  double state_pwm[3][2]; // normal pos, ratio per pwm (rad)
  double state_target[3]; // x, y, z
  double computed_angle[3]; // teta, beta, alpha (rad)

public:
  LinkSpider_Leg() {
    state_anchor[0] = 0;
    state_anchor[1] = 0;
    state_anchor[2] = 0;
    state_anchor[3] = 0;

    state_frame[0] = 4;
    state_frame[1] = 4;
    state_frame[2] = 4;

    state_pwm[0][0] = 0;
    state_pwm[0][1] = 1000 / M_PI;
    state_pwm[1][0] = 0;
    state_pwm[1][1] = 1000 / M_PI;
    state_pwm[2][0] = 0;
    state_pwm[2][1] = 1000 / M_PI;
  }

public:
  void compute () {
    double diffX = state_target[0] - state_anchor[0];
    double diffY = state_target[1] - state_anchor[1];
    double diffZ = state_target[2] - state_anchor[2];

    double rotatedX = diffX * cos(state_anchor[3]) + diffY * sin(state_anchor[3]);
    double rotatedY = diffY * cos(state_anchor[3]) - diffX * sin(state_anchor[3]);

    double r = sqrt(pow(rotatedY, 2) + pow(rotatedX, 2));
    double p = r - state_frame[0];
    double u = sqrt(pow(p, 2) + pow(diffZ, 2));
    double gama = p > 0 ? atan(-diffZ / p) : M_PI + atan(-diffZ / p);

    computed_angle[0] = atan(rotatedY / rotatedX);
    computed_angle[1] = acos((pow(u, 2) + pow(state_frame[1], 2) - pow(state_frame[2], 2)) / (2 * state_frame[1] * u)) - gama;
    computed_angle[2] = acos((pow(state_frame[2], 2) + pow(state_frame[1], 2) - pow(u, 2)) / (2 * state_frame[2] * state_frame[1]));
  }

public:
  void setAnchorPos (double x0, double y0, double z0) {
    state_anchor[0] = x0;
    state_anchor[1] = y0;
    state_anchor[2] = z0;
  }

public:
  void setAnchorRotRad (double rad) {
    state_anchor[3] = rad;
  }

public:
  void setAnchorRotDeg (double deg) {
    state_anchor[3] = deg * M_PI / 180;
  }

public:
  void setFrameLength (double coxa, double femur, double tibia) {
    state_frame[0] = coxa;
    state_frame[1] = femur;
    state_frame[2] = tibia;
  }

public:
  void setNormalPosPWM (unsigned int index, double value) {
    state_pwm[index][0] = value;
  }

public:
  void setRatioRadPWM (unsigned int index, double radPerPWM) {
    state_pwm[index][1] = radPerPWM;
  }

public:
  void setRatioDegPWM (unsigned int index, double degPerPWM) {
    state_pwm[index][1] = degPerPWM * M_PI / 180;
  }

public:
  void setTipPos (double x, double y, double z) {
    state_target[0] = x;
    state_target[1] = y;
    state_target[2] = z;
  }

public:
  double getAngleRad (unsigned int index) {
    return computed_angle[index];
  }

public:
  double getAngleDeg (unsigned int index) {
    return computed_angle[index] * 180 / M_PI;
  }

public:
  double getAnglePWM (unsigned int index) {
    return state_pwm[index][0] + computed_angle[index] / state_pwm[index][1];
  }
};

class LinkSpider_Posture
{
protected:
  double state_frs;
  double state_ms;
  double state_l;
  double state_h;
  double state_rx;
  double state_ry;
  double state_rz;
  double computed_legs[6][3];

public:
  LinkSpider_Posture () {
    state_frs = 0;
    state_ms = 0;
    state_l = 0;
    state_h = 0;
    state_rx = 0;
    state_ry = 0;
    state_rz = 0;
  }

public:
  void compute () {
    computed_legs[0][0] = - state_frs / 2;
    computed_legs[0][1] = state_l / 2;
    computed_legs[0][2] = - state_h;

    computed_legs[2][0] = - state_ms / 2;
    computed_legs[2][1] = 0;
    computed_legs[2][2] = - state_h;

    computed_legs[4][0] = - state_frs / 2;
    computed_legs[4][1] = - state_l / 2;
    computed_legs[4][2] = - state_h;

    computed_legs[1][0] = state_frs / 2;
    computed_legs[1][1] = state_l / 2;
    computed_legs[1][2] = - state_h;

    computed_legs[3][0] = state_ms / 2;
    computed_legs[3][1] = 0;
    computed_legs[3][2] = - state_h;

    computed_legs[5][0] = state_frs / 2;
    computed_legs[5][1] = - state_l / 2;
    computed_legs[5][2] = - state_h;

    for (int i = 0; i < 6; i++) {
      double x0 = computed_legs[i][0];
      double y0 = computed_legs[i][1];
      double z0 = computed_legs[i][2];

      // process pitch
      double x1 = x0;
      double y1 = y0 * cos(state_rx) - z0 * sin(state_rx);
      double z1 = y0 * sin(state_rx) + z0 * cos(state_rx);

      // process roll
      double x2 = x1 * cos(state_ry) + z1 * sin(state_ry);
      double y2 = y1;
      double z2 = - x1 * sin(state_ry) + z1 * cos(state_ry);

      // process yaw
      double x3 = x2 * cos(state_rz) - y2 * sin(state_rz);
      double y3 = x2 * sin(state_rz) + y2 * cos(state_rz);
      double z3 = z2;

      computed_legs[i][0] = x3;
      computed_legs[i][1] = y3;
      computed_legs[i][2] = z3;
    }
  }

public:
  void setNormalPos (double frs, double ms, double l, double h) {
    // frs = front and rear span, ms = middle span, l = length, h = height
    state_frs = frs;
    state_ms = ms;
    state_l = l;
    state_h = h;
  }

public:
  void setRotationRad (double rx, double ry, double rz) {
    // rx = pitch, ry = roll, rz = yaw
    state_rx = rx;
    state_ry = ry;
    state_rz = rz;
  }

public:
  void setRotationDeg (double rx, double ry, double rz) {
    rx = rx * M_PI / 180;
    ry = ry * M_PI / 180;
    rz = rz * M_PI / 180;
    setRotationRad(rx, ry, rz);
  }

public:
  double getCoordinate (unsigned int legIndex, unsigned int vectorIndex) {
    // legIndex -> [L1, R1, L2, R2, L3, R3], vectorIndex -> [x, y, z]
    return computed_legs[legIndex][vectorIndex];
  }
};

class LinkSpider_ConnectorSSC32
{
protected:
  unsigned int state_pins[6][3];
  unsigned int state_values[6][3];
  unsigned int state_interval;
  char computed_printable[200]; // a 200 chars buffer string

public:
  LinkSpider_ConnectorSSC32 () {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 3; j++) {
        state_pins[i][j] = i * 3 + j;
        state_values[i][j] = 1500;
      }
    }

    state_interval = 200;
  }

public:
  void setInterval (unsigned int time) {
    state_interval = time;
  }

public:
  void setServoPin (unsigned int legIndex, unsigned int servoIndex, unsigned int pin) {
    state_pins[legIndex][servoIndex] = pin;
  }

public:
  void setServoValue (unsigned int legIndex, unsigned int servoIndex, double value) {
    double roundedValue = round(abs(value));
    state_values[legIndex][servoIndex] = (int) roundedValue;
  }

public:
  void compute () {
    sprintf(
      computed_printable,
      "#%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d #%dP%d T%d",
      state_pins[0][0], state_values[0][0],
      state_pins[0][1], state_values[0][1],
      state_pins[0][2], state_values[0][2],
      state_pins[1][0], state_values[1][0],
      state_pins[1][1], state_values[1][1],
      state_pins[1][2], state_values[1][2],
      state_pins[2][0], state_values[2][0],
      state_pins[2][1], state_values[2][1],
      state_pins[2][2], state_values[2][2],
      state_pins[3][0], state_values[3][0],
      state_pins[3][1], state_values[3][1],
      state_pins[3][2], state_values[3][2],
      state_pins[4][0], state_values[4][0],
      state_pins[4][1], state_values[4][1],
      state_pins[4][2], state_values[4][2],
      state_pins[5][0], state_values[5][0],
      state_pins[5][1], state_values[5][1],
      state_pins[5][2], state_values[5][2],
      state_interval
    );
  }

public:
  char * getPrintable () {
    return computed_printable;
  }
};

#endif
