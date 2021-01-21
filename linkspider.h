#ifndef LINK_SPIDER
#define LINK_SPIDER

#include <math.h>

class LinkSpider_Leg
{
protected:
  double anchor[4]; // x0, y0, z0, r0 (rad)
  double frame[3]; // coxa, femur, tibia (cm)
  double pwm[3][2]; // normal pos, ratio per pwm (rad)
  double angle[3]; // teta, beta, alpha (rad)

public:
  LinkSpider_Leg() {
    anchor[0] = 0;
    anchor[1] = 0;
    anchor[2] = 0;
    anchor[3] = 0;

    frame[0] = 4;
    frame[1] = 4;
    frame[2] = 4;

    pwm[0][0] = 0;
    pwm[0][1] = 1000 / M_PI;
    pwm[1][0] = 0;
    pwm[1][1] = 1000 / M_PI;
    pwm[2][0] = 0;
    pwm[2][1] = 1000 / M_PI;

    angle[0] = 0;
    angle[1] = 0;
    angle[2] = 0;
  }

public:
  void setAnchorPos (double x0, double y0, double z0) {
    anchor[0] = x0;
    anchor[1] = y0;
    anchor[2] = z0;
  }

public:
  void setAnchorRotRad (double rad) {
    anchor[3] = rad;
  }

public:
  void setAnchorRotDeg (double deg) {
    anchor[3] = deg * M_PI / 180;
  }

public:
  void setFrameLength (double coxa, double femur, double tibia) {
    frame[0] = coxa;
    frame[1] = femur;
    frame[2] = tibia;
  }

public:
  void setNormalPosPWM (unsigned int index, double value) {
    pwm[index][0] = value;
  }

public:
  void setRatioRadPWM (unsigned int index, double radPerPWM) {
    pwm[index][1] = radPerPWM;
  }

public:
  void setRatioDegPWM (unsigned int index, double degPerPWM) {
    pwm[index][1] = degPerPWM * M_PI / 180;
  }

public:
  void setTipPos (double x, double y, double z) {
    double diffX = x - anchor[0];
    double diffY = y - anchor[1];
    double diffZ = z - anchor[2];

    double rotatedX = diffX * cos(anchor[3]) + diffY * sin(anchor[3]);
    double rotatedY = diffY * cos(anchor[3]) - diffX * sin(anchor[3]);

    double r = sqrt(pow(rotatedY, 2) + pow(rotatedX, 2));
    double p = r - frame[0];
    double u = sqrt(pow(p, 2) + pow(diffZ, 2));
    double gama = p > 0 ? atan(-diffZ / p) : M_PI + atan(-diffZ / p);

    angle[0] = atan(rotatedY / rotatedX);
    angle[1] = acos((pow(u, 2) + pow(frame[1], 2) - pow(frame[2], 2)) / (2 * frame[1] * u)) - gama;
    angle[2] = acos((pow(frame[2], 2) + pow(frame[1], 2) - pow(u, 2)) / (2 * frame[2] * frame[1]));
  }

public:
  double getAngleRad (unsigned int index) {
    return angle[index];
  }

public:
  double getAngleDeg (unsigned int index) {
    return angle[index] * 180 / M_PI;
  }

public:
  double getAnglePWM (unsigned int index) {
    return pwm[index][0] + angle[index] / pwm[index][1];
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
    rx = rx * 180 / M_PI;
    ry = ry * 180 / M_PI;
    rz = rz * 180 / M_PI;
    setRotationRad(rx, ry, rz);
  }

public:
  double getCoordinate (unsigned int legIndex, unsigned int vectorIndex) {
    // legIndex -> [L1, R1, L2, R2, L3, R3], vectorIndex -> [x, y, z]
    return computed_legs[legIndex][vectorIndex];
  }
};

#endif
