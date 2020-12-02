#ifndef LINK_SPIDER
#define LINK_SPIDER

#include <math.h>

struct LinkSpiderAngular_t {
  double teta;
  double beta;
  double alpha;
};

class LinkSpiderLeg {
public:
  double tibia;
  double femur;
  double coxa;
  double x0;
  double y0;
  double z0;

protected:
  double rotated; // Radian, Counter Clockwise

public:
  LinkSpiderLeg () {
    tibia = 9;
    femur = 5;
    coxa = 2;
    x0 = 0;
    y0 = 0;
    z0 = 0;
    rotated = 0;
  }

public:
  void setRotatedDeg (double deg) {
    rotated = deg * M_PI / 180;
  }

public:
  void setRotatedRad (double rad) {
    rotated = rad;
  }

public:
  LinkSpiderAngular_t convert (double x, double y, double z) {
    LinkSpiderAngular_t angularValues;

    double diffX = x - x0;
    double diffY = y - y0;
    double diffZ = z - z0;

    double rotatedX = diffX * cos(rotated) + diffY * sin(rotated);
    double rotatedY = diffY * cos(rotated) - diffX * sin(rotated);

    double r = sqrt(pow(rotatedY, 2) + pow(rotatedX, 2));
    double p = r - coxa;
    double u = sqrt(pow(p, 2) + pow(diffZ, 2));
    double gama = p > 0 ? atan(-diffZ / p) : M_PI + atan(-diffZ / p);

    angularValues.teta = atan(rotatedY / rotatedX);
    angularValues.alpha = acos((pow(tibia, 2) + pow(femur, 2) - pow(u, 2)) / (2 * tibia * femur));
    angularValues.beta = acos((pow(u, 2) + pow(femur, 2) - pow(tibia, 2)) / (2 * femur * u)) - gama;

    return angularValues;
  }
};

#endif
