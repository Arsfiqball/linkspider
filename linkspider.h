#include <math.h>

namespace LinkSpider {
  // Unit: Radians
  struct angular_t {
    double teta;
    double beta;
    double alpha;
  };

  double getR (double y, double x) {
    return sqrt(pow(y, 2) + pow(x, 2));
  }

  double getTeta (double y, double x) {
    return atan(y / x);
  }

  double getP (double r, double c) {
    return r - c;
  }

  double getU (double p, double z) {
    return sqrt(pow(p, 2) + pow(z, 2));
  }

  double getAlpha (double t, double f, double u) {
    return acos((pow(t, 2) + pow(f, 2) - pow(u, 2)) / (2 * t * f));
  }

  double getGama (double p, double z) {
    return atan(-z / p);
  }

  double getBeta (double t, double f, double u, double alpha, double gama) {
    return acos((pow(u, 2) + pow(f, 2) - pow(t, 2)) / (2 * f * u)) - gama;
  }

  // Unit input: centimeter
  angular_t getAngularOfCartesian (double c, double f, double t, double x, double y, double z) {
    double r = getR(y, x);
    double teta = getTeta(y, x);
    double p = getP(r, c);
    double u = getU(p, z);
    double alpha = getAlpha(t, f, u);
    double gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
    double beta = getBeta(t, f, u, alpha, gama);

    angular_t angularValues;
    angularValues.teta = teta;
    angularValues.beta = beta;
    angularValues.alpha = alpha;

    return angularValues;
  }
}
