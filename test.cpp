#include <iostream>
#include "linkspider.h"

bool testAngularOfCartesian (double c, double f, double t, double x, double y, double z, double teta, double beta, double alpha, double tolerance) {
  std::cout << "getAngularOfCartesian(" << x << ", " << y << ", " << z << "):" << std::endl;

  LinkSpider::angular_t angularValues = LinkSpider::getAngularOfCartesian(c, t, f, x, y, z);

  if (angularValues.teta - teta > tolerance) {
    std::cout << "- teta  > actual   : " << angularValues.teta << std::endl;
    std::cout << "- teta  > expected : " << teta << std::endl;
    return false;
  }

  if (angularValues.beta - beta > tolerance) {
    std::cout << "- beta  > actual   : " << angularValues.beta << std::endl;
    std::cout << "- beta  > expected : " << beta << std::endl;
    return false;
  }

  if (angularValues.alpha - alpha > tolerance) {
    std::cout << "- alpha > actual   : " << angularValues.alpha << std::endl;
    std::cout << "- alpha > expected : " << alpha << std::endl;
    return false;
  }

  std::cout << "---- [OK]" << std::endl;
  return true;
}

int main(int argc, char const *argv[]) {
  const double tolerance = .005;
  const double coxa = 2; // in centimeter
  const double femur = 5; // in centimeter
  const double tibia = 9; // in centimeter

  if (!testAngularOfCartesian(coxa, femur, tibia, 2.77, 3.04, -4.93, 0.8318, 0.9374, 0.5392, tolerance)) return 1;
  if (!testAngularOfCartesian(coxa, femur, tibia, 2.67, 7.56, -3.25, 1.2316, 1.2102, 0.8532, tolerance)) return 1;
  if (!testAngularOfCartesian(coxa, femur, tibia, 2.51, 1.65, -5.86, 0.5819, 0.5248, 0.6676, tolerance)) return 1;
  if (!testAngularOfCartesian(coxa, femur, tibia, 0.48, 1.91, -4.38, 1.3245, 0.9889, 0.2677, tolerance)) return 1;

  return 0;
}
