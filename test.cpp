#include <cmath>
#include <iostream>
#include "submodules/acutest/include/acutest.h"
#include "linkspider.h"

using namespace std;

void test_single_leg_static_position (void) {
  double tolerance = .005;
  double tolerancePWM = 1;
  double normalPWM = 1500;
  double ratioPWM = (M_PI / 2) / 500;

  LinkSpider_Leg leg;

  leg.setNormalPosPWM(0, normalPWM);
  leg.setNormalPosPWM(1, normalPWM);
  leg.setNormalPosPWM(2, normalPWM);
  leg.setRatioRadPWM(0, ratioPWM);
  leg.setRatioRadPWM(1, ratioPWM);
  leg.setRatioRadPWM(2, ratioPWM);

  TEST_CASE("ro = 0, anchor = <0, 0, 0>, c = 2.5, f = 4.5, t = 7.5");

  leg.setAnchorRotRad(0);
  leg.setAnchorPos(0, 0, 0);
  leg.setFrameLength(2.5, 4.5, 7.5);

  leg.setTipPos(8.5608, 1.6711, -3.0502);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.1928) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.1353) < tolerance);

  leg.setTipPos(8.6064, -9.2439, 2.4098);
  TEST_CHECK(abs(leg.getAngleRad(0) - (-0.8211)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.0635) < tolerance);

  leg.setTipPos(0.6049, 0.8983, -7.9660);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.9782) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-0.5926)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.4066) < tolerance);

  leg.setTipPos(3.1619, -12.3883, 5.9890);
  TEST_CHECK(abs(leg.getAngleRad(0) - (-1.3209)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.6926) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.8774) < tolerance);

  TEST_CASE("Check getAnglePWM on <xyz> = 3.1619, -12.3883, 5.9890");

  TEST_CHECK(abs(leg.getAnglePWM(0) - (normalPWM + (-1.3209) / ratioPWM)) < tolerancePWM);
  TEST_CHECK(abs(leg.getAnglePWM(1) - (normalPWM + (0.6926) / ratioPWM)) < tolerancePWM);
  TEST_CHECK(abs(leg.getAnglePWM(2) - (normalPWM + (2.8774) / ratioPWM)) < tolerancePWM);

  TEST_CASE("Check getAngleDeg on <xyz> = 3.1619, -12.3883, 5.9890");

  TEST_CHECK(abs(leg.getAngleDeg(0) - (-1.3209 * 180 / M_PI)) < tolerance);
  TEST_CHECK(abs(leg.getAngleDeg(1) - (0.6926 * 180 / M_PI)) < tolerance);
  TEST_CHECK(abs(leg.getAngleDeg(2) - (2.8774 * 180 / M_PI)) < tolerance);

  TEST_CASE("ro = 0, anchor = <2, -2.5, 0>, c = 4.5, f = 3, t = 5.5");

  leg.setAnchorRotRad(0);
  leg.setAnchorPos(2, -2.5, 0);
  leg.setFrameLength(4.5, 3, 5.5);

  leg.setTipPos(8.5443, -0.0057, -6.2693);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.3641) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-0.2642)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.7636) < tolerance);

  leg.setTipPos(10.0220, -4.1852, -2.8176);
  TEST_CHECK(abs(leg.getAngleRad(0) - (-0.2071)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9068) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.0067) < tolerance);

  leg.setTipPos(7.8976, 6.8226, -5.1861);
  TEST_CHECK(abs(leg.getAngleRad(0) - 1.0067) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-0.4070)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.7346) < tolerance);

  leg.setTipPos(12.3428, -0.1726, 5.7981);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.2213) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9496) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.8489) < tolerance);

  TEST_CASE("ro = -1.47, anchor = <1, 2, 0>, c = 6.5, f = 8.5, t = 4.5");

  leg.setAnchorRotRad(-1.47);
  leg.setAnchorPos(1, 2, 0);
  leg.setFrameLength(6.5, 8.5, 4.5);

  leg.setTipPos(6.2085, -12.9338, 9.0196);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.2356) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.8211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.9917) < tolerance);

  leg.setTipPos(16.1342, 1.8919, 5.6148);
  TEST_CHECK(abs(leg.getAngleRad(0) - 1.4637) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 1.0210) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.7493) < tolerance);

  leg.setTipPos(11.9597, -4.7161, 0.0309);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.5498) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 0.8211) < tolerance);

  leg.setTipPos(7.6058, -2.0480, -10.5817);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-1.0353)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.8493) < tolerance);
}

void test_posture_static_position (void) {
  double tolerance = .05;
  LinkSpider_Posture posture;

  TEST_CASE("frs = 17, ms = 22, l = 24, h = 5");
  posture.setNormalPos(17, 22, 24, 5);

  posture.setRotationRad(0.2106, -0.1321, 0.2606);
  posture.compute();
  TEST_CHECK(abs(posture.getCoordinate(0, 0) - (-11.1315)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(0, 1) - (10.2583)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(0, 2) - (-3.4793)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(1, 0) - (5.1513)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(1, 1) - (14.6005)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(1, 2) - (-1.2403)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(2, 0) - (-10.1831)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(2, 1) - (-1.6336)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(2, 2) - (-6.2956)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(3, 0) - (10.8888)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(3, 1) - (3.9856)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(3, 2) - (-3.3981)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(4, 0) - (-4.4455)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(4, 1) - (-12.2485)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(4, 2) - (-8.4534)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(5, 0) - (11.8372)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(5, 1) - (-7.9062)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(5, 2) - (-6.2144)) < tolerance);

  posture.setRotationRad(0.3963, 0.0678, -0.3534);
  posture.compute();
  TEST_CHECK(abs(posture.getCoordinate(0, 0) - (-3.4555)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(0, 1) - (15.1312)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(0, 2) - (0.5952)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(1, 0) - (12.4570)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(1, 1) - (9.2607)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(1, 2) - (-0.5569)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(2, 0) - (-9.9217)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(2, 1) - (5.7173)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(2, 2) - (-3.8563)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(3, 0) - (10.6710)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(3, 1) - (-1.8797)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(3, 2) - (-5.3474)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(4, 0) - (-11.7076)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(4, 1) - (-5.4231)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(4, 2) - (-8.6469)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(5, 0) - (4.2048)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(5, 1) - (-11.2936)) < tolerance);
  TEST_CHECK(abs(posture.getCoordinate(5, 2) - (-9.7991)) < tolerance);
}

TEST_LIST = {
  { "Test Single Leg Static Position", test_single_leg_static_position },
  { "Test Posture Static Position", test_posture_static_position },
  { NULL, NULL }
};
