#include <math.h>
#include <string.h>
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
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.1928) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.1353) < tolerance);

  leg.setTipPos(8.6064, -9.2439, 2.4098);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - (-0.8211)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.0635) < tolerance);

  leg.setTipPos(0.6049, 0.8983, -7.9660);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.9782) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-0.5926)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.4066) < tolerance);

  leg.setTipPos(3.1619, -12.3883, 5.9890);
  leg.compute();
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
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.3641) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-0.2642)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.7636) < tolerance);

  leg.setTipPos(10.0220, -4.1852, -2.8176);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - (-0.2071)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9068) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.0067) < tolerance);

  leg.setTipPos(7.8976, 6.8226, -5.1861);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 1.0067) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - (-0.4070)) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.7346) < tolerance);

  leg.setTipPos(12.3428, -0.1726, 5.7981);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.2213) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9496) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.8489) < tolerance);

  TEST_CASE("ro = -1.47, anchor = <1, 2, 0>, c = 6.5, f = 8.5, t = 4.5");

  leg.setAnchorRotRad(-1.47);
  leg.setAnchorPos(1, 2, 0);
  leg.setFrameLength(6.5, 8.5, 4.5);

  leg.setTipPos(6.2085, -12.9338, 9.0196);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.2356) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.8211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 2.9917) < tolerance);

  leg.setTipPos(16.1342, 1.8919, 5.6148);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 1.4637) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 1.0210) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 1.7493) < tolerance);

  leg.setTipPos(11.9597, -4.7161, 0.0309);
  leg.compute();
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.9211) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.5498) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 0.8211) < tolerance);

  leg.setTipPos(7.6058, -2.0480, -10.5817);
  leg.compute();
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

void test_connector_ssc32 (void) {
  LinkSpider_ConnectorSSC32 ssc;
  ssc.setServoPin(0, 0, 1);
  ssc.setServoPin(0, 1, 2);
  ssc.setServoPin(0, 2, 3);
  ssc.setServoPin(1, 0, 24);
  ssc.setServoPin(1, 1, 23);
  ssc.setServoPin(1, 2, 22);
  ssc.setServoPin(2, 0, 4);
  ssc.setServoPin(2, 1, 5);
  ssc.setServoPin(2, 2, 6);
  ssc.setServoPin(3, 0, 21);
  ssc.setServoPin(3, 1, 20);
  ssc.setServoPin(3, 2, 19);
  ssc.setServoPin(4, 0, 7);
  ssc.setServoPin(4, 1, 8);
  ssc.setServoPin(4, 2, 9);
  ssc.setServoPin(5, 0, 18);
  ssc.setServoPin(5, 1, 17);
  ssc.setServoPin(5, 2, 16);
  ssc.setServoValue(0, 0, 1500);
  ssc.setServoValue(0, 1, 1000);
  ssc.setServoValue(0, 2, 2000);
  ssc.setServoValue(1, 0, 1500);
  ssc.setServoValue(1, 1, 1500);
  ssc.setServoValue(1, 2, 1000);
  ssc.setServoValue(2, 0, 2000);
  ssc.setServoValue(2, 1, 2000);
  ssc.setServoValue(2, 2, 1500);
  ssc.setServoValue(3, 0, 2000);
  ssc.setServoValue(3, 1, 2000);
  ssc.setServoValue(3, 2, 1500);
  ssc.setServoValue(4, 0, 1500);
  ssc.setServoValue(4, 1, 1000);
  ssc.setServoValue(4, 2, 1500);
  ssc.setServoValue(5, 0, 1500);
  ssc.setServoValue(5, 1, 1000);
  ssc.setServoValue(5, 2, 2000);
  ssc.setInterval(400);
  ssc.compute();

  char expected[200];
  strcpy(expected, "#1P1500 #2P1000 #3P2000 #24P1500 #23P1500 #22P1000 #4P2000 #5P2000 #6P1500 #21P2000 #20P2000 #19P1500 #7P1500 #8P1000 #9P1500 #18P1500 #17P1000 #16P2000 T400");
  TEST_CHECK(strcmp(ssc.getPrintable(), expected) == 0);
}

TEST_LIST = {
  { "Test Single Leg Static Position", test_single_leg_static_position },
  { "Test Posture Static Position", test_posture_static_position },
  { "Test Connector SSC32", test_connector_ssc32 },
  { NULL, NULL }
};
