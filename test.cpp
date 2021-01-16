#include <cmath>
#include "submodules/acutest/include/acutest.h"
#include "linkspider.h"

using namespace std;

void test_single_leg_static_position (void) {
  double tolerance = .005;
  double angle[3];
  LinkSpider_Leg leg;

  leg.setFrameLength(2, 5, 9);

  leg.setTipPos(2.77, 3.04, -4.93);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.8318) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9374) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 0.5392) < tolerance);

  leg.setTipPos(2.67, 7.56, -3.25);
  TEST_CHECK(abs(leg.getAngleRad(0) - 1.2316) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 1.2102) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 0.8532) < tolerance);

  leg.setTipPos(2.51, 1.65, -5.86);
  TEST_CHECK(abs(leg.getAngleRad(0) - 0.5819) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.5248) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 0.6676) < tolerance);

  leg.setTipPos(0.48, 1.91, -4.38);
  TEST_CHECK(abs(leg.getAngleRad(0) - 1.3245) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(1) - 0.9889) < tolerance);
  TEST_CHECK(abs(leg.getAngleRad(2) - 0.2677) < tolerance);
}

TEST_LIST = {
  { "Test Single Leg Static Position", test_single_leg_static_position },
  { NULL, NULL }
};
