# LinkSpider API
The API have several class which have different usage and should be constructed
together to make a correct hexapod control. The API was designed to be as
flexible as possible. In general, lower level classes have four step to follow:
* Initialization step: declaring object
* Configuration step: all methods prefixed with **set...**
* Compute step: **compute** method
* Access computed property step: all methods prefixed with **get...**

Example:
```cpp
#include <cmath>
#include "submodules/linkspider/linkspider.h"

// ...

// Initialization step
LinkSpider_Leg leg;

// Configuration step
leg.setAnchorRotRad(0);
leg.setAnchorPos(0, 0, 0);
leg.setFrameLength(2.5, 4.5, 7.5);
leg.setTipPos(8.5608, 1.6711, -3.0502);

// Compute step
leg.compute();

// Access computed property step
leg.getAngleRad(0) // 0.1928
leg.getAngleRad(1) // 0.9211
leg.getAngleRad(2) // 1.1353

// ...
```

#### LinkSpider_Leg leg(void)
This class initializes configuration and controls single leg static position.
```cpp
// initialize the object first.
LinkSpider_Leg leg;
```

##### void leg.compute()
Doing all internal computation

##### void leg.setAnchorPos(double x0, double y0, double z0)
Setting coordinate of first servo as the anchor relative to Center of Gravity. Positive x0 is to the right sign, positive y0 is forward, positive z0 is upward. For example, if your first servo point is placed on {x0, y0, z0} = {-4, 9, 0}:
```cpp
leg.setAnchorPos(-4, 9, 0);
```
> Note: It currently does not support z0 to be other than 0

##### void leg.setAnchorRotRad(double rad)
Setting the align of the first servo (in radian). Position rad = 0 is to the right sign (x-axis positive). For example, if your first servo aligned forward (90 deg):
```cpp
#include <cmath>
// ...
leg.setAnchorRotRad(M_PI / 2); // 90 deg = PI / 2
```

##### void leg.setAnchorRotDeg(double deg)
Same as setAnchorRotRad but with degree angular unit as parameterer instead of radian. Example for forward aligned first servo:
```cpp
leg.setAnchorRotDeg(90);
```

##### void leg.setFrameLength(double coxa, double femur, double tibia)
Set the length of leg frames.
* coxa: first servo (anchor) to second servo
* femur: second servo to third servo
* tibia: third servo to the tip of the leg
```cpp
leg.setFrameLength(2, 5, 9);
```

##### void leg.setNormalPosPWM(unsigned int index, double value)
Set the PWM on servo normalized position.
* index: index of the servo (0 = first servo, 1 = second servo, 2 = third servo)
* value: the PWM value
```cpp
leg.setNormalPosPWM(0, 1500);
```

##### void leg.setRatioRadPWM(unsigned int index, double radPerPWM)
Set ratio radian per PWM of servo.
* index: index of the servo (0 = first servo, 1 = second servo, 2 = third servo)
* radPerPWM: radian per PWM value
```cpp
#include <cmath>
// ...
leg.setRatioRadPWM(0, M_PI / 1000);
```

##### void leg.setRatioDegPWM(unsigned int index, double degPerPWM)
Same as setRatioRadPWM but using degree angular unit as parameter
* index: index of the servo (0 = first servo, 1 = second servo, 2 = third servo)
* degPerPWM: degree per PWM value
```cpp
leg.setRatioDegPWM(0, 180 / 1000);
```

##### void leg.setTipPos(double x, double y, double z)
Set tip of the leg target coordinate relative to Center of Gravity. Positive x0 is to the right sign, positive y0 is forward, positive z0 is upward.
```cpp
leg.setTipPos(20, 8, -7)
```

##### double leg.getAngleRad(unsigned int index)
Get calculated angle value (in radian) of servo (0 = first servo, 1 = second servo, 2 = third servo)
```cpp
double rad = leg.getAngleRad(0)
```

##### double leg.getAngleDeg(unsigned int index)
Get calculated angle value (in degree) of servo (0 = first servo, 1 = second servo, 2 = third servo)
```cpp
double deg = leg.getAngleDeg(0)
```

##### double leg.getAnglePWM(unsigned int index)
Get calculated angle value (in PWM) of servo (0 = first servo, 1 = second servo, 2 = third servo)
```cpp
double pwm = leg.getAnglePWM(0)
```

#### LinkSpider_Posture posture(void)
This class initializes configuration and controls hexapod posture body position.
```cpp
// initialize the object first.
LinkSpider_Posture posture;
```

##### void posture.compute()
Doing all internal computation.

##### void posture.setNormalPos(double frs, double ms, double l, double h)
Set normal standing posture.
* frs: front and rear legs span (x-axis)
* ms: middle legs span (x-axis)
* l: length (y-axis)
* h: standing height (z-axis)

Example:
```cpp
posture.setNormalPos(17, 22, 24, 5);
```

##### void posture.setRotationRad(double rx, double ry, double rz)
Set pitch (rx), roll (ry), yaw (rz) of the body (in Radian). Positive is clockwise rotation transform.
```cpp
#include <cmath>
posture.setRotationRad(M_PI / 12, - M_PI / 12, M_PI / 12); // 15deg, -15deg, 15deg
```

##### void posture.setRotationDeg(double rx, double ry, double rz)
Set pitch (rx), roll (ry), yaw (rz) of the body (in Degree). Positive is clockwise rotation transform.
```cpp
posture.setRotationDeg(15, - 15, 15);
```

##### double posture.getCoordinate(unsigned int legIndex, unsigned int vectorIndex)
Get coordinate of each leg tips
* legIndex: index of leg [0 = L1, 1 = R1, 2 = L2, 3 = R2, 4 = L3, 5 = R3]
* vectorIndex: index of vector [0 = x, 1 = y, 2 = z]
```cpp
posture.getCoordinate(3, 2); // R2, z
```

#### LinkSpider_ConnectorSSC32 ssc(void)
This class initializes configuration and controls for SSC 32 connection.
```cpp
// initialize the object first.
LinkSpider_ConnectorSSC32 ssc;
```

##### void ssc.compute()
Doing all internal computation.

##### void ssc.setInterval(unsigned int time)
Set SSC-32 time (T) command which controls how quick the servo should move (in miliseconds)
```cpp
ssc.setInterval(200); // 200 ms
```

##### void ssc.setServoPin (unsigned int legIndex, unsigned int servoIndex, unsigned int pin)
Set SSC-32 pin configuration (P) command for each servos.
* legIndex: [0 = L1, 1 = R1, 2 = L2, 3 = R2, 4 = L3, 5 = R3]
* servoIndex: [0 = teta, 1 = beta, 2 = alpha]
* pin: which pin address the servo is connected to
```cpp
ssc.setServoPin(1, 2, 22); // R1, alpha, pin 22
```

##### void ssc.setServoValue(unsigned int legIndex, unsigned int servoIndex, double value)
Set servo PWM value.
* legIndex: [0 = L1, 1 = R1, 2 = L2, 3 = R2, 4 = L3, 5 = R3]
* servoIndex: [0 = teta, 1 = beta, 2 = alpha]
* value: PWM value, will be rounded to integer
```cpp
ssc.setServoPin(4, 0, 1400); // L3, teta, 1400 micro seconds
```

##### char * ssc.getPrintable()
Get printable SSC-32 serial command. Implementation example:
```cpp
SoftwareSerial com(1, 2); // TX, RX
com.println(ssc.getPrintable());
```
