C ++ based inverse kinematics solver and algorithm for static positioning, walking sequence and adaptability of hexapod robot, with support for PWM based servo driver

### Features
* [x] Convert cartesian coordinates of each leg tips to 3DOF servo angle.
* [x] Convert servo angle into PWM (+ calibration feature)
* [ ] Move forward / backward
* [ ] Rotate clockwise / counter clockwise
* [ ] Shift left / right
* [ ] Turn left / right
* [ ] Adjustable speed
* [x] Adjustable robot height
* [ ] Adjustable step span
* [x] Adjustable leg span
* [x] Rotation at X / Y / Z axis

### Getting Started
Download and copy this repository to your modules folder or just clone this repository as a git submodule.
```sh
git submodule add https://github.com/Arsfiqball/linkspider.git submodules/linkspider
```
Read the [Definitions](/docs/Definitions.md) and [API Documentation](/docs/API.md) for detailed usage.

### Run Tests
```sh
g++ -o test test.cpp
./test
```

### Related Research
* [LinkSpider: Single Leg Simulation for Evaluation](https://observablehq.com/@arsfiqball/linkspider-single-leg-simulation-for-evaluation)
* [LinkSpider: Posture Simulation for Evaluation](https://observablehq.com/@arsfiqball/linkspider-single-posture-simulation-for-evaluation)

### References
* [Hexapod Inverse Kinematics Equations](https://toglefritz.com/hexapod-inverse-kinematics-equations/) by Toglefritz
* [Hexapod Inverse Kinematics Simulator](https://toglefritz.com/hexapod-inverse-kinematics-simulator/) (Spreadsheet) by Toglefritz
* [Inverse dan Body Kinematics pada Robot Hexapod](https://jurnal.polban.ac.id/index.php/proceeding/article/download/1050/854) Journal by Indra Agustian Kurniawan, Feriyonika, and Sabar Pramono at Politeknik Negeri Bandung
* [Hexapod Simulation (Python)](https://github.com/mithi/hexapod-robot-simulator) by Mithi Sevilla
* [Hexapod Simulation (JavaScript)](https://github.com/mithi/hexapod) by Mithi Sevilla
* [Tirtapod Leg Simulation v1](https://observablehq.com/@arsfiqball/tirtapod-leg-simulation-wip) by Iqbal Mohammad Abdul Ghoni
* [Tirtapod-X](https://github.com/Arsfiqball/tirtapods-x) by Iqbal Mohammad Abdul Ghoni
