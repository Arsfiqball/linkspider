C ++ based inverse kinematics solver and algorithm for static positioning, walking sequence and adaptability of hexapod robot, with support for PWM based servo driver

### Features
* [ ] Convert cartesian coordinates of each leg tips to 3DOF servo angle.
* [ ] Convert servo angle into PWM (+ calibration feature)
* [ ] Move forward / backward
* [ ] Rotate clockwise / counter clockwise
* [ ] Shift left / right
* [ ] Turn left / right
* [ ] Adjustable speed
* [ ] Adjustable robot height
* [ ] Adjustable step span
* [ ] Adjustable leg span
* [ ] Rotation at X / Y / Z axis

### Run Tests
```sh
g++ -o test test.cpp
./test
```

### References
* [Hexapod Inverse Kinematics Equations](https://toglefritz.com/hexapod-inverse-kinematics-equations/) by Toglefritz
* [Hexapod Inverse Kinematics Simulator](https://toglefritz.com/hexapod-inverse-kinematics-simulator/) (Spreadsheet) by Toglefritz
* [Inverse dan Body Kinematics pada Robot Hexapod](https://jurnal.polban.ac.id/index.php/proceeding/article/download/1050/854) Journal by Indra Agustian Kurniawan, Feriyonika, and Sabar Pramono at Politeknik Negeri Bandung
* [Hexapod Simulation (Python)](https://github.com/mithi/hexapod-robot-simulator) by Mithi Sevilla
* [Hexapod Simulation (JavaScript)](https://github.com/mithi/hexapod) by Mithi Sevilla
* [Tirtapod Leg Simulation v1](https://observablehq.com/@arsfiqball/tirtapod-leg-simulation-wip) by Iqbal Mohammad Abdul Ghoni
* [Tirtapod-X](https://github.com/Arsfiqball/tirtapods-x) by Iqbal Mohammad Abdul Ghoni
