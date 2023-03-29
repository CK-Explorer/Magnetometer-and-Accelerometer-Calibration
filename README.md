## :star2: Magnetometer and Accelerometer Calibration

* [MATLAB scripts](Calibration%20MATLAB%20script) and [C++ library](Calibration%20C%2B%2B%20Library) (use separately) that calibrate accelerometer and magnetometer's outputs.

## :robot: Brief Description

The algorithm implemented in this project minimizes the following problems in both accelerometer and magnetometer:
* For each sensor:
    * The non-orthoganality of all axes,
    * Different scale factors of all axes,
    * Centre biases.
* For magnetometer (when used with accelerometer):
    * Misalignment of three magnetometer's axes from three accelerometer's axes respectively.

<div align="center"><img src="Images/Magnetometer data plot.png" width="415" height="auto" class="center"></div>

After calibration, both sensors' readings should fit on the surface of spheres with their centres at the origin, but not on ellipsoids, like in the figure above.

All the mathematical details involved in this algorithm are available in this [pdf](Working%20Principle/Working%20Principle.pdf).

## 	:toolbox: Getting Started

### :bangbang: Dependencies

* For C++,
    * [ALGLIB](https://www.alglib.net/) math library, requires version 3.20.
    * [Eigen](https://eigen.tuxfamily.org/) math library.
    * [ellipsoid-fit](https://github.com/CK-Explorer/ellipsoid-fit) library, forked from [BenjaminNavarro](https://github.com/BenjaminNavarro/ellipsoid-fit).

* For MATLAB,
    * [ellipsoid_fit_new](https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit) function.

### :running: Using the library

* For C++, please refer to [this page](Calibration%20C%2B%2B%20Library/Brief%20explanation.md) and its [header file](Calibration%20C%2B%2B%20Library/Calibration.h) for more details. 
* For MATLAB, please refer to [this page](Calibration%20MATLAB%20script/Brief%20explanation.md) and their function [files](Calibration%20MATLAB%20script/) for more explanations. 

### :test_tube: Examples

* Demo for calibrating accelerometer or magnetometer when it is used solely,
    * first include [acc_log.csv](Data%20Examples/acc_log.csv).as data file.
    * [Accelerometer Calibration Example.cpp](Calibration%20C%2B%2B%20Library/Examples/Accelerometer%20Calibration%20Example.cpp) for C++.
    * [calibration_acc_interface.m](Calibration%20MATLAB%20script/Examples/calibration_acc_interface.m) for MATLAB.
* Demo for calibrating magnetometer with well-calibrated accelerometer,
    * first include [acc_mag_log.csv](Data%20Examples/acc_mag_log.csv) as data file.
    * [Magnetometer Calibration Example.cpp](Calibration%20C%2B%2B%20Library/Examples/Magnetometer%20Calibration%20Example.cpp) for C++.
    * [calibration_mag.m](Calibration%20MATLAB%20script/Examples/calibration_mag_interface.m) for MATLAB.

## :scroll: Version History

* 1.0.0
    * Initial Release

## :warning: License

This project is licensed under the Apache-2.0 license - see the [LICENSE](LICENSE) file for details.
