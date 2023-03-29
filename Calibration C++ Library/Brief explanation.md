## :star2: A Brief Explanation of the Steps of Using This Library

As stated in the title, this page is just a sufficient summary of guidelines to use this C++ library, hence please refer to this [C++ header file](Calibration.h) and this [documentation](../Working%20Principle/Working%20Principle.pdf) if you have any doubt.

# :notebook_with_decorative_cover: Chapters
1. [Mathematical Notations](#one-Mathematical-Notations)
2. [Accelerometer Calibration Cpp API](#two-Accelerometer-Calibration-Cpp-API)
3. [Magnetometer Calibration Cpp API](#three-Magnetometer-Calibration-Cpp-API)
4. [Tips for collecting data](#four-Tips-for-collecting-data)

## :one: Mathematical Notations

First, letters $a$ and $m$ correspond to accelerometer and magnetometer readings. 

Subscript is used to represent the axis' name, e.g. $a_x$ means x-axis accelerometer reading.

For reading before calibration, we use non-circumflex or non-hat to denote it, i.e. 

accelerometer and magnetometer reading before calibration respectively = 
```math
\begin{pmatrix}a_x\\a_y\\a_z\end{pmatrix} \ , \   \begin{pmatrix}m_x\\m_y\\m_z\end{pmatrix}
```

while for calibrated readings, circumflex or hat on the letter is used, i.e.

accelerometer and magnetometer reading after calibration respectively = 
```math
\begin{pmatrix}\hat{a_x}\\\hat{a_y}\\\hat{a_z}\end{pmatrix} \ , \   \begin{pmatrix}\hat{m_x}\\\hat{m_y}\\\hat{m_z}\end{pmatrix}
```

## :two: Accelerometer Calibration Cpp API

**Note:** This function is also suitable for calibrating magnetometer, when it is used without the presence of accelerometer in your project. Else if accelerometer exists, then it is recommended to use the following magnetometer calibration [below](#three-Magnetometer-Calibration-Cpp-API) instead.

Refer to [Calibration.h](Calibration.h), we have this static function

```
Calibration::accelerometer_calibration(const Eigen::MatrixXd& acc_points, Calibration::acc_parameters& acc_var);
```

Suppose we have $n$ sets of uncalibrated accelerometer data. We first store these data into ```acc_points```, which is a n x 3 matrix or array and the first, second and third columns represents x, y and z axis respectively.

Then we pass it into the above function. It will produce ```acc_var``` which is a struct that stores two members, i.e. ```n_o``` (3 x 3 array) and ```centre``` (array of 3 dimensions). ```n_o``` is the non-orthoganality matrix, while ```centre``` is the centre biases for each axis.

We apply those parameters to correct our readings reported from accelerometer later using the following equation:
```math
\begin{pmatrix}\hat{a_x}\\\hat{a_y}\\\hat{a_z}\end{pmatrix}_{normalised}=
\begin{pmatrix}
n\_o[0][0]&0&0\\
n\_o[1][0]&n\_o[1][1]&0\\
n\_o[2][0]&n\_o[2][1]&n\_o[2][2]
\end{pmatrix}
\begin{pmatrix}a_x - centre[0]\\a_y - centre[1]\\a_z - centre[2]\end{pmatrix}
```
since
```math
n\_o[0][1] = n\_o[0][2] = n\_o[1][2] = 0
```

The above calibrated reading is a normalised reading. To make it unnormalised, we assume x-axis has an accurate scale, then an extra step is needed:

```math
\begin{pmatrix}\hat{a_x}\\\hat{a_y}\\\hat{a_z}\end{pmatrix}_{unnormalised}=
\frac{1}{n\_o[0][0]}
\begin{pmatrix}\hat{a_x}\\\hat{a_y}\\\hat{a_z}\end{pmatrix}_{normalised}
```

## :three: Magnetometer Calibration Cpp API

**Note:** This section requires the accelerometer to be calibrated beforehand.

In [Calibration.h](Calibration.h), we have this static function

```
Calibration::magnetometer_calibration(const Eigen::MatrixXd& acc_points, const Eigen::MatrixXd& mag_points, 
    Calibration::mag_parameters& mag_var, 
    double& beta, const double initial_quaternion[4]);
```

Suppose we have $n$ sets of accelerometer (calibrated) and magnetometer (uncalibrated) readings, with both are obtained simultaneously in the same positions. We first store accelerometer data into ```acc_points```, and magnetometer data into ```mag_points```, of which both variables are n x 3 matrices. The first, second and third columns corresponds to x, y and z-axes of these sensors.

Now, we need to pass the initial estimates for the least square minimisation happened inside the function. The first estimate is ```beta```, which represents the cosine of theta and this theta is the projection or dot product angle between gravity and Earth's magnetic field vector. It should set based on your local magnetic inclination angle plus or subtract with $\frac{\pi}{2}$ that depends on the local reference frame convention. Please refer to Chapter 3 of the [documentation](../Working%20Principle/Working%20Principle.pdf).

The second estimate is ```initial_quaternion```, which is an array of dimension four. This variable is a quaternion that represents a rotation matrix to align the magnetometer to the accelerometer. We could set this variable with the value of {1, 0, 0, 0}, which forms an identity rotation matrix by means of $\mathbf{q} = q + \vec{q} = 1$. This is based on the assumption that both accelerometer and magnetometer are almost aligned.

The outputs of this function are ```mag_var``` struct and ```beta```. ```mag_var``` is a struct that consists of ```transform_matrix``` (3 x 3 array), ```centre``` (array of 3 dimensions), ```n_o``` (3 x 3 array) and ```R_cov``` (3 x 3 array). Their defintions are as below:
* ```transform_matrix``` is the total transformation matrix, which incorporates the misalignment, ```R_cov``` and non-orthoganality correction, ```n_o``` matrices, .
* ```centre``` stores the centre biases of x, y and z axes respectively.
* ```R_cov``` is the misalignment correction matrix.
* ```n_o``` is the non-orthoganality matrix.
* ```beta``` is the estimated beta after least square minimisation.

We only need to apply ```transform_matrix``` and ```centre``` to correct the magnetometer readings in our project, as follows:
```math
\begin{pmatrix}\hat{m_x}\\\hat{m_y}\\\hat{m_z}\end{pmatrix}_{normalised}
=
\begin{pmatrix}
transform\_matrix[0][0]&transform\_matrix[0][1]&transform\_matrix[0][2]\\
transform\_matrix[1][0]&transform\_matrix[1][1]&transform\_matrix[1][2]\\
transform\_matrix[2][0]&transform\_matrix[2][1]&transform\_matrix[2][2]
\end{pmatrix}
\begin{pmatrix}m_x - centre[0]\\m_y - centre[1]\\m_z - centre[2]\end{pmatrix}
```

Similarly, the above output vector is a normalised vector. So, by assuming x-axis has an accurate scale, the following extra step is performed:

```math
\begin{pmatrix}\hat{m_x}\\\hat{m_y}\\\hat{m_z}\end{pmatrix}_{unnormalised}=
\frac{1}{n\_o[0][0]}
\begin{pmatrix}\hat{m_x}\\\hat{m_y}\\\hat{m_z}\end{pmatrix}_{normalised}
```

## :four: Tips for collecting data

* To ensure high accuracies for all these calibration parameters, we need to obtain the data in all possible orienatation that are able to cover or describe most of the ellipsoid's surface, e.g.

<div align="center"><img src="../Images/accelerometer_plot.png" width="325" height="auto" class="center"></div>

* For accelerometer calibration using ```accelerometer_calibration``` as in [chapter 2](#two-Accelerometer-Calibration-Cpp-API), all the data should be obtained at stationary state, as to avoid any possible acceleration introduced to the data. This accelerometer calibration only needs to be performed once, since it is rarely interfered with external factors.

* For magnetometer calibration using ```accelerometer_calibration``` as in [chapter 2](#two-Accelerometer-Calibration-Cpp-API), the data do not need to be captured under stationary condition.

* If using ```magnetometer_calibration``` as in [chapter 3](#three-Magnetometer-Calibration-Cpp-API) for magnetometer calibration, the IMU should be tumbled in all directions gently, as to avoid any acceleration interfered with the accuracy of accelerometer's readings.

* The magnetometer calibration needs to be performed again, when we move to an environment which has different magnetic interference than the previous environment where we performed the calibration before.