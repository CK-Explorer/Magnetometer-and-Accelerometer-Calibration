/* GitHub Page: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration
*  API tutorial: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Calibration%20C%2B%2B%20Library/Brief%20explanation.md
*  Documentation: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Working%20Principle/Working%20Principle.pdf
*/

#pragma once

#define stop_cond 1.0e-12 	//EpsX or stopping condition from ALGLIB documentation
#define max_iteration 500	//maximum number of iterations for least square minimisation
#define diff_step 0.0001	//numerical differentiation step for least square minimisation

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/*ALGLIB header file for least square fitting and other math operations
* From: https://www.alglib.net/
*/
#include "stdafx.h"
#include "interpolation.h"

/*Eigen header file for matrix operations
* From: https://eigen.tuxfamily.org/
*/
#include "Eigen/Dense"

/*ellipsoid fit header file
* From: https://github.com/BenjaminNavarro/ellipsoid-fit
*/
#include "ellipsoid/fit.h"

class Calibration
{
public:
	/*A struct to hold the accelerometer's calibration parameters:
	*
	* n_o (3x3) = non-orthogonal and scaling factor correction matrix 
	* centre (3) = center biases
	* 
	* Way of using outputs:
	* accelerometer data = [ax ay az]
	* accelerometer data with shifted bias, a_b (3x1) = [ ax - center(1) 
	*                                                     ay - center(2)
	*                                                     az - center(3) ]
	* corrected normalised accelerometer data (3x1) = n_o * a_b
	*/
	struct acc_parameters
	{
		double n_o[3][3];
		double centre[3];
	};

	/*A struct to hold the magnetometer's calibration parameters:
	*
	* Important:
	* transform_matrix (3x3) = transformation matrix from R_cov * n_o
	* centre (3) = center biases
	* 
	* For reference:
	* n_o (3x3) = non-orthogonal and scaling factor correction matrix 
	* R_cov (3x3) = correcting magnetometer misalignment rotation matrix 
	* 
	* Way of using outputs:
	* magnetometer data = [mx my mz]
	* accelerometer data with shifted bias, m_b (3x1) = [ mx - center(1) 
	*                                                     my - center(2)
	*                                                     mz - center(3) ]
	* corrected normalised magnetometer data (3x1) = transform_matrix * m_b
	*/
	struct mag_parameters
	{
		double transform_matrix[3][3];
		double n_o[3][3];
		double R_cov[3][3];
		double centre[3];
	};

public:
	/* An accelerometer calibration function that corrects
	*   * accelerometer's center biases
	*   * accelerometer's axes scaling factors
	*   * non-orthogonality between accelerometer's axes
	*
	*  Input: 
	*  acc_points (nx3) = accelerometer data
	*  
	*  Output:
	*  acc_var = acc_parameters struct from above
	*  
	*  Return value = 0, execution successful.
	*
	*  Note: This is also suitable to calibrate magnetometer 
	*  if it is used solely.
	*/
	static int accelerometer_calibration(
		const Eigen::MatrixXd& acc_points,
		acc_parameters& acc_var);

	/* A magnetometer calibration script that corrects
	*   * magnetometer's center biases
	*   * magnetometer's axes scaling factors
	*   * non-orthogonality between magnetometer's axes
	*   * misaligment between magnetometer and accelerometer
	*
	*  Inputs: 
	*  acc_points (nx3) = accelerometer data
	*  mag_points (nx3) = magnetometer data
	*  beta = 	an initial estimate of cos(theta),in which 
	*  		theta is an estimated dot product angle between 
	*		gravity and Earth's magnetic field
	*  initial_quaternion (4) = initial estimate of R_cov expressed in quaternion				 
	*  
	*  Outputs:
	*  mag_var = mag_parameters struct from above
	*  beta = estimated beta after least square operation (for reference)
	*
	*  Return value = 0, execution successful.
	*  Return value = -1, the number of row of accelerometer and 
	*  magnetometer data are different.
	* 
	*  Note: This function requires both accelerometer and magnetometer 
	*  		 data captured simultaneously when they are in the same 
	*		 orientations.	
	*/
	/*return value = -1*/
	static int magnetometer_calibration(
		const Eigen::MatrixXd& acc_points,
		const Eigen::MatrixXd& mag_points,
		mag_parameters& mag_var,
		double& beta, const double initial_quaternion[4]);

private:
	/* A function that determines non-orthoganality, scale-factors, and
	*  centre biases matrices after ellipsoid fitting.
	* 
	*  Input: point (nx3) = IMU data for ellipsoid fitting.
	*  Outpits:
	*  n_o (3x3) = non-orthoganality and scale-factors correction matrix.
	*  centre (3) = centre biases of each axis.
	*/
	static void non_orthogonality(
		const Eigen::MatrixXd& points,
		alglib::real_2d_array& n_o, double centre[3]);

	/* A function that determines correcting magnetometer misalignment
	*  rotation matrix.
	*
	*  Inputs:
	*  data (nx6) = pairs of calibrated accelerometer (nx3) and magnetometer data (nx3)
	*  data_no = number of pairs of data, n
	*  beta = 	an initial estimate of cos(theta),in which theta is estimated 
	*		dot product angles between gravity and Earth's magnetic field
	*  initial_quaternion (4) = initial estimate of R_cov expressed in quaternion	
	*  
	*  Outputs:
	*  info = output after performing least square minimisation.
	*  state = output after performing least square minimisation.	
	*  rep = output after performing least square minimisation.	
	*
	*  Return value (4x1) = estimated parameters from least square in the form of
	*			[X Y Z beta]
	*/
	static alglib::real_1d_array misalignment_correction(
		const alglib::real_2d_array& data, int data_no,
		double beta, const double initial_quaternion[4],
		alglib::ae_int_t& info, alglib::lsfitstate& state,
		alglib::lsfitreport& rep);

	/* Cost function passed into alglib::lsfitfit
	*/
	static void misalignment_correction_func(
		const alglib::real_1d_array& para,
		const alglib::real_1d_array& data, double& func, void* ptr);
	
	/* Convert quaternion to parameters in stereographic projection
	*
	*  Input: quat (4x1) = quaternion
	*  Output: x (3x1) = parameters of stereographic projection
	*/
	static void quat2stereographic(const alglib::real_1d_array& quat,
		alglib::real_1d_array& x);

	/* Convert parameters of stereographic projection to quaternion and 
	*  finally to rotation matrix
	*  
	*  Input: x (3x1) = parameters of stereographic projection
	*  Output: R_cov (3x3) = rotation matrix
	*/
	static inline void stereographic2rotm(const alglib::real_1d_array& x,
		alglib::real_2d_array& R_cov);

	/* Convert 3x3 alglib::real_2d_array to 3x3 C-array
	*
	*  Input: alglib_array (3x3 of alglib::real_2d_array)
	*  Output: c_array (3x3 C-type array)
	*/
	static inline void alglib_array2c_array_3x3(
		const alglib::real_2d_array& alglib_array,
		double c_array[3][3]);
};

