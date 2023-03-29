/*GitHub Page: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration
*/

/* Remember to include all required dependencies
*  and load acc_mag_log.csv from https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/tree/main/Data%20Examples
*/
#include "Calibration.h"

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <exception>

/* A function that extracts data from file and stored them 
*  in acc_points and mag_points
*/
int dataExtractForMagCal(const char* fileName, const int data_number,
    Eigen::MatrixXd& acc_points, Eigen::MatrixXd& mag_points);

/*Printing functions*/
void printVector3d(const char* msg, const double v3x1[3]);
void printMatrix3x3(const char* msg, const double m3x1[3][3]);

int main()
{
    int data_number = 12000;

    /* Matrices to store accelerometer and magnetometer in 
    *  acc_points and mag_points respectively.
    */
    Eigen::MatrixXd acc_points(data_number, 3);
    Eigen::MatrixXd mag_points(data_number, 3);

    /* Extract data from acc_mag_log.csv
    */
    if (dataExtractForMagCal("acc_mag_log.csv", data_number,
            acc_points, mag_points))
    {
        std::cerr << "Error: File problem.\n";
        return -1;
    }

    try
    {
        /* mag_parameters struct to store all magnetometer 
        *  calibration parameters.
        */
        Calibration::mag_parameters mag; 

        /* Initial estimate for beta or cos(theta), 
        *  which theta = projection angle between gravity and 
        *  magnetic field vectors.
        */
        double beta = 0;
        /* Initial estimate for quaternion that represent the 
        *  misalignment correction matrix.
        */
        double initial_quaternion[4] = { 1, 0, 0, 0 };
        /* Running magnetometer calibration
        */
        Calibration::magnetometer_calibration(acc_points, mag_points,
            mag, beta, initial_quaternion);

        std::cout << "---------------------------------------\n";
        std::cout << "Magnetometer calibration parameters: \n\n";

        printVector3d("Center = ", mag.centre);
        printMatrix3x3("Transformation matrix = ", mag.transform_matrix);
        printMatrix3x3("Non-orthoganality matrix = ", mag.n_o);
        printMatrix3x3("Misaligment matrix = ", mag.R_cov);

        std::cout << "Estimated beta = " << beta << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception thrown: " << e.what() << '\n';
        std::cerr << "Error: Failed to compute magnetometer calibration parameters.\n";
        return -1;
    }
    
	return 0;
}

int dataExtractForMagCal(const char* fileName, const int data_number,
    Eigen::MatrixXd& acc_points, Eigen::MatrixXd& mag_points)
{
    auto ax = acc_points.col(0);
    auto ay = acc_points.col(1);
    auto az = acc_points.col(2);

    auto mx = mag_points.col(0);
    auto my = mag_points.col(1);
    auto mz = mag_points.col(2);

    try
    {
        std::ifstream file;
        file.open(fileName);
        if (file.is_open())
        {
            std::string line;
            int i = 0;
            int j = 0;
            bool first_line = true;
            while (std::getline(file, line))
            {
                if (first_line)
                {
                    first_line = false;
                    continue;
                }

                std::stringstream str(line);
                std::string number;
                while (std::getline(str, number, ','))
                {
                    switch (i)
                    {
                    case 0:
                        ax(j) = std::stod(number);
                        break;
                    case 1:
                        ay(j) = std::stod(number);
                        break;
                    case 2:
                        az(j) = std::stod(number);
                        break;
                    case 3:
                        mx(j) = std::stod(number);
                        break;
                    case 4:
                        my(j) = std::stod(number);
                        break;
                    case 5:
                        mz(j) = std::stod(number);
                        break;
                    default:
                        break;
                    }
                    i++;
                }
                i = 0;
                j++;
            }
            std::cout << data_number << " of data have been extracted from "
                << fileName << ".\n";
            file.close();
        }
        else
        {
            std::cerr << "Please make sure " << fileName
                << " is available.\n";
            return -2;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception thrown: " << e.what() << '\n';
        return -1;
    }

    return 0;
}

void printVector3d(const char* msg, const double v3x1[3])
{
    std::cout << msg << '\n';
    for (int i = 0; i < 3; i++)
    {
        std::cout << v3x1[i];
        if (i != 2)
        {
            std::cout << " , ";
        }
    }
    std::cout << "\n\n";
}

void printMatrix3x3(const char* msg, const double m3x1[3][3])
{
    std::cout << msg << '\n';
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3;j++)
        {
            std::cout << m3x1[i][j];
            if (j != 2)
            {
                std::cout << " , ";
            }
        }

        std::cout << '\n';
    }
    std::cout << '\n';
}