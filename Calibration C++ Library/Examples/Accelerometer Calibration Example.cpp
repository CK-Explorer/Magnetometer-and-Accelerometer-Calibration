/*GitHub Page: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration
*/

/* Remember to include all required dependencies
*  and load acc_log.csv from https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/tree/main/Data%20Examples
*/
#include "Calibration.h"

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <exception>

/* A function that extracts data from file and stored them in acc_points
*/
int dataExtractForAccCal(const char* fileName, const int data_number,
    Eigen::MatrixXd& acc_points);

/*Printing functions*/
void printVector3d(const char* msg, const double v3x1[3]);
void printMatrix3x3(const char* msg, const double m3x1[3][3]);

int main()
{
    int data_number = 16000;

    /* Matrices to store accelerometer in acc_points
    */
    Eigen::MatrixXd acc_points(data_number, 3);

    /* Extract data from acc_log.csv
    */
    if (dataExtractForAccCal("acc_log.csv", data_number, 
            acc_points))
    {
        std::cerr << "Error: File problem.\n";
        return -1;
    }

    try
    {
        /* acc_parameters struct to store all accelerometer 
        *  calibration parameters.
        */
        Calibration::acc_parameters acc; 

        /* Running accelerometer calibration
        */
        Calibration::accelerometer_calibration(acc_points, acc);

        std::cout << "---------------------------------------\n";
        std::cout << "Accelerometer calibration parameters: \n\n";

        printVector3d("Center = ", acc.centre);
        printMatrix3x3("Transformation matrix = ", acc.n_o);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception thrown: " << e.what() << '\n';
        std::cerr << "Error: Failed to compute accelerometer calibration parameters.\n";
        return -1;
    }
    
	return 0;
}

int dataExtractForAccCal(const char* fileName, const int data_number,
    Eigen::MatrixXd& acc_points)
{
    auto ax = acc_points.col(0);
    auto ay = acc_points.col(1);
    auto az = acc_points.col(2);

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
            std::cerr << "Please make sure " << fileName << " is available.\n";
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
