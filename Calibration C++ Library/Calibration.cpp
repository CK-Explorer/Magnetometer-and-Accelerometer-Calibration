/* GitHub Page: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration
*  API tutorial: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Calibration%20C%2B%2B%20Library/Brief%20explanation.md
*  Documentation: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Working%20Principle/Working%20Principle.pdf
*/

#include "Calibration.h"

int Calibration::accelerometer_calibration(
    const Eigen::MatrixXd& acc_points, acc_parameters& acc_var)
{
    alglib::real_2d_array n_o;
    non_orthogonality(acc_points, n_o, acc_var.centre);
    alglib_array2c_array_3x3(n_o, acc_var.n_o);

    return 0;
}

int Calibration::magnetometer_calibration(
    const Eigen::MatrixXd& acc_points,
    const Eigen::MatrixXd& mag_points,
    mag_parameters& mag_var,
    double& beta, const double initial_quaternion[4])
{
    alglib::real_2d_array n_o;
    non_orthogonality(mag_points, n_o, mag_var.centre);
    alglib_array2c_array_3x3(n_o, mag_var.n_o);

    if (acc_points.rows() != mag_points.rows())
        return -1;

    int data_number = acc_points.rows();
    alglib::real_2d_array data;
    data.setlength(data_number, 6);

    for (int i = 0; i < data_number; i++)
    {
        data[i][0] = acc_points(i, 0);
        data[i][1] = acc_points(i, 1);
        data[i][2] = acc_points(i, 2);
        double mx_ = mag_points(i, 0) - mag_var.centre[0];
        double my_ = mag_points(i, 1) - mag_var.centre[1];
        double mz_ = mag_points(i, 2) - mag_var.centre[2];
        data[i][3] = n_o[0][0] * mx_;
        data[i][4] = n_o[1][0] * mx_ + n_o[1][1] * my_;
        data[i][5] = n_o[2][0] * mx_ + n_o[2][1] * my_ + n_o[2][2] * mz_;
    }

    alglib::ae_int_t info;
    alglib::lsfitstate state;
    alglib::lsfitreport rep;

    alglib::real_1d_array para = misalignment_correction(
        data, data_number, beta, initial_quaternion, info, state, rep);

    alglib::real_2d_array R_cov; R_cov.setlength(3, 3);
    stereographic2rotm(para, R_cov);
    alglib_array2c_array_3x3(R_cov, mag_var.R_cov);

    alglib::real_2d_array transform; transform.setlength(3, 3);
    alglib::rmatrixgemm(3, 3, 3, 1,
        R_cov, 0, 0, 0,
        n_o, 0, 0, 0,
        0, transform, 0, 0);

    beta = para[3];
    alglib_array2c_array_3x3(transform, 
        mag_var.transform_matrix);

    return 0;
}

void Calibration::non_orthogonality(
    const Eigen::MatrixXd& points,
    alglib::real_2d_array& n_o, double centre[3])
{
    Eigen::Matrix<double, 10, 1> algebraicForm;
    auto parameters = ellipsoid::fit(points, 
        [&](const Eigen::Matrix<double, 10, 1>& v) { algebraicForm = v; });

    Eigen::Matrix3d k_matrix;
    k_matrix << 
        algebraicForm(0), algebraicForm(3), algebraicForm(4),
        algebraicForm(3), algebraicForm(1), algebraicForm(5),
        algebraicForm(4), algebraicForm(5), algebraicForm(2);

    double value = parameters.center.transpose() * k_matrix * parameters.center
        - algebraicForm(9);
    
    algebraicForm = algebraicForm / value;

    for (int i = 0; i < 3; i++)
    {
        centre[i] = parameters.center(i);
    }

    n_o = "[[0, 0, 0],[0, 0, 0],[0, 0, 0]]";

    n_o[2][2] = sqrt(algebraicForm(2));
    n_o[2][1] = algebraicForm(5) / n_o[2][2];
    n_o[2][0] = algebraicForm(4) / n_o[2][2];
    n_o[1][1] = sqrt(algebraicForm(1) - n_o[2][1] * n_o[2][1]);
    n_o[1][0] = (algebraicForm(3) - n_o[2][1] * n_o[2][0]) / n_o[1][1];
    n_o[0][0] = sqrt(algebraicForm(0) - n_o[1][0] * n_o[1][0] - n_o[2][0] * n_o[2][0]);
}

alglib::real_1d_array Calibration::misalignment_correction(
    const alglib::real_2d_array& data, int data_no,
    double beta, const double initial_quaternion[4],
    alglib::ae_int_t& info, alglib::lsfitstate& state, 
    alglib::lsfitreport& rep)
{
    alglib::real_1d_array y; y.setlength(data_no);
    for (int i = 0; i < y.length(); i++)
    {
        y[i] = 0;
    }

    /*Normalize estimators*/
    double quat_magnitude = 0;
    for (int i = 0; i < 4; i++)
    {
        quat_magnitude += initial_quaternion[i] * initial_quaternion[i];
    }
    quat_magnitude = sqrt(quat_magnitude);
    
    /*Initial estimators*/
    alglib::real_1d_array q; q.setlength(4);
    for (int i = 0; i < q.length(); i++)
    {
        q[i] = initial_quaternion[i] / quat_magnitude;
    }

    alglib::real_1d_array para;
    para.setlength(4);
    para[3] = beta;
    quat2stereographic(q, para);

    /*Procedure for fitting*/
    alglib::lsfitcreatef(data, y, para, diff_step, state);
    alglib::lsfitsetcond(state, stop_cond, max_iteration);
    alglib::lsfitfit(state, Calibration::misalignment_correction_func);
    alglib::lsfitresults(state, info, para, rep);

    return para;
}

void Calibration::misalignment_correction_func(
    const alglib::real_1d_array& para, const alglib::real_1d_array& data, 
    double& func, void* ptr)
{
    alglib::real_2d_array R_cov; R_cov.setlength(3, 3);
    stereographic2rotm(para, R_cov);

    alglib::real_2d_array data_acc; data_acc.setlength(3, 1);
    alglib::real_2d_array data_mag; data_mag.setlength(3, 1);
    for (int i = 0; i < 3; i++)
    {
        data_acc[i][0] = data[i];
        data_mag[i][0] = data[i + 3];
    }

    alglib::real_2d_array temp; temp.setlength(3, 1);
    alglib::rmatrixgemm(3, 1, 3, 1,
        R_cov, 0, 0, 0,
        data_mag, 0, 0, 0,
        0, temp, 0, 0);

    alglib::real_2d_array mul; mul.setlength(1, 1);
    alglib::rmatrixgemm(1, 1, 3, 1,
        data_acc, 0, 0, 1,
        temp, 0, 0, 0,
        0, mul, 0, 0);

    alglib::real_2d_array acc_val; acc_val.setlength(1, 1);
    alglib::rmatrixgemm(1, 1, 3, 1,
        data_acc, 0, 0, 1,
        data_acc, 0, 0, 0,
        0, acc_val, 0, 0);

    alglib::real_2d_array mag_val; mag_val.setlength(1, 1);
    alglib::rmatrixgemm(1, 1, 3, 1,
        data_mag, 0, 0, 1,
        data_mag, 0, 0, 0,
        0, mag_val, 0, 0);

    func = mul[0][0] - sqrt(acc_val[0][0] * mag_val[0][0]) * para[3];
}

void Calibration::quat2stereographic(const alglib::real_1d_array& quat,
    alglib::real_1d_array& x)
{
    double denom = 1 - quat[1];
    x[0] = quat[0] / denom;
    x[1] = quat[2] / denom;
    x[2] = quat[3] / denom;
}

inline void Calibration::stereographic2rotm(const alglib::real_1d_array& x, 
    alglib::real_2d_array& R_cov)
{
    double denom = 1 + x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
    double q_1 = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - 1) / denom;
    double q_0 = 2 * x[0] / denom;
    double q_2 = 2 * x[1] / denom;
    double q_3 = 2 * x[2] / denom;

    R_cov[0][0] = 1 - 2 * (q_2 * q_2 + q_3 * q_3);
    R_cov[0][1] = 2 * (q_1 * q_2 - q_3 * q_0);
    R_cov[0][2] = 2 * (q_1 * q_3 + q_2 * q_0);
    R_cov[1][0] = 2 * (q_1 * q_2 + q_3 * q_0);
    R_cov[1][1] = 1 - 2 * (q_1 * q_1 + q_3 * q_3);
    R_cov[1][2] = 2 * (q_2 * q_3 - q_1 * q_0);
    R_cov[2][0] = 2 * (q_1 * q_3 - q_2 * q_0);
    R_cov[2][1] = 2 * (q_2 * q_3 + q_1 * q_0);
    R_cov[2][2] = 1 - 2 * (q_1 * q_1 + q_2 * q_2);
}

inline void Calibration::alglib_array2c_array_3x3(
    const alglib::real_2d_array& alglib_array, 
    double c_array[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3;j++)
        {
            c_array[i][j] = alglib_array[i][j];
        }
    }
}


