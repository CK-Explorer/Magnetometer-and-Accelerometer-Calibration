function [transform_matrix, center, n_o, R, beta] = calibration_mag(ax, ay, az,...
    mx, my, mz, q_est, beta)
%
% For more information, pls refer to:
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration
% Function tutorial: 
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Calibration%20MATLAB%20script/Brief%20explanation.md
% Documentation: 
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Working%20Principle/Working%20Principle.pdf
%
% A magnetometer calibration script that corrects
%   * magnetometer's center biases
%   * magnetometer's axes scaling factors
%   * non-orthogonality between magnetometer's axes
%   * misaligment between magnetometer and accelerometer
% 
% Parameters:
% ax, ay, az (all are nx1) = x, y, z axes accelerometer data
% mx, my, mz (all are nx1)= x, y, z axes magnetometer data
% q_est (4x1 or 1x4) = quaternion estimate for rotation matrix
% beta (1x1) = estimated beta
% 
% Outputs:
% center (3x1) = center biases
% transform_matrix (3x3) =  transformation matrix for the magnetometer data
%                           for shifted biases
% n_o (3x3) = non-orthogonal and scaling factor correction matrix 
% R (3x3) = rotation matrix to align the magnetometer with the accelerometer
% beta (1x1) = cosine of theta, which theta is the projection angle between
%              the gravity and magnetic field's vectors. 
% 
% Way of using outputs:
% magnetometer data with shifted bias, m_b (3x1) = [ mx - center(1) 
%                                                    my - center(2)
%                                                    mz - center(3) ]
% corrected normalised magnetometer data (3x1) = transform_matrix * m_b
% while n_o, R and beta are just for reference.
%

% Fitting the magnetometer data onto the surface of ellipsoid
% See : https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
[center, ~, ~, v, ~ ] = ellipsoid_fit_new( [mx my mz] );

% Determine the non-orthogonal and scaling factor correction matrix 
% for the magnetometer axes
k_matrix = [v(1) v(4) v(5); v(4) v(2) v(6); v(5) v(6) v(3)];
k_normal = center'*k_matrix*center - v(10);
v = v / k_normal;
n_o = zeros(3,3);
n_o(3,3) = sqrt(v(3));
n_o(3,2) = v(6)/n_o(3,3);
n_o(3,1) = v(5)/n_o(3,3);
n_o(2,2) = sqrt(v(2) - n_o(3,2).^2);
n_o(2,1) = (v(4) - n_o(3,2) * n_o(3,1))/n_o(2,2);
n_o(1,1) = sqrt(v(1) - n_o(2,1).^2 - n_o(3,1).^2);

% Shifting and transforming the magnetometer data using center and n_o 
% respectively.   
mag_shifted_center = [mx-center(1,1), my-center(2,1), mz-center(3,1)];
m_transform = (n_o*mag_shifted_center')';

% Using the least square solver, the misallignment correction matrix is
% determined to rotate the axes of magnetometer to the accelerometer 
data = [ax ay az m_transform(:,1) m_transform(:,2) m_transform(:,3)];
ydata = zeros(size(data,1),1);
initial_guess = quat2stereographic(normalize(q_est));
x0 = [initial_guess(1) initial_guess(2) initial_guess(3) beta];
opts = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');
opts = optimoptions(opts, 'Display', 'iter');
opts = optimoptions(opts, 'FunctionTolerance', 1e-22);
x = lsqcurvefit(@misallignment_correction,x0,data,ydata,[],[],opts);
R = stereographic2rotm([x(1) x(2) x(3)]);
beta = x(4);

% Total transformation matrix to be applied to magnetometer.
transform_matrix = R*n_o;
end


function y = misallignment_correction(x, data)
%
% The cost function that computes the dot product invariance between
% accelerometer and magnetometer.
%
% Parameters:
% data = [ax ay az mx my mz], which a = accelerometer, m = magnetometer
%

denom = (1 + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
q_1 = (x(1)*x(1) + x(2)*x(2) + x(3)*x(3) - 1) / denom;
q_0 = 2*x(1) / denom;
q_2 = 2*x(2) / denom;
q_3 = 2*x(3) / denom;
k = x(4);

quat = [q_0 q_1 q_2 q_3];
R = quat2rotm(quat);

mul = zeros(size(data,1),1);

for index = 1:size(data,1)
    mul(index) = [data(index,1) data(index,2) data(index,3)]*R*...
        [data(index,4) data(index,5) data(index,6)]';
end

magnitude = zeros(size(data,1),1);

for index = 1:size(data,1)
    accel_val = sqrt(data(index,1)*data(index,1) + data(index,2)*data(index,2)...
        + data(index,3)*data(index,3));
    mag_val = sqrt(data(index,4)*data(index,4) + data(index,5)*data(index,5) ...
        + data(index,6)*data(index,6));
    magnitude(index) = accel_val*mag_val;
end

y = mul - magnitude*k;

end


function x = quat2stereographic(quat)
%
% Quaternion to sterographic projection conversion
%
% Parameter:
% quat (4x1 or 1x4) = quaternion
%
% Outputs:
% x (3x1 or 1x3) = quaternion in stereographic projection form
%

denom = 1 - quat(2);
x(1) = quat(1) / denom;
x(2) = quat(3) / denom;
x(3) = quat(4) / denom;
end


function R = stereographic2rotm(x)
%
% Stereographic projection to quaternion, then lastly to rotation matrix
% conversion
%
% Parameter:
% x (3x1 or 1x3) = quaternion in stereographic projection form
%
% Output:
% R (3x3) = rotation matrix converted from x
%

denom = (1 + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));

q_1 = (x(1)*x(1) + x(2)*x(2) + x(3)*x(3) - 1) / denom;
q_0 = 2*x(1) / denom;
q_2 = 2*x(2) / denom;
q_3 = 2*x(3) / denom;

quat = [q_0 q_1 q_2 q_3];
R = quat2rotm(quat);
end
