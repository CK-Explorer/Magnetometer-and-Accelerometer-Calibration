function [center, n_o] = calibration_acc(ax, ay, az)
%
% For more information, pls refer to:
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration
% Function tutorial: 
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Calibration%20MATLAB%20script/Brief%20explanation.md
% Documentation: 
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/blob/main/Working%20Principle/Working%20Principle.pdf
%
% An accelerometer calibration script that corrects
%   * accelerometer's center biases
%   * accelerometer's axes scaling factors
%   * non-orthogonality between accelerometer's axes
% 
% Parameters:
% ax, ay, az (all are nx1) = x, y, z axes accelerometer data
% 
% Outputs:
% center (3x1) = center biases
% n_o (3x3) = non-orthogonal and scaling factor correction matrix 
% 
% Way of using outputs:
% accelerometer data with shifted bias, a_b (3x1) = [ ax - center(1) 
%                                                     ay - center(2)
%                                                     az - center(3) ]
% corrected normalised accelerometer data (3x1) = n_o * a_b
%

% Fitting the accelerometer data onto the surface of ellipsoid
% See : https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
[center, ~, ~, v, ~ ] = ellipsoid_fit_new( [ax ay az] );

% Determine the non-orthogonal and scaling factor correction matrix 
% for the accelerometer axes
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
end

