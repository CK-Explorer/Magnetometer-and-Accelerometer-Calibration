% GitHub Page: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration

% Require ellipsoid_fit_new function,
% From: https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit

% An example to run calibration_acc function from
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/tree/main/Calibration%20MATLAB%20script

% Please load acc_log.csv from 
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/tree/main/Data%20Examples

ax = acclog.ax;
ay = acclog.ay;
az = acclog.az;

[center, n_o] = calibration_acc(ax, ay, az)

%%
% For displaying non-calibrated accelerometer data and fitting a sphere for
% comparison

r = n_o(1,1);

[e1,e2,e3] = sphere;
h1 = surfl(e1/r + center(1,1), e2/r + center(2,1), e3/r + center(3,1)); 
set(h1, 'FaceAlpha', 0.5)
shading interp
hold on

scatter3(ax, ay, az)
axis equal

%%
% For displaying calibrated accelerometer data (un-normalised) and fitting
% a sphere for comparison

h2 = surfl(e1/r, e2/r, e3/r); 
set(h2, 'FaceAlpha', 0.5)
shading interp
hold on

acc_shifted_center = [ax-center(1,1), ay-center(2,1), az-center(3,1)];
acc_transform = (n_o*acc_shifted_center')'/r;
scatter3(acc_transform(:,1), acc_transform(:,2), acc_transform(:,3))
axis equal

%%
% Obtain the ellipsoid parameter by fitting the calibrated magnetometer 
% to the ellipsoid

[centerNew, radiiNew, ~, vNew, ~ ] = ellipsoid_fit_new(acc_transform)