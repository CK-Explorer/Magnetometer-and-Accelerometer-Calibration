% GitHub Page: https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration

% Require ellipsoid_fit_new function,
% From: https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit

% An example to run calibration_mag function from
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/tree/main/Calibration%20MATLAB%20script

% Please load acc_mag_log.csv from
% https://github.com/CK-Explorer/Magnetometer-and-Accelerometer-Calibration/tree/main/Data%20Examples

ax = accmaglog.ax;
ay = accmaglog.ay;
az = accmaglog.az;

mx = accmaglog.mx;
my = accmaglog.my;
mz = accmaglog.mz;

[transform_matrix, center, n_o, R, beta] = calibration_mag(ax, ay, az, ...
    mx, my, mz, [1 0 0 0], 0)

%%
% For displaying non-calibrated magnetometer data and fitting a sphere for
% comparison.

r = n_o(1,1);

[e1,e2,e3] = sphere;
h1 = surfl(e1/r + center(1,1), e2/r + center(2,1), e3/r + center(3,1)); 
set(h1, 'FaceAlpha', 0.5)
shading interp
hold on

scatter3(mx, my, mz)
axis equal

%%
% For displaying calibrated magnetometer data (un-normalised) and fitting a
% sphere for comparison

h2 = surfl(e1/r, e2/r, e3/r); 
set(h2, 'FaceAlpha', 0.5)
shading interp
hold on

mag_shifted_center = [mx-center(1,1), my-center(2,1), mz-center(3,1)];
m_transform = (transform_matrix*mag_shifted_center')'/r;
scatter3(m_transform(:,1), m_transform(:,2), m_transform(:,3))
axis equal

%%
% Obtain the ellipsoid parameter by fitting the calibrated magnetometer 
% to the ellipsoid

[centerNew, radiiNew, ~, vNew, ~ ] = ellipsoid_fit_new(m_transform)