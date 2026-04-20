function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt, wheel_base)
%EKF_PREDICT Summary of this function goes here

vel_r = u(1);
vel_l = u(2);

linear = (vel_l + vel_r) / 2;
angular = (vel_r - vel_l) / wheel_base;

x = mu(1);
y = mu(2);
dir = mu(3);

x_new = x + linear * cos(dir) * dt;
y_new = y + linear * sin(dir) * dt;
dir_new = dir + angular * dt;
dir_new = atan2(sin(dir_new), cos(dir_new));

new_mu = [x_new; y_new; dir_new];

G = [1, 0, -linear * sin(dir) * dt;
     0, 1,  linear * cos(dir) * dt;
     0, 0,  1 ];

new_sigma = G * sigma * G' + kf.R;

end

