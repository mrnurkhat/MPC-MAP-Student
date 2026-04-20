function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here
C = kf.C;
Q = kf.Q;

error = z' - (C * mu);

S = C * sigma * C' + Q;
K = (sigma * C') / S;

new_mu = mu + K * error;

I = eye(size(sigma));
new_sigma = (I - K * C) * sigma;
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
end

