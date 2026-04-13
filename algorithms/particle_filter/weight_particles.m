function [weights] = weight_particles(particle_measurements, lidar_distances, public_vars)
% Evaluates particles by comparing predicted and actual sensor data

num_particles = size(particle_measurements, 1);
sigma = public_vars.sensor_noise;
weights = public_vars.weights;

for i = 1:num_particles
    errors = lidar_distances - particle_measurements(i, :);
    log_w = -errors.^2 / (2 * sigma^2);
    weights(i) = sum(log_w);
end

weights = exp(weights - max(weights));
weights = weights / sum(weights);

end

