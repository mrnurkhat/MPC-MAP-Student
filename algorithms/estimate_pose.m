function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here
particles = public_vars.particles;

mean_x = mean(particles(:, 1));
mean_y = mean(particles(:, 2));

mean_sin = mean(sin(particles(:, 3)));
mean_cos = mean(cos(particles(:, 3)));
mean_dir = atan2(mean_sin, mean_cos);
    
estimated_pose = [mean_x, mean_y, mean_dir];

end

