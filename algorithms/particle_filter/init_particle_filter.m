function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

N = public_vars.particles_count;

x_min = read_only_vars.map.limits(1);
y_min = read_only_vars.map.limits(2);

x_max = read_only_vars.map.limits(3);
y_max = read_only_vars.map.limits(4);

particles_x = x_min + (x_max - x_min) * rand(N, 1);
particles_y = y_min + (y_max - y_min) * rand(N, 1);
particles_dir = -pi + 2 * pi * rand(N, 1);

public_vars.particles = [particles_x, particles_y, particles_dir];
public_vars.weights = ones(N, 1) / N;

end

