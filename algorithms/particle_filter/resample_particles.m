function [new_particles] = resample_particles(particles, weights)
% Performs systematic resampling to maintain particle diversity.
% It replaces low-weight particles with copies of high-weight ones.

num_particles = length(weights);

new_particles = zeros(num_particles, size(particles, 2));

cdf = cumsum(weights);

step = 1 / num_particles;
current_tooth = rand() * step;

new_idx = 1;
idx = 1;

while new_idx <= num_particles
    if cdf(idx) >= current_tooth
        current_tooth = current_tooth + step;
        new_particles(new_idx, :) = particles(idx, :);
        new_idx = new_idx + 1;
    else
        idx = idx + 1;
    end
end

end

