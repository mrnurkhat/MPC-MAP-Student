function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    path_points = 200;

    % Circle
    x0 = 3.5;
    y0 = 6.5;
    R = 2.5;
    omeg = linspace(0, 2 * pi, path_points);
    x = x0 + R * cos(omeg);
    y = y0 + R * sin(omeg);
    circle = [x', y'];

    public_vars.path = circle; % the trajectory to follow
    public_vars.look_ahead_dist = 0.5;% Lh dist for pure pursuit
    public_vars.target_point = 1;   % starting index in waypoints array
    public_vars.desired_speed = 0.3; % constant velocity m/s
    public_vars.slow_down_k = 0.5; % gain for braking at the end
    public_vars.max_w = 1.0; % angular velocity limit
    public_vars.finish_treshold = 1; % distance to stop the robo

    public_vars.particles_count = 500;
    public_vars.motion_noise = 0.7;
    public_vars.sensor_noise = 4;
    
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

% 9. Update particle filter
public_vars = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
%public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end

