function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

if read_only_vars.counter == 1
    % Trajectory between the start pose and the goal location
    path_points = 200;
    x0 = 9;
    y0 = 2.5;
    R = 7;
    omeg = linspace(pi, -pi/30, path_points);
    x = x0 + R * cos(omeg);
    y = y0 + R * sin(omeg);
    circle = [x', y'];
    
    % Motion planning parameters
    public_vars.path = circle;
    public_vars.look_ahead_dist = 0.5;
    public_vars.target_point = 1;  
    public_vars.slow_down_k = 0.5;
    public_vars.max_w = 1.0; 
    public_vars.finish_treshold = 1; 
end

%  Initialization procedure
if read_only_vars.counter <= 100
    public_vars.desired_speed = 0.0;  
elseif (read_only_vars.counter == 101)
    public_vars.desired_speed = 0.3;
    public_vars = init_kalman_filter(read_only_vars, public_vars);
else 
    % Update Kalman filter
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

    % Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

    % Plan next motion command
    public_vars = plan_motion(read_only_vars, public_vars);

end
end

