function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

target = get_target(public_vars.estimated_pose, public_vars.path);

x = 1;
y = 2; 
o = 3;

pos_global = read_only_vars.mocap_pose;
trajectory = public_vars.path;
found_target = false;

for i = public_vars.target_point:size(trajectory, 1)
    D = sqrt((pos_global(x) - trajectory(i, x))^2 + (pos_global(y) - trajectory(i, y))^2);
    if (D > public_vars.look_ahead_dist)
        found_target = true;
        public_vars.target_point = i;
        break;
    end
end

if found_target == false 
    public_vars.target_point = size(trajectory, 1);
    dist_to_finish = sqrt((pos_global(x) - trajectory(public_vars.target_point, x))^2 + ...
        (pos_global(y) - trajectory(public_vars.target_point, y))^2);

    w = 0;
    if dist_to_finish < public_vars.finish_treshold
        public_vars.desired_speed = 0;
    end
else
    dx = trajectory(public_vars.target_point, x) - pos_global(x);
    dy = trajectory(public_vars.target_point, y) - pos_global(y);
    theta = pos_global(o);
    
    y_local = -dx * sin(theta) + dy * cos(theta);
        
    gamma = 2 * y_local / public_vars.look_ahead_dist^2;
    
    w = gamma * public_vars.desired_speed;
    w = max(-public_vars.max_w, min(public_vars.max_w, w));
end

speed_r = public_vars.desired_speed + w * read_only_vars.agent_drive.interwheel_dist / 2;
speed_l = public_vars.desired_speed - w * read_only_vars.agent_drive.interwheel_dist / 2;
        
public_vars.motion_vector = [speed_r, speed_l];

end 