function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars, std_vel)
% This function estimates the next pose of a differential drive robot by 
% converting wheel velocities into linear and angular velocities

x = old_pose(1);
y = old_pose(2);
dir = old_pose(3);

vel_r = motion_vector(1);
vel_l = motion_vector(2);

wheel_base = read_only_vars.agent_drive.interwheel_dist;
dt = read_only_vars.sampling_period;

vel_l = vel_l + vel_l * std_vel * randn();
vel_r = vel_r + vel_r * std_vel * randn();

linear = (vel_l + vel_r) / 2;
angular = (vel_r - vel_l) / wheel_base;

new_x = x + linear * cos(dir) * dt;
new_y = y + linear * sin(dir) * dt;
new_dir = dir + angular * dt;

new_dir = atan2(sin(new_dir), cos(new_dir));

new_pose = [new_x, new_y, new_dir];

end

