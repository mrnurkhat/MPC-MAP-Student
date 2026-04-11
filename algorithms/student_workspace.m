function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

%% Week 2 - Uncertainty

% Init
if ~isfield(public_vars, 'initialized_uncertainty')
    public_vars.lidar_meas_history = zeros(100, 8);
    public_vars.gnss_meas_history = zeros(100, 2);
    public_vars.std_lidar = zeros(1, 8);
    public_vars.std_gnss = zeros(1, 2);
    
    setup_content = fileread('setup.m');
    public_vars.is_indoor = contains(setup_content, "indoor");
    public_vars.initialized_uncertainty = true;
end

is_indoor = public_vars.is_indoor;

if read_only_vars.counter <= 100
    if is_indoor
        public_vars.lidar_meas_history(read_only_vars.counter, :) = read_only_vars.lidar_distances;
    else
        public_vars.gnss_meas_history(read_only_vars.counter, :) = read_only_vars.gnss_position;
    end
elseif read_only_vars.counter == 101
    
    if is_indoor
        % LiDAR Processing
        public_vars.std_lidar = std(public_vars.lidar_meas_history);
        public_vars.cov_lidar = cov(public_vars.lidar_meas_history);
        
        % Histogram
        figure('Name', 'LiDAR Analysis');
        subplot(2,1,1);
        histogram(public_vars.lidar_meas_history(:, 8), 20);
        title('Histogram of LiDAR (Ch 8)');
        
        % PDF
        current_std = public_vars.std_lidar(1);
        x = linspace(-4 * current_std, 4 * current_std, 100);
        pdf_val = norm_pdf(x, 0, public_vars.std_lidar(1));
        
        subplot(2,1,2);
        plot(x, pdf_val, 'r', 'LineWidth', 2);
        title("PDF of LiDAR Noise");
        grid on;
        
    else
        GNSS Processing 
        public_vars.std_gnss = std(public_vars.gnss_meas_history);
        public_vars.cov_gnss = cov(public_vars.gnss_meas_history);
        
        % Histogram
        figure('Name', 'GNSS Analysis');
        subplot(2,1,1);
        histogram(public_vars.gnss_meas_history(:, 2), 20);
        title('Histogram of GNSS (Y Axis)');
        
        % PDF
        current_std = public_vars.std_gnss(2); 
        x = linspace(-4 * current_std, 4 * current_std, 100);
        pdf_val = norm_pdf(x, 0, public_vars.std_gnss(1));
        
        subplot(2,1,2);
        plot(x, pdf_val, 'b', 'LineWidth', 2);
        title("PDF of GNSS Noise");
        grid on;
    end
end
%% ------------------------------------------------------------------------

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)    
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end

