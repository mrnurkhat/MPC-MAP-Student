function new_path = smooth_path(old_path, public_vars)
    new_path = old_path; 
    
    tolerance = public_vars.tolerance;
    alpha = public_vars.weight_data;
    beta = public_vars.weight_smooth;

    change = tolerance;
    
    while change >= tolerance
        change = 0.0;

        for i = 2:size(old_path, 1) - 1
            for j = 1:size(old_path, 2)
                
                old_val = new_path(i, j);
              
                new_path(i, j) = new_path(i, j) + alpha * (old_path(i, j) - new_path(i, j)) + ...
                    beta * (new_path(i-1, j) + new_path(i+1, j) - 2.0 * new_path(i, j));
                
                change = change + abs(old_val - new_path(i, j));
            end
        end
    end
end