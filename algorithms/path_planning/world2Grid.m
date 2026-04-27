function [j, i] = world2Grid(x, y, read_only_vars)
    min_x = read_only_vars.map.limits(1);
    min_y = read_only_vars.map.limits(2);
    res = read_only_vars.map.discretization_step;

    j = floor((y - min_y) / res) + 1;  % y → row
    i = floor((x - min_x) / res) + 1;  % x → col
end