function [x, y] = grid2World(j, i, read_only_vars)
    min_x = read_only_vars.map.limits(1);
    min_y = read_only_vars.map.limits(2);
    res = read_only_vars.map.discretization_step;

    x = min_x + (i - 0.5) * res;  % col → x
    y = min_y + (j - 0.5) * res;  % row → y
end