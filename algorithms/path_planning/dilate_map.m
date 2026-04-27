function dilated_map = dilate_map(map, radius_px)
[rows, cols] = size(map);

dilated_map = map;

for j = 1:rows
    for i = 1:cols
        if map(j, i) ~= 0
            for dj = -radius_px:radius_px
                for di = -radius_px:radius_px
                    if hypot(dj, di) <= radius_px
                        nj = j + dj;
                        ni = i + di;

                        if nj >= 1 && nj <= rows && ni >= 1 && ni <= cols
                            dilated_map(nj, ni) = 1;
                        end
                    end
                end
            end
        end
    end
end
end