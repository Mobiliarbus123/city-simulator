function [x, y, z, full] = get_model_size(vertices)
    full.min_x = min(vertices(:, 1));
    full.max_x = max(vertices(:, 1));
    full.min_y = min(vertices(:, 2));
    full.max_y = max(vertices(:, 2));
    full.min_z = min(vertices(:, 3));
    full.max_z = max(vertices(:, 3));
    x = full.max_x - full.min_x;
    y = full.max_y - full.min_y;
    z = full.max_z - full.min_z;
end
