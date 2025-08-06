function [x, y, z] = get_model_size(vertices)
    min_x = min(vertices(:, 1));
    max_x = max(vertices(:, 1));
    min_y = min(vertices(:, 2));
    max_y = max(vertices(:, 2));
    min_z = min(vertices(:, 3));
    max_z = max(vertices(:, 3));
    x = max_x - min_x;
    y = max_y - min_y;
    z = max_z - min_z;
end
