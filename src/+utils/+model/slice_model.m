function [vertices_new, faces_new] = slice_model(vertices, faces, xmin, xmax, ymin, ymax)
    inbox = vertices(:, 1) >= xmin & vertices(:, 1) <= xmax & ...
        vertices(:, 2) >= ymin & vertices(:, 2) <= ymax;

    faces_inbox = inbox(faces);

    xy_mask = all(faces_inbox, 2);

    % 获取初步筛选后的面片
    faces_preliminary = faces(xy_mask, :);

    % 如果初步筛选后没有面片了，直接返回空
    if isempty(faces_preliminary)
        vertices_new = [];
        faces_new = [];
        return;
    end

    vertex_indices = faces_preliminary(:);
    z_coords_flat = vertices(vertex_indices, 3);
    z_vals_per_face = reshape(z_coords_flat, [], 3);

    max_z_per_face = max(z_vals_per_face, [], 2);
    min_z_per_face = min(z_vals_per_face, [], 2);

    z_delta_mask = (max_z_per_face - min_z_per_face) <= 10;
    z_height_mask = max_z_per_face < 50;

    z_mask = ~(z_delta_mask & z_height_mask);

    faces_new = faces_preliminary(z_mask, :);

    if isempty(faces_new)
        vertices_new = [];
        faces_new = [];
        return;
    end

    used_vertices = unique(faces_new(:));
    vertices_new = vertices(used_vertices, :);
    [~, new_idx] = ismember(faces_new, used_vertices);
    faces_new = new_idx;
end
