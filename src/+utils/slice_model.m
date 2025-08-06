function [vertices_new, faces_new] = slice_model(vertices, faces, xmin, xmax, ymin, ymax)
    inbox = vertices(:, 1) >= xmin & vertices(:, 1) <= xmax & ...
        vertices(:, 2) >= ymin & vertices(:, 2) <= ymax;
    faces_in = inbox(faces);
    faces_mask = all(faces_in, 2); % 只保留全部顶点在范围内的面
    faces_new = faces(faces_mask, :);

    used_vertices = unique(faces_new(:));
    vertices_new = vertices(used_vertices, :);
    [~, new_idx] = ismember(faces_new, used_vertices);
    faces_new = new_idx;
end
