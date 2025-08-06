function UDF_grid = get_distance(vertices, faces, min_bounds, max_bounds)
    % 创建网格坐标向量
    x_vec = min_bounds(1):SDF_RESOLUTION:max_bounds(1);
    y_vec = min_bounds(2):SDF_RESOLUTION:max_bounds(2);
    z_vec = min_bounds(3):SDF_RESOLUTION:max_bounds(3);
    
    grid_dims = [length(y_vec), length(x_vec), length(z_vec)];
    total_voxels = prod(grid_dims);
    
    % 将坐标向量转换为完整的网格点矩阵
    [X, Y, Z] = meshgrid(x_vec, y_vec, z_vec);
    query_points = [X(:), Y(:), Z(:)];
    
    % --- 并行计算距离 ---
    distances = zeros(total_voxels, 1);
    parfor i = 1:total_voxels
    % 使用point2trimesh计算每个点到网格的最近距离
    distances(i) = point2trimesh(triangulation(faces, vertices), query_points(i,:), 'QueryType', 'point');
    end
    
    % 将一维的距离向量重塑为三维的SDF网格
    UDF_grid = reshape(distances, grid_dims); % Unsigned Distance Field
end