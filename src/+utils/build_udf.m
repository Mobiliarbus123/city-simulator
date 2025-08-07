function [UDF_grid, context] = build_udf(vertices, faces, padding, sdf_resolution)
    T = triangulation(faces, vertices);
    % --- 2. ➡️ 从对象中提取数据，创建函数所需的 "转接头" 结构体 ---
    FV.Faces = T.ConnectivityList;    % 提取面数据
    FV.Vertices = T.Points;         % 提取顶点数据

    % context: min_bounds, max_bounds, grid_dims, x_vec, y_vec, z_vec
    context.min_bounds = min(vertices, [], 1) - padding;
    context.max_bounds = max(vertices, [], 1) + padding;
    
    % 创建网格坐标向量
    context.x_vec = context.min_bounds(1):sdf_resolution:context.max_bounds(1);
    context.y_vec = context.min_bounds(2):sdf_resolution:context.max_bounds(2);
    context.z_vec = context.min_bounds(3):sdf_resolution:context.max_bounds(3);

    context.grid_dims = [length(context.y_vec), length(context.x_vec), length(context.z_vec)];
    total_voxels = prod(context.grid_dims);

    % 将坐标向量转换为完整的网格点矩阵
    [X, Y, Z] = meshgrid(context.x_vec, context.y_vec, context.z_vec);
    query_points = [X(:), Y(:), Z(:)];

    % --- 并行计算距离 ---
    distances = zeros(total_voxels, 1);
    [distances, ~] = point2trimesh(FV, 'QueryPoints', query_points, 'Algorithm', 'parallel');

    UDF_grid = reshape(distances, context.grid_dims);
end
