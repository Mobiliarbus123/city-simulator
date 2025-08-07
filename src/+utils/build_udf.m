function [UDF_grid, context] = build_udf(vertices, faces, padding, sdf_resolution)
    FV.faces = faces; % 提取面数据
    FV.vertices = vertices; % 提取顶点数据

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

    % % --- 并行计算距离 ---
    % [distances, ~] = point2trimesh(FV, 'QueryPoints', query_points, 'Algorithm', 'parallel');

    ppm = ParforProgressbar(total_voxels, 'progressBarUpdatePeriod', 3);

    distances = zeros(total_voxels, 1);

    parfor i = 1:total_voxels
        [dist_val, ~] = point2trimesh(FV, 'QueryPoints', query_points(i, :));
        distances(i) = dist_val;
        ppm.increment();
    end

    delete(ppm);

    UDF_grid = reshape(distances, context.grid_dims);
end
