function UDF = build_udf(vertices, faces, padding, sdf_resolution, bounds, ground)
    % bounds: {min: [x_min, y_min, z_min], max: [x_max, y_max, z_max]}
    % context: min_bounds, max_bounds, grid_dims, x_vec, y_vec, z_vec

    context.min_bounds = bounds.min - padding;
    context.max_bounds = bounds.max + padding;

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
    bvh = utils.SimpleBVH(vertices, faces);

    parfor i = 1:total_voxels
        p = query_points(i, :);
        dist_val = min(bvh.query_single_point(p), abs(p(3) - ground));
        distances(i) = dist_val;
        ppm.increment();
    end

    delete(ppm);

    UDF_grid = reshape(distances, context.grid_dims);
    UDF = utils.UDF(UDF_grid, context);
end
