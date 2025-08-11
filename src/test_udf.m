init.init_env;

% --- 1. 加载原始STL模型数据 ---
fprintf('正在加载STL模型...\n');
[vertices, faces] = utils.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% --- 2. 加载 UDF ---
fprintf('正在加载UDF数据...\n');
udf = utils.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));

% --- 3. 从UDF中提取“表面”点 ---
fprintf('正在从UDF中提取零距离表面...\n');
% 找到距离场中的最小值
min_dist = min(udf.grid(:));
% 设置一个小的容差，找到所有接近表面的点
% 容差可以设置为分辨率的一半，以确保捕捉到最接近的体素
tolerance = 1;
% 找到所有值在 [min_dist, min_dist + tolerance] 范围内的点的线性索引
surface_indices = find(udf.grid <= (min_dist + tolerance));

if isempty(surface_indices)
    warning('警告: 未能在UDF中找到距离值接近于零的点。');
    udf_surface_points = [];
else
    % 将这些点的线性索引转换为三维下标索引 [y, x, z]
    [y_idx, x_idx, z_idx] = ind2sub(udf.grid_dims, surface_indices);
    % 从坐标向量中获取这些点的真实世界坐标
    udf_surface_points = [udf.x_vec(x_idx)', udf.y_vec(y_idx)', udf.z_vec(z_idx)'];
    fprintf('成功从UDF中提取了 %d 个表面点。\n', size(udf_surface_points, 1));
end

% --- 4. 可视化对比验证 ---
fprintf('正在进行可视化对比...\n');
figure('Name', 'UDF 正确性检验', 'NumberTitle', 'off');
hold on;

% 步骤 A: 绘制原始STL模型 (半透明)
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% 步骤 B: 绘制从UDF中提取出的表面点 (红色实心点)
if ~isempty(udf_surface_points)
    plot3(udf_surface_points(:, 1), udf_surface_points(:, 2), udf_surface_points(:, 3), ...
        'r.', 'MarkerSize', 1); % 使用红色的小点绘制
end

% --- 设置显示效果 ---
axis equal; view(3); grid on; rotate3d on;
lighting gouraud; camlight('headlight');
title('UDF 生成结果检验');
legend('原始STL模型 (半透明)', '从UDF提取的表面点');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off;
