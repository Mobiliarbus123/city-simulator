init.init_env;
init.init_db;

% 加载 STL 模型
try
    [vertices, faces] = utils.load_model(stl_file);
catch ME
    fprintf(2, '!!! 模型加载失败: %s\n', ME.message);
    return;
end

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, 1000, 1700, 500, 1300);

% 检查数据尺寸
[size_x, size_y, size_z] = utils.get_model_size(vertices);
fprintf('\n--- 数据尺寸检查 ---\n');
fprintf('模型包围盒尺寸: X=%.5f, Y=%.5f, Z=%.5f\n', size_x, size_y, size_z);
fprintf('Vertices 矩阵的尺寸: %d x %d\n', size(vertices, 1), size(vertices, 2));
fprintf('Faces 矩阵的尺寸: %d x %d\n', size(faces, 1), size(faces, 2));
fprintf('--------------------\n\n');

% 可视化
utils.show_model(vertices, faces, "New York");
