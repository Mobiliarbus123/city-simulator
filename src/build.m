init.init_env;

% 加载 STL 模型
[vertices, faces] = utils.model.load_model(stl_file);

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.model.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));
bound.min = [model_range.x(1), model_range.y(1), model_range.z(1)];
bound.max = [model_range.x(2), model_range.y(2), model_range.z(2)];

% 检查数据尺寸
[size_x, size_y, size_z] = utils.model.get_model_size(vertices);
fprintf('--- 数据尺寸检查 ---\n');
fprintf('模型包围盒尺寸: X=%.5f, Y=%.5f, Z=%.5f\n', size_x, size_y, size_z);
fprintf('Vertices 矩阵的尺寸: %d x %d\n', size(vertices, 1), size(vertices, 2));
fprintf('Faces 矩阵的尺寸: %d x %d\n', size(faces, 1), size(faces, 2));

fprintf('--------------------\n\n');
clear size_x size_y size_z;

% 构建 UDF 网格
fprintf('--- 构建 UDF 网格 ---\n');
fprintf("开始构建 UDF 网格...\n");
tic;
udf = utils.udf.build_udf(vertices, faces, PADDING, SDF_RESOLUTION, bound, 35);
fprintf('UDF 网格构建完成。\n');
toc;
fprintf('--------------------\n\n');

% 构建高度信息
fprintf('--- 构建高度信息 ---\n');
max_height_map = utils.udf.MaxHMap(udf);
fprintf('最大高度图构建完成。\n');
fprintf('--------------------\n\n');

% 丢弃无效数据
clear vertices faces;

save(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)), "udf", "max_height_map", "-v7.3");
fprintf('UDF 网格已保存到文件。\n');
