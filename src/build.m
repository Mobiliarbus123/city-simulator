init.init_env;
init.init_db;

save_to_file = true;

% 加载 STL 模型
[vertices, faces] = utils.load_model(stl_file);

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));
bound.min = [model_range.x(1), model_range.y(1), model_range.z(1)];
bound.max = [model_range.x(2), model_range.y(2), model_range.z(2)];

% 检查数据尺寸
[size_x, size_y, size_z] = utils.get_model_size(vertices);
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
udf = utils.build_udf(vertices, faces, PADDING, SDF_RESOLUTION, bound, 35);
fprintf('UDF 网格构建完成。\n');
toc;
fprintf('--------------------\n\n');

% 丢弃无效数据
clear vertices faces;

% 将 UDF 网格写入数据库
if save_to_file
    save(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)), 'udf', '-v7.3');
    fprintf('UDF 网格已保存到文件。\n');
else
    save(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)), 'udf', '-v7.3');
    utils.write_udf(conn, udf, MODEL_NAME_IN_DB, CHUNK_SIZE, SRID, SDF_RESOLUTION);
    fprintf('UDF 网格数据已成功写入数据库。\n');
end

% % 可视化
% utils.show_model(vertices, faces, "New York");
