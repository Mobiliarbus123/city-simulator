init.init_env;
init.init_db;

MODEL_NAME_IN_DB = 'newyork_udf_v1'; % 数据库中模型的名称
SDF_RESOLUTION = 2.0; % 米/体素。分辨率越高，计算越慢，数据量越大。建议从较大值开始测试。
PADDING = 50.0; % 米。在模型边界外额外扩展的距离，确保无人机有足够空间。
CHUNK_SIZE = 32; % 将SDF切割成 32x32x32 的小块。建议为2的幂（16, 32, 64）。
SRID = 0; % 空间参考ID。0代表局部笛卡尔坐标系。

% 加载 STL 模型
[vertices, faces] = utils.load_model(stl_file);

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, 1000, 1700, 500, 1300);

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
[UDF_grid, UDF_context] = utils.build_udf(vertices, faces, PADDING, SDF_RESOLUTION);
fprintf('UDF 网格构建完成。\n');
toc;
fprintf('--------------------\n\n');

% 丢弃无效数据
clear vertices faces;

% 将 UDF 网格写入数据库
if save_to_file
    save(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)), 'UDF_grid', 'UDF_context', '-v7.3');
    fprintf('UDF 网格已保存到文件。\n');
else
    save(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)), 'UDF_grid', 'UDF_context', '-v7.3');
    utils.write_udf(UDF_grid, MODEL_NAME_IN_DB, UDF_context, CHUNK_SIZE, SRID);
    fprintf('UDF 网格数据已成功写入数据库。\n');
end

% % 可视化
% utils.show_model(vertices, faces, "New York");
