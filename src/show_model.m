init.init_env;

% 加载 STL 模型
[vertices, faces] = utils.model.load_model(stl_file);

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.model.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% 打印尺寸信息
[~, ~, ~, bound] = utils.model.get_model_size(vertices);
fprintf("模型尺寸信息：\n");
fprintf("X轴范围: %.2f - %.2f\n", bound.min_x, bound.max_x);
fprintf("Y轴范围: %.2f - %.2f\n", bound.min_y, bound.max_y);
fprintf("Z轴范围: %.2f - %.2f\n", bound.min_z, bound.max_z);

% 可视化
utils.model.show_model(vertices, faces, "New York");
