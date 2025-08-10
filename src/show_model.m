init.init_env;

% 加载 STL 模型
[vertices, faces] = utils.load_model(stl_file);

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% 可视化
utils.show_model(vertices, faces, "New York");
