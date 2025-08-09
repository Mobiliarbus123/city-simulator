init.init_env;

% 加载 STL 模型
[vertices, faces] = utils.load_model(stl_file);

% 数据预处理
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% for i = 1:size(faces, 1)
%     idx = faces(i, :);
%     pts = vertices(idx, :);
%     z_vals = pts(:, 3);

%     if max(z_vals) - min(z_vals) <= 10 && max(z_vals) < 50
%         fprintf('((%.4f,%.4f,%.4f), (%.4f,%.4f,%.4f), (%.4f,%.4f,%.4f))\n', ...
%             pts(1, 1), pts(1, 2), pts(1, 3), ...
%             pts(2, 1), pts(2, 2), pts(2, 3), ...
%             pts(3, 1), pts(3, 2), pts(3, 3));
%     end

% end

% 可视化
utils.show_model(vertices, faces, "New York");
