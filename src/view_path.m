init.init_env;

env.reverse = false;

% 加载 STL 模型
fprintf('加载 STL 模型...\n');
[vertices, faces] = utils.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% 加载路径
fprintf('加载路径...\n');
load_path = init.build_path(sprintf("run/%s_history.mat", MODEL_NAME_IN_DB));
load(load_path, "history", "step");

% 加载反向路径
if env.reverse
    fprintf('加载反向路径...\n');
    load_path = init.build_path(sprintf("run/%s_reverse_history.mat", MODEL_NAME_IN_DB));
    reverse_data = load(load_path);
    history_reverse = reverse_data.history;
    step_reverse = reverse_data.step;
end

% 绘图
fprintf('绘制模型...\n');
figure('Name', '模拟', 'NumberTitle', 'off');
hold on;
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
axis equal; view(3); grid on; rotate3d on;
lighting gouraud; camlight('headlight');
title('模拟');
legend('off');
xlabel('X'); ylabel('Y'); zlabel('Z');

fprintf('绘制路径...\n');

for i = 1:step
    plot3(history(i, 1), history(i, 2), history(i, 3), 'r.', 'MarkerSize', 3);
end

if env.reverse
    fprintf('绘制反向路径...\n');

    for i = 1:step_reverse
        plot3(history_reverse(i, 1), history_reverse(i, 2), history_reverse(i, 3), 'g.', 'MarkerSize', 3);
    end

end
