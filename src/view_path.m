init.init_env;

env.reverse = false;
env = utils.Ref(env);
tag = "multi_2x";

% 加载 STL 模型
fprintf('加载 STL 模型...\n');
[vertices, faces] = utils.model.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.model.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% 加载路径
fprintf('加载路径...\n');
load_path = init.build_path(sprintf("run/%s_%s_history.mat", MODEL_NAME_IN_DB, tag));
load(load_path, "swarm");

% 加载反向路径
if env.value.reverse
    fprintf('加载反向路径...\n');
    load_path = init.build_path(sprintf("run/%s_%s_reverse_history.mat", MODEL_NAME_IN_DB, tag));
    reverse_data = load(load_path);
    swarm_reverse = reverse_data.swarm;
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

for i = 1:length(swarm.drones)
    drone = swarm.drones(i);
    history = drone.history;
    step = size(history, 1);

    for j = 1:step
        plot3(history(j, 1), history(j, 2), history(j, 3), 'r.', 'MarkerSize', 3);
    end

end

if env.value.reverse
    fprintf('绘制反向路径...\n');

    for i = 1:length(swarm_reverse.drones)
        drone = swarm_reverse.drones(i);
        history = drone.history;
        step = size(history, 1);

        for j = 1:step
            plot3(history(j, 1), history(j, 2), history(j, 3), 'g.', 'MarkerSize', 3);
        end

    end

end
