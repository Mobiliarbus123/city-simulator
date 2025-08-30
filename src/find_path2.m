init.init_env;

udf = utils.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));
env.boundary.x = [model_range.x(1) - 25, model_range.x(2) + 25];
env.boundary.y = [model_range.y(1) - 25, model_range.y(2) + 25];
env.boundary.z = [model_range.z(1) - 25, model_range.z(2) + 25];
env.debug = true; 
debug = @(varargin) (env.debug && fprintf(varargin{:}));

% 定义力的权重函数
env.weights.repulsion = @(d) 50000 / (d ^ 3);
env.weights.boundary = @(d) 5000 / ((d + 3) ^ 5);
env.weights.task = @(d) (d < 100) .* 5 + 5;
env.weights.ground = @(d) exp(- ((d / 100) - 1));
env.weights.up = 0.5;
env.weights.drone_repulsion = @(d) 80000 / (d ^ 4); %调

% 仿真参数
max_counted_distance = 200;
step_length = 1;
step_count=0;
ends_tolerance = 10;
drone_collision_distance = 200; 

% 在这里定义每架无人机的起点和终点
swarm_definitions = { ...
    struct('id', 1, 'start', [-400, -1400, 200], 'end', [-1900, -200, 200]); ...
    struct('id', 2, 'start', [-1900, -200, 200], 'end', [-400, -1400, 200]); ...
    %struct('id', 3, 'start', [500, 200, 200], 'end', [1800, 1400, 250]); ...
    % 您可以在此添加更多无人机
};

num_drones = length(swarm_definitions);
drones(num_drones) = utils.drone; % 预分配对象数组以提高性能

for i = 1:num_drones
    def = swarm_definitions{i};
    drones(i) = utils.drone(def.id, def.start, def.end);
end

% --- 3. 可视化设置 ---
[vertices, faces] = utils.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));
figure('Name', '机群仿真', 'NumberTitle', 'off');
hold on;
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
axis equal; view(3); grid on; rotate3d on;
lighting gouraud; camlight('headlight');
title('机群仿真'); xlabel('X'); ylabel('Y'); zlabel('Z');

% 绘制起点、终点和无人机当前位置
colors = lines(num_drones); % 为每架无人机分配不同颜色
drone_plots = gobjects(num_drones, 1);
for i = 1:num_drones
    plot3(drones(i).start_point(1), drones(i).start_point(2), drones(i).start_point(3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 8);
    plot3(drones(i).end_point(1), drones(i).end_point(2), drones(i).end_point(3), 'x', 'Color', colors(i,:), 'MarkerSize', 10, 'LineWidth', 2);
    drone_plots(i) = plot3(drones(i).loc(1), drones(i).loc(2), drones(i).loc(3), '.', 'Color', colors(i,:), 'MarkerSize', 25); % 当前位置的大点
end
legend('off');

% --- 4. 运行主仿真循环 ---
fprintf("--- 开始机群仿真 ---\n");
start_time = tic;

while true   
    % 检查是否所有无人机都已完成任务
    if all([drones.is_finished])
        fprintf("所有无人机均已到达目的地。\n");
        break;
    end
    
    % 实时更新可视化
    for i = 1:num_drones
        udf_value = udf.get_value(drones(i).loc);
        if udf_value > max_counted_distance
            F_repulsion = [0, 0, 0];
        else
            grad = udf.get_gradient(drones(i).loc);
            F_repulsion = grad * env.weights.repulsion(udf_value);
        end
        F_boundary = utils.boundary_vector(drones(i).loc, env.boundary, env.weights.boundary, 20);
        F_ground = [0, 0, env.weights.ground(drones(i).loc(3))];
        task_vec = (drones(i).end_point - drones(i).loc);
        task_dist = norm(task_vec);
        if task_dist > 0
            task_vec_norm = task_vec / task_dist;
        else
            task_vec_norm = [0,0,0];
        end
        F_task = task_vec_norm * env.weights.task(task_dist);
        F_drones = [0, 0, 0];
        for j = 1:num_drones
            if drones(j).id == drones(i).id
                continue; % 跳过自己
            end
            vec_to_other = drones(i).loc - drones(j).loc;
            dist_to_other = norm(vec_to_other);
            % 如果距离小于设定的碰撞阈值，则计算排斥力
            if dist_to_other < drone_collision_distance
                repulsion_force = (vec_to_other / dist_to_other) * env.weights.drone_repulsion(dist_to_other);
                F_drones = F_drones + repulsion_force;
                debug("无人机 %d 机间排斥力: %.2f, %.2f, %.2f\n", drones(i).id, F_drones(1), F_drones(2), F_drones(3));
            end
        end
        
        F_total = F_repulsion + F_boundary + F_ground + F_task + F_drones;
        debug("id: %d 合力:%.2f, %.2f, %.2f\n", drones(i).id, F_total(1), F_total(2), F_total(3));
        if drones(i).step > 8 && norm(drones(i).loc - drones(i).history(drones(i).step - 5, :)) < 1
            debug("无人机 %d 可能卡住了，尝试向上飞。\n", drones(i).id);
            if norm(F_total) > 0
                F_total = F_total / norm(F_total);
            end
            F_total(3) = F_total(3) + env.weights.up;
        end
        drones(i).move(F_total, step_length, ends_tolerance);
        debug("当前位置:%.2f, %.2f, %.2f\n", drones(i).loc(1), drones(i).loc(2), drones(i).loc(3));
        step_count = step_count + 1;
        plot3(drones(i).loc(1), drones(i).loc(2), drones(i).loc(3), '.', 'Color', colors(i,:), 'MarkerSize', 3);
        drawnow;
    end
end

% --- 5. 仿真结束 ---
time_elapsed = toc(start_time);
fprintf("--- 仿真结束 ---\n");
fprintf("总步数: %d\n", step_count);
fprintf("总时间: %.2f 秒\n", time_elapsed);

for i = 1:num_drones
    fprintf("无人机 %d | 是否完成: %s\n", drones(i).id, string(drones(i).is_finished));
end

