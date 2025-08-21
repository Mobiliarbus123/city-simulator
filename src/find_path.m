init.init_env;

env.debug = true;
env.reverse = true;
debug = @(varargin) (env.debug && fprintf(varargin{:}));

% % -2k
% START_POINT = [-200, -100, 50];
% END_POINT = [-1500, -1800, 50];

% 2k
START_POINT = [400, 1400, 50];
END_POINT = [1900, 200, 50];
INTERMEDIATE_POINTS = [1500, 1400, 300];

if env.reverse
    temp = START_POINT;
    START_POINT = END_POINT;
    END_POINT = temp;
    INTERMEDIATE_POINTS = flipud(INTERMEDIATE_POINTS);
end

repulsion_weight = @(distance) 50000 / (distance ^ 3);
boundary_weight = @(distance) 5000 / ((distance + 3) ^ 5);
task_weight = @(distance) (distance < 100) .* 5 + 5;
ground_weight = @(distance) exp(- ((distance / 100) - 1));
up_weight = 0.5;

max_counted_distance = 200;
step_length = 1;
ends_tolerance = 10;
max_ends_tolerance = 30;
move_speed_tolerance = 0.05;

boundary.x = [model_range.x(1) - 25, model_range.x(2) + 25];
boundary.y = [model_range.y(1) - 25, model_range.y(2) + 25];
boundary.z = [model_range.z(1) - 25, model_range.z(2) + 25];

udf = utils.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));

[vertices, faces] = utils.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));
figure('Name', '模拟', 'NumberTitle', 'off');
hold on;
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
axis equal; view(3); grid on; rotate3d on;
lighting gouraud; camlight('headlight');
title('模拟');
legend('off');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% 模拟
total_distance = 0;
progress = 0;

way_points = [START_POINT; INTERMEDIATE_POINTS; END_POINT];

for i = 1:size(way_points, 1) - 1
    total_distance = total_distance + norm(way_points(i + 1, :) - way_points(i, :));
end

loc = way_points(1, :);
% 预分配
% TODO: 自动扩增
history_length = floor(total_distance / step_length * 1.5);
history = zeros(history_length, 3);
capacity_step = floor(total_distance / step_length * 0.5);

fprintf("--- 开始模拟 ---\n");
fprintf("总距离：%.2f\n", total_distance);
start_time = tic;

i = 0;

check_stop_interval = floor(1 / move_speed_tolerance);
is_succeed = @(loc, target) abs(norm(loc - target)) < max_ends_tolerance;

bar = waitbar(0, '正在导航');

for n = 1:size(way_points, 1) - 1

    target = way_points(n + 1, :);
    current_distance = norm(way_points(n + 1, :) - way_points(n, :));

    while true

        % 结束条件
        if abs(norm(loc - target)) < ends_tolerance
            break;
        end

        if i > (check_stop_interval + 5) && (abs(norm(loc - history(i - check_stop_interval, :))) < 1)
            break;
        end

        % 寻路
        i = i + 1;
        % 计算建筑物排斥力
        udf_value = udf.get_value(loc);
        debug("建筑物距离: %.2f\n", udf_value);

        if udf_value > max_counted_distance
            F_repulsion = [0, 0, 0];
        else
            grad = udf.get_gradient(loc);
            F_repulsion = grad * repulsion_weight(udf_value);
        end

        % 计算边界排斥力
        F_boundary = utils.boundary_vector(loc, boundary, boundary_weight, 20);
        debug("边界排斥力: %.2f, %.2f, %.2f\n", F_boundary(1), F_boundary(2), F_boundary(3));

        % 计算地面排斥力
        F_ground = [0, 0, ground_weight(loc(3))];
        debug("地面排斥力: %.2f, %.2f, %.2f\n", F_ground(1), F_ground(2), F_ground(3));

        % 计算目标吸引力
        task_vec = (target - loc);
        task_vec = task_vec / norm(task_vec);
        F_task = task_vec * task_weight(abs(norm(target - loc)));

        % 合成总力
        debug("F_repulsion: %.2f, %.2f, %.2f, F_boundary: %.2f, %.2f, %.2f, F_task: %.2f, %.2f, %.2f\n", F_repulsion(1), F_repulsion(2), F_repulsion(3), F_boundary(1), F_boundary(2), F_boundary(3), F_task(1), F_task(2), F_task(3));
        F_total = F_repulsion + F_boundary + F_ground + F_task;
        debug("F_total: %.2f, %.2f, %.2f, norm: %.2f\n", F_total(1), F_total(2), F_total(3), norm(F_total));

        if i > 3
            debug("上上次水平移动距离: %.2f\n", norm(loc(2:3) - history(i - 3, 2:3)));
        end

        % 没思路就往上飞一点
        if abs(norm(F_task - F_total) / norm(F_task)) > 0.3 ...
                && (loc(3) - target(3)) / norm(loc - target) < 0.7 ...
                && abs(F_task(3) / norm(F_task)) < 0.8 ...
                && ((abs(norm(F_total)) < 0.2) ...
                || abs(norm(dot(F_repulsion(2:3) / norm(F_repulsion(2:3)), F_task(2:3) / norm(F_task(2:3))))) > 0.5 ...
                || (i > 3 && norm(loc(2:3) - history(i - 3, 2:3)) < 0.3) ...
                || (norm(F_boundary) + norm(F_repulsion)) / norm(F_repulsion + F_boundary) < 0.5 ...
                || (i > 8 && norm(loc - history(i - 5, :)) < 1))
            debug("往上飞一点\n");
            F_total = F_total / norm(F_total);
            F_total(3) = up_weight;
            F_total = F_total / norm(F_total);
        end

        F_total = F_total / norm(F_total);

        % 更新位置
        loc = loc + F_total * step_length;

        % 记录历史位置
        debug("总力: %.2f, %.2f, %.2f  当前位置：%.2f, %.2f, %.2f\n", F_total(1), F_total(2), F_total(3), loc(1), loc(2), loc(3));
        debug("\n");

        if i > history_length
            history(history_length + 1:history_length + capacity_step, :) = 0;
            history_length = history_length + capacity_step;
        end

        history(i, :) = loc;
        waitbar((current_distance - abs(norm(loc - target)) + progress) / total_distance, bar, sprintf('正在导航: %d 步', i));
        plot3(loc(1), loc(2), loc(3), 'r.', 'MarkerSize', 3);
    end

    progress = progress + current_distance;

    if ~is_succeed(loc, target)
        break;
    end

end

close(bar);

time_elapsed = toc(start_time);
fprintf("--- 模拟结束 ---\n");
fprintf("总步数：%d\n", i);
fprintf("总时间：%.2f秒\n", time_elapsed);
fprintf("平均速度：%.2f单位/秒\n", total_distance / time_elapsed);
fprintf("最终位置：%.2f, %.2f, %.2f\n", loc(1), loc(2), loc(3));
fprintf("是否成功到达终点：%s\n", string(is_succeed(loc, END_POINT)));
fprintf('--------------------\n\n');

% 储存过程
step = i;

if env.reverse
    save_path = init.build_path(sprintf("run/%s_reverse_history.mat", MODEL_NAME_IN_DB));
else
    save_path = init.build_path(sprintf("run/%s_history.mat", MODEL_NAME_IN_DB));
end

save(save_path, 'history', 'step', '-v7.3');
