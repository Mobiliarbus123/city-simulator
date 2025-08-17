init.init_env;

env.debug = true;
env.reverse = false;
debug = @(varargin) (env.debug && fprintf(varargin{:}));

% START_POINT = [1100, 1200, 50];
% % END_POINT = [1650, 1025, 50];
% END_POINT = [1320, 840, 100];
% START_POINT = [1700, 1000, 50];
% END_POINT = [1800, 1550, 70];
START_POINT = [1100, 1200, 90];
END_POINT = [1675, 1050, 160];
% START_POINT = [400, 1200, 50];
% END_POINT = [600, 1700, 280];
% END_POINT = [400, 600, 80];

% START_POINT = [700, 1200, 50];
% END_POINT = [1420, 1580, 80];

if env.reverse
    temp = START_POINT;
    START_POINT = END_POINT;
    END_POINT = temp;
end

% 建筑物排斥力强度
% repulsion_weight = @(distance) 38000 / (distance ^ 3);  % 原50000
repulsion_weight = @(distance) ...
    (distance < 20) .* (48000 / (distance ^ 3 + 1)) + ...  % 极近距离强排斥
    (distance >= 20 & distance < 100) .* (32000 / (distance ^ 3)) + ...
    (distance >= 100) .* (15000 / (distance ^ 3));

% 边界排斥权重
boundary_weight = @(distance) 3000 / ((distance + 3) ^ 5);

% 目标吸引
task_weight = @(distance) (distance < 100) .* 5 + 6 + (distance > 300) .* (distance / 60); 
ground_weight = @(distance) exp(- ((distance / 100) - 1));
up_weight = 0.15;  % 原0.3

max_counted_distance = 180;  % 原为200
step_length = 1;
ends_tolerance = 15;
max_ends_tolerance = 30;
move_speed_tolerance = 0.04;

% 边界设置
boundary.x = [model_range.x(1) - 25, model_range.x(2) + 25];
boundary.y = [model_range.y(1) - 25, model_range.y(2) + 25];
boundary.z = [model_range.z(1) + 10, min(model_range.z(2), 300) + 5];

% 加载模型和UDF
udf = utils.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));
total_distance = norm(END_POINT - START_POINT);

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
loc = START_POINT;
history_length = floor(total_distance / step_length * 1.5);
history = zeros(history_length, 3);
capacity_step = floor(total_distance / step_length * 0.5);

fprintf("--- 开始模拟 ---\n");
fprintf("总距离：%.2f\n", total_distance);
start_time = tic;

i = 0;

check_stop_interval = floor(1 / move_speed_tolerance);
shouldStop = @(i, loc, history) ...
    abs(norm(loc - END_POINT)) < ends_tolerance ...
    || i > (check_stop_interval + 5) && (abs(norm(loc - history(i - check_stop_interval, :))) < 1);
is_succeed = @(loc) abs(norm(loc - END_POINT)) < max_ends_tolerance;

f = waitbar(0, '正在导航');

while ~shouldStop(i, loc, history)
    i = i + 1;
    % 计算建筑物排斥力
    udf_value = udf.get_value(loc);
    debug("建筑物距离: %.2f\n", udf_value);

    if udf_value > max_counted_distance
        F_repulsion = [0, 0, 0];
    else
        grad = utils.gradient(udf, loc);
        F_repulsion = grad * repulsion_weight(udf_value);
    end

    % 计算边界排斥力
    F_boundary = utils.boundary_vector(loc, boundary, boundary_weight, 20);
    debug("边界排斥力: %.2f, %.2f, %.2f\n", F_boundary(1), F_boundary(2), F_boundary(3));

    % 地面排斥力，添加高度限制修正力
    F_ground = [0, 0, ground_weight(loc(3))];
    % 新增：高度过高时添加向下修正力（高度>220开始生效）
    if loc(3) > 220
        over_height = loc(3) - 220;
        down_force = 0.1 + (over_height / 60);  % 高度越高，向下力越大
        % down_force = 0.1 + (over_height / 80);
        F_ground(3) = F_ground(3) - down_force;  % 抵消向上力，添加向下力
    end
    debug("地面排斥力(含高度修正): %.2f, %.2f, %.2f\n", F_ground(1), F_ground(2), F_ground(3));

    % 计算目标吸引力
    task_vec = (END_POINT - loc);
    task_vec = task_vec / norm(task_vec);
    F_task = task_vec * task_weight(abs(norm(END_POINT - loc)));

    % 合成总力
    F_total = F_repulsion + F_boundary + F_ground + F_task;
    debug("F_total: %.2f, %.2f, %.2f, norm: %.2f\n", F_total(1), F_total(2), F_total(3), norm(F_total));


    % 局部最小值处理逻辑
    % 收紧触发条件（减少无效向上飞），并添加高度约束
    height_constraint = loc(3) < 280;  % 仅允许在高度<280时向上飞
    if height_constraint && ...
            abs(norm(F_task - F_total) / norm(F_task)) > 0.4 ...
            && (loc(3) - END_POINT(3)) / norm(loc - END_POINT) < 0.7 ...
            && abs(F_task(3) / norm(F_task)) < 0.8 ...
            && ((abs(norm(F_total)) < 0.2) ...
            || abs(norm(dot(F_repulsion(2:3) / norm(F_repulsion(2:3)), F_task(2:3) / norm(F_task(2:3))))) > 0.5 ...
            || (i > 3 && norm(loc(2:3) - history(i - 3, 2:3)) < 0.3) ...
            || (norm(F_boundary) + norm(F_repulsion)) / norm(F_repulsion + F_boundary) < 0.5 ...
            || (i > 8 && norm(loc - history(i - 5, :)) < 1))
        debug("受限向上飞（高度<280）\n");
        F_total = F_total / norm(F_total);
        F_total(3) = up_weight;  % 向上力权重
        F_total = F_total + 0.1*task_vec; %先加上试试
        F_total = F_total / norm(F_total);
    end

    % 归一化总力，更新位置
    F_total = F_total / norm(F_total);
    loc = loc + F_total * step_length;

    % 强制限制高度不超过300（最后一道保险）
    if loc(3) > 300
        loc(3) = 300;
        debug("高度超限，强制限制为300\n");
    end


    if i > history_length
        history(history_length + 1:history_length + capacity_step, :) = 0;
        history_length = history_length + capacity_step;
    end
    history(i, :) = loc;
    waitbar(1 - abs(norm(loc - END_POINT)) / total_distance, f, sprintf('正在导航: %d 步', i));
    plot3(loc(1), loc(2), loc(3), 'r.', 'MarkerSize', 3);
end

% 后续结果输出部分保持不变
close(f);
time_elapsed = toc(start_time);
fprintf("--- 模拟结束 ---\n");
fprintf("总步数：%d\n", i);
fprintf("总时间：%.2f秒\n", time_elapsed);
fprintf("平均速度：%.2f单位/秒\n", total_distance / time_elapsed);
fprintf("最终位置：%.2f, %.2f, %.2f\n", loc(1), loc(2), loc(3));
fprintf("是否成功到达终点：%s\n", string(is_succeed(loc)));
fprintf('--------------------\n\n');

step = i;
if env.reverse
    save_path = init.build_path(sprintf("run/%s_reverse_history.mat", MODEL_NAME_IN_DB));
else
    save_path = init.build_path(sprintf("run/%s_history.mat", MODEL_NAME_IN_DB));
end
save(save_path, 'history', 'step', '-v7.3');