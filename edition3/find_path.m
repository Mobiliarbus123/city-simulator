init.init_env;

env.debug = true;
env.reverse = false;
debug = @(varargin) (env.debug && fprintf(varargin{:}));

% 起点和终点设置
START_POINT = [1100, 1200, 70];
END_POINT = [1675, 1050, 80];

% 预定义中转点（可自己手动添加）
PREDEFINED_WAYPOINTS = [
    % 1350, 1200, 150;
    % 1435, 1125, 200;
];

if env.reverse
    temp = START_POINT;
    START_POINT = END_POINT;
    END_POINT = temp;
end


% 力场参数设置（包含优化）
% 建筑物排斥力权重（分段函数）
repulsion_weight = @(distance) ...
    (distance < 20) .* (48000 / (distance ^ 3 + 1)) + ...  % 极近距离强排斥
    (distance >= 20 & distance < 100) .* (32000 / (distance ^ 3)) + ...
    (distance >= 100) .* (15000 / (distance ^ 3));

% 边界排斥权重
boundary_weight = @(distance) 3000 / ((distance + 3) ^ 5);

% 目标吸引力权重（动态调整）
task_weight = @(distance) (distance < 100) .* 5 + 6 + (distance > 300) .* (distance / 60); 

% 地面排斥力及高度约束
ground_weight = @(distance) exp(- ((distance / 100) - 1));
up_weight = 0.15;  % 向上力权重

% 其他参数
max_counted_distance = 180;  % 排斥力最大作用距离
step_length = 1;
ends_tolerance = 15;
max_ends_tolerance = 30;
move_speed_tolerance = 0.05;

% 边界设置
boundary.x = [model_range.x(1) - 25, model_range.x(2) + 25];
boundary.y = [model_range.y(1) - 25, model_range.y(2) + 25];
boundary.z = [model_range.z(1) - 15, min(model_range.z(2), 300) + 5];

% 中转点参数
waypoint_udf_threshold = 35;  % 中转点最小障碍物距离
waypoint_max_height = 200;    % 中转点最大高度
num_waypoint_candidates = 5;  % 每次生成的候选中转点数量
max_stuck_steps = 20;         % 判定为停滞的最大步数
max_waypoint_distance = 300;  % 中转点之间的最大距离
waypoint_tolerance = 10;      % 到达中转点的容忍距离

% 加载模型和UDF
udf = utils.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));
total_distance = norm(END_POINT - START_POINT);

% 可视化初始化
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

%% 中转点生成函数
function waypoints = generate_waypoints(current_loc, target_loc, udf, num_candidates)
    waypoints = [];
    % 沿直线方向采样候选点
    step = (target_loc - current_loc) / (num_candidates + 1);
    for k = 1:num_candidates
        candidate = current_loc + step * k;
        % 检查高度约束（<200）
        if candidate(3) >= waypoint_max_height
            candidate(3) = waypoint_max_height - 10;  % 强制调整高度
        end
        % 检查障碍物距离（udf_value > 35）
        udf_val = udf.get_value(candidate);
        if ~isnan(udf_val) && udf_val > waypoint_udf_threshold
            waypoints = [waypoints; candidate];  % 符合条件则加入中转点列表
        end
    end
    % 如果无候选点，尝试在垂直方向微调后再采样
    if isempty(waypoints)
        for k = 1:num_candidates
            candidate = current_loc + step * k;
            candidate(3) = max(50, min(waypoint_max_height - 10, candidate(3) + 30 * (-1)^k));  % 上下微调
            udf_val = udf.get_value(candidate);
            if ~isnan(udf_val) && udf_val > waypoint_udf_threshold
                waypoints = [waypoints; candidate];
            end
        end
    end
end

%% 模拟主逻辑
loc = START_POINT;
history_length = floor(total_distance / step_length * 1.5);
history = zeros(history_length, 3);
capacity_step = floor(total_distance / step_length * 0.5);

% 中转点管理
waypoints = PREDEFINED_WAYPOINTS;
current_target = END_POINT;  % 初始目标为终点
stuck_counter = 0;          % 停滞计数器
prev_distance = norm(loc - current_target);

fprintf("--- 开始模拟 ---\n");
fprintf("总距离：%.2f\n", total_distance);
start_time = tic;

i = 0;
f = waitbar(0, '正在导航');

% 停止条件函数
shouldStop = @(i, loc, history) ...
    abs(norm(loc - current_target)) < waypoint_tolerance ...
    || i > (check_stop_interval + 5) && (abs(norm(loc - history(i - check_stop_interval, :))) < 1);
is_succeed = @(loc) abs(norm(loc - END_POINT)) < max_ends_tolerance;
check_stop_interval = floor(1 / move_speed_tolerance);

while true
    % 检查是否到达最终终点
    if abs(norm(loc - END_POINT)) < ends_tolerance
        break;
    end
    
    % 检查是否到达当前目标（中转点或终点）
    if abs(norm(loc - current_target)) < waypoint_tolerance
        if isequal(current_target, END_POINT)
            break;  % 到达最终终点
        else
            % 切换到下一个目标
            if ~isempty(waypoints)
                current_target = waypoints(1, :);
                waypoints = waypoints(2:end, :);
                debug("切换到中转点: (%.2f, %.2f, %.2f)\n", current_target(1), current_target(2), current_target(3));
                plot3(current_target(1), current_target(2), current_target(3), 'go', 'MarkerSize', 8);  % 标记中转点
            else
                current_target = END_POINT;  % 无中转点，直接指向终点
            end
            prev_distance = norm(loc - current_target);
            stuck_counter = 0;
        end
    end

    % 检测是否陷入局部最小值
    current_distance = norm(loc - current_target);
    if abs(current_distance - prev_distance) < move_speed_tolerance
        stuck_counter = stuck_counter + 1;
    else
        stuck_counter = 0;
    end
    prev_distance = current_distance;

    % 触发中转点生成：停滞或绕路严重
    if stuck_counter > max_stuck_steps || (current_distance > max_waypoint_distance && ...
            norm(loc - START_POINT) > 1.5 * norm(START_POINT - current_target))
        debug("触发中转点生成...\n");
        new_waypoints = generate_waypoints(loc, current_target, udf, num_waypoint_candidates);
        if ~isempty(new_waypoints)
            % 选择最优中转点（距离当前位置近且朝向目标）
            [~, idx] = min(vecnorm(new_waypoints - loc, 2, 2) + ...
                0.3 * vecnorm(new_waypoints - current_target, 2, 2));
            waypoints = [new_waypoints(idx, :); waypoints];  % 插入中转点列表
            debug("新增中转点: (%.2f, %.2f, %.2f)\n", waypoints(1,1), waypoints(1,2), waypoints(1,3));
            plot3(waypoints(1,1), waypoints(1,2), waypoints(1,3), 'yo', 'MarkerSize', 6);  % 标记候选中转点
            stuck_counter = 0;
        end
    end

    % 计算力场
    % 1. 建筑物排斥力
    udf_value = udf.get_value(loc);
    debug("建筑物距离: %.2f\n", udf_value);
    if udf_value > max_counted_distance
        F_repulsion = [0, 0, 0];
    else
        grad = utils.gradient(udf, loc);
        F_repulsion = grad * repulsion_weight(udf_value);
    end

    % 2. 边界排斥力
    F_boundary = utils.boundary_vector(loc, boundary, boundary_weight, 20);
    debug("边界排斥力: %.2f, %.2f, %.2f\n", F_boundary(1), F_boundary(2), F_boundary(3));

    % 3. 地面排斥力（含高度修正）
    F_ground = [0, 0, ground_weight(loc(3))];
    if loc(3) > 220
        over_height = loc(3) - 220;
        down_force = 0.1 + (over_height / 80);
        F_ground(3) = F_ground(3) - down_force;
    end
    debug("地面排斥力: %.2f, %.2f, %.2f\n", F_ground(1), F_ground(2), F_ground(3));

    % 4. 目标吸引力（指向当前目标）
    task_vec = (current_target - loc);
    if norm(task_vec) > 1e-6
        task_vec = task_vec / norm(task_vec);
    end
    F_task = task_vec * task_weight(abs(norm(current_target - loc)));

    % 合成总力
    F_total = F_repulsion + F_boundary + F_ground + F_task;
    debug("总力: %.2f, %.2f, %.2f, 模长: %.2f\n", F_total(1), F_total(2), F_total(3), norm(F_total));

    % 局部最小值处理
    height_constraint = loc(3) < 270;
    if height_constraint && ...
            abs(norm(F_task - F_total) / norm(F_task)) > 0.4 ...
            && (loc(3) - current_target(3)) / norm(loc - current_target) < 0.7 ...
            && abs(F_task(3) / norm(F_task)) < 0.8 ...
            && ((abs(norm(F_total)) < 0.2) ...
            || abs(norm(dot(F_repulsion(2:3) / norm(F_repulsion(2:3)), F_task(2:3) / norm(F_task(2:3))))) > 0.5 ...
            || (i > 3 && norm(loc(2:3) - history(i - 3, 2:3)) < 0.3) ...
            || (norm(F_boundary) + norm(F_repulsion)) / norm(F_repulsion + F_boundary) < 0.5 ...
            || (i > 8 && norm(loc - history(i - 5, :)) < 1))
        debug("触发向上力脱离局部最小值\n");
        F_total = F_total / norm(F_total);
        F_total(3) = up_weight;
        F_total = F_total / norm(F_total);
    end

    % 更新位置
    F_total = F_total / norm(F_total);
    loc = loc + F_total * step_length;

    % 高度限制
    if loc(3) > 300
        loc(3) = 300;
        debug("高度超限，强制限制为300\n");
    end

    % 记录路径
    i = i + 1;
    if i > history_length
        history(history_length + 1:history_length + capacity_step, :) = 0;
        history_length = history_length + capacity_step;
    end
    history(i, :) = loc;
    waitbar(1 - abs(norm(loc - END_POINT)) / total_distance, f, sprintf('正在导航: %d 步', i));
    plot3(loc(1), loc(2), loc(3), 'r.', 'MarkerSize', 3);
end

% 结果输出
close(f);
time_elapsed = toc(start_time);
fprintf("--- 模拟结束 ---\n");
fprintf("总步数：%d\n", i);
fprintf("总时间：%.2f秒\n", time_elapsed);
fprintf("平均速度：%.2f单位/秒\n", total_distance / time_elapsed);
fprintf("最终位置：%.2f, %.2f, %.2f\n", loc(1), loc(2), loc(3));
fprintf("是否成功到达终点：%s\n", string(is_succeed(loc)));

% 保存路径
step = i;
if env.reverse
    save_path = init.build_path(sprintf("run/%s_reverse_history.mat", MODEL_NAME_IN_DB));
else
    save_path = init.build_path(sprintf("run/%s_history.mat", MODEL_NAME_IN_DB));
end
save(save_path, 'history', 'step', '-v7.3');