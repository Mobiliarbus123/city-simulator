init.init_env;

% 环境配置
env.debug = false;
env.reverse = false;
debug = @(varargin) (env.debug && fprintf(varargin{:}));

% 起点和终点设置
% START_POINT = [1100, 1200, 50];
% % END_POINT = [1650, 1025, 50];
% END_POINT = [1320, 840, 100];
% START_POINT = [1700, 1000, 50];
% END_POINT = [1800, 1550, 70];
START_POINT = [1100, 1200, 90];
END_POINT = [1625, 1050, 160];
% START_POINT = [400, 1200, 50];
% END_POINT = [600, 1700, 280];
% END_POINT = [400, 600, 80];

% START_POINT = [700, 1200, 50];
% END_POINT = [1420, 1580, 80];

% 处理反向路径
if env.reverse
    temp = START_POINT;
    START_POINT = END_POINT;
    END_POINT = temp;
end

% 核心参数优化 - 平衡平滑性与避障
MAX_HEIGHT = 300;        % 最大飞行高度限制
MIN_HEIGHT = 35;         % 最小飞行高度限制
SAFE_DISTANCE = 20;      % 障碍物安全距离
BASE_STEP = 2;           % 基础步长
MIN_STEP = 1;            % 最小步长
MAX_STEP = 5;            % 最大步长
GOAL_TOLERANCE = 18;      % 到达终点的容差
MAX_ITERATIONS = 8000;   % 增加最大迭代次数，降低提前终止概率
REPLAN_THRESHOLD = 4000; % 重新规划阈值

% 改进势场函数
% repulsion_weight = @(distance) ...
%     (distance < SAFE_DISTANCE*0.5) .* (100000 ./ (distance.^3 + 1e-6)) + ...  % 极近时强排斥
%     (distance >= SAFE_DISTANCE*0.5 & distance < SAFE_DISTANCE) .* (30000 ./ (distance.^3 + 1e-6)) + ...
%     (distance >= SAFE_DISTANCE & distance < SAFE_DISTANCE*1.5) .* (5000 ./ (distance.^2 + 1e-6)) + ...
%     (distance >= SAFE_DISTANCE*1.5 & distance < SAFE_DISTANCE*2) .* (1000 ./ (distance + 1e-6)) + ...
%     (distance >= SAFE_DISTANCE*2) .* 0;

repulsion_weight = @(distance) ...
    (distance < 20) .* (48000 / (distance ^ 3 + 1)) + ...  % 极近距离强排斥
    (distance >= 20 & distance < 100) .* (32000 / (distance ^ 3)) + ...
    (distance >= 100) .* (12000 / (distance ^ 3));

% 目标吸引力
% task_weight = @(distance) ...
%     (distance > 300) .* 28 + ...          % 增加远距离吸引力，提高寻路效率
%     (distance <= 300 & distance > 100) .* (20 - 5*(300-distance)/200) + ...
%     (distance <= 100 & distance > 20) .* (15 - 5*(100-distance)/80) + ...
%     (distance <= 20) .* 9;               % 接近目标时适当增强吸引力
task_weight = @(distance) (distance < 100) .* 16 + 12 + (distance > 300) .* (distance / 8); 

% 高度约束势场
height_upper_weight = @(distance) 50000 ./ ((distance + 1) .^5);  % 接近MAX_HEIGHT时的排斥力
height_lower_weight = @(distance) 30000 ./ ((distance + 1) .^5);  % 接近MIN_HEIGHT时的排斥力

% 环境边界
boundary.x = [model_range.x(1) - 25, model_range.x(2) + 25];
boundary.y = [model_range.y(1) - 25, model_range.y(2) + 25];
boundary.z = [MIN_HEIGHT + 3, MAX_HEIGHT - 5];

% 边界排斥力 - 提前预警
% boundary_weight = @(distance) 15000 ./ ((distance + 1) .^5);
boundary_weight = @(distance) 3000 / ((distance + 3) ^ 5);

% 加载UDF和模型
udf = utils.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));
[vertices, faces] = utils.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));

% 可视化设置
figure('Name', '优化路径规划', 'NumberTitle', 'off');
hold on;
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot3(START_POINT(1), START_POINT(2), START_POINT(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(END_POINT(1), END_POINT(2), END_POINT(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
axis equal; view(3); grid on; rotate3d on;
lighting gouraud; camlight('headlight');
title('优化路径规划');
xlabel('X'); ylabel('Y'); zlabel('Z');
zlim([MIN_HEIGHT-50, MAX_HEIGHT+50]);

% 初始化
loc = START_POINT;
total_distance = norm(END_POINT - START_POINT);
history = zeros(MAX_ITERATIONS, 3);  % 路径历史，大小与最大迭代次数匹配
i = 0;
prev_locations = zeros(15, 3);       % 存储最近位置用于检测局部最优和计算平滑方向
prev_directions = zeros(5, 3);       % 存储最近方向用于平滑处理
closest_dist = total_distance;       % 记录到目标的最近距离
closest_point = loc;                 % 记录最近点位置

% 局部最优检测与逃离参数
stuck_counter = 0;
STUCK_THRESHOLD = 20;                % 多少步几乎不动判定为陷入局部最优
escape_strategy = 0;                 % 0:正常, 1:向上逃离, 2:向右逃离, 3:向左逃离
escape_counter = 0;
MAX_ESCAPE_STEPS = 25;               % 增加逃离策略持续步数

% 开始路径规划
fprintf("--- 开始路径规划 ---\n");
start_time = tic;
f = waitbar(0, '正在规划路径');

while true
    i = i + 1;
    if i > MAX_ITERATIONS
        warning('达到最大迭代次数，可能未找到完整路径');
        break;
    end
    
    % 检查是否到达终点
    current_dist_to_goal = norm(loc - END_POINT);
    if current_dist_to_goal < GOAL_TOLERANCE
        fprintf("已到达终点附近\n");
        break;
    end
    
    % 记录到目标的最近距离和位置
    if current_dist_to_goal < closest_dist
        closest_dist = current_dist_to_goal;
        closest_point = loc;
    end
    
    % 重新规划策略：如果长时间没有进展，尝试直接飞向最近点
    if i > REPLAN_THRESHOLD && mod(i, 500) == 0
        fprintf("长时间未找到路径，尝试重新规划...\n");
        loc = closest_point;  % 回到曾经到达的最近点
        stuck_counter = 0;
        escape_strategy = 0;
    end
    
    % 保存最近位置和方向用于平滑处理
    prev_idx = mod(i-1, 15) + 1;
    prev_locations(prev_idx, :) = loc;
    dir_idx = mod(i-1, 5) + 1;
    
    % 计算障碍物距离和排斥力
    udf_value = udf.get_value(loc);
    debug("障碍物距离: %.2f\n", udf_value);
    
    % 增强避障：提前预警并减速
    if udf_value < SAFE_DISTANCE*1.5
        % 距离障碍物较近，计算排斥力
        grad = utils.gradient(udf, loc);
        F_repulsion = grad * repulsion_weight(udf_value);
        
        % 极近距离时强制向上规避
        if udf_value < SAFE_DISTANCE*0.7 && loc(3) < MAX_HEIGHT - 30
            F_repulsion(3) = abs(F_repulsion(3)) + 25;  % 更强的向上力
        end
    else
        F_repulsion = [0, 0, 0];
    end
    
    % 计算边界排斥力
    F_boundary = utils.boundary_vector(loc, boundary, boundary_weight, 50);
    debug("边界排斥力: %.2f, %.2f, %.2f\n", F_boundary(1), F_boundary(2), F_boundary(3));
    
    % 计算高度约束势场
    dist_to_upper = MAX_HEIGHT - loc(3);
    if dist_to_upper < 40  % 更早开始约束
        F_upper = [0, 0, -height_upper_weight(40 - dist_to_upper)];
    else
        F_upper = [0, 0, 0];
    end
    
    dist_to_lower = loc(3) - MIN_HEIGHT;
    if dist_to_lower < 40  % 更早开始约束
        F_lower = [0, 0, height_lower_weight(40 - dist_to_lower)];
    else
        F_lower = [0, 0, 0];
    end
    F_height = F_upper + F_lower;
    
    % 计算目标吸引力 - 更强的导向性
    task_vec = (END_POINT - loc);
    task_dist = norm(task_vec);
    if task_dist > 0
        task_vec_normalized = task_vec / task_dist;
        F_task = task_vec_normalized * task_weight(task_dist);
    else
        F_task = [0, 0, 0];
    end
    
    % 检测局部最优
    local_stuck = false;
    if i > 10
        recent_movement = norm(loc - prev_locations(prev_idx, :));
        if recent_movement < 0.8
            stuck_counter = stuck_counter + 1;
            if stuck_counter >= STUCK_THRESHOLD
                debug("检测到局部最优，启动逃离策略\n");
                escape_strategy = mod(escape_strategy + 1, 4) + 1;  % 增加一种逃离策略
                stuck_counter = 0;
                escape_counter = 0;
                local_stuck = true;
            end
        else
            stuck_counter = 0;
        end
    end
    
    % 应用逃离策略 - 增加一种斜向逃离策略
    F_escape = [0, 0, 0];
    if escape_strategy > 0 && escape_counter < MAX_ESCAPE_STEPS
        if escape_strategy == 1 && loc(3) < MAX_HEIGHT - 50
            F_escape = [0, 0, 8];  % 向上逃离，增强力度
        elseif escape_strategy == 2
            F_escape = [8, 0, 6]; % 向右上逃离
        elseif escape_strategy == 3
            F_escape = [-8, 0, 6];% 向左上逃离
        elseif escape_strategy == 4  % 斜向逃离
            F_escape = [5, 5, 5]; % 向右前上逃离
        end
        escape_counter = escape_counter + 1;
    else
        escape_strategy = 0;
    end
    
    % 合成总力
    F_total = F_repulsion + F_boundary + F_height + F_task + F_escape;
    debug("总力: %.2f, %.2f, %.2f\n", F_total(1), F_total(2), F_total(3));
    
    % 规范化总力方向
    if norm(F_total) > 1e-6
        F_total = F_total / norm(F_total);
    else
        F_total = rand(1,3) - 0.5;
        F_total = F_total / norm(F_total);
        F_total(3) = abs(F_total(3));  % 优先向上
    end
    
    % 方向平滑处理 - 基于历史方向的加权平均，减少波动
    if i > 1
        prev_directions(dir_idx, :) = F_total;
        if i >= 5
            % 对最近5个方向进行加权平均，增强平滑性
            weights = [0.1, 0.2, 0.3, 0.4, 0.5];  % 近期方向权重更高
            weights = weights / sum(weights);
            smoothed_dir = sum(prev_directions .* weights', 1);
            smoothed_dir = smoothed_dir / norm(smoothed_dir);
            
            % 只有在非局部最优时才平滑方向
            if ~local_stuck
                F_total = 0.7*smoothed_dir + 0.3*F_total;  % 混合平滑方向和当前方向
                F_total = F_total / norm(F_total);
            end
        end
    end
    
    % 动态调整步长：平衡效率与安全性
    if udf_value < SAFE_DISTANCE*1.2
        % 接近障碍物时减速
        current_step = MIN_STEP + (SAFE_DISTANCE*1.2 - udf_value)/(SAFE_DISTANCE*1.2)*(BASE_STEP - MIN_STEP);
    elseif current_dist_to_goal < 100
        % 接近目标时减速，提高精度
        current_step = MIN_STEP + (current_dist_to_goal/100)*(BASE_STEP - MIN_STEP);
    else
        % 正常情况，根据距离目标远近调整步长
        current_step = BASE_STEP + min((total_distance - current_dist_to_goal)/total_distance, 1)*(MAX_STEP - BASE_STEP);
    end
    
    % 预测新位置并进行碰撞检查
    predicted_loc = loc + F_total * current_step;
    predicted_loc(3) = max(MIN_HEIGHT, min(predicted_loc(3), MAX_HEIGHT));  % 强制高度约束
    
    % 增强避障：路径碰撞预检测与修正
    collision_avoided = false;
    if udf.get_value(predicted_loc) < SAFE_DISTANCE
        debug("检测到潜在碰撞，调整路径\n");
        
        % 尝试向上调整
        adjusted_loc = predicted_loc;
        for adjust = 5:5:50  % 逐步提升高度
            adjusted_loc(3) = min(adjusted_loc(3) + adjust, MAX_HEIGHT);
            if udf.get_value(adjusted_loc) >= SAFE_DISTANCE
                predicted_loc = adjusted_loc;
                collision_avoided = true;
                break;
            end
        end
        
        % 向上调整失败，尝试横向调整
        if ~collision_avoided
            lateral_dirs = [
                [F_total(2), -F_total(1), 0];  % 右向垂直
                [-F_total(2), F_total(1), 0];  % 左向垂直
                [F_total(2), -F_total(1), 0.3];% 右向斜上
                [-F_total(2), F_total(1), 0.3] % 左向斜上
            ];
            
            for dir = 1:size(lateral_dirs, 1)
                lateral_dir = lateral_dirs(dir, :);
                lateral_dir = lateral_dir / norm(lateral_dir);
                
                for adjust = 5:5:30
                    adjusted_loc = predicted_loc + lateral_dir * adjust;
                    adjusted_loc(3) = max(MIN_HEIGHT, min(adjusted_loc(3), MAX_HEIGHT));
                    if udf.get_value(adjusted_loc) >= SAFE_DISTANCE
                        predicted_loc = adjusted_loc;
                        collision_avoided = true;
                        break;
                    end
                end
                if collision_avoided
                    break;
                end
            end
        end
        
        % 所有调整都失败，大幅减速并谨慎移动
        if ~collision_avoided
            current_step = MIN_STEP;
            predicted_loc = loc + F_total * current_step;
            predicted_loc(3) = max(MIN_HEIGHT, min(predicted_loc(3), MAX_HEIGHT));
        end
    end
    
    % 更新位置
    loc = predicted_loc;
    
    % 记录历史并可视化
    history(i, :) = loc;
    plot3(loc(1), loc(2), loc(3), 'r.', 'MarkerSize', 5);
    
    % 更新进度条
    progress = 1 - min(current_dist_to_goal / total_distance, 1);
    waitbar(progress, f, sprintf('路径规划中: 第%d步, 距离终点: %.1f', i, current_dist_to_goal));
    
    % 定期刷新图像
    if mod(i, 30) == 0
        drawnow limitrate;
    end
end

% 清理
close(f);
time_elapsed = toc(start_time);

% 修复数组索引越界问题：确保i不超过MAX_ITERATIONS
valid_idx = min(i, MAX_ITERATIONS);
history = history(1:valid_idx, :);  % 裁剪多余空间

% 如果未到达终点，尝试从最近点继续规划一小段
if norm(history(end,:) - END_POINT) >= GOAL_TOLERANCE
    fprintf("未到达终点，尝试从最近点延伸路径...\n");
    extra_steps = 50;
    extra_history = zeros(extra_steps, 3);
    current_loc = closest_point;
    
    for s = 1:extra_steps
        % 直接朝向终点移动
        dir = (END_POINT - current_loc);
        if norm(dir) > 0
            dir = dir / norm(dir);
        end
        
        new_loc = current_loc + dir * MIN_STEP;
        new_loc(3) = max(MIN_HEIGHT, min(new_loc(3), MAX_HEIGHT));
        
        % 检查碰撞
        if udf.get_value(new_loc) < SAFE_DISTANCE
            new_loc(3) = min(new_loc(3) + 5, MAX_HEIGHT);  % 向上调整
        end
        
        extra_history(s, :) = new_loc;
        current_loc = new_loc;
        
        % 如果到达终点附近则停止
        if norm(current_loc - END_POINT) < GOAL_TOLERANCE
            extra_history = extra_history(1:s, :);
            break;
        end
    end
    
    % 将额外路径添加到历史记录
    history = [history; extra_history];
end

% 高级路径平滑处理 - 增强平滑效果同时保证安全
if size(history, 1) > 10
    fprintf("平滑路径...\n");
    history = smooth_path(history, 7, udf, SAFE_DISTANCE, MIN_HEIGHT, MAX_HEIGHT);
end

% 结果输出
fprintf("--- 路径规划完成 ---\n");
fprintf("总步数: %d\n", size(history, 1));
fprintf("总时间: %.2f秒\n", time_elapsed);
fprintf("起点: [%.1f, %.1f, %.1f]\n", history(1,1), history(1,2), history(1,3));
fprintf("终点: [%.1f, %.1f, %.1f]\n", history(end,1), history(end,2), history(end,3));
fprintf("路径总长度: %.2f\n", sum(sqrt(sum(diff(history).^2, 2))));
fprintf("最小高度: %.1f, 最大高度: %.1f\n", min(history(:,3)), max(history(:,3)));
fprintf("是否成功到达: %s\n", string(norm(history(end,:) - END_POINT) < GOAL_TOLERANCE*2));

% 绘制最终路径
plot3(history(:,1), history(:,2), history(:,3), 'g-', 'LineWidth', 1.5);

% 保存结果
step = size(history, 1);
if env.reverse
    save_path = init.build_path(sprintf("run/%s_reverse_history.mat", MODEL_NAME_IN_DB));
else
    save_path = init.build_path(sprintf("run/%s_history.mat", MODEL_NAME_IN_DB));
end
save(save_path, 'history', 'step', '-v7.3');

% 改进的路径平滑函数 - 确保平滑后仍避障
function smoothed = smooth_path(path, window_size, udf, safe_dist, min_h, max_h)
    n = size(path, 1);
    if n <= window_size
        smoothed = path;
        return;
    end
    
    smoothed = path;
    % 使用高斯滤波进行平滑
    weights = gausswin(2*window_size+1);
    weights = weights / sum(weights);
    
    % 平滑处理，对不安全的点不进行平滑
    for i = window_size+1 : n-window_size
        % 计算滑动窗口内的加权平均
        smooth_point = [0, 0, 0];
        for j = -window_size : window_size
            smooth_point = smooth_point + path(i+j, :) * weights(j+window_size+1);
        end
        
        % 检查平滑点是否安全
        if udf.get_value(smooth_point) >= safe_dist * 0.9 && ...
           smooth_point(3) >= min_h && smooth_point(3) <= max_h && ...
           norm(smooth_point - path(i, :)) < safe_dist * 0.5  % 平滑幅度限制
            smoothed(i, :) = smooth_point;
        end
    end
    
    % 二次平滑处理，减少残留锯齿
    for i = window_size+1 : n-window_size
        if norm(smoothed(i, :) - smoothed(i-1, :)) > safe_dist * 0.3 && ...
           norm(smoothed(i, :) - smoothed(i+1, :)) > safe_dist * 0.3
            % 对尖锐拐角进行额外平滑
            smoothed(i, :) = 0.3*smoothed(i-1, :) + 0.4*smoothed(i, :) + 0.3*smoothed(i+1, :);
        end
    end
end
