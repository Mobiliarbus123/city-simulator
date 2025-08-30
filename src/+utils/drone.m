classdef drone < handle
    properties
        id              % 编号
        start_point     % 起点
        end_point       % 终点
        loc             % 当前坐标
        history         % 历史轨迹
        step            % 已经走的步数
        is_finished     % 是否到达终点
    end

    methods
        % 构造函数
        function obj = drone(id, start_point, end_point)
            if nargin > 0 
                obj.id = id;
                obj.start_point = start_point;
                obj.end_point = end_point;
                obj.loc = start_point;
                obj.history = start_point;
                obj.step = 0;
                obj.is_finished = false;
            end
        end

        % % 移动无人机
        function move(obj, F_total, step_length, ends_tolerance)
            if obj.is_finished
                return;
            end

            % 2. 根据合力方向进行移动
            if norm(F_total) > 0
                direction = F_total / norm(F_total);
            else
                direction = [0, 0, 0]; % 如果没有力，保持原地
            end
            new_loc = obj.loc + direction * step_length;
            
            % 3. 更新无人机状态
            obj.loc = new_loc;
            obj.history(end+1, :) = new_loc;
            obj.step = obj.step + 1;
            
            % 4. 判断是否到达终点
            if norm(obj.loc - obj.end_point) < ends_tolerance
                obj.is_finished = true;
                fprintf("无人机 %d 到达终点。\n", obj.id);
            end
        end

        % % 计算合力
        % function F_total = compute_forces(obj, all_drones, env, drone_collision_distance)
        %     debug = @(varargin) (env.debug && fprintf(varargin{:}));

        %     % (1) 建筑物排斥力 
        %     udf_value = env.udf.get_value(obj.loc);
        %     max_counted_distance = 200; % From your original file
        %     if udf_value > max_counted_distance
        %         F_repulsion = [0, 0, 0];
        %     else
        %         grad = env.udf.get_gradient(obj.loc);
        %         F_repulsion = grad * env.weights.repulsion(udf_value);
        %     end

        %     % (2) 边界排斥力
        %     F_boundary = utils.boundary_vector(obj.loc, env.boundary, env.weights.boundary, 20);

        %     % (3) 地面排斥力
        %     F_ground = [0, 0, env.weights.ground(obj.loc(3))];

        %     % (4) 目标吸引力
        %     task_vec = (obj.end_point - obj.loc);
        %     task_dist = norm(task_vec);
        %     if task_dist > 0
        %         task_vec_norm = task_vec / task_dist;
        %     else
        %         task_vec_norm = [0,0,0];
        %     end
        %     F_task = task_vec_norm * env.weights.task(task_dist);

        %     % (5) 【新增】机间排斥力
        %     F_drones = [0, 0, 0];
        %     for other_drone = all_drones
        %         if other_drone.id == obj.id
        %             continue; % 跳过自己
        %         end
                
        %         vec_to_other = obj.loc - other_drone.loc;
        %         dist_to_other = norm(vec_to_other);
                
        %         % 如果距离小于设定的碰撞阈值，则计算排斥力
        %         if dist_to_other < drone_collision_distance
        %             repulsion_force = (vec_to_other / dist_to_other) * env.weights.drone_repulsion(dist_to_other);
        %             F_drones = F_drones + repulsion_force;
        %         end
        %     end
        %     debug("无人机 %d 机间排斥力: %.2f, %.2f, %.2f\n", obj.id, F_drones(1), F_drones(2), F_drones(3));

        %     % 合成总力
        %     F_total = F_repulsion + F_boundary + F_ground + F_task + F_drones;

        %     % "卡住"处理逻辑：如果长时间位置变化不大，则尝试向上飞
        %     if obj.step > 8 && norm(obj.loc - obj.history(obj.step - 5, :)) < 1
        %         debug("无人机 %d 可能卡住了，尝试向上飞。\n", obj.id);
        %         if norm(F_total) > 0
        %             F_total = F_total / norm(F_total);
        %         end
        %         F_total(3) = F_total(3) + env.weights.up;
        %     end
        % end
    end
end