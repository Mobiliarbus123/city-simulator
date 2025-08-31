classdef Drone < handle

    properties
        id
        start_point
        end_point
        intermediate_point
        weights
        color

        way_points
        target_nth
        target

        loc
        step
        is_finished
        history

        progress_current
        progress_total
        total_distance

        start_time
        time_elapsed

        boundary

        debug
    end

    properties (Access = private)
        history_length
        capacity_step

        current_distance
        progress
    end

    methods
        % 构造函数
        function obj = Drone(id, start_point, end_point, intermediate_point, weights, color, env, boundary)

            obj.id = id;
            obj.start_point = start_point;
            obj.end_point = end_point;
            obj.intermediate_point = intermediate_point;
            obj.weights = weights;
            obj.color = color;

            obj.way_points = [start_point; intermediate_point; end_point];
            obj.target_nth = 2;
            obj.target = obj.way_points(2, :);

            obj.loc = start_point;
            obj.step = 0;
            obj.is_finished = false;

            obj.progress_total = 0;

            for i = 1:size(obj.way_points, 1) - 1
                obj.progress_total = obj.progress_total + norm(obj.way_points(i + 1, :) - obj.way_points(i, :));
            end

            obj.total_distance = obj.progress_total;

            obj.history_length = floor(obj.progress_total / obj.weights.step_length * 1.5);
            obj.history = zeros(obj.history_length, 3);
            obj.capacity_step = floor(obj.progress_total / obj.weights.step_length * 0.5);

            obj.current_distance = norm(obj.way_points(2, :) - obj.way_points(1, :));
            obj.progress_current = 0;
            obj.progress = 0;

            obj.boundary = boundary;

            obj.debug = @(varargin) (env.value.debug && fprintf(varargin{:}));

        end

        function push_history(obj, loc)

            i = obj.step;

            if i > obj.history_length
                obj.history(obj.history_length + 1:obj.history_length + obj.capacity_step, :) = 0;
                obj.history_length = size(obj.history, 1);
            end

            obj.history(i, :) = loc;

        end

        function move(obj, udf, swarm)

            if obj.is_finished
                return;
            end

            i = obj.step;

            if abs(norm(obj.loc - obj.target)) < obj.weights.ends_tolerance
                obj.is_finished = true;
            end

            if i > (obj.weights.check_stop_interval + 5) && (abs(norm(obj.loc - obj.history(i - obj.weights.check_stop_interval, :))) < 1)
                obj.is_finished = true;
            end

            if obj.is_finished

                if obj.target_nth < size(obj.way_points, 1)
                    obj.target_nth = obj.target_nth + 1;
                    obj.target = obj.way_points(obj.target_nth, :);
                    obj.current_distance = norm(obj.way_points(obj.target_nth, :) - obj.way_points(obj.target_nth - 1, :));
                    obj.progress = obj.progress + obj.current_distance;
                    obj.is_finished = false;
                else
                    obj.progress_total = obj.progress_current;
                    obj.time_elapsed = toc(obj.start_time.value);
                    return;
                end

            end

            obj.step = obj.step + 1;
            i = obj.step;

            % 计算建筑物排斥力
            udf_value = udf.get_value(obj.loc);
            obj.debug("建筑物距离: %.2f\n", udf_value);

            if udf_value > obj.weights.max_counted_distance
                F_repulsion = [0, 0, 0];
            else
                grad = udf.get_gradient(obj.loc);
                F_repulsion = grad * obj.weights.repulsion_weight(udf_value);
            end

            % 计算边界排斥力
            F_boundary = utils.path.boundary_vector(obj.loc, obj.boundary, obj.weights.boundary_weight, 20);
            obj.debug("边界排斥力: %.2f, %.2f, %.2f\n", F_boundary(1), F_boundary(2), F_boundary(3));

            % 计算地面排斥力
            F_ground = [0, 0, obj.weights.ground_weight(obj.loc(3))];
            obj.debug("地面排斥力: %.2f, %.2f, %.2f\n", F_ground(1), F_ground(2), F_ground(3));

            % 计算无人机间排斥力
            if nargin < 3
                F_drones = [0, 0, 0];
            else
                F_drones = swarm.calc_mutual_repulsion(obj, obj.weights);
            end

            obj.debug("无人机间排斥力: %.2f, %.2f, %.2f\n", F_drones(1), F_drones(2), F_drones(3));

            % 计算目标吸引力
            task_vec = (obj.target - obj.loc);
            task_vec = task_vec / norm(task_vec);
            F_task = task_vec * obj.weights.task_weight(abs(norm(obj.target - obj.loc)));

            % 合成总力
            obj.debug("F_repulsion: %.2f, %.2f, %.2f, F_boundary: %.2f, %.2f, %.2f, F_task: %.2f, %.2f, %.2f\n", F_repulsion(1), F_repulsion(2), F_repulsion(3), F_boundary(1), F_boundary(2), F_boundary(3), F_task(1), F_task(2), F_task(3));
            F_total = F_repulsion + F_boundary + F_ground + F_drones + F_task;
            obj.debug("F_total: %.2f, %.2f, %.2f, norm: %.2f\n", F_total(1), F_total(2), F_total(3), norm(F_total));

            if i > 3
                obj.debug("上上次水平移动距离: %.2f\n", norm(obj.loc(2:3) - obj.history(i - 3, 2:3)));
            end

            % 没思路就往上飞一点
            if abs(norm(F_task - F_total) / norm(F_task)) > 0.3 ...
                    && (obj.loc(3) - obj.target(3)) / norm(obj.loc - obj.target) < 0.7 ...
                    && abs(F_task(3) / norm(F_task)) < 0.8 ...
                    && ((abs(norm(F_total)) < 0.2) ...
                    || abs(norm(dot(F_repulsion(2:3) / norm(F_repulsion(2:3)), F_task(2:3) / norm(F_task(2:3))))) > 0.5 ...
                    || (i > 3 && norm(obj.loc(2:3) - obj.history(i - 3, 2:3)) < 0.3) ...
                    || (norm(F_boundary) + norm(F_repulsion)) / norm(F_repulsion + F_boundary) < 0.5 ...
                    || (i > 8 && norm(obj.loc - obj.history(i - 5, :)) < 1))
                obj.debug("往上飞一点\n");
                F_total = F_total / norm(F_total);
                F_total(3) = obj.weights.up_weight;
                F_total = F_total / norm(F_total);
            end

            F_total = F_total / norm(F_total);

            % 更新位置
            obj.loc = obj.loc + F_total * obj.weights.step_length;
            obj.debug("总力: %.2f, %.2f, %.2f  当前位置：%.2f, %.2f, %.2f\n", F_total(1), F_total(2), F_total(3), obj.loc(1), obj.loc(2), obj.loc(3));
            obj.debug("\n");

            obj.push_history(obj.loc);
            obj.progress_current = obj.current_distance - abs(norm(obj.loc - obj.target)) + obj.progress;

            % 画图
            utils.path.draw("path", obj.loc, obj.color);

        end

        function value = is_succeed(obj)
            value = all(abs(obj.loc - obj.target) < obj.weights.max_ends_tolerance);
        end

    end

end
