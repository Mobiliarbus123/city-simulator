classdef drone < handle
    properties
        id              % ���
        start_point     % ���
        end_point       % �յ�
        loc             % ��ǰ����
        history         % ��ʷ�켣
        step            % �Ѿ��ߵĲ���
        is_finished     % �Ƿ񵽴��յ�
    end

    methods
        % ���캯��
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

        % % �ƶ����˻�
        function move(obj, F_total, step_length, ends_tolerance)
            if obj.is_finished
                return;
            end

            % 2. ���ݺ�����������ƶ�
            if norm(F_total) > 0
                direction = F_total / norm(F_total);
            else
                direction = [0, 0, 0]; % ���û����������ԭ��
            end
            new_loc = obj.loc + direction * step_length;
            
            % 3. �������˻�״̬
            obj.loc = new_loc;
            obj.history(end+1, :) = new_loc;
            obj.step = obj.step + 1;
            
            % 4. �ж��Ƿ񵽴��յ�
            if norm(obj.loc - obj.end_point) < ends_tolerance
                obj.is_finished = true;
                fprintf("���˻� %d �����յ㡣\n", obj.id);
            end
        end

        % % �������
        % function F_total = compute_forces(obj, all_drones, env, drone_collision_distance)
        %     debug = @(varargin) (env.debug && fprintf(varargin{:}));

        %     % (1) �������ų��� 
        %     udf_value = env.udf.get_value(obj.loc);
        %     max_counted_distance = 200; % From your original file
        %     if udf_value > max_counted_distance
        %         F_repulsion = [0, 0, 0];
        %     else
        %         grad = env.udf.get_gradient(obj.loc);
        %         F_repulsion = grad * env.weights.repulsion(udf_value);
        %     end

        %     % (2) �߽��ų���
        %     F_boundary = utils.boundary_vector(obj.loc, env.boundary, env.weights.boundary, 20);

        %     % (3) �����ų���
        %     F_ground = [0, 0, env.weights.ground(obj.loc(3))];

        %     % (4) Ŀ��������
        %     task_vec = (obj.end_point - obj.loc);
        %     task_dist = norm(task_vec);
        %     if task_dist > 0
        %         task_vec_norm = task_vec / task_dist;
        %     else
        %         task_vec_norm = [0,0,0];
        %     end
        %     F_task = task_vec_norm * env.weights.task(task_dist);

        %     % (5) �������������ų���
        %     F_drones = [0, 0, 0];
        %     for other_drone = all_drones
        %         if other_drone.id == obj.id
        %             continue; % �����Լ�
        %         end
                
        %         vec_to_other = obj.loc - other_drone.loc;
        %         dist_to_other = norm(vec_to_other);
                
        %         % �������С���趨����ײ��ֵ��������ų���
        %         if dist_to_other < drone_collision_distance
        %             repulsion_force = (vec_to_other / dist_to_other) * env.weights.drone_repulsion(dist_to_other);
        %             F_drones = F_drones + repulsion_force;
        %         end
        %     end
        %     debug("���˻� %d �����ų���: %.2f, %.2f, %.2f\n", obj.id, F_drones(1), F_drones(2), F_drones(3));

        %     % �ϳ�����
        %     F_total = F_repulsion + F_boundary + F_ground + F_task + F_drones;

        %     % "��ס"�����߼��������ʱ��λ�ñ仯�����������Ϸ�
        %     if obj.step > 8 && norm(obj.loc - obj.history(obj.step - 5, :)) < 1
        %         debug("���˻� %d ���ܿ�ס�ˣ��������Ϸɡ�\n", obj.id);
        %         if norm(F_total) > 0
        %             F_total = F_total / norm(F_total);
        %         end
        %         F_total(3) = F_total(3) + env.weights.up;
        %     end
        % end
    end
end