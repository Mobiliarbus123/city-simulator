function [direction, F_total] = calculate_apf_direction(q_current, q_goal, params, grid, context)
% CALCULATE_APF_DIRECTION - 计算某点在人工势场中的合力方向
%
% 输入:
%   q_current - 1x3 向量, 当前点 [x, y, z]
%   q_goal    - 1x3 向量, 目标点 [x, y, z]
%   params    - 结构体, 包含APF参数:
%               .eta  - 引力系数
%               .k    - 斥力系数
%               .rho0 - 障碍物影响范围
%               .h    - 有限差分步长
%  grid
%  context
% 输出:
%   direction - 1x3 单位向量, 表示合力的方向。如果合力为0，则返回 [0, 0, 0]。
%   F_total   - 1x3 向量, 未经归一化的原始合力向量 (可选，用于调试)。
%% 计算引力 (Attractive Force)
    F_att = -params.eta * (q_current - q_goal);
    %% 计算斥力 (Repulsive Force)
    F_rep = [0, 0, 0]; 
    % 获取到障碍物的距离
    dist_to_obs = interp3(context.x_vec, context.y_vec, context.z_vec, grid, ...
                          q_current(2), q_current(1), q_current(3), 'linear');
    % 如果在影响范围内，则计算斥力
    if ~isnan(dist_to_obs) && dist_to_obs < params.rho0
        % 按需计算梯度
        grad_vec = calculate_gradient_from_udf(grid, context, q_current);
        if norm(grad_vec) > 1e-9
            rep_magnitude = params.k * (1/dist_to_obs - 1/params.rho0) * (1/dist_to_obs^2);
            rep_direction = grad_vec / norm(grad_vec);
            F_rep = rep_magnitude * rep_direction;
        end
    end
    
    %% 计算合力并归一化
    F_total = F_att + F_rep;
    
    norm_F_total = norm(F_total);
    
    if norm_F_total > 1e-9
        direction = F_total / norm_F_total;
    else
        direction = [0, 0, 0];
    end
end