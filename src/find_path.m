init.init_env;
init.init_drone;

env.debug = true;
env.reverse = false;
tag = "multi_2x";
env = utils.Ref(env);
debug = @(varargin) (env.value.debug && fprintf(varargin{:}));

udf = utils.udf.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));

% 在这里定义每架无人机的起点和终点
swarm_definitions = ...
    { ...
     struct('start', [-400, -1400, 200], 'end', [-1900, -200, 200], 'intermediate', []); ...
     struct('start', [-1900, -200, 200], 'end', [-400, -1400, 200], 'intermediate', []); ...
 };

if env.value.reverse

    for i = 1:length(swarm_definitions)
        temp = swarm_definitions{i}.start;
        swarm_definitions{i}.start = swarm_definitions{i}.end;
        swarm_definitions{i}.end = temp;
        swarm_definitions{i}.intermediate = flipud(swarm_definitions{i}.intermediate);
    end

end

swarm = utils.path.Swarm(swarm_definitions, weights, env, boundary);

% 绘制模型
[vertices, faces] = utils.model.load_model(stl_file);
vertices = [vertices(:, 1), vertices(:, 3), vertices(:, 2)] * 60;
[vertices, faces] = utils.model.slice_model(vertices, faces, model_range.x(1), model_range.x(2), model_range.y(1), model_range.y(2));
figure('Name', '机群仿真', 'NumberTitle', 'off');
hold on;
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
axis equal; view(3); grid on; rotate3d on;
lighting gouraud; camlight('headlight');
title('机群仿真'); xlabel('X'); ylabel('Y'); zlabel('Z');

% 绘制起点、终点
swarm.foreach(@(drone) utils.path.draw("start", drone.start_point, drone.color));
swarm.foreach(@(drone) utils.path.draw("end", drone.end_point, drone.color));

fprintf("--- 开始机群仿真 ---\n");
start_time = utils.Ref(tic);
swarm.set_tic(start_time);

bar = waitbar(0, '正在导航');

while true
    % 检查是否所有无人机都已完成任务
    if all([swarm.drones.is_finished])
        break;
    end

    % 移动
    swarm.foreach(@(drone) drone.move(udf, swarm));
    [progress, step] = swarm.get_progress();
    waitbar(progress, bar, sprintf('正在导航: %d 步', step));

end

% 结束
close(bar);
time_elapsed = toc(start_time.value);

fprintf("--- 模拟结束 ---\n");

success_count = 0;

for i = 1:swarm.len
    drone = swarm.drones(i);
    success_count = success_count + drone.is_succeed();

    fprintf("无人机 %d | 总步数：%d\n", drone.id, drone.step);
    fprintf("无人机 %d | 平均速度：%.2f单位/秒\n", drone.id, drone.total_distance / drone.time_elapsed);
    fprintf("无人机 %d | 最终位置：%.2f, %.2f, %.2f\n", drone.id, drone.loc(1), drone.loc(2), drone.loc(3));
    fprintf("无人机 %d | 是否成功到达终点：%s\n", drone.id, string(drone.is_succeed()));
end

fprintf("总时间：%.2f秒\n", time_elapsed);
fprintf("成功数量：%d/%d\n", success_count, swarm.len);
fprintf('--------------------\n\n');

% 储存过程
if env.value.reverse
    save_path = init.build_path(sprintf("run/%s_%s_reverse_history.mat", MODEL_NAME_IN_DB, tag));
else
    save_path = init.build_path(sprintf("run/%s_%s_history.mat", MODEL_NAME_IN_DB, tag));
end

save(save_path, 'swarm', '-v7.3');
