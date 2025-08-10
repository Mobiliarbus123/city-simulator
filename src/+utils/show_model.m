function show_model(vertices, faces, name)

    if nargin < 3
        name = 'STL Model Viewer';
    end

    try
        figure('Name', name, 'NumberTitle', 'off');
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

        % 设置显示效果
        axis equal;
        view(3);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        set(gca, 'YDir', 'reverse');
        title(name, 'Interpreter', 'none'); % 防止文件名中的下划线被转义
        grid on;
        rotate3d on; % 允许用鼠标旋转视角
        lighting gouraud;
        camlight('headlight');

    catch ME
        fprintf(2, '!!! patch 函数绘图失败: %s\n', ME.message);
    end

end
