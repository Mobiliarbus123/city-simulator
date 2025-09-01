function draw(type, point, color)

    switch type
        case "start"
            plot3(point(1), point(2), point(3), 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 8);
        case "end"
            plot3(point(1), point(2), point(3), 'x', 'Color', color, 'MarkerSize', 10, 'LineWidth', 2);
        case "path"
            plot3(point(:, 1), point(:, 2), point(:, 3), '.', 'Color', color, 'MarkerSize', 3);
    end

end
