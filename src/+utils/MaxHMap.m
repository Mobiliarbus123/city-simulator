classdef MaxHMap < handle

    properties
        grid
        x_vec
        y_vec
    end

    methods

        function obj = MaxHMap(udf)
            obj.x_vec = udf.x_vec;
            obj.y_vec = udf.y_vec;
            obj.grid = zeros(length(obj.x_vec), length(obj.y_vec));

            mask = udf.grid < 1;

            z = reshape(udf.z_vec, 1, 1, []);

            z = repmat(z, size(udf.grid, 1), size(udf.grid, 2), 1);
            z(~mask) = -Inf;

            obj.grid = max(z, [], 3);

            obj.grid(obj.grid == -Inf) = 0;

        end

        function value = get_value(obj, point)
            % 获取指定 (x, y) 坐标的最大高度值（z），使用双线性插值

            px = point(1);
            py = point(2);

            % --- 1. 边界检查 ---
            if px < obj.x_vec(1) || px > obj.x_vec(end) || ...
                    py < obj.y_vec(1) || py > obj.y_vec(end)
                value = NaN;
                return;
            end

            % --- 2. 计算索引 ---
            dx = obj.x_vec(2) - obj.x_vec(1);
            dy = obj.y_vec(2) - obj.y_vec(1);

            ix = floor((px - obj.x_vec(1)) / dx) + 1;
            iy = floor((py - obj.y_vec(1)) / dy) + 1;

            % --- 3. 处理上边界情况 ---
            if ix >= length(obj.x_vec); ix = length(obj.x_vec) - 1; end
            if iy >= length(obj.y_vec); iy = length(obj.y_vec) - 1; end

            x1 = obj.x_vec(ix);
            y1 = obj.y_vec(iy);

            xd = (px - x1) / dx;
            yd = (py - y1) / dy;

            % --- 4. 提取四个角点的值 ---
            v00 = obj.grid(ix, iy);
            v10 = obj.grid(ix + 1, iy);
            v01 = obj.grid(ix, iy + 1);
            v11 = obj.grid(ix + 1, iy + 1);

            % --- 5. 双线性插值 ---
            c0 = v00 * (1 - xd) + v10 * xd;
            c1 = v01 * (1 - xd) + v11 * xd;

            value = c0 * (1 - yd) + c1 * yd;
        end

    end

end
