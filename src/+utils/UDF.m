classdef UDF < handle

    properties
        grid
        min_bounds
        max_bounds
        grid_dims
        x_vec
        y_vec
        z_vec

        dx
        dy
        dz
    end

    methods

        function obj = UDF(grid, context)
            obj.grid = grid;
            obj.min_bounds = context.min_bounds;
            obj.max_bounds = context.max_bounds;
            obj.grid_dims = context.grid_dims;
            obj.x_vec = context.x_vec;
            obj.y_vec = context.y_vec;
            obj.z_vec = context.z_vec;

            obj.dx = obj.x_vec(2) - obj.x_vec(1);
            obj.dy = obj.y_vec(2) - obj.y_vec(1);
            obj.dz = obj.z_vec(2) - obj.z_vec(1);

        end

        function distance = get_value(obj, point)
            % 获取指定坐标的 UDF 值 (坐标轴是{x,y,z}_vec)

            px = point(1);
            py = point(2);
            pz = point(3);

            % --- 1. 边界检查 ---
            % 检查点是否在网格边界之外
            if px < obj.min_bounds(1) || px > obj.max_bounds(1) || ...
                    py < obj.min_bounds(2) || py > obj.max_bounds(2) || ...
                    pz < obj.min_bounds(3) || pz > obj.max_bounds(3)
                distance = NaN; % 与 interp3 行为一致，界外点返回 NaN
                return;
            end

            % --- 2. 直接计算索引 (i, j, k) ---
            % 因为网格是均匀的，所以这是最快的 O(1) 方法
            % +1 是因为 MATLAB 是1-based索引

            ix = floor((px - obj.x_vec(1)) / obj.dx) + 1;
            iy = floor((py - obj.y_vec(1)) / obj.dy) + 1;
            iz = floor((pz - obj.z_vec(1)) / obj.dz) + 1;

            % --- 3. 处理上边界情况 ---
            % 如果点正好落在上边界，索引ix会等于grid_dims(1)，ix+1会越界
            if ix >= obj.grid_dims(1); ix = obj.grid_dims(1) - 1; end
            if iy >= obj.grid_dims(2); iy = obj.grid_dims(2) - 1; end
            if iz >= obj.grid_dims(3); iz = obj.grid_dims(3) - 1; end

            % --- 4. 计算插值权重 (xd, yd, zd) ---
            % 点在立方体单元内的归一化坐标 (0到1)
            x1 = obj.x_vec(ix);
            y1 = obj.y_vec(iy);
            z1 = obj.z_vec(iz);

            xd = (px - x1) / obj.dx;
            yd = (py - y1) / obj.dy;
            zd = (pz - z1) / obj.dz;

            % --- 5. 提取8个角点的值 ---
            v000 = obj.grid(ix, iy, iz);
            v100 = obj.grid(ix + 1, iy, iz);
            v010 = obj.grid(ix, iy + 1, iz);
            v110 = obj.grid(ix + 1, iy + 1, iz);
            v001 = obj.grid(ix, iy, iz + 1);
            v101 = obj.grid(ix + 1, iy, iz + 1);
            v011 = obj.grid(ix, iy + 1, iz + 1);
            v111 = obj.grid(ix + 1, iy + 1, iz + 1);

            % --- 6. 执行三线性插值 ---
            c00 = v000 * (1 - xd) + v100 * xd;
            c10 = v010 * (1 - xd) + v110 * xd;
            c01 = v001 * (1 - xd) + v101 * xd;
            c11 = v011 * (1 - xd) + v111 * xd;

            c0 = c00 * (1 - yd) + c10 * yd;
            c1 = c01 * (1 - yd) + c11 * yd;

            distance = c0 * (1 - zd) + c1 * zd;
        end

    end

end
