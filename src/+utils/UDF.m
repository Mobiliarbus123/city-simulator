classdef UDF < handle

    properties
        grid
        context
        min_bounds
        max_bounds
        grid_dims
        x_vec
        y_vec
        z_vec
    end

    methods

        function obj = UDF(grid, context)
            obj.grid = grid;
            obj.context = context;
            obj.min_bounds = context.min_bounds;
            obj.max_bounds = context.max_bounds;
            obj.grid_dims = context.grid_dims;
            obj.x_vec = context.x_vec;
            obj.y_vec = context.y_vec;
            obj.z_vec = context.z_vec;
        end

        function distance = get_value(obj, point)
            % 获取指定坐标的 UDF 值 (坐标轴是{x,y,z}_vec)
            % 不存在的点线性插值

            distance = interp3(obj.x_vec, obj.y_vec, obj.z_vec, obj.grid, ...
                point(1), point(2), point(3), 'linear');
        end

    end

end
