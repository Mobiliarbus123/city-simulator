classdef Weights < handle

    properties
        repulsion_weight
        boundary_weight
        task_weight
        ground_weight
        drone_weight
        up_weight

        max_counted_distance
        step_length
        ends_tolerance
        max_ends_tolerance
        move_speed_tolerance

        check_stop_interval
        is_succeed
    end

    methods

        function obj = Weights()
            obj.repulsion_weight = @(distance) 50000 / (distance ^ 3);
            obj.boundary_weight = @(distance) 5000 / ((distance + 3) ^ 5);
            obj.task_weight = @(distance) (distance < 100) .* 5 + 5;
            obj.ground_weight = @(distance) exp(- ((distance / 100) - 1));
            obj.drone_weight = @(d) 80000 / (d ^ 4);
            obj.up_weight = 0.5;

            obj.max_counted_distance = 200;
            obj.step_length = 1;
            obj.ends_tolerance = 10;
            obj.max_ends_tolerance = 30;
            obj.move_speed_tolerance = 0.05;

            obj.check_stop_interval = floor(1 / obj.move_speed_tolerance);
            obj.is_succeed = @(loc, target) abs(norm(loc - target)) < obj.max_ends_tolerance;
        end

    end

end
