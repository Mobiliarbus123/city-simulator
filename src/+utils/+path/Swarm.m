classdef Swarm < handle

    properties
        drones
        len
    end

    methods

        function obj = Swarm(drones, weights, env, boundary)
            obj.drones = utils.path.Drone.empty();
            obj.len = length(drones);

            colors = lines(obj.len);

            for i = 1:obj.len
                def = drones{i};
                obj.drones(i) = utils.path.Drone(i, def.start, def.end, def.intermediate, weights, colors(i, :), env, boundary);
            end

        end

        function foreach(obj, func)

            for i = 1:obj.len
                func(obj.drones(i));
            end

        end

        function set_tic(obj, tic)

            for i = 1:obj.len
                obj.drones(i).start_time = tic;
            end

        end

        function force = calc_mutual_repulsion(obj, drone, weights)

            force = [0, 0, 0];

            for i = 1:obj.len

                if obj.drones(i) == drone
                    continue;
                end

                dist = drone.loc - obj.drones(i).loc;

                force = force + (dist / norm(dist)) .* weights.drone_weight(norm(dist));

            end

        end

        function [progress, steps] = get_progress(obj)

            progress = sum([obj.drones.progress_current]) / sum([obj.drones.progress_total]);
            steps = sum([obj.drones.step]);

        end

    end

end
