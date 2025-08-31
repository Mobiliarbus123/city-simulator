function vec = boundary_vector(point, boundary, weighter, max_distance)
    % boundary: Ref
    vec = [0, 0, 0];

    if abs(point(1) - boundary.value.x(1)) < max_distance
        vec(1) = abs(boundary.value.x(1) - point(1));
    elseif abs(point(1) - boundary.value.x(2)) < max_distance
        vec(1) = -abs(boundary.value.x(2) - point(1));
    end

    if abs(point(2) - boundary.value.y(1)) < max_distance
        vec(2) = abs(boundary.value.y(1) - point(2));
    elseif abs(point(2) - boundary.value.y(2)) < max_distance
        vec(2) = -abs(boundary.value.y(2) - point(2));
    end

    if abs(point(3) - boundary.value.z(1)) < max_distance
        vec(3) = abs(boundary.value.z(1) - point(3));
    elseif abs(point(3) - boundary.value.z(2)) < max_distance
        vec(3) = -abs(boundary.value.z(2) - point(3));
    end

    if norm(vec) < 1e-6
        return; % 如果向量几乎为零，则不需要进一步处理
    end

    vec = vec .* weighter(norm(vec));
end
