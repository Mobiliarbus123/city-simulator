function grad = gradient(udf, p)

    px = p(1); py = p(2); pz = p(3);

    dx = udf.x_vec(2) - udf.x_vec(1);
    dy = udf.y_vec(2) - udf.y_vec(1);
    dz = udf.z_vec(2) - udf.z_vec(1);

    % 找到左下近似格点索引
    ix0 = floor((px - udf.x_vec(1)) / dx) + 1;
    iy0 = floor((py - udf.y_vec(1)) / dy) + 1;
    iz0 = floor((pz - udf.z_vec(1)) / dz) + 1;

    % 限制范围
    ix0 = max(1, min(ix0, length(udf.x_vec) - 1));
    iy0 = max(1, min(iy0, length(udf.y_vec) - 1));
    iz0 = max(1, min(iz0, length(udf.z_vec) - 1));

    ix1 = ix0 + 1;
    iy1 = iy0 + 1;
    iz1 = iz0 + 1;

    % 归一化坐标 (0~1)
    tx = (px - udf.x_vec(ix0)) / dx;
    ty = (py - udf.y_vec(iy0)) / dy;
    tz = (pz - udf.z_vec(iz0)) / dz;

    % 取出 8 个顶点值
    c000 = udf.grid(iy0, ix0, iz0);
    c100 = udf.grid(iy0, ix1, iz0);
    c010 = udf.grid(iy1, ix0, iz0);
    c110 = udf.grid(iy1, ix1, iz0);
    c001 = udf.grid(iy0, ix0, iz1);
    c101 = udf.grid(iy0, ix1, iz1);
    c011 = udf.grid(iy1, ix0, iz1);
    c111 = udf.grid(iy1, ix1, iz1);

    % 三线性插值的梯度
    dfdx0 = (c100 - c000) * (1 - ty) * (1 - tz) + (c110 - c010) * ty * (1 - tz) ...
        + (c101 - c001) * (1 - ty) * tz + (c111 - c011) * ty * tz;
    dfdx = dfdx0 / dx;

    dfdy0 = (c010 - c000) * (1 - tx) * (1 - tz) + (c110 - c100) * tx * (1 - tz) ...
        + (c011 - c001) * (1 - tx) * tz + (c111 - c101) * tx * tz;
    dfdy = dfdy0 / dy;

    dfdz0 = (c001 - c000) * (1 - tx) * (1 - ty) + (c101 - c100) * tx * (1 - ty) ...
        + (c011 - c010) * (1 - tx) * ty + (c111 - c110) * tx * ty;
    dfdz = dfdz0 / dz;

    grad = [dfdx, dfdy, dfdz];
end
