function filtered_points = get_point(udf)
    % 找到距离场中的最小值
    min_dist = min(udf.grid(:));
    % 设置一个小的容差，找到所有接近表面的点
    % 容差可以设置为分辨率的一半，以确保捕捉到最接近的体素
    tolerance = 1;
    % 找到所有值在 [min_dist, min_dist + tolerance] 范围内的点的线性索引
    surface_indices = find(udf.grid <= (min_dist + tolerance));
    
    if isempty(surface_indices)
        warning('警告: 未能在UDF中找到距离值接近于零的点。');
        udf_surface_points = [];
    else
        % 将这些点的线性索引转换为三维下标索引 [y, x, z]
        [y_idx, x_idx, z_idx] = ind2sub(udf.grid_dims, surface_indices);
        % 从坐标向量中获取这些点的真实世界坐标
        udf_surface_points = [udf.x_vec(x_idx)', udf.y_vec(y_idx)', udf.z_vec(z_idx)'];
        fprintf('成功从UDF中提取了 %d 个表面点。\n', size(udf_surface_points, 1));
    end
    mask = udf_surface_points(:,3) > 300;  
    filtered_points = udf_surface_points(mask,:); 
end