function [distance] = get_point_distance_v1(vertices, faces, point)
% GET_POINT_DISTANCE - 计算一个点到STL网格表面的最短无符号距离 (UDF)
%
% 语法:
%   [distance] = get_point_distance(vertices, faces, point)
%
% 输入:
%   vertices - N x 3 的矩阵，包含STL模型的所有顶点坐标。
%   faces    - M x 3 的矩阵，定义了M个三角面片，每行是顶点的索引。
%   point    - 1 x 3 的行向量，表示要计算距离的空间点。
%
% 输出:
%   distance - 一个标量，表示 point 到网格表面的最短距离。
%
% 算法:
%   该函数遍历模型中的每一个三角形，计算点到该三角形的最近距离。
%   它通过比较距离的平方来维护全局的最小距离，以避免在循环中
%   进行昂贵的 sqrt 计算，从而提高性能。
%   点到单个三角形的距离计算被分解为点到其顶点、边和面的距离计算。
%
% 作者: Gemini
% 日期: 2025-08-09

    % 初始化最小距离的平方为一个非常大的数
    min_dist_sq = inf;
    
    num_faces = size(faces, 1);
    
    % 遍历每一个面片 (三角形)
    for i = 1:num_faces
        % 1. 获取当前三角形的三个顶点坐标
        face_indices = faces(i, :);
        v0 = vertices(face_indices(1), :);
        v1 = vertices(face_indices(2), :);
        v2 = vertices(face_indices(3), :);
        
        % 2. 计算点到当前三角形的距离的平方
        dist_sq = point_to_triangle_dist_sq(point, v0, v1, v2);
        
        % 3. 更新全局最小距离的平方
        if dist_sq < min_dist_sq
            min_dist_sq = dist_sq;
        end
    end
    
    % 4. 对最终结果开方，得到真实距离
    distance = sqrt(min_dist_sq);

end

% --- 辅助函数：计算点到单个三角形的距离的平方 ---
function dist_sq = point_to_triangle_dist_sq(p, a, b, c)
    % 该函数计算点p到由a,b,c定义的三角形的最近距离的平方。
    % 算法通过判断点在Voronoi区域的位置来确定最近特征（点、边、面）。
    
    % 从顶点a出发的向量
    ab = b - a;
    ac = c - a;
    ap = p - a;
    
    % --- 检查最近点是否为顶点 a ---
    d1 = dot(ab, ap);
    d2 = dot(ac, ap);
    if d1 <= 0 && d2 <= 0
        % 在顶点a的Voronoi区域
        dist_sq = dot(ap, ap);
        return;
    end
    
    % --- 检查最近点是否为顶点 b ---
    bp = p - b;
    d3 = dot(ab, bp);
    d4 = dot(ac, bp);
    if d3 >= 0 && d4 <= d3
        % 在顶点b的Voronoi区域
        dist_sq = dot(bp, bp);
        return;
    end
    
    % --- 检查最近点是否在边 ab 上 ---
    vc = d1 * d4 - d3 * d2;
    if vc <= 0 && d1 >= 0 && d3 <= 0
        % 在边ab的Voronoi区域
        v = d1 / (d1 - d3);
        closest_point_on_ab = a + v * ab;
        dist_sq = dot(p - closest_point_on_ab, p - closest_point_on_ab);
        return;
    end
    
    % --- 检查最近点是否为顶点 c ---
    cp = p - c;
    d5 = dot(ab, cp);
    d6 = dot(ac, cp);
    if d6 >= 0 && d5 <= d6
        % 在顶点c的Voronoi区域
        dist_sq = dot(cp, cp);
        return;
    end
    
    % --- 检查最近点是否在边 ac 上 ---
    vb = d5 * d2 - d1 * d6;
    if vb <= 0 && d2 >= 0 && d6 <= 0
        % 在边ac的Voronoi区域
        w = d2 / (d2 - d6);
        closest_point_on_ac = a + w * ac;
        dist_sq = dot(p - closest_point_on_ac, p - closest_point_on_ac);
        return;
    end

    % --- 检查最近点是否在边 bc 上 ---
    va = d3 * d6 - d5 * d4;
    if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0
        % 在边bc的Voronoi区域
        w_bc = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        closest_point_on_bc = b + w_bc * (c - b);
        dist_sq = dot(p - closest_point_on_bc, p - closest_point_on_bc);
        return;
    end
    
    % --- 如果以上都不是，则最近点在三角形内部（投影到面上） ---
    % 点到平面的距离的平方
    normal = cross(ab, ac);
    dist_sq = dot(normal, ap)^2 / dot(normal, normal);
end