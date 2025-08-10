classdef SimpleBVH < handle
% SimpleBVH - 一个简单的层次包围盒(BVH)加速结构。
%
% 用于快速查询一个或多个点到三角网格的无符号距离 (UDF)。
%
% 版本: 2.0 (修正了最根本的“值类 vs 句柄类”的错误，确保状态正确更新)

    properties (SetAccess = private)
        Vertices, Faces, Tree, RootNodeIndex
    end
    
    properties (Access = private)
        NextNodeIndex
    end
    
    methods
        function obj = SimpleBVH(vertices, faces)
            % 构造函数：构建BVH树
            obj.Vertices = vertices;
            obj.Faces = faces;
            if size(faces, 2) ~= 3, error('面片必须是三角形。'); end
            num_faces = size(faces, 1);
            if num_faces == 0, error('面片数量为0。'); end
            
            face_indices = (1:num_faces)';
            
            % 预分配足够的空间
            obj.Tree = repmat(struct('AABB', zeros(2,3), 'child1', 0, 'child2', 0, 'face_indices', []), 2 * num_faces, 1);
            
            % 初始化节点计数器
            obj.NextNodeIndex = 1;
            % 因为是句柄类，recursive_build会直接修改obj本身
            obj.RootNodeIndex = obj.recursive_build(face_indices); 
            
            % 裁剪未使用的树空间
            obj.Tree = obj.Tree(1:obj.NextNodeIndex-1);
        end
        
        function distances = query(obj, points)
            num_points = size(points, 1);
            distances = zeros(num_points, 1);
            use_parallel = num_points > 1000 && ~isempty(ver('parallel'));
            if use_parallel
                parfor i = 1:num_points, distances(i) = obj.query_single_point(points(i, :)); end
            else
                for i = 1:num_points, distances(i) = obj.query_single_point(points(i, :)); end
            end
        end
        
        function dist = query_single_point(obj, point)
            min_dist_sq = obj.query_recursive(point, obj.RootNodeIndex, inf);
            dist = sqrt(min_dist_sq);
        end
    end
    
    methods (Access = private)
        function node_idx = recursive_build(obj, face_indices)
            % 因为是句柄类，obj.NextNodeIndex的修改会全局可见
            node_idx = obj.NextNodeIndex;
            obj.NextNodeIndex = obj.NextNodeIndex + 1;
            
            num_faces = numel(face_indices);
            
            vertex_indices_for_faces = obj.Faces(face_indices, :);
            active_vertices = obj.Vertices(vertex_indices_for_faces(:), :);
            node_aabb = [min(active_vertices, [], 1); max(active_vertices, [], 1)];
            
            obj.Tree(node_idx).AABB = node_aabb;
            
            if num_faces <= 8
                obj.Tree(node_idx).face_indices = face_indices;
            else
                aabb_size = node_aabb(2,:) - node_aabb(1,:);
                [~, split_axis] = max(aabb_size);
                
                v_indices = obj.Faces(face_indices, :);
                reshaped_coords = reshape(obj.Vertices(v_indices(:), :), num_faces, 3, 3);
                centroids = squeeze(mean(reshaped_coords, 2));
                
                [~, sorted_order] = sort(centroids(:, split_axis));
                sorted_face_indices = face_indices(sorted_order);
                
                mid_point = floor(num_faces / 2);
                left_indices = sorted_face_indices(1:mid_point);
                right_indices = sorted_face_indices(mid_point+1:end);
                
                child1_idx = obj.recursive_build(left_indices);
                child2_idx = obj.recursive_build(right_indices);
                
                obj.Tree(node_idx).child1 = child1_idx;
                obj.Tree(node_idx).child2 = child2_idx;
            end
        end

        function min_dist_sq = query_recursive(obj, point, node_idx, min_dist_sq)
            node = obj.Tree(node_idx);
            
            dist_to_box_sq = obj.point_to_box_dist_sq(point, node.AABB);
            if dist_to_box_sq >= min_dist_sq
                return;
            end
            
            if node.child1 == 0 
                for i = 1:numel(node.face_indices)
                    face_idx = node.face_indices(i);
                    v0 = obj.Vertices(obj.Faces(face_idx, 1), :);
                    v1 = obj.Vertices(obj.Faces(face_idx, 2), :);
                    v2 = obj.Vertices(obj.Faces(face_idx, 3), :);
                    dist_sq = obj.point_to_triangle_dist_sq(point, v0, v1, v2);
                    if dist_sq < min_dist_sq
                        min_dist_sq = dist_sq;
                    end
                end
            else 
                dist1_sq = obj.point_to_box_dist_sq(point, obj.Tree(node.child1).AABB);
                dist2_sq = obj.point_to_box_dist_sq(point, obj.Tree(node.child2).AABB);
                
                if dist1_sq < dist2_sq
                    min_dist_sq = obj.query_recursive(point, node.child1, min_dist_sq);
                    min_dist_sq = obj.query_recursive(point, node.child2, min_dist_sq);
                else
                    min_dist_sq = obj.query_recursive(point, node.child2, min_dist_sq);
                    min_dist_sq = obj.query_recursive(point, node.child1, min_dist_sq);
                end
            end
        end
    end
    
    methods (Static, Access = private)
        function dist_sq = point_to_box_dist_sq(p, aabb)
            dx = max(0, aabb(1,1) - p(1)) + max(0, p(1) - aabb(2,1));
            dy = max(0, aabb(1,2) - p(2)) + max(0, p(2) - aabb(2,2));
            dz = max(0, aabb(1,3) - p(3)) + max(0, p(3) - aabb(2,3));
            dist_sq = dx*dx + dy*dy + dz*dz;
        end

        function dist_sq = point_to_triangle_dist_sq(p, a, b, c)
            ab = b - a; ac = c - a; ap = p - a;
            d1 = dot(ab, ap); d2 = dot(ac, ap);
            if d1 <= 0 && d2 <= 0, dist_sq = dot(ap, ap); return; end
            bp = p - b; d3 = dot(ab, bp); d4 = dot(ac, bp);
            if d3 >= 0 && d4 <= d3, dist_sq = dot(bp, bp); return; end
            vc = d1 * d4 - d3 * d2;
            if vc <= 0 && d1 >= 0 && d3 <= 0, v = d1 / (d1 - d3); dist_sq = dot(ap - v * ab, ap - v * ab); return; end
            cp = p - c; d5 = dot(ab, cp); d6 = dot(ac, cp);
            if d6 >= 0 && d5 <= d6, dist_sq = dot(cp, cp); return; end
            vb = d5 * d2 - d1 * d6;
            if vb <= 0 && d2 >= 0 && d6 <= 0, w = d2 / (d2 - d6); dist_sq = dot(ap - w * ac, ap - w * ac); return; end
            va = d3 * d6 - d5 * d4;
            if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0, w_bc = (d4 - d3) / ((d4 - d3) + (d5 - d6)); dist_sq = dot(bp - w_bc * (c - b), bp - w_bc * (c - b)); return; end
            dist_sq = dot(cross(ab, ac), ap)^2 / dot(cross(ab, ac), cross(ab, ac));
        end
    end
end