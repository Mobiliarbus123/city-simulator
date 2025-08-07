function [vertices, faces] = load_model(model_path)
    model = stlread(model_path);
    % fprintf('文件读取成功。\n');

    if isa(model, 'triangulation')
        vertices = model.Points;
        faces = model.ConnectivityList;
        % fprintf('从 triangulation 对象中提取 .Points 和 .ConnectivityList 成功。\n');

        % 兼容旧版的 stlread，判断是否为普通结构体
    elseif isstruct(model) && isfield(model, 'vertices') && isfield(model, 'faces')
        vertices = model.vertices;
        faces = model.faces;
        % fprintf('从 struct 结构体中提取 .vertices 和 .faces 成功。\n');
    else
        error('无法识别 stlread 返回的数据格式，请检查 model 变量。');
    end

end
