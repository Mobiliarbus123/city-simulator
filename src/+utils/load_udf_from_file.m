function [udf, max_height_map] = load_udf_from_file(file_path)
    % 从指定文件加载 UDF 数据
    data = load(file_path);

    if isfield(data, 'udf')
        udf = data.udf;

        if ~isfield(udf, 'dx')
            udf = utils.UDF(udf.grid, udf);
        end

    else
        % 兼容旧版本
        udf = utils.UDF(data.UDF_grid, data.UDF_context);
    end

    if isfield(data, 'max_height_map')
        max_height_map = data.max_height_map;
    else
        max_height_map = utils.MaxHMap(udf);
    end

end
