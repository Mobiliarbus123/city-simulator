function write_udf(conn, udf, model_name, chunk_size, sr_id, sdf_resolution)

    try
        % 首先检查是否已存在同名模型，如果存在则删除旧数据（或提示错误）
        exec(conn, 'DELETE FROM sdf_metadata WHERE model_name = ?', {model_name});

        meta_sql = ['INSERT INTO sdf_metadata (model_name, resolution, grid_dim_x, grid_dim_y, grid_dim_z, chunk_size, min_bound_x, min_bound_y, min_bound_z, sr_id) ' ...
                    'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)'];
        exec(conn, meta_sql, {model_name, sdf_resolution, udf.grid_dims(2), udf.grid_dims(1), udf.grid_dims(3), chunk_size, udf.min_bounds(1), udf.min_bounds(2), udf.min_bounds(3), sr_id});

        curs = exec(conn, 'SELECT id FROM sdf_metadata WHERE model_name = ?', {model_name});
        curs = fetch(curs);
        metadata_id = curs.Data{1, 1};
        fprintf('  元数据存储成功，模型 [%s] 的ID为: %d\n\n', model_name, metadata_id);
    catch ME
        rollback(conn); % 出错则回滚事务并关闭连接
        disp(ME);
        throw(ME);
    end

    sql_insert_chunk = 'INSERT INTO sdf_chunks_postgis (metadata_id, chunk_bbox, chunk_data) VALUES (?, ST_GeomFromText(?, ?), ?)';
    chunk_count = 0;
    num_chunks_total = ceil(udf.grid_dims(3) / chunk_size) * ceil(udf.grid_dims(1) / chunk_size) * ceil(udf.grid_dims(2) / chunk_size);

    try

        for z_start = 1:chunk_size:udf.grid_dims(3)

            for y_start = 1:chunk_size:udf.grid_dims(1)

                for x_start = 1:chunk_size:udf.grid_dims(2)
                    % 提取数据块
                    x_end = min(x_start + chunk_size - 1, udf.grid_dims(2));
                    y_end = min(y_start + chunk_size - 1, udf.grid_dims(1));
                    z_end = min(z_start + chunk_size - 1, udf.grid_dims(3));
                    chunk_data_matrix = udf.grid(y_start:y_end, x_start:x_end, z_start:z_end);

                    % 计算边界盒世界坐标
                    min_x = udf.x_vec(x_start); max_x = udf.x_vec(x_end);
                    min_y = udf.y_vec(y_start); max_y = udf.y_vec(y_end);
                    min_z = udf.z_vec(z_start); max_z = udf.z_vec(z_end);

                    % 生成立方体边界盒的WKT字符串
                    bbox_wkt = sprintf('POLYHEDRALSURFACE Z (((%f %f %f,%f %f %f,%f %f %f,%f %f %f,%f %f %f)),((%f %f %f,%f %f %f,%f %f %f,%f %f %f,%f %f %f)),((%f %f %f,%f %f %f,%f %f %f,%f %f %f,%f %f %f)),((%f %f %f,%f %f %f,%f %f %f,%f %f %f,%f %f %f)),((%f %f %f,%f %f %f,%f %f %f,%f %f %f,%f %f %f)),((%f %f %f,%f %f %f,%f %f %f,%f %f %f,%f %f %f)))', ...
                        min_x, min_y, min_z, max_x, min_y, min_z, max_x, max_y, min_z, min_x, max_y, min_z, min_x, min_y, min_z, min_x, min_y, max_z, min_x, max_y, max_z, max_x, max_y, max_z, max_x, min_y, max_z, min_x, min_y, max_z, min_x, min_y, min_z, min_x, max_y, min_z, min_x, max_y, max_z, min_x, min_y, max_z, min_x, min_y, min_z, max_x, min_y, min_z, max_x, min_y, max_z, max_x, max_y, max_z, max_x, max_y, min_z, max_x, min_y, min_z, min_x, min_y, min_z, min_x, min_y, max_z, max_x, min_y, max_z, max_x, min_y, min_z, min_x, min_y, min_z, min_x, max_y, min_z, max_x, max_y, min_z, max_x, max_y, max_z, min_x, max_y, max_z, min_x, max_y, min_z);

                    % 序列化并插入
                    chunk_data_blob = getByteStreamFromArray(single(chunk_data_matrix)); % 使用单精度浮点数以节省空间
                    exec(conn, sql_insert_chunk, {metadata_id, bbox_wkt, sr_id, chunk_data_blob});

                    chunk_count = chunk_count + 1;

                    if mod(chunk_count, 100) == 0 % 每处理100个块，打印一次进度
                        fprintf('  已处理 %d / %d 个数据块...\n', chunk_count, num_chunks_total);
                    end

                end

            end

        end

        commit(conn); % 全部成功后，提交事务
        fprintf('所有 %d 个数据块及空间边界盒均已成功存入数据库！\n', chunk_count);

    catch ME
        rollback(conn); % 如果任何一步出错，回滚所有操作
        fprintf(2, '!!! 数据块存储过程中发生错误，事务已回滚 !!!\n');
        error('错误信息: %s', ME.message);
    end

end
