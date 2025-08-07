-- 确保PostGIS扩展已启用
CREATE EXTENSION IF NOT EXISTS postgis;

CREATE TABLE sdf_metadata (
    id SERIAL PRIMARY KEY,
    model_name VARCHAR(255) UNIQUE NOT NULL,
    resolution FLOAT NOT NULL,
    grid_dim_x INT NOT NULL,
    grid_dim_y INT NOT NULL,
    grid_dim_z INT NOT NULL,
    chunk_size INT NOT NULL,
    min_bound_x FLOAT NOT NULL,
    min_bound_y FLOAT NOT NULL,
    min_bound_z FLOAT NOT NULL,
    srid INT NOT NULL, -- 新增：空间参考ID
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE TABLE sdf_chunks_postgis (
    id BIGSERIAL PRIMARY KEY, -- 使用BIGSERIAL以支持海量数据块
    metadata_id INT NOT NULL REFERENCES sdf_metadata(id) ON DELETE CASCADE,
    
    -- 使用一个GEOMETRY字段来描述数据块的三维边界盒
    -- 存储为3D多边形 (立方体)，SRID与元数据表保持一致
    chunk_bbox GEOMETRY(POLYHEDRALSURFACEZ, 0) NOT NULL, 
    
    -- 实际的SDF浮点数矩阵，依然使用二进制存储
    chunk_data BYTEA NOT NULL
);

-- !!! 这是性能优化的关键：创建3D GIST空间索引 !!!
CREATE INDEX idx_sdf_chunks_postgis_bbox ON sdf_chunks_postgis USING GIST(chunk_bbox);

COMMENT ON TABLE sdf_chunks_postgis IS '存储被分块的SDF数据，并使用PostGIS进行空间索引';
COMMENT ON COLUMN sdf_chunks_postgis.chunk_bbox IS '数据块的三维空间边界盒(Bounding Box)';