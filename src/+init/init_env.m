clear;
clc;
close all;

stl_file = init.build_path('data/newyork.stl');
model_range.x = [-2000, 0];
model_range.y = [-2000, 0];
model_range.z = [0, 350];

MODEL_NAME_IN_DB = 'newyork_udf_v2_-2k'; % 数据库中模型的名称
SDF_RESOLUTION = 2.0; % 米/体素。分辨率越高，计算越慢，数据量越大。建议从较大值开始测试。
PADDING = 25.0; % 米。在模型边界外额外扩展的距离，确保无人机有足够空间。
CHUNK_SIZE = 32; % 将SDF切割成 32x32x32 的小块。建议为2的幂（16, 32, 64）。
SRID = 0; % 空间参考ID。0代表局部笛卡尔坐标系。
