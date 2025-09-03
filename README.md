## Requirements

Matlab >= 2024a

- Instrument Control Toolbox
- Parallel Computing Toolbox
- point2trimesh( ) — Distance Between Point and Triangulated Surface
- PARFOR progress monitor (progress bar) v4

## Run

Matlab root: `src/`

Entries:

- show_model: 显示模型
- build: 构建 UDF
- test_udf: 验证 UDF
- find_path: 寻路 (需要预构建 UDF)
- view_path: 从历史记录加载并查看路径

## Env

`init.init_env`:

MODEL_NAME_IN_DB: 模型名称，用于标识生成的 UDF (以及相关的寻路历史)
model_range(x, y, z): 定义模型的空间范围

`find_path`:

env.debug: 输出调试信息 (详细寻路过程)
env.reverse: 反向寻路
swarm_definitions: 无人机群的起点终点和中转点 (若有) 定义
