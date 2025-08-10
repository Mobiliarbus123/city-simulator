init.init_env; 
udf_data=load("newyork_udf_v2.mat");
x_coords = data.UDF_context.x_vec;
y_coords = data.UDF_context.y_vec;
z_coords = data.UDF_context.z_vec;
grid = udf_data.UDF_grid;
