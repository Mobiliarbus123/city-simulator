% Update udf from old format to new

init.init_env;

[udf, max_height_map] = utils.udf.load_udf_from_file(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)));

save(init.build_path(sprintf("run/%s.mat", MODEL_NAME_IN_DB)), "udf", "max_height_map", "-v7.3");