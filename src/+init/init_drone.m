weights = utils.path.Weights();

boundary.x = [model_range.x(1) - 25, model_range.x(2) + 25];
boundary.y = [model_range.y(1) - 25, model_range.y(2) + 25];
boundary.z = [model_range.z(1) - 25, model_range.z(2) + 25];

boundary = utils.Ref(boundary);