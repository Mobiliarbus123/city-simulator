function fullPath = build_path(relativePath)
    fullPath = fullfile(fileparts(mfilename('fullpath')), relativePath);
end