function s_mapped = mapToStruct(s, lambda)
    % s = mapToStruct(s, lambda)  Applies the given lambda function to every field of the given struct
    % lambda = @(s, fieldname)( ... )
    % function returns modified struct
    s_mapped = struct;
    
    names = fieldnames(s);
    for idx = 1:length(names)
        field = names{idx};

        s_mapped.(field) = lambda(s, field);
    end
end