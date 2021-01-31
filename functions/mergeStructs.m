function [mergedStruct] = mergeStructs(varargin)
%MERGESTRUCTS Merges given structs in the order of the arguments
mergedStruct = struct;
for i=1:nargin
    fields = fieldnames(varargin{i});
    for idx = 1:length(fields)
        mergedStruct.(fields{idx}) = varargin{i}.(fields{idx});
    end
end


