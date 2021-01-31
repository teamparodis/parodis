function [extractedStruct] = extractScenario(inputStruct, scenario)
%EXTRACTSCENARIO Extracts a single struct from a scenario struct
%                   a scenario is a struct where s.field = cell(numScenarios, 1)
    extractedStruct = struct;
    fields = fieldnames(inputStruct);
    for idx=1:length(fields)
        extractedStruct.(fields{idx}) = inputStruct.(fields{idx}){scenario};
    end
end

