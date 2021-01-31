function [funName] = getFunName(functionHandle)
%GETNAME Summary of this function goes here
%   Detailed explanation goes here
functionStats = functions(functionHandle);
funName = functionStats.function;
end

