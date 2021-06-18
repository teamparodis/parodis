function x_ref = param_x_ref(~, ~, agents, N_S)
x_ref = cell(1, N_S);
offset = 2;
for s=1:N_S
    x_ref{s} = agents.leader.previousStatus.xPred{s}(1, :) - offset;
end