function x_ref = param_x_ref(~, ~, agents, ~)
offset = 2;
N_pred = size(agents.leader.previousStatus.uPred, 2);

leader_trolley_trajectory = agents.leader.previousStatus.xPred{1}(1, :);
follower_trolley_trajectory = leader_trolley_trajectory - offset;

x_ref = {
    [ follower_trolley_trajectory; 
      zeros(3, N_pred+1) ]
};