function [transformedPlane, distance, BP_pts] = paretoFrontProjection(normFront)
%PARETOFRONTPROJECTION Summary of this function goes here
%   Detailed explanation goes here
distance = [];

dim = size(normFront,2);

intercept = normFront(1:dim,:)\ones(3,1);
normal_vector = intercept/norm(intercept);

P = [intercept' 0; eye(dim) normal_vector];

for i = 1:length(normFront)
    plane_dist_vec = P^-1*[1; normFront(i,:)'];
    distance(i,1) = plane_dist_vec(end);
    BP_pts(i,:) = plane_dist_vec(1:end-1)';
end

% EP3d = BP_pts(1:dim,:);
EP3d = [0 1 1;1 0 1; 1 1 0];
EPproj = [0 1; 1 1; 0.5 0];
EP_base = EP3d\EPproj;

transformedPlane = BP_pts*EP_base;

end

