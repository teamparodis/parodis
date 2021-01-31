function borderPoints = getBorderPoints(pf)
%Transforms the points onto the plane best describing them and finds the points of the tightest
%boundary.

dim = size(pf,2); % dimension of a singular Pareto-optimal point

borderPoints = []; % vector of indices of the border points
planeDescription = []; % matrix of planar vectors dim dimensions

% calculate the axis intercepts of the plane the Pareto-optimal points are
% projected to
intercept = ((pf'*pf)\pf'*ones(size(pf,1),1))';

% calculate the plane's normal vector
normal_vector = intercept'/norm(intercept);

% calculate the projection matrix
P = [intercept 0; eye(dim) normal_vector];

% project all Pareto-optimal points to the plane
plane_dist_vec = P^-1*[ones(1,size(pf,1)); pf'];
distance = plane_dist_vec(end,:)';
planeDescription = plane_dist_vec(1:end-1,:)';

% calculate the normalized directions in the plane from any point
planeVectorsAll = planeDescription-planeDescription(1,:);
planeVectorsCut = uniquetol(planeVectorsAll(2:end,:)./vecnorm(planeVectorsAll(2:end,:),2,2),0.05,'ByRows',true);

% get the first direction of the plane vectors as the first base direction
planeVectors = planeVectorsCut(1,:)/norm(planeVectorsCut(1,:));

% calculate the orthonormal base in the plane to get the new representation
% using the matrix-definition of the cross product
for i = 2:dim-1
    planeVectors = [planeVectors; ParetoController.crossn([planeVectors;normal_vector';planeVectorsCut(i+1:dim-1,:)])];
end

% get the plane representation of the projected Pareto-optimal points
dimReducMatrix = [eye(dim-1);zeros(1,dim-1)];
planeRepresentation = (planeVectorsAll*dimReducMatrix)/(planeVectors*dimReducMatrix);

% apply the boundary algorithm to fronts with dim equal 3 or 4
if dim == 2
    [~,borderPoints(1)] = min(planeRepresentation);
    [~,borderPoints(2)] = max(planeRepresentation);
elseif dim == 3 || dim == 4
    borderPoints = boundary(planeRepresentation,1)';
else
    error("not suited for 4+ dimensions");
end

end