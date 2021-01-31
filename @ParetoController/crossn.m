function normalvec = crossn(RowVectorMatrix)
% n-dimensional cross product 
%   The matrix definition of the cross product is applied to calculate the
%   cross product between n-1 n-dimensional row vectors stored in
%   RowVectorMatrix
%
%   Input:
%       RowVectorMatrix: matrix of row vectors to calculate the cross-product for
%        
%   Output:
%       normalvec: n-dimensional vector perpendicular to the vectors in RowVectorMatrix

normalvec = zeros(1, size(RowVectorMatrix,2));

ObjDirections=sym('f',[1,size(RowVectorMatrix,2)]);
CrossProdMat=vertcat(RowVectorMatrix,ObjDirections);
SumNotation=det(CrossProdMat);

[val,objective] = coeffs(SumNotation,ObjDirections);

normalvec(ismember(ObjDirections,objective)) = val;

end