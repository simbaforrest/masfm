function Y=rigidTransform(R,t,X)
% euclidian transformation
% Yi=R*Xi+t, i=1...n
% X<3xn|2xn>: 3D|2D points to be transformed
% R<3x3|2x2>, t<3x1|2x1>: rotation and translation
% Y<3xn|2xn>: output 3D|2D points
assert(size(R,1)==size(X,1) && size(R,1)==size(t,1));

Y = R*X+repmat(t,1,size(X,2));
end