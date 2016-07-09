function vs=skew(v)
% convert between 3x1 vector and its cross-product matrix
% v<3x1 or 3x3>: the vector or the matrix
% vs<3x3 or 3x1>: the matrix or the vector
[m,n]=size(v);
if m*n==3
  vs=[
    0,-v(3),v(2);
    v(3),0,-v(1);
    -v(2),v(1),0];
elseif m==3 && n==3
  vs=[v(3,2);v(1,3);v(2,1)];
else
  error('v should be either 3x1 or 3x3!');
end
end