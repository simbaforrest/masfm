function [T,hX]=pgConditioner(X)
% condition X such that the center is at 0 and std is 1
% X<dimxn>: inhomogeneous points
% T<(dim+1)x(dim+1)>: Condition matrix
% hX<(dim+1)xn>
%
% See also pgDlt1, pgDlt2, pgDlt3
[dim,~]=size(X);
m=mean(X,2);
s=std(X,1,2); %std(X,flag,2), set flag=1(or 0) means normalize by N (or N-1)
s=s+(s==0); %handle cases where a row of X are all the same
T=[diag(1./s), -m./s;
  zeros(1,dim),1];
if nargout>1
  hX=T*pgHomogenize(X);
end
end