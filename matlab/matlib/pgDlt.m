function P=pgDlt(U,Xw)
% dlt algorithm for 2D|1D camera estimation or 2D homography estimation
% U <2xn|1xn|2xn|1xn>: image points
% Xw <3xn|2xn|2xn|1xn>: world points
% P <3x4|2x3|3x3|2x2>: projection matrix, either perspective or homography
[um,un]=size(U);
[xm,xn]=size(Xw);
assert(un==xn);
if um==2 && xm==3 % 2D camera
  P=pgDlt23(U,Xw);
elseif um==1 && xm==2 % 1D camera
  P=pgDlt12(U,Xw);
elseif um==2 && xm==2 % 2D homography
  P=pgDlt2(U,Xw);
elseif um==1 && xm==1 % 1D homography
  P=pgDlt1(U,Xw);
else
  error('input matrix dimension wrong!');
end
end

function P=pgDlt23(U,Xw)
% find 2D camera projection matrix P such that
% U=P*Xw ([U_i;1]~P*[X_i;1] \forall i\in[1,n])
% U<2xn>: measured 2d image points
% Xw<3xn>: world 3d points
%
% See also pgDlt12, pgDlt2, pgDecomposeP11, pgDecomposeP10
n=size(Xw,2);
assert(n==size(U,2) && size(U,1)==2 && size(Xw,1)==3);
if n<6
  error('at least 6 correspondences are needed for estimating 2D camera!');
end

[T,Xw]=pgConditioner(Xw);
[C,U]=pgConditioner(U);

A=zeros(2*n,12);
A(1:2:end,5:8)=-Xw';
A(1:2:end,9:12)=repmat(U(2,:)',1,4).*Xw';
A(2:2:end,1:4)=Xw';
A(2:2:end,9:12)=-repmat(U(1,:)',1,4).*Xw';

[~,~,v]=svd(A);

P=reshape(v(:,end),[4,3])';
P=C\P*T;
end

function H=pgDlt2(U,X)
% find 2D homography matrix H such that
% U=H*X ([U_i;1]~H*[X_i;1] \forall i\in[1,n])
% U<2xn>
% X<2xn>
%
% See also pgDlt12, pgDlt23
n=size(X,2);
assert(n==size(U,2) && size(U,1)==2 && size(X,1)==2);
if n<4
  error('at least 4 correspondences are needed for estimating 2D homography!');
end

[T,X]=pgConditioner(X);
[C,U]=pgConditioner(U);

A=zeros(2*n,9);
A(1:2:end,4:6)=-X';
A(1:2:end,7:9)=repmat(U(2,:)',1,3).*X';
A(2:2:end,1:3)=X';
A(2:2:end,7:9)=-repmat(U(1,:)',1,3).*X';

[~,~,v]=svd(A);

H=reshape(v(:,end),[3,3])';
H=C\H*T;
end

function H=pgDlt1(U,X)
% find 1D homography matrix H such that
% U=H*X ([U_i;1]~H*[X_i;1] \forall i\in[1,n])
% U<1xn>
% X<1xn>
%
% See also pgDlt2
n=size(X,2);
assert(n==size(U,2) && size(U,1)==1 && size(X,1)==1);
if n<3
  error('at least 3 correspondences are needed for estimating 1D homography!');
end

[T,X]=pgConditioner(X);
[C,U]=pgConditioner(U);

A=zeros(n,4);
A(:,1)=X(1,:)';
A(:,2)=ones(n,1);
A(:,3)=-X(1,:)'.*U(1,:)';
A(:,4)=-U(1,:)';

[~,~,v]=svd(A);

H=reshape(v(:,end),[2,2])';
H=C\H*T;
end

function P=pgDlt12(U,Xw)
% find 1D camera projection matrix P such that
% U=P*Xw ([U_i;1]~P*[X_i;1] \forall i\in[1,n])
% U <1xn>: 1D image points
% Xw <2xn>: 2D world points
% P <2x3>: 1D camera matrix, P=[f,c;0,1]*[cos(th),-sin(th),tx;sin(th),cos(th),ty]
% See also pgDlt2, pgDlt23
n=size(U,2);
assert(size(Xw,1)==2 && size(Xw,2)==n);
if n<5
  error('at least 5 correspondences are needed for estimating 1D camera!');
end

[T,hXw]=pgConditioner(Xw);
[C,hU]=pgConditioner(U);

A=zeros(n,6);
A(:,1:3)=repmat(hU(1,:)',1,3).*hXw';
A(:,4:6)=-hXw'; %should be hU(2,:) but it is just a row of ones

[~,~,v]=svd(A);

P=reshape(v(:,end),[3,2])';
P=C\P*T;
end