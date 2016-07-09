function U=pgProject(varargin)
% interface for pgHX, pgPXw and pgKRtXw
% U=pgProject(H,X)
% U=pgProject(P,Xw)
% U=pgProject(K,T,Xw)
% U=pgProject(K,R,t,Xw)
% U=pgProject(K,d,T,Xw)
% U=pgProject(K,d,R,t,Xw)
switch(nargin)
  case 5
    U=pgKdRtXw(varargin{:});
  case 4
    if isvector(varargin{3}) %t
      U=pgKRtXw(varargin{:});
    else
      U=pgKdTXw(varargin{:});
    end
  case 3
    U=pgKTXw(varargin{:});
  case 2
    M=varargin{1}; %projection matrix, either P or H
    X=varargin{2}; %inhomogeneous points to be projected, either Xw or X
    [m,n]=size(M);
    if m==n % must be homography matrix H
      U=pgHX(M,X);
    elseif m+1==n % must be perspective projection matrix P
      U=pgPXw(M,X);
    else
      error('[pgProject error] unrecognized input argument');
    end
  otherwise
    error('[pgProject error] unrecognized input argument');
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function U=pgPXw(P,Xw)
% 2D or 1D camera projection
%   [U_i;1]~P*[X_i;1], \forall i\in[1,n]
% P <3x4|2x3>: camera projection matrix
% Xw <3xn|2xn>: inhomogeneous world (3D|2D) points to be projected on image
% U <2xn|1xn>: inhomogeneous projected points on image
%
% See also pgKRtXw, pgHX
assert(size(Xw,1)==size(P,1));
U=pgInhomogenize(P*pgHomogenize(Xw));
end

function U=pgKRtXw(K,R,t,Xw)
% project world points to image points (3D->2D|2D->1D)
%   [U_i;1]~[K*R*X_i+K*t], \forall i\in[1,n]
% K <3x3|2x2>: calibration matrix
% R <3x3|2x2>: ^{c}R_{w}, each column is the world axis in camera coord frame
% t <3x1|2x1>: ^{c}t_{w}, world orgin in camera coord frame
% Xw <3xn|2xn>: world (3D|2D) points to be projected on image
% U <2xn|1xn>: projected points on image
%
% note: This is K*R*Xw+K*t, which is NOT equal to K*(R*Xw+t)
% i.e. the order of multiplication matters in terms of
% error of 1e-14 order of magnitude
%
% See also pgErrUKRtXw
assert(size(Xw,1)==size(K,1));
U=pgInhomogenize(K*R*Xw+K*repmat(t,1,size(Xw,2)));
end

function U=pgKTXw(K,T,Xw)
% T<4x4>: [R,t;0,0,0,1]
% See also pgKRtXw
U=pgKRtXw(K,T(1:3,1:3),T(1:3,4),Xw);
end

function U=pgKdRtXw(K,d,R,t,Xw)
% projection with distortion
% d <8x1|7x1|...|1x1|0x1>: ([k1,k2,p1,p2,k3,k4,k5,k6])
% Reference:
% http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
assert(size(Xw,1)==size(K,1));

nDistCoeffs=numel(d);
if nDistCoeffs>=8
  d=d(:); d=d(1:8);
else
  d=[d(:);zeros(8-nDistCoeffs,1)];
end
D=num2cell(d);
[k1,k2,p1,p2,k3,k4,k5,k6]=deal(D{:});

Xc=R*Xw+repmat(t,1,size(Xw,2));
Xn=pgInhomogenize(Xc);
r2=sum(Xn.^2);
xn=Xn(1,:);
yn=Xn(2,:);
factor=(1+((k3*r2+k2).*r2+k1).*r2)./(1+((k6*r2+k5).*r2+k4).*r2);
xnyn=xn.*yn;
xnp=xn.*factor+2*p1*xnyn+p2*(r2+2*xn.^2);
ynp=yn.*factor+p1*(r2+2*yn.^2)+2*p2*xnyn;
U=[K(1,1)*xnp+K(1,2)*ynp+K(1,3);K(2,2)*ynp+K(2,3)]/K(3,3);
end

function U=pgKdTXw(K,d,T,Xw)
% T<4x4>: [R,t;0,0,0,1]
% See also pgKRtXw, pgKdRtXw
U=pgKdRtXw(K,d,T(1:3,1:3),T(1:3,4),Xw);
end

function U=pgHX(H,X)
% 2D or 1D homography projection
% X<2xn|1xn>: inhomogeneous points to be projected by homography H
% H<3x3|2x2>: homography matrix
% U<2xn|1xn>: inhomogeneous projected points ([U_i;1]~H*[X_i;1], \forall i\in[1,n])
%
% See also pgErrUHX
assert(size(X,1)==(size(H,1)-1));
U=pgInhomogenize(H*pgHomogenize(X));
end