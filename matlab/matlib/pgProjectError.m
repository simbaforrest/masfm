function err=pgProjectError(varargin)
% interface for pgErrUHX, pgErrUPXw and pgErrUKRtXw and pgErrUKdRtXw
% err=pgProjectError(U,H,X,sORv)
% err=pgProjectError(U,P,Xw,sORv)
% err=pgProjectError(U,K,R,t,Xw,sORv), notice U should be undistorted
% err=pgProjectError(U,K,T,Xw,sORv), notice U should be undistorted
% err=pgProjectError(U,K,d,R,t,Xw,sORv), notice U should be distorted/raw
% err=pgProjectError(U,K,d,T,Xw,sORv), notice U should be distorted/raw
if nargin==5
  err=pgErrUKTXw(varargin{:});
elseif nargin==6
  if isvector(varargin{4}) % t
    err=pgErrUKRtXw(varargin{:});
  else
    err=pgErrUKdTXw(varargin{:});
  end
elseif nargin==7
	err=pgErrUKdRtXw(varargin{:});
else
  M=varargin{2};
  [m,n]=size(M);
  if m==n
    err=pgErrUHX(varargin{:});
  elseif m==n-1
    err=pgErrUPXw(varargin{:});
  else
    error('[pgProjectError error] unrecognized input argument');
  end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function err=pgErrUPXw(U,P,Xw,sORv)
% error of 2D or 1D camera projection (rms=sqrt(err^2/n))
%   [U_i;1]~P*[X_i;1], \forall i\in[1,n]
% U <2xn|1xn>: measured points on image
% P <3x4|2x3>: camera projection matrix
% Xw <3xn|2xn>: world (3D|2D) points to be projected on image
% sORv [char,'s'/'v']: return scalar error/error vector
% err <1x1/2nx1|nx1>: reprojection scalar error/error vector
%
% See also pgProject
assert(size(Xw,1)==size(P,1) && size(U,1)==(size(Xw,1)-1));

if nargin<4
  sORv='s';
end

if sORv=='s' % scalar error
  err=sqrt(sum(reshape(U-pgProject(P,Xw), numel(U),1).^2)/size(U,2));
else % error vector [err_x_1,err_y_1,err_x_2,err_y_2,...]
  err=reshape(U-pgProject(P,Xw), numel(U),1);
end
end

function err=pgErrUKdRtXw(U,K,d,R,t,Xw,sORv)
% error of 2D camera projection with distortion (rms=sqrt(err^2/n))
% U <2xn>: measured image points
% K <3x3>: calibration matrix
% d <8x1>: (k1,k2,p1,p2,k3,k4,k5,k6)
% R <3x3>: ^{c}R_{w}, each column is the world axis in camera coord frame
% t <3x1>: ^{c}t_{w}, world orgin in camera coord frame
% Xw <3xn>: world (3D) points to be projected
% sORv [char,'s'/'v']: return scalar error/error vector
% err <1x1/2nx1>: reprojection scalar error/error vector
%
% See also pgProject
assert(size(Xw,1)==size(K,1) && size(U,1)==(size(Xw,1)-1));

if nargin<6
  sORv='s';
end

if sORv=='s' % scalar error
  err=sqrt(sum(reshape(U-pgProject(K,d,R,t,Xw),numel(U),1).^2)/size(U,2));
else % error vector [err_x_1,err_y_1,err_x_2,err_y_2,...]
  err=reshape(U-pgProject(K,d,R,t,Xw),numel(U),1);
end
end

function err=pgErrUKdTXw(U,K,d,T,Xw,sORv)
% T<4x4>: [R,t;0,0,0,1]
err=pgErrUKdRtXw(U,K,d,T(1:3,1:3),T(1:3,4),Xw,sORv);
end

function err=pgErrUKRtXw(U,K,R,t,Xw,sORv)
% error of 2D or 1D camera projection (rms=sqrt(err^2/n))
%   [U_i;1]~[K*R*X_i+K*t], \forall i\in[1,n]
% U <2xn|1xn>: measured image points
% K <3x3|2x2>: calibration matrix
% R <3x3|2x2>: ^{c}R_{w}, each column is the world axis in camera coord frame
% t <3x1|2x1>: ^{c}t_{w}, world orgin in camera coord frame
% Xw <3xn|2xn>: world (3D|2D) points to be projected
% sORv [char,'s'/'v']: return scalar error/error vector
% err <1x1/2nx1|nx1>: reprojection scalar error/error vector
%
% See also pgProject
assert(size(Xw,1)==size(K,1) && size(U,1)==(size(Xw,1)-1));

if nargin<6
  sORv='s';
end

if sORv=='s' % scalar error
  err=sqrt(sum(reshape(U-pgProject(K,R,t,Xw),numel(U),1).^2)/size(U,2));
else % error vector [err_x_1,err_y_1,err_x_2,err_y_2,...]
  err=reshape(U-pgProject(K,R,t,Xw),numel(U),1);
end
end

function err=pgErrUKTXw(U,K,T,Xw,sORv)
% T<4x4>: [R,t;0,0,0,1]
err=pgErrUKRtXw(U,K,T(1:3,1:3),T(1:3,4),Xw,sORv);
end

function err=pgErrUHX(U,H,X,sORv)
% error of 2D or 1D homography projection (rms=sqrt(err^2/n))
% U <2xn|1xn>: measured projected points
% X <2xn|1xn>: points to be projected by homography H
% H <3x3|2x2>: homography matrix
% sORv [char,'s'/'v']: return scalar error/error vector
% err <1x1/2nx1|nx1>: reprojection scalar error/error vector
%
% See also pgProject
assert(size(U,1)==(size(H,1)-1) && size(U,1)==size(X,1));

if nargin<4
  sORv='s';
end

if sORv=='s' % scalar error
  err=sqrt(sum(reshape(U-pgProject(H,X),numel(U),1).^2)/size(U,2));
else % error vector [err_x_1,err_y_1,err_x_2,err_y_2,...]
  err=reshape(U-pgProject(H,X),numel(U),1);
end
end