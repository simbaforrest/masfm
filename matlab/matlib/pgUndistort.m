function U=pgUndistort(K,d,V)
% undistort image points V using intrinsics K and d
% K<3x3>: calibration matrix
% d<8x1|7x1|...|1x1|0x1>: ([k1,k2,p1,p2,k3,k4,k5,k6])
% V<2xN>: N image points
% Reference:
% http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
%
% See also pgKdRtXw in pgProject

nDistCoeffs=numel(d);
if nDistCoeffs==0; U=V; return; end;
if nDistCoeffs>=8
  d=d(:); d=d(1:8);
else
  d=[d(:);zeros(8-nDistCoeffs,1)];
end
D=num2cell(d);
[k1,k2,p1,p2,k3,k4,k5,k6]=deal(D{:});

X0=pgInhomogenize(K\pgHomogenize(V));%pgInhomogenize(inv(K)*pgHomogenize(V));
iter=5; % from opencv code
for i=1:iter
  r2=sum(V.^2);
  ifactor=(1+((k6*r2+k5).*r2+k4).*r2)./(1+((k3*r2+k2).*r2+k1).*r2);
  dV=[2*p1*V(1,:).*V(2,:)+p2*(r2+2*V(1,:).*V(1,:));
    p1*(r2+2*V(2,:).*V(2,:))+2*p2*V(1,:).*V(2,:)];
  V=(X0-dV).*repmat(ifactor,2,1);
end
U=pgInhomogenize(K*pgHomogenize(V));
end