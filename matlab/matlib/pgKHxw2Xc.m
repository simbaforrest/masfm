function [Xc,Rwc,twc,err]=pgKHxw2Xc(K,H,xw)
% compute the 3D coordinates (expressed in camera frame) of world plane
% points xw  based on K and H
% K<3x3>: calibration matrix
% H<3x3>: homography matrix
% xw<dimxn>: 2D points on world plane, dim=2 or 3; if dim=2 then it will be
%   homogenize first
% Xc<3xn>: points on world plane expressed in camera frame
% Rwc<3x3>, twc<3x1>: pose of camera in world frame
% err<3xn>: registration error
% Note: this method is different from pgKH2Rt
dim=size(xw,1);
if dim==2
  Xw=pgHomogenize(xw);
elseif dim==3
  Xw=xw;
  xw=xw(1:2,:);
else
  error('size(xw,1) should be either 2 or 3');
end

A=K\H;%inv(K)*H
s=sqrt(norm(A(:,1)*norm(A(:,2))));
if A(3,3)<0 %ensure Xc(3,:)>0
  A=-A;
end
Xc=A*Xw/s;
[Rwc,twc,err]=rigidRegistration([xw;zeros(1,size(xw,2))], Xc);
end