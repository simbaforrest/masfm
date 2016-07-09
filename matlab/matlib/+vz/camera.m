function hcamera=camera(K,Rwc,twc,w,varargin)
% plot camera K*[Rwc,twc] in world frame with view
% (0...w]x(0...h] for 2D image or
% (0...w] for 1D image
% if 2D, the varargin{1} must be h
% K <3x3|2x2>: calibration matrix
% Rwc <3x3/3x1|2x2/1x1>: rotation matrix/rotation vector|rotation matrix/rotation angle
if length(twc)==3 % 3D camera, i.e. 2D image
  h=varargin{1};
  if numel(Rwc)==3 % a rotation vector in angle-axis form theta*[kx,ky,kz]
    Rwc=rot.rodrigues(Rwc);
  end
  hcamera=vz.frame(Rwc',-Rwc'*twc,varargin{2:end},'iscam',true,...
    'fovx',w/K(1,1),'fovy',h/K(2,2));
elseif length(twc)==2 % 2D camera, i.e. 1D image
  if numel(Rwc)==1 % a rotation angle theta
    Rwc=rot.e2R(Rwc);
  end
  hcamera=vz.frame(Rwc',-Rwc'*twc,varargin{:},'iscam',true,...
    'fovx',w/K(1,1));
else
  error('only 2D or 3D camera can be visualized!');
end
end