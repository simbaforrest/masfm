function [h,p]=ellipsoid(c, E, r, varargin)
% plot (p-c)^T*E*(p-c)=r^2, either as 3D ellipsoid
% or as 2D ellipse
% c<3x1|2x1>: center of ellipsoid/ellipse
% E<3x3|2x2>: ellipsoid/ellipse
% r<1x1>: radius
if length(c)==3 % 3D ellipsoid
  assert(size(E,1)==3 && size(E,2)==3);
  [Q,D]=eig(E);
  if prod(diag(D))<=0
    warning('matlib:warn','E is not PD!');
  end
  [x,y,z]=sphere(16);
  sz=size(x);
  clr=z;
  p=[x(:)';y(:)';z(:)']*r;
  p=Q*diag(1./sqrt(diag(D)))*p;
  x=reshape(p(1,:)+c(1),sz);
  y=reshape(p(2,:)+c(2),sz);
  z=reshape(p(3,:)+c(3),sz);
  h=mesh(x, y, z, clr, varargin{:});
  alpha(0.3); %see occluded part also
elseif length(c)==2 % 2D ellipse
  assert(size(E,1)==2 && size(E,2)==2);
  [Q,D]=eig(E);
  if prod(diag(D))<=0
    warning('matlib:warn','E is not PD!');
  end
  th=pi*(0:360)/180.0;
  p=Q*diag(1./sqrt(diag(D)))*[r*cos(th); r*sin(th)];
  p=[p(1,:)+c(1);p(2,:)+c(2)];
  h=plot(p(1,:),p(2,:), varargin{:});
else
  error('only 2D ellipse or 3D ellipsoid can be visualized');
end
end