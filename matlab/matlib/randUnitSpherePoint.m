function p=randUnitSpherePoint(n,z_range,th_range)
% sample 3d points uniformly distributed on an unit sphere (or part of)
% n<1x1>: number of samples
% [z_range<2x1>=[-1;1]]: z range of sample points, e.g. z \in [0;1]
% [th_range<2x1>=[0;2*pi]]: theta range, e.g. theta \in [0;pi]
% p<3xn>: sample points
if nargin<1
  n=1;
end
if nargin<2
  z_range=[-1;1];
end
if nargin<3
  th_range=[0;2*pi];
end
p=method3(n,z_range,th_range);
end

%% algorithms
function p=method1(n)
% Reference
% 1959.CACM.MullerM.A note on a method for generating points uniformly on n-dimensional spheres
p=randn(3,n);
pn=sqrt(sum(p.^2));
p=p./repmat(pn,3,1);
end

function p=method2(n)
% Reference
% http://mathworld.wolfram.com/SpherePointPicking.html
u=rand(1,n);
v=rand(1,n);
theta=2*pi*u;
phi=acos(2*v-1);
p=[
  cos(theta).*sin(phi);
  sin(theta).*sin(phi);
  cos(phi)
  ];
end

function p=method3(n,z_range,th_range)
% Reference
% http://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d#44691
th=rand(1,n)*abs(th_range(2)-th_range(1))+min(th_range(1),th_range(2));
z=rand(1,n)*abs(z_range(2)-z_range(1))+min(z_range(1),z_range(2));
len=sqrt(1-z.*z);
p=[
  len.*cos(th);
  len.*sin(th);
  z];
end