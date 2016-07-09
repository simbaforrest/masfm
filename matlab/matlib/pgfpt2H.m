function H=pgfpt2H(fpt)
% fpt<8x1>: 4pt parameterization of 2D homography matrix H, where
%  fpt=[u1;v1;u2;v2;u3;v3;u4;v4] and [ui;vi;1]~H[xi;yi;1] where
%  x=[-1,1,1,-1], y=[-1,-1,1,1], i.e. by the homography matrix H
%  (u1,v1)<->(-1,-1)
%  (u2,v2)<->( 1,-1)
%  (u3,v3)<->( 1, 1)
%  (u4,v4)<->(-1, 1)
% H<3x3>: homography matrix corresponding to fpt
%
% See also pgH2fpt
assert(length(fpt)==8);
u1=fpt(1); v1=fpt(2);
u2=fpt(3); v2=fpt(4);
u3=fpt(5); v3=fpt(6);
u4=fpt(7); v4=fpt(8);
h=[
 u1*u3*v2 - u2*u3*v1 - u1*u4*v2 + u2*u4*v1 - u1*u3*v4 + u1*u4*v3 + u2*u3*v4 - u2*u4*v3;
 u1*u3*v2 - u1*u2*v3 + u1*u2*v4 - u2*u4*v1 - u1*u3*v4 + u3*u4*v1 + u2*u4*v3 - u3*u4*v2;
 u1*u2*v3 - u2*u3*v1 - u1*u2*v4 + u1*u4*v2 - u1*u4*v3 + u3*u4*v1 + u2*u3*v4 - u3*u4*v2;
 u1*v2*v3 - u2*v1*v3 - u1*v2*v4 + u2*v1*v4 - u3*v1*v4 + u4*v1*v3 + u3*v2*v4 - u4*v2*v3;
 u3*v1*v2 - u2*v1*v3 + u1*v2*v4 - u4*v1*v2 - u1*v3*v4 + u4*v1*v3 + u2*v3*v4 - u3*v2*v4;
 u1*v2*v3 - u3*v1*v2 - u2*v1*v4 + u4*v1*v2 - u1*v3*v4 + u3*v1*v4 + u2*v3*v4 - u4*v2*v3;
                         u1*v3 - u3*v1 - u1*v4 - u2*v3 + u3*v2 + u4*v1 + u2*v4 - u4*v2;
                         u1*v2 - u2*v1 - u1*v3 + u3*v1 + u2*v4 - u4*v2 - u3*v4 + u4*v3;
                         u1*v2 - u2*v1 - u1*v4 + u2*v3 - u3*v2 + u4*v1 + u3*v4 - u4*v3
  ];
H=reshape(h,[3,3])';
end

%% below are equations for non-canonical 4pt parameterization of H
% in which the 4 base points are (l,b),(r,b),(r,t),(l,t) instead of
% (-1,-1),(1,-1),(1,1),(-1,1). This is more generalized equations
% h=[
%   -((b - t)*(u1*u3*v2 - u2*u3*v1 - u1*u4*v2 + u2*u4*v1 - u1*u3*v4 + u1*u4*v3 + u2*u3*v4 - u2*u4*v3))
%   ((l - r)*(u1*u2*v3 - u1*u3*v2 - u1*u2*v4 + u2*u4*v1 + u1*u3*v4 - u3*u4*v1 - u2*u4*v3 + u3*u4*v2))
%   (b*l*u1*u3*v2 - b*l*u2*u3*v1 - b*l*u1*u3*v4 + b*l*u3*u4*v1 + b*l*u2*u3*v4 - b*l*u3*u4*v2 - b*r*u1*u4*v2 + b*r*u2*u4*v1 + b*r*u1*u4*v3 - b*r*u3*u4*v1 - b*r*u2*u4*v3 + b*r*u3*u4*v2 - l*t*u1*u2*v3 + l*t*u2*u3*v1 + l*t*u1*u2*v4 - l*t*u2*u4*v1 - l*t*u2*u3*v4 + l*t*u2*u4*v3 + r*t*u1*u2*v3 - r*t*u1*u3*v2 - r*t*u1*u2*v4 + r*t*u1*u4*v2 + r*t*u1*u3*v4 - r*t*u1*u4*v3)
%   -((b - t)*(u1*v2*v3 - u2*v1*v3 - u1*v2*v4 + u2*v1*v4 - u3*v1*v4 + u4*v1*v3 + u3*v2*v4 - u4*v2*v3))
%   ((l - r)*(u2*v1*v3 - u3*v1*v2 - u1*v2*v4 + u4*v1*v2 + u1*v3*v4 - u4*v1*v3 - u2*v3*v4 + u3*v2*v4))
%   (b*l*u1*v2*v3 - b*l*u2*v1*v3 - b*l*u1*v3*v4 + b*l*u4*v1*v3 + b*l*u2*v3*v4 - b*l*u4*v2*v3 - b*r*u1*v2*v4 + b*r*u2*v1*v4 + b*r*u1*v3*v4 - b*r*u3*v1*v4 - b*r*u2*v3*v4 + b*r*u3*v2*v4 - l*t*u1*v2*v3 + l*t*u3*v1*v2 + l*t*u1*v2*v4 - l*t*u4*v1*v2 - l*t*u3*v2*v4 + l*t*u4*v2*v3 + r*t*u2*v1*v3 - r*t*u3*v1*v2 - r*t*u2*v1*v4 + r*t*u4*v1*v2 + r*t*u3*v1*v4 - r*t*u4*v1*v3)
%   -((b - t)*(u1*v3 - u3*v1 - u1*v4 - u2*v3 + u3*v2 + u4*v1 + u2*v4 - u4*v2))
%   -((l - r)*(u1*v2 - u2*v1 - u1*v3 + u3*v1 + u2*v4 - u4*v2 - u3*v4 + u4*v3))
%   (b*l*u1*v2 - b*l*u2*v1 - b*l*u1*v4 + b*l*u4*v1 + b*l*u2*v4 - b*l*u4*v2 - b*r*u1*v2 + b*r*u2*v1 + b*r*u1*v3 - b*r*u3*v1 - b*r*u2*v3 + b*r*u3*v2 - l*t*u1*v3 + l*t*u3*v1 + l*t*u1*v4 - l*t*u4*v1 - l*t*u3*v4 + l*t*u4*v3 + r*t*u2*v3 - r*t*u3*v2 - r*t*u2*v4 + r*t*u4*v2 + r*t*u3*v4 - r*t*u4*v3)
%   ];