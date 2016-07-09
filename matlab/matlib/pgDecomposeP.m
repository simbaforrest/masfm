function [K,R,t,Rraw]=pgDecomposeP(P,p10OR11)
% decomposition of camera projection matrix P
% P <3x4>: camera projection matrix
% p10OR11 [scalar,10|11]: use 10|11 parameters decomposition
% K <3x3>: calibration matrix, [a 0 c;0 b d;0 0 1]|[a e c;0 b d;0 0 1]
% R <3x3>: strict rotation matrix \in SO(3), by polar decomposition of Rraw
% t <3x1>: translation vector, P=K[Rraw,t]
% Rraw <3x3>: raw rotation matrix contaminated by noise, if p10OR11==11 then Rraw==R
if nargin<2
  p10OR11=10;
end

if p10OR11==10
  [K,R,t,Rraw]=pgDecomposeP10(P);
else
  [K,R,t]=pgDecomposeP11(P);
  Rraw=R;
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [K,R,t]=pgDecomposeP11(P)
% 11 parameters decomposition of 2D camera projection matrix P<3x4>
% K=[a e c;0 b d;0 0 1]
% R<3x3>: rotation matrix
% t<3x1>: translation vector, P=K[R,t]
P=P/norm(P(3,1:3));
if(det(P(1:3,1:3))<0)
  P=-P;
end
r789=P(3,1:3);
t3=P(3,4);
d=dot(P(2,1:3),r789);
b=norm(P(2,1:3)-d*r789);
r456=(P(2,1:3)-d*r789)/b;
r123=cross(r456,r789);
a=dot(P(1,1:3),r123);
e=dot(P(1,1:3),r456);
c=dot(P(1,1:3),r789);
t2=(P(2,4)-d*t3)/b;
t1=(P(1,4)-e*t2-c*t3)/a;
R=[r123;r456;r789];
t=[t1;t2;t3];
K=[a e c;0 b d;0 0 1];
end

function [K,R,t,Rraw]=pgDecomposeP10(P)
% 10 parameters decomposition of 2D camera projection matrix P<3x4>
% K=[a 0 c;0 b d;0 0 1]
% R<3x3>: rotation matrix, if Rraw=U*S*V' then R=U*V'
% t<3x1>: translation vector, P=K[Rraw,t]
P=P/norm(P(3,1:3));
if(det(P(1:3,1:3))<0)
  P=-P;
end
r789=P(3,1:3);
t3=P(3,4);
c=dot(P(1,1:3),r789);
d=dot(P(2,1:3),r789);
a=norm(P(1,1:3)-c*r789);
b=norm(P(2,1:3)-d*r789);
K=[a 0 c;0 b d;0 0 1];
Rraw=[(P(1,1:3)-c*r789)/a;
  (P(2,1:3)-d*r789)/b;
  r789];
% polar decomposition to get a valid rotation matrix
[u,~,v]=svd(Rraw);
R=u*v';
t=[(P(1,4)-c*t3)/a;(P(2,4)-d*t3)/b;t3];
end