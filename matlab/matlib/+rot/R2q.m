function q=R2q(R)
% convert from rotation matrix to quaternion (xi,yj,zk,w)
% R <3x3>: rotation matrix, v_new=R*v_old
% q <4x1>: quaternion
% See also rot.q2R
r=rot.rodrigues(R);
theta=norm(r);
if abs(theta)>eps
  k=r/theta;
  q=[k*sin(theta/2.0);cos(theta/2.0)];
else
  q=[0;0;0;1];
end
end