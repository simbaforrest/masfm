function R=q2R(q)
% convert from quaternion (xi,yj,zk,w) to rotation matrix
% q <4x1>: quaternion
% R <3x3>: rotation matrix, v_new=R*v_old
% See also rot.R2q
s=norm(q(1:3));
if s>eps
  theta=2*atan2(s,q(4));
  k=q(1:3)/s;
  R=rot.rodrigues(theta*k);
else
  R=eye(3);
end
end