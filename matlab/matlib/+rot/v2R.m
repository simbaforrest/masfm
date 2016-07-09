function R=v2R(v_from, v_to)
% return rotation matrix R such that v_to=R*v_from
% v_from, v_to<3x1>
% R<3x3>
% Reference:
% http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula#Conversion_to_rotation_matrix
assert(size(v_from,1)==3 && size(v_to,1)==3);
v_from=v_from/norm(v_from);
v_to=v_to/norm(v_to);
ks=cross(v_from, v_to); %note here the ks=k*sin(t)
s=norm(ks);
c=dot(v_from,v_to);
kx=skew(ks);
R=eye(3)+kx+kx^2*(1-c)/s^2;
end
