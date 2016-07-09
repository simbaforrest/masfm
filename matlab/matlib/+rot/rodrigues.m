function r=rodrigues(R)
% convert between rotation matrix R and angle-axis representation
% R <3x3|3x1/2x2|1x1>: rotation matrix (3x3) or angle-axis form (3x1)
% r <3x1|3x3/1x1|2x2>: the other form
nR=numel(R);
if nR==9 %->angle-axis
  % reference: http://en.wikipedia.org/wiki/Axis_angle#Log_map_from_SO.283.29_to_so.283.29
  theta=acos((trace(R)-1)/2.0);
  k=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/(2*sin(theta));
  r=theta*k/norm(k);
elseif nR==3 %->rotation matrix
  % reference: http://en.wikipedia.org/wiki/Rodrigues'_rotation_formula#Conversion_to_rotation_matrix
  theta=norm(R);
  if theta<1e-8 %too small, return I
    r=eye(3);
    return;
  end
  kx=skew(R/theta);
  r=eye(3)+kx*sin(theta)+(1-cos(theta))*kx*kx;
elseif nR==4 % 2D rotation
  r=atan2(R(2,1),R(1,1));
elseif nR==1 % 2D rotation angle
  r=rot.e2R(R);
else
  error('input size must be either 3x3, 3x1, 2x2 or 1x1!');
end
end
