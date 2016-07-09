function world(scale)
if nargin<1; scale=1; end;
vz.frame(eye(3),zeros(3,1),'name','world','scale',scale);
end