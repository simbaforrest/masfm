function R=e2R(r, seqStr)
% Rx      =e2R(pi,'x')
% Rx*Ry*Rz=e2R([pi,0,0],'xyz')
% Rz*Ry*Rz=e2R([pi/2,pi/3,0],'zyz')
%
% note: column vector and pre-multiplication convention
% i.e. v_new=R*v_old
% while matlab's angle2dcm in aero toolbox uses row vector
% and post-multiplication convenction: v_new=v_old*R
% see reference: http://en.wikipedia.org/wiki/Rotation_matrix#Ambiguities

if nargin~=2 && nargin~=1
  error('nargin should be 1 or 2: e2R(pi) or e2R(pi,''x'') or e2R([pi,0,0],''xyz'')');
end

if nargin==1 %handle 2D rotation
  R=rot(r(1));
  for i=2:length(r)
    R=R*rot(r(i));
  end
  return;
end

if length(r)~=length(seqStr)
  error('length of r should be equal to length of seqStr')
end

seqStr=lower(seqStr);
R=feval(['rot',seqStr(1)], r(1));
for i=2:length(seqStr)
  R=R*feval(['rot',seqStr(i)], r(i));
end
end

%%%%%%%%%%%%%%%%%%%%%
function R=rot(r)
c=cos(r); s=sin(r);
R=[c,-s;
  s,c];
end

function Rx=rotx(r)
c=cos(r); s=sin(r);
Rx=[1,0,0;
  0,c,-s;
  0,s,c];
end

function Ry=roty(r)
c=cos(r); s=sin(r);
Ry=[c,0,s;
  0,1,0;
  -s,0,c];
end

function Rz=rotz(r)
c=cos(r); s=sin(r);
Rz=[c,-s,0;
  s,c,0;
  0,0,1];
end