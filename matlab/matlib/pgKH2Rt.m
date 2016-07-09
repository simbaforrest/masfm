function [R,t]=pgKH2Rt(K,H,doPolarDecomp)
% compute plane pose (R and t) from K and Homography
if numel(H)==9
  if(nargin<3), doPolarDecomp=false; end
  [R,t]=pgKH2Rt3x3(K,H,doPolarDecomp);
elseif numel(H)==4
  [R,t]=pgKH2Rt2x2(K,H);
else
  error('[pgKH2Rt] can only handle 2D or 1D homography!');
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D homography
function [R,t]=pgKH2Rt3x3(K,H,doPolarDecomp)
% H = const * K[r1,r2,t]
A = K\H;%inv(K) * H;
s1 = norm(A(:,1));
s2 = norm(A(:,2));
s = sqrt(s1*s2);
A = A/s; % get rid of const by apply ||r1||=1 && ||r2||=1
if A(3,3)<0 %ensure t(3)>0, since we assume world frame's origin is in front of the camera
  A=-A;
end
if abs(A(3,3))<1e-8
  warning('matlib:warning','[pgKH2Rt] t(3) close to zero, please check!');
end
r1 = A(:,1);
r2 = A(:,2);
t  = A(:,3);
r3 = cross(r1,r2);
R = [r1,r2,r3];
if(doPolarDecomp)
  [u,s,v]=svd(R);
%   fprintf('%f\n',sum(diag(s))-3);
  R = u*v';
end
end

% 1D homography
function [R,t]=pgKH2Rt2x2(K,H)
assert(K(2,1)==0);
f=K(1,1)/K(2,2);
c=K(1,2)/K(2,2);
if H(2,2)<0 %ensure t(2)>0 since assume world frame's origin in front of the camera
  H=-H;
end
h1=H(1,1); h2=H(1,2); h3=H(2,1); h4=H(2,2);
lambda=sqrt(h3^2+((h1-c*h3)/f)^2);
th=atan2(h3, (h1-c*h3)/f);
ty=h4/lambda;
tx=(h2-c*h4)/lambda/f;
R=[cos(th),-sin(th);sin(th),cos(th)];
t=[tx;ty];
end
% 7:04 PM 9/25/13 Chen Feng <simbaforrest@gmail.com>
% - renamed from RTfromKH to pgKH2Rt
% 3:33 PM 11/5/13 Chen Feng <simbaforrest@gmail.com>
% - add support for both 1D and 2D camera