function fpt=pgH2fpt(H)
% H<3x3>: homography matrix
% fpt<8x1>: 4pt parameterization of homography matrix H, where
%  fpt=[u1;v1;u2;v2;u3;v3;u4;v4] and [ui;vi;1]~H[xi;yi;1] where
%  x=[-1,1,1,-1], y=[-1,-1,1,1], i.e. by the homography matrix H
%  (u1,v1)<->(-1,-1)
%  (u2,v2)<->( 1,-1)
%  (u3,v3)<->( 1, 1)
%  (u4,v4)<->(-1, 1)
%
% See also pgfpt2H
X=[-1,-1;1,-1;1,1;-1,1]';
U=pgHX(H,X);
fpt=U(:);
end

%% below are non-canonical X, see also pgfpt2H
% canonical X are special cases in which l=b=-1, r=t=1
% X=[l,b;r,b;r,t;l,t]';