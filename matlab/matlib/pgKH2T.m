function T=pgKH2T(K,H,doPolarDecomp)
% See also pgKH2Rt
[R,t]=pgKH2Rt(K,H,doPolarDecomp);
T=[R,t;zeros(size(t))',1];
end