function T=p2T(p)
R=rot.rodrigues(p(1:3));
t=p(4:6);
T=Rt2T(R,t);
end