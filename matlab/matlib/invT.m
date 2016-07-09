function Ti=invT(T)
% T<4x4>: 3D euclidean transformation
% Ti<4x4>: inverse of T
[R,t]=T2Rt(T);
[Ri,ti]=invRt(R,t);
Ti=Rt2T(Ri,ti);
end