function p=T2p(T)
[R,t]=T2Rt(T);
p=[rot.rodrigues(R);t(:)];
end