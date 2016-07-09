function rt=kh2rt(k,h)
K=[k(1),0,k(3);0,k(2),k(4);0,0,1];
H=reshape(h,3,3);
[R,t]=pgKH2Rt(K,H,true);
rt=[rot.rodrigues(R);t];
end