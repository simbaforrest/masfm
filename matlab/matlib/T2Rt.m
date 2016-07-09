function [R,t]=T2Rt(T)
R=T(1:end-1,1:end-1);
t=T(1:end-1,end);
end