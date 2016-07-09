function [Rab,Tab,err]=rigidRegistration(Pa,Pb)
% find [Rab,Tab] such that Pb=Rab*Pa+Tab
% Pa<3xn>
% Pb<3xn>
% err<3xn>: registration residual
assert(size(Pa,1)==3 && size(Pb,1)==3);
Ca = mean(Pa,2);
Cb = mean(Pb,2);

Qa = Pa-repmat(Ca,1,size(Pa,2));
Qb = Pb-repmat(Cb,1,size(Pb,2));

K = Qa*Qb';
[U,~,V]=svd(K);

Rab=U*[1 0 0;0 1 0;0 0 det(U*V')]*V';
Rab=Rab';
Tab=Cb-Rab*Ca;

Pd=Pb-Rab*Pa-repmat(Tab,1,size(Pa,2));
err=Pd;%norm(reshape(Pd,numel(Pd),1));
end