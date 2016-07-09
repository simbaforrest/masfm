function x=pgInhomogenize(hx)
% hx<dimxn>    : points to be made inhomogeneous
% x <(dim-1)xn>: inhomogeneous points(hx_i=[x_i;1], \forall i\in[1,n])
% See also pgHomogenize
[dim,~]=size(hx);
x=hx(1:dim-1,:)./repmat(hx(dim,:),dim-1,1);
end