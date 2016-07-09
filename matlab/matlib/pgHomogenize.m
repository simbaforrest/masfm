function hx=pgHomogenize(x)
% x <dimxn>    : points to be made homogeneous
% hx<(dim+1)xn>: homogeneous points(hx_i=[x_i;1], \forall i\in[1,n])
% See also pgInhomogenize
[~,n]=size(x);
hx=[x;ones(1,n)];
end