function K=pgk2K(k,nk)
% convert between K matrix and it's parameter form
% k <4x1/5x1>: parameter form of K matrix [fx;fy;cx;cy;a]
% nk [1x1]: 4 or 5, noting the output k's length, default is 4
% K <3x3>: K matrix, [fx,a,cx;0,fy,cy;0,0,1]
if nargin<2; nk=4; end;
[m,n]=size(k);
if m==3 && n==3
  K=[k(1,1);k(2,2);k(1,3);k(2,3);k(1,2)]/k(3,3);
  if nk==4; K=K(1:4); end;
elseif m==1 || n==1
  if numel(k)==4
    K=[k(1),0,k(3);0,k(2),k(4);0,0,1];
  elseif numel(k)==5
    K=[k(1),k(5),k(3);0,k(2),k(4);0,0,1];
  else
    error('k should be either 4x1 or 5x1 or 3x3!');
  end
end
end