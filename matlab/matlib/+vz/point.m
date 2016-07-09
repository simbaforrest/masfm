function h=point(vertex,varargin)
% vertex<dimxn>: dim=2 or 3
if mod(nargin,2)==1
  tmp=vz.linestrip(vertex,'.',varargin{:});
else
  tmp=vz.linestrip(vertex,varargin{:});
end
if nargout>0
  h=tmp;
end
end