function h=linestrip(vertex,varargin)
% vertex<dimxn>: dim=2 or 3
dim=size(vertex,1);
if dim==2
  tmp=plot(vertex(1,:),vertex(2,:),varargin{:});
elseif dim==3
  tmp=plot3(vertex(1,:),vertex(2,:),vertex(3,:),varargin{:});
else
  error('size(vertex,1) should be either 2 or 3!');
end
if nargout>0
  h=tmp;
end
end
