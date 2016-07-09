function h=line(vertex_start, vertex_end, varargin)
% vertex<dimxn>: dim=2 or 3
assert(size(vertex_start,2)==size(vertex_end,2));
dim=size(vertex_start,1);
if dim==2
  tmp=arrayfun(@(sx,sy,ex,ey) plot([sx,ex],[sy,ey],varargin{:}),...
    vertex_start(1,:), vertex_start(2,:),...
    vertex_end(1,:), vertex_end(2,:));
elseif dim==3
  tmp=arrayfun(@(sx,sy,sz,ex,ey,ez) plot3([sx,ex],[sy,ey],[sz,ez],varargin{:}),...
    vertex_start(1,:), vertex_start(2,:), vertex_start(3,:),...
    vertex_end(1,:), vertex_end(2,:), vertex_end(3,:));
else
  error('size(vertex,1) should be either 2 or 3!');
end
if nargout>0
  h=tmp;
end
end
