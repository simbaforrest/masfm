function h=lineloop(vertex,varargin)
% vertex<dimxn>: dim=2 or 3
dim=size(vertex,1);
if dim==2
  h=plot([vertex(1,:),vertex(1,1)],[vertex(2,:),vertex(2,1)],varargin{:});
elseif dim==3
  h=plot3([vertex(1,:),vertex(1,1)],[vertex(2,:),vertex(2,1)],[vertex(3,:),vertex(3,1)],varargin{:});
else
  error('size(vertex,1) should be either 2 or 3!');
end
end
