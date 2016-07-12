function hframe=frame(R,t,varargin)
% plot reference frame

%handle varargin
assert(mod(length(varargin),2)==0);
opts=struct('name','',...
  'scale',1,...
  'linetype','-',...
  'linewidth',1,...
  'iscam',false,...
  'fovx',1,...
  'fovy',1,...
  'facecolor','r',...
  'fontsize',10,...
  'hittest','on');
for pair = reshape(varargin,2,[]) % pair is {key;value}
  key=lower(pair{1});
  opts.(key)=pair{2};
end

%handle hold on/off
prev_is_hold=ishold;
if ~prev_is_hold
  hold on;
end

if exist('OCTAVE_VERSION', 'builtin') == 0
  hframe=hgtransform('HitTest',opts.hittest);
else
  hframe=gcf;
end
addChild=@(hc) set(hc, 'Parent', hframe);

if length(t)==3 % plot 3d frame
  if numel(R)==3 % a rotation vector in angle-axis form theta*[kx,ky,kz]
    R=rot.rodrigues(R);
  end
  p=([zeros(3,1),R]*opts.scale+repmat(t,1,4))';
  addChild(...
    plot3(p([1 2],1),p([1 2],2),p([1 2],3),opts.linetype,'Color','r','LineWidth',opts.linewidth));
  addChild(...
    plot3(p([1 3],1),p([1 3],2),p([1 3],3),opts.linetype,'Color','g','LineWidth',opts.linewidth));
  addChild(...
    plot3(p([1 4],1),p([1 4],2),p([1 4],3),opts.linetype,'Color','b','LineWidth',opts.linewidth));
  if ~isempty(opts.name)
    addChild(text(p(1,1),p(1,2),p(1,3),opts.name,'FontSize',opts.fontsize));
  end
  if opts.iscam
    w=opts.fovx*opts.scale/2;
    h=opts.fovy*opts.scale/2;
    q=(R*[0,0,0;w,h,opts.scale;w,-h,opts.scale;-w,-h,opts.scale;-w,h,opts.scale]'+repmat(t,1,5))'; %image frustum
    idx=[2,3,4,5,2,1,3,4,1,5];
    addChild(...
      plot3(q(idx,1),q(idx,2),q(idx,3),opts.linetype,'Color','k','LineWidth',opts.linewidth));
    idx2=[2,3,4,5];
    addChild(...
      fill3(q(idx2,1),q(idx2,2),q(idx2,3),opts.facecolor,'FaceAlpha',0.8));
  end
elseif length(t)==2 % plot 2d frame
  if numel(R)==1 % a rotation angle theta
    R=[cos(R),-sin(R);sin(R),cos(R)];
  end
  p=([zeros(2,1),R]*opts.scale+repmat(t,1,3))';
  addChild(...
    plot(p([1 2],1),p([1 2],2),opts.linetype,'Color','r','LineWidth',opts.linewidth));
  addChild(...
    plot(p([1 3],1),p([1 3],2),opts.linetype,'Color','g','LineWidth',opts.linewidth));
  if ~isempty(opts.name)
    addChild(text(p(1,1),p(1,2),opts.name,'FontSize',opts.fontsize));
  end
  if opts.iscam
    w=opts.fovx*opts.scale/2;
    q=(R*[0,0;w,opts.scale;-w,opts.scale]'+repmat(t,1,3))'; %image triangle
    idx=[1,2,3,1];
    addChild(...
      plot(q(idx,1),q(idx,2),opts.linetype,'Color','k','LineWidth',opts.linewidth));
  end
else
  error('only 2D or 3D frame can be visualized!');
end

%handle hold on/off
if ~prev_is_hold
  hold off;
end
end