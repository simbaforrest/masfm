function plot_cmgraph(G,doNewFig,doPlotMarkers,doPlotEdges)
if ~exist('doNewFig','var'); doNewFig=true; end;
if ~exist('doPlotMarkers','var'); doPlotMarkers=true; end;
if ~exist('doPlotEdges','var'); doPlotEdges=1; end;

if doNewFig
  figure, rotate3d on, grid on, axis equal;
end

hold on;
sc = abs(G.marker_x(1,1)-G.marker_x(1,2));
if doPlotMarkers
  %markers
  nM = length(G.markers);
  for i=1:nM
    mn = G.markers(i);
    [R,t]=T2Rt(mn.T);
    thename = sprintf('m%d',sscanf(mn.name,'Tag36h11.%d'));
    vz.frame(R,t,'name',thename,'scale',sc,'linewidth',2);
    X=rigidTransform(R,t,G.marker_X);
    vz.linestrip(X(:,1:2), 'k');
    vz.linestrip(X(:,2:3), 'g');
    vz.linestrip(X(:,3:4), 'b');
    vz.linestrip(X(:,[4,1]), 'r');
    vz.ellipsoid(t,mn.Cp(4:6,4:6)\eye(3),3);
  end
end
if ~isempty(doPlotEdges)
  %views
  nV = length(G.views);
  viewflags=false(1,nV);
  viewflags(doPlotEdges)=true;
  for i=doPlotEdges%1:nV
    vn = G.views(i);
    [R,t]=T2Rt(vn.T);
    vz.frame(R,t,'name',vn.name,'scale',sc,'linewidth',1,'linetype',':');
    vz.ellipsoid(t,vn.Cp(4:6,4:6)\eye(3),3);
  end
  %edges
  M = length(G.edges);
  for j=1:M
    ed = G.edges(j);
    if ~viewflags(ed.vid); continue; end;
    vn = G.views(ed.vid);
    mn = G.markers(ed.mid);
    vz.line(vn.T(1:3,4), mn.T(1:3,4), 'k--');
  end
end
end