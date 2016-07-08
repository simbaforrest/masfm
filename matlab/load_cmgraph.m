function G = load_cmgraph(fname, G)
if ~exist('G','var') || isempty(G)
  G = cmgraph;
end

if ~exist('fname','var') || isempty(fname)
  [FileName,PathName]=uigetfile;
  if ~FileName
    return;
  end
  fname=fullfile(PathName,FileName);
end

fid=fopen(fname,'r');
if fid<0
  error('cannot open: %s',fname);
end
theCleanupObj=onCleanup(@()fclose(fid));

%load markers
nmarkers=fscanf(fid,'%d',1);
for mi=1:nmarkers
  marker = load_node(fid);
  
  G.markers(end+1)=marker;
  G.name2mid(marker.name)=length(G.markers);
end

%load views
nviews=fscanf(fid,'%d',1);
for vi=1:nviews
  view = load_node(fid);
  
  G.views(end+1)=view;
end

%load edges
nedges=fscanf(fid,'%d',1);
for ei=1:nedges
  e.vid=fscanf(fid,'%d',1)+1; %note: c++ use 0-based idx
  e.mid=fscanf(fid,'%d',1)+1;
  e.u=[]; %not saved yet
  
  G.edges(end+1)=e;
end
end

function nd = load_node(fid)
nd.name=fscanf(fid,'%s',1);
nd.p=fscanf(fid,'%f',6);
Cp=fscanf(fid,'%f',[6,6]);
nd.Cp=(Cp+Cp')/2; %make sure it is self-adjoint

nedges=fscanf(fid,'%d',1);
nd.vmeids=fscanf(fid,'%d',[nedges,1]);

nd.T=p2T(nd.p);
end