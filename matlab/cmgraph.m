function this=cmgraph(K, d, l)
%Create an empty cmgraph with given camera calibration (K,d) and half
%marker size (i.e., marker's black edge length is 2*l)
% the created cmgraph contains following fields:
% markers, views
%   struct array of nodes stores each marker or view as a node of the graph
%   each node contains the following fields:
%     p : [a,b,c,x,y,z]', the pose of the node in world coordinate frame,
%         i.e. T(p) = [R(a,b,c),t(x,y,z);0,1]
%     Cp: Cov[p], i.e., covariance matrix of p
%     vmeids: edges' ids linked to this node
%     name: name of the node
% edges
%   struct array of edges stores the view->marker observations,
%   i.e., 2D coordinates of four corners of a marker
%   each edge contains the following fields:
%     vid: id of the view
%     mid: id of the marker
%     u: 2x4 matrix, each column is a 2D corner position
% calib
%   calibration of the camera, containing the following field:
%   k: [fx,fy,cx,cy], focal length and principal point
%   d: [k1, k2], distortion parameters
%   K: [fx,0,cx;0,fy,cy;0,0,1], matrix version of k
%   Ck: Cov[k], covariance matrix of k (not used right now)
%   Cd: Cov[d], covariance matrix of d (not used right now)
% name2mid
%   map from marker name to marker id
% marker_x, marker_X
%   all marker's local 2D and 3D coordinates
%
% Author: Chen Feng <simbaforrest@gmail.com>

if ~exist('K','var') || isempty(K)
  K=eye(3);
end
if ~exist('d','var') || isempty(d)
  d=zeros(2,1);
end
if ~exist('l','var') || isempty(l)
  l=1;
end

this.markers = createEmptyNodeArray();
this.views = createEmptyNodeArray();

%TODO: each edge should be associate with a weight/covariance
this.edges = createEmptyEdgeArray();

if isvector(K)
  this.calib.K = pgk2K(K);
  this.calib.k = K;
else
  this.calib.K = K;
  this.calib.k = pgk2K(K);
end
this.calib.d = d;
this.calib.Ck=eye(length(this.calib.k));
this.calib.Cd=eye(length(this.calib.d));

if exist('OCTAVE_VERSION', 'builtin') == 0
  this.name2mid = containers.Map('KeyType','char','ValueType','int32');
end

this.marker_x = [-1,-1;1,-1;1,1;-1,1]'*l;
this.marker_X = [[-1,-1;1,-1;1,1;-1,1]';zeros(1,4)]*l;
end

%% helpers
function nd = createEmptyNodeArray
nd = struct('name',{},'p',{},'Cp',{},'vmeids',{},'T',{});
end

function e = createEmptyEdgeArray
e = struct('vid',{},'mid',{},'u',{});
end