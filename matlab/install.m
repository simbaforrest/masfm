function install(doTest)
if ~exist('doTest','var'); doTest=false; end;

addpath(pwd);
addpath(genpath(fullfile(pwd,'matlib')));
addpath(fullfile(pwd,'bin'));

if ~doTest; return; end;
if ~exist('data','dir')
  error('cannot find data folder under current working directory!');
end

try
  [K,d]=run_calibration('photo://data/calibration/*.png');
  fprintf('calibration test: pass\n');
  G=run_masfm(K,d,158/2,'photo://data/masfm/*.png','Tag36h11.1');
  fprintf('masfm test: pass\n');
  plot_cmgraph(G);
  fprintf('masfm plot test: pass\n');
  if exist('OCTAVE_VERSION', 'builtin') == 0
    trackball on;
  end
catch err
  fprintf('basic test failed:\n%s\n', err.message);
end

fprintf('================================\n');
try
  atf=AprilTagFinder; %#ok<NASGU>
  clear('atf');
  fprintf('real-time demo test: pass\n');
catch err
  fprintf('real-time demo test failed:\n%s\n', err.message);
end

close all;
end