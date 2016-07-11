function G=run_masfm(K,d,half_marker_length,url,fixed_marker_name)
if ~exist('fixed_marker_name','var')
  fixed_marker_name='null';
end

G=cmgraph(K,d,half_marker_length);

if ~exist('url','var') || isempty(url)
    url='camera://0';
end
exe=fullfile(pwd,'bin','masfm.exe');
tagfamiliesID='4';
outputDir=fullfile(pwd,'bin','log','masfm');
if exist(outputDir,'dir')
    rmdir(outputDir,'s');
end
mkdir(outputDir);

fprintf('================================\n');
my_setenv('input',url);
my_setenv('tagfamiliesID',tagfamiliesID);
my_setenv('output',fullfile(outputDir,'masfm.out'));
my_setenv('outputDir',outputDir);
my_setenv('marker_half_size',num2str(half_marker_length));
my_setenv('fixed_marker_name',fixed_marker_name);

k=pgk2K(K);
my_setenv('fx',num2str(k(1)));
my_setenv('fy',num2str(k(2)));
my_setenv('cx',num2str(k(3)));
my_setenv('cy',num2str(k(4)));
my_setenv('k1',num2str(d(1)));
my_setenv('k2',num2str(d(2)));

my_setenv('ImageSource:pause','false');
my_setenv('ImageSource:loop','false');

cmd=sprintf('%s %s',exe,fullfile(pwd,'bin','masfm.ini'));
fprintf('executing: %s\n', cmd);

[status, result]=system(cmd);
fid=fopen(fullfile(outputDir,'masfm.log'),'w');
if fid<0
  disp(result);
else
  fprintf(fid,'%s',result);
  fclose(fid);
end
if status~=0
    error(result);
end

G=load_cmgraph(fullfile(outputDir,'masfm.out'),G);
end

%%helper
function my_setenv(name, str)
setenv(['masfm_',name], str);
end