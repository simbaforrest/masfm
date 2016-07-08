function G=run_masfm(K,d,half_marker_length,url)
G=cmgraph(K,d,half_marker_length);

if ~exist('url','var') || isempty(url)
    url='camera://0';
end
exe=fullfile('bin','masfm.exe');
tagfamiliesID='4';
outputDir=fullfile('bin','log','masfm');
if exist(outputDir,'dir')
    rmdir(outputDir,'s');
end
mkdir(outputDir);

fprintf('================================\n');
setenv('masfm_input',url);
setenv('masfm_tagfamiliesID',tagfamiliesID);
setenv('masfm_output',fullfile(outputDir,'masfm.out'));
setenv('masfm_ImageSource:pause','false');
setenv('masfm_ImageSource:loop','false');
fprintf('executing: %s >c:/users/Chen/downloads/test.log\n',exe);
[status, result]=system(exe);
if status~=0
    error(result);
end

G=load_cmgraph(fullfile(outputDir,'masfm.out'),G);
end