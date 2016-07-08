function [K,distCoeffs]=run_calibration(url)
if ~exist('url','var') || isempty(url)
    url='camera://0';
end
exe=fullfile('bin','AprilCalib.exe');
tagfamiliesID='4';
outputDir=fullfile('bin','log','AprilCalib');
if exist(outputDir,'dir')
    rmdir(outputDir,'s');
end

fprintf('================================\n');
cmd=sprintf('%s url=%s tagfamiliesID=%s outputDir=%s',...
    exe,url,tagfamiliesID,outputDir);
fprintf('executing: %s\n',cmd);
[status, result]=system(cmd);
if status~=0
    error(result);
end

logs=dir(fullfile(outputDir,'AprilCalib_log_*.m'));
if length(logs)<10
    error('please capture at least 10 images for calibration!\n');
end
lastLog=fullfile(outputDir,sprintf('AprilCalib_log_%05d.m',length(logs)-1));
fprintf('loading the last log: %s\n',lastLog);
[K,distCoeffs]=readAprilCalibLog(lastLog);
end