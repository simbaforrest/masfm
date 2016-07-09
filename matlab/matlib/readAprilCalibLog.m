function [K,distCoeffs]=readAprilCalibLog(fpath)
[folder,fname,~]=fileparts(fpath);
logscript = fullfile(folder,fname);
clear(logscript); % need to clear cache in matlab, see http://www.mathworks.com/matlabcentral/answers/102386
run(logscript);
end