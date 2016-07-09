function H = pgKRt2H(K,R,t)
% assume world point [X,Y,Z] with all Z=0, the perspective 2D camera can be
% represented by a homography; this equation can be used for 1D camera also
% K <3x3|2x2>: camera calibration matrix
% R <3x3|2x2>: rotation matrix, Rwc
% t <3x1|2x1>: translation vector, twc
% H <3x3|2x2>: homography equivalent to perspective camera K*[R,t]
H = K*[R(:,1:end-1),t];
end