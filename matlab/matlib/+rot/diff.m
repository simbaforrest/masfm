function err=diff(Ra,Rb,unit)
% return 3d or 2d rotation's different
% Ra<3x3|4x1|3x1/2x2|1x1>, Rb<3x3|4x1|3x1/2x2|1x1>: 3d rotation
%   matrices|quaternions|rodrigues vector/2d rotation|rotation angle
% err<1x1,radian>: how many radians/degrees to rotation from Ra to Rb
%   (or equally: Rb to Ra)
% unit['r'/'d']: radian/degree, default 'r'
assert(all(size(Ra)==size(Rb)));
nRa=numel(Ra);
if nRa==3 % rvec -> R
	Ra=rot.rodrigues(Ra);
	Rb=rot.rodrigues(Rb);
end

if nRa==9 % R diff
	err=acos((trace(Ra'*Rb)-1)*0.5);
elseif all(size(Ra)==[4,1]) || all(size(Ra)==[1,4]) %quat diff
	cs = 2*dot(Ra,Rb)^2-1;
	if abs(cs)>1 %due to numerical error, cs might be slightly larger than 1
		%e.g. when qa=[ 0.00688739 -0.99996772 -0.00171941  0.00376439]
		%diff(qa,qa) will give a cs slightly larger than 1
		cs = sign(cs);
	end
	err=acos(cs);
elseif all(size(Ra)==[2,2])
  err=acos(dot(Ra(:,1),Rb(:,1)));
elseif nRa==1
  err=mod((Ra-Rb)+pi,2*pi)-pi;
else
	error('[rot.diff] Ra/Rb must be 3x3, 4x1, 3x1, 2x2 or 1x1!');
end

if nargin==3 && unit=='d'
  err = err*180.0/pi;
end
end
