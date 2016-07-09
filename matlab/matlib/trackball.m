function trackball(varargin)
%TRACKBALL Interactively rotate the view of a 3D plot with a trackball.
%   If 'FindJobj' is installed, the view can also be scaled (2nd mouse
%   button) and dragged (3rd mouse button).
%
%   TRACKBALL ON turns on mouse-based 3D rotation.
%   TRACKBALL OFF turns if off.
%   TRACKBALL by itself toggles the state.
%
%   TRACKBALL(FIG,...) works on the figure FIG.
%
%   Example:
%     sphere(36);
%     axis equal
%     trackball on
%
%   Known Bugs:
%    - Rotation is not intuitive if axis are not equal, e.g.:
%       t = 0:pi/6:4*pi; [x,y,z] = cylinder(4+cos(t),30); surf(x,y,z)
%
%   See also ROTATE3D.
%
%   Author: Tobias Maier, tobias.maier@stud.unibas.ch
%   Contributor: Chen Feng <simbaforrest@gmail.com>
%                - improve function start
%                - add function pan
%                - switch operation order:
%                   left button: rotate
%                   right button: zoom
%                   left double-click: pan
%                   middle button: drag
%                - add temp visualization of camtarget between
%                   start to stop
%                - add toggletool button on default toolbar
%

if nargin >= 1
    arg = 'toggle';
    if ishandle(varargin{1})
        h = varargin{1};
        if ~ (isa(handle(h), 'figure'))
            error('MATLAB:trackball:InvalidHandle', 'Invalid handle');
        end
        if nargin > 1
            arg = varargin{2};
        end
    else
        % Get handle of curreng figure
        h = gcf;
        arg = varargin{1};
    end
    
    switch(lower(arg))
        case 'on',     state = 'on';
        case 'off',    state = 'off';
        case 'toggle', state = 'toggle';
        otherwise
            error('MATLAB:trackball:ActionStringUnknown', 'Unknown action string.');
    end
else
    h = gcf;
    state = 'toggle';
end

if strcmp(state, 'toggle')
    data = get(h, 'UserData');
    if isfield(data, 'trackball') && isfield(data.trackball, 'active')
        state = 'off';
    else
        state = 'on';
    end
end

% Set or unset callbacks
if strcmp(state, 'off')
    hstart  = '';
    hstop   = '';
    hjstart = '';
    
    data = get(h, 'UserData');
    data.trackball = [];
    data.trackball_gui.hcenter=[];
    set(h, 'Name', '');
    htoggle=findall(h,'tag','uitoggletool_trackball');
    if ~isempty(htoggle)
        set(htoggle,'State','off');
    end
    set(h, 'UserData', data);
else
    try
        zoom off;
        rotate3d off;
        pan off;
        plotedit off;
        brush off;
        
        htoggle=findall(h,'tag','uitoggletool_trackball');
        if ~isempty(htoggle)
            set(htoggle,'State','on');
        else %no toggle tool, create one then
            hToolbar = findall(h,'tag','FigureToolBar');
            if ~isempty(hToolbar)
                uitoggletool(hToolbar,...
                    'CData',rand(16,16,3),'Separator','on',...
                    'TooltipString','trackball',...
                    'ClickedCallback','trackball',...
                    'tag','uitoggletool_trackball',...
                    'State','on');
            end
        end
    catch e %#ok<NASGU>
    end
    hstart  = @start;
    hstop   = @stop;
    hjstart = {@start, h};
    
    data = get(h, 'UserData');
    data.trackball = [];
    data.trackball.active = 1;
    data.trackball_gui.hcenter=[];

    set(h, 'Name', 'trackball: left>rotate, right>zoom, middle>drag, double-click>pan');
    set(h, 'UserData', data);
end

try
    jhandle = findjobj(h);
    set(jhandle, 'MousePressedCallback', hjstart);
catch e
    if strcmp(e.identifier, 'MATLAB:UndefinedFunction')
        % no 'findjobj' installed
        set(h, 'WindowButtonDownFcn', hstart);
    else
        rethrow(e);
    end
end

set(h, 'WindowButtonUpFcn', hstop);

end


function start(h, eventData, handle)
if nargin == 3
    % If figure is docked findjobj returns the handle of the main
    % window, so check click comes from figure or somewhere else
    classpath = java.lang.String('com.mathworks.hg');
    class= h.getClass().getName();
    if ~class.startsWith(classpath);
        return
    end
    
    % Called by java callback, set correct handle
    h = handle;
    
    if eventData.isMetaDown
        % Right
        set(h,'WindowButtonMotionFcn',@tb_zoom);
%         fprintf('zoom\n');
    elseif eventData.isAltDown
        % Middle
        set(h,'WindowButtonMotionFcn',@tb_drag);
%         fprintf('drag\n');
    else
        % Left
        set(h,'WindowButtonMotionFcn',@tb_rotate);
%         fprintf('rotate\n');
    end
else
    % Rotate as default
    try
        selectionType=get(h,'SelectionType');
        switch selectionType
            case 'normal' %left click
                set(h,'WindowButtonMotionFcn',@tb_rotate);
%                 fprintf('rotate2\n');
            case 'alt' %right click
                set(h,'WindowButtonMotionFcn',@tb_zoom);
%                 fprintf('zoom2\n');
            case 'extend' % middle click
                set(h,'WindowButtonMotionFcn',@tb_drag);
%                 fprintf('drag2\n');
            case 'open' %double-click
                set(h,'WindowButtonMotionFcn',@tb_pan);
%                 fprintf('pan2\n');
        end
    catch e
        disp(e);
        set(h,'WindowButtonMotionFcn',@tb_rotate);
%         fprintf('rotate3\n');
    end
end

% Figure dimensions
pos = get(h,'Position');
width = pos(3);
height = pos(4);

center = [width height] / 2;
radius = sqrt(width^2 + height^2) / 2;

% Mouse xyz-coordinates on virtual sphere
xy = get(h, 'CurrentPoint');
xy_ctr = xy - center;
xyz = get_xyz(xy_ctr(1), xy_ctr(2), radius);

% Rotation from eye to world coordinates
v = normr(camtarget - campos);
u = normr(camup);
r = normr(cross(v, u));
R0 = [r; u; -v]';

% Save data
data = get(h, 'UserData');

data.trackball.xy0    = xy;
data.trackball.xyz0   = xyz;
data.trackball.width  = width;
data.trackball.height = height;
data.trackball.center = center;
data.trackball.radius = radius;

data.trackball.campos0    = campos;
data.trackball.camup0     = camup;
data.trackball.camtarget0 = camtarget;
data.trackball.camva0     = camva;
data.trackball.R0         = R0(1:3,1:3);
data.trackball.daspect0   = daspect;

data.haxis = gca;
data.axis0 = axis(gca);

target = camtarget;
needHold=~ishold;
if needHold; hold on; end;
if ~isempty(data.trackball_gui.hcenter)...
        && ishghandle(data.trackball_gui.hcenter)
    delete(data.trackball_gui.hcenter);
end
data.trackball_gui.hcenter=...
    plot3(target(1),target(2),target(3),'o',...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','r',...
    'MarkerSize',10);
uistack(data.trackball_gui.hcenter, 'top');
if needHold; hold off; end;

set(h, 'UserData', data);

end


function stop(h, ~)
set(h, 'WindowButtonMotionFcn', '');

data = get(h, 'UserData');
data.trackball = [];
data.trackball.active = 1;

delete(data.trackball_gui.hcenter);
data.trackball_gui.hcenter=[];

set(h, 'UserData', data);

end


function tb_rotate(h, ~)
% TODO: take care of data aspect ratio
data = get(h, 'UserData');
tb = data.trackball;

% Mouse xyz-coordinates on virtual sphere
xy = get(h, 'CurrentPoint') - tb.center;
xyz = get_xyz(xy(1), xy(2), tb.radius);

% Rotation axis in eye coordinates
v = cross(xyz, tb.xyz0);
v = normc(v(:));

% Transform rotation axis to world coordinates
v = tb.R0 * v;
v_world = v; %#ok<NASGU>

% Rotation angle
a = acos(tb.xyz0 * xyz');

% Rotate the camera
R = makehgtform('axisrotate', v, a);
campos(R(1:3,1:3) * tb.campos0');
camup(R(1:3,1:3) * tb.camup0');

end

function tb_pan(h, ~)
data = get(h, 'UserData');
tb = data.trackball;

% Relative Position
xy = get(h, 'CurrentPoint');
delta = tb.xy0 - xy;
dx = 2 * delta(1) / tb.width  ;
dy = 2 * delta(2) / tb.height  ;

c_up=camup;
c_dir=camtarget-campos;
c_dir=c_dir/norm(c_dir);
c_right=cross(c_dir,c_up);

selx=zeros(1,3);
[~,ix]=max(abs(c_right));
selx(ix)=sign(c_right(ix));

sely=zeros(1,3);
[~,iy]=max(abs(c_up));
sely(iy)=sign(c_up(iy));

newaxis=reshape(data.axis0,2,3)+dx*repmat(selx,2,1)+dy*repmat(sely,2,1);
axis(data.haxis,newaxis(:)');
end


function tb_drag(h, ~)
data = get(h, 'UserData');
tb = data.trackball;

% Relative Position
xy = get(h, 'CurrentPoint');
delta = tb.xy0 - xy;
dx = 2 * delta(1) / tb.width  ;
dy = 2 * delta(2) / tb.height  ;

% View, right and up directions
v = (tb.camtarget0 - tb.campos0) ./ tb.daspect0;
r = normr(cross(v, tb.camup0 ./ tb.daspect0));
u = normr(tb.camup0 ./ tb.daspect0);

% Field of view
fov = 2 * norm(v) * tan(tb.camva0/2*pi/180);

% Delta relative to field of view
delta = tb.daspect0 .* (fov/2 .* ((dx * r) + (dy * u)));

% Update camera and targets position
campos(tb.campos0 + delta);
camtarget(tb.camtarget0 + delta);

end


function tb_zoom(h, ~)
data = get(h, 'UserData');
tb = data.trackball;

% Use up/down to zoom
xy = get(h, 'CurrentPoint');
delta = 2 * (xy(2) - tb.xy0(2)) / tb.height;

% get linear zoom factor
zoom_factor = 1 + abs(delta);
if delta < 0
    zoom_factor = 1/zoom_factor;
end

% Calculate new camera view angle
view_angle = tb.camva0 * zoom_factor;

% View angle must not be to small
if view_angle < .1;
    view_angle = .1;
end

% Update the camera view angle
camva(view_angle);

end


function xyz = get_xyz(x, y, r)
% Returns xyz coordinates on a virtual sphere with radius r
x = x/r;
y = y/r;

if x^2 + y^2 < 1
    z = sqrt( 1 - (x^2 + y^2) );
    xyz = [x y z];
else
    xyz = normr([x y 0]);
end

end