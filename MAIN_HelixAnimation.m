% Helix visualization with projections as sine and cosine
%   Video: https://www.youtube.com/watch?v=2uESlFxGVDM
%
%   Order:
%   1) Commonly-altered settings, parameters for the helix
%   2) Make figure and plot objects (mostly empty)
%   3) Precalculate the helix and all other lines to be drawn.
%   4) ANIMATION LOOP
%       a) Interpolate and assign line data to the plot objects
%       b) Interpolate and update camera
%       b) Advance time and (if desired) record video frames
%
% @author Matthew Sheen, 2016
%

close all;

timefactor = 2; % Animation speed. This scaling between real time and the parameter "t" used to define the helix. Higher timefactor means faster.

% Stuff for making movies (if turned on -- warning, slow)
renderMP4 = false;
fRate = 60;
if renderMP4
    vw = VideoWriter('helixanimation.mp4','MPEG-4');
    vw.FrameRate = fRate;
    open(vw);
end

% Set up the equation for a parametric helix.
r = 1;
slope = 0.4;
tLow = 0;
tHigh = 60*pi;
tRes = 0.01;
spiralEqn = @(t)([r*cos(t), r*sin(t), slope*t]);

% New figure
fig = figure;
fig.Position = [100,50,1920,1080];
fig.Color = [1 1 1];
hold on; % Keep hold on until all shapes have been added.

% Plot for the helix itself
spiralPlot = plot3(0,0,0,'k');
spiralPlot.LineWidth = 1.5;
spiralPt = plot3(0,0,0,'.k');
spiralPt.MarkerSize = 20;

% For plane 1
x1 = [0 0 1]; % basis vectors for the new plane. Could align arbitrarily if desired.
y1 = [0 1 0];

bottomLeftCorner1 = [-r*1.5 -r*1.8 0];
planeXExtent1 =  tHigh;
planeYExtent1 = 4*r;

planeCoords1 = [bottomLeftCorner1;
    bottomLeftCorner1 + x1*planeXExtent1;
    bottomLeftCorner1 + x1*planeXExtent1 + y1*planeYExtent1;
    bottomLeftCorner1 + y1*planeYExtent1;
    bottomLeftCorner1];

plane1 = patch(planeCoords1(:,1),planeCoords1(:,2),planeCoords1(:,3),[0.8 0 0]); % Make the red plane
plane1.FaceAlpha = 0.2;
plane1.EdgeAlpha = 0.2;

% Projections onto plane 1
planewave1 = plot3(0,0,0,'r'); % The full projection
planewavept1 = plot3(0,0,0,'.r'); % The "cursor" which is the active drawing point projected
planewavept1.MarkerSize = 12;
planewaveconnect1 = plot3(0,0,0,':r'); % A dotted line that connects the projection to the actual helix

% For plane 2
x2 = [0 0 1]; % basis vectors for the new plane.
y2 = [1 0 0];

bottomLeftCorner2 = [-r*1.8 -r*1.5 0];
planeXExtent2 =  tHigh;
planeYExtent2 = 4*r;

planeCoords2 = [bottomLeftCorner2;
    bottomLeftCorner2 + x2*planeXExtent2;
    bottomLeftCorner2 + x2*planeXExtent2 + y2*planeYExtent2;
    bottomLeftCorner2 + y2*planeYExtent2;
    bottomLeftCorner2];

plane2 = patch(planeCoords2(:,1),planeCoords2(:,2),planeCoords2(:,3),[0 0 0.8]); % Make the blue plane
plane2.FaceAlpha = 0.2;
plane2.EdgeAlpha = 0.2;

% Projections onto plane 2
planewave2 = plot3(0,0,0,'b');
planewavept2 = plot3(0,0,0,'.b');
planewaveconnect2 = plot3(0,0,0,':b');
planewavept2.MarkerSize = 12;

% Make sure everything fits in view (could be done better)
axis([-2*r,2*r,-2*r,2*r,tLow - (tHigh-tLow)*0.1,tHigh + (tHigh-tLow)*0.1]);
axis equal
hold off % Done with new lines/patches

% Fix up the axes (mostly disable visual stuff)
ax = fig.Children;
ax.Visible = 'off';
ax.Projection = 'orthographic'; % Perspective view actually screws things up -- the waves don't line up with the projection planes.
ax.Box = 'off';
ax.ClippingStyle = 'rectangle';

camva(40); % Set the camera view angle to something reasonable.

%%%%% Precompute all the lines -- We'll only interpolate inside the
%%%%% animation loop

tRange = transpose(tLow:tRes:tHigh); % Range of the parameter in the helix equation. Trying to keep everything as columns here.
timeRange = tRange/timefactor; % Same thing, but scaled by a 'fastforward' factor
endtime = tHigh/timefactor;

% Coordinates of the helix
spCoords = spiralEqn(tRange);

% Coordinates of the projection on plane 1
wave1coords = repmat(spCoords*x1',[1,3]).*repmat(x1,[size(spCoords,1),1]) + repmat(spCoords*y1',[1,3]).*repmat(y1,[size(spCoords,1),1]) + repmat(dot(cross(x1,y1),bottomLeftCorner1)*cross(x1,y1),[size(spCoords,1),1]);

% Coordinates of the projection on plane 2
wave2coords = repmat(spCoords*x2',[1,3]).*repmat(x2,[size(spCoords,1),1]) + repmat(spCoords*y2',[1,3]).*repmat(y2,[size(spCoords,1),1]) + repmat(dot(cross(x2,y2),bottomLeftCorner2)*cross(x2,y2),[size(spCoords,1),1]);

tic % Start the timer
currentTime = toc;

frameCount = 0;

% Animation loop -- display frames as often as possible. Interpolate
% precomputer line data and assign to the plot objects.
while currentTime < endtime
    
    % If you close the window early, stop trying to do stuff and make
    % errors.
    if ~ishandle(fig)
        break;
    end
    
    
    %%%%%% Coordinate Updates %%%%%%
    
    % Update the spiral
    % Tracer point -- interpolating to make this smooth
    spPt = (interp1(timeRange,spCoords,currentTime)); % Interpolate in time to find the tracer pt's value.
    spiralPt.XData = spPt(1);
    spiralPt.YData = spPt(2);
    spiralPt.ZData = spPt(3);
    
    % Entire spiral -- nearest neighbor
    spIdx = find(timeRange < currentTime); % We want to display the line of all points up to this point in time too. Since the resolution is good, we won't bother interpolating. Just nearest neighbor.
    spiralPlot.XData = spCoords(spIdx,1);
    spiralPlot.YData = spCoords(spIdx,2);
    spiralPlot.ZData = spCoords(spIdx,3);
    
    % Update the wave on plane1
    % Tracer point -- interpolating to make this smooth
    wPt1 = interp1(timeRange,wave1coords,currentTime);
    planewavept1.XData = wPt1(1);
    planewavept1.YData = wPt1(2);
    planewavept1.ZData = wPt1(3);
    
    % Entire wave -- again just doing nearest neighbor
    planewave1.XData = wave1coords(spIdx,1);
    planewave1.YData = wave1coords(spIdx,2);
    planewave1.ZData = wave1coords(spIdx,3);
    
    planewaveconnect1.XData = [wPt1(1),spPt(1)];
    planewaveconnect1.YData = [wPt1(2),spPt(2)];
    planewaveconnect1.ZData = [wPt1(3),spPt(3)];
    
    % Update the wave on plane2
    % Tracer point -- interpolating to make this smooth
    wPt2 = interp1(timeRange,wave2coords,currentTime);
    planewavept2.XData = wPt2(1);
    planewavept2.YData = wPt2(2);
    planewavept2.ZData = wPt2(3);
    
    % Entire wave -- again just doing nearest neighbor
    planewave2.XData = wave2coords(spIdx,1);
    planewave2.YData = wave2coords(spIdx,2);
    planewave2.ZData = wave2coords(spIdx,3);
    
    planewaveconnect2.XData = [wPt2(1),spPt(1)];
    planewaveconnect2.YData = [wPt2(2),spPt(2)];
    planewaveconnect2.ZData = [wPt2(3),spPt(3)];
    
    
    %%%% Timings and transitions for cameras and objects fading in %%%%
    
    % This is inside the loop since some of the targets are state-based and
    % some are constant
    
    % Interpolate the alpha values on the planes to get them to "fade-in"
    % Plane 1
    plane1AlphaTiming = [0;8;
        12;endtime];
    plane1Alpha = [0;0
        0.2;0.2];
    % Plane 2
    plane2AlphaTiming = [0;40;
        44;endtime];
    plane2Alpha = [0;0
        0.2;0.2];
    
    % Camera
    % Camera positions, targets, up-vectors are interpolated in time to get
    % nice smooth transitions.
    
    camTimings = [0; 15; % Initial view
        20; 30; % Zoom to perpendicular plane1, scroll up to tracer point
        32; 33; % Stay at plane 1 for a sec
        38; 44; % Go back to some angled view
        50; 56; % Perpendicular to plane 2
        68; 68.5; % Hold at some ortho view
        88; endtime]; % Watch the "unit-circle" at the end
    
    camTargets = [0 0 spPt(3); 0 0 spPt(3); 
        0 0 spPt(3)-8; 0 0 spPt(3); 
        0 0 spPt(3); 0 0 spPt(3); 
        0 0 spPt(3); 0 0 spPt(3);
        0 0 spPt(3); 0 0 spPt(3);
        0 0 spPt(3); 0 0 spPt(3);
        0 0 spPt(3); 0 0 spPt(3)];
    
    camPositions = [12 28 spPt(3)+25; 12 28 spPt(3)+25;
        10 0 spPt(3)-8; 10 0 spPt(3);
        10 0 spPt(3); 10 0 spPt(3);
        0.6*[28 18 spPt(3)+50; 28 18 spPt(3)+50];
        0 10 spPt(3); 0 10 spPt(3);
        0.6*[10 10 spPt(3)+70; 10 10 spPt(3)+70];
        0 0 spPt(3)+10; 0 0 spPt(3)+10];
    
    camUps = [0 0 1; 0 0 1;
        0 -1 0; 0 -1 0;
        0 -1 0; 0 -1 0;
        0 0 1; 0 0 1;
        1 0 0; 1 0 0;
        1 1 0; 1 1 0;
        0 1 0; 0 1 0];
    
    % Transparency stuff -- plane 1
    % Interpolate the transparencies and assign them to the correct
    % objects.
    p1Alpha = interp1(plane1AlphaTiming,plane1Alpha,currentTime,'linear');
    plane1.FaceAlpha = p1Alpha;
    plane1.EdgeAlpha = p1Alpha;
    planewave1.Color(4) = min([1,p1Alpha*4]);
    planewave1.Color(4) = min([1,p1Alpha*4]);
    planewaveconnect1.Color(4) = min([1,p1Alpha*4]);
    
    if p1Alpha > 0.1
        planewavept1.Visible = 'on'; % Can't seem to set alpha on points. Just turning it visible/invisible
    else
        planewavept1.Visible = 'off';
    end
    
    % Transparency stuff -- plane 2
        % Interpolate the transparencies and assign them to the correct
    % objects.
    p2Alpha = interp1(plane2AlphaTiming,plane2Alpha,currentTime,'linear');
    plane2.FaceAlpha = p2Alpha;
    plane2.EdgeAlpha = p2Alpha;
    planewave2.Color(4) = min([1,p2Alpha*4]);
    planewave2.Color(4) = min([1,p2Alpha*4]);
    planewaveconnect2.Color(4) = min([1,p2Alpha*4]);
    
    if p2Alpha > 0.1
        planewavept2.Visible = 'on'; % Can't seem to set alpha on points. Just turning it visible/invisible
    else
        planewavept2.Visible = 'off';
    end
    
    % Camera stuff - Do camera interpolation and assign to this axis.
    campos(interp1(camTimings,camPositions,currentTime,'linear'));
    camtarget(interp1(camTimings,camTargets,currentTime,'linear'));
    camup(interp1(camTimings,camUps,currentTime,'linear'));
    
    drawnow;
    
    frameCount = frameCount + 1;
    % Time updates + video rendering if turned on
    if renderMP4
        frame = getframe(fig);
        writeVideo(vw,frame);
        
        currentTime = currentTime + 1/fRate; % If we're rendering for a video, step forward a fixed amount for the frame rate.
        if currentTime > endtime, close(vw); end
    else
        % Don't render a gif. Just do real-time run
        currentTime = toc; % If not rendering, update based on the clock so it runs in real-time.
    end
end