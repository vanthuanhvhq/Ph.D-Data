clear all;
cam = gigecam
s = serialport("COM15", 115200);
cam.Timeout = 25;

countFinal = 170;
countInitial = 1;

%% Serial Line Config
% Relative position od Region of Interest RoI
centerLocationRoI = [1951 344];% [544 274]
wayPoint=[669 304; 610 1592; 1902 1596; 1951 344; 1951 344];
wayPointNo = size(wayPoint);
wayPointNo = wayPointNo(1,1);
text_strWP = {'1', '2', '3', '4', '4' };
box_color = {'yellow'};
actualWP = 1;
sizeRoI = [400 400];

%% Declaration of cell for final Image - problem
img = zeros(2056,2464,3,'uint8');
imgFinalMerge = cell(countInitial,countFinal);
imgRAW = cell(countInitial,countFinal);
imgFinalMerge(countInitial:countFinal)={img};
imgRAW(countInitial:countFinal)={img};

%% [rows columns numberOfColorBands] = size(objectImage);
tmp = size(img);
rows = tmp(:,1);
columns = tmp(:,2);
numberOfColorBands = tmp(:,3);

rows2 = sizeRoI(:,1);
columns2 = sizeRoI(:,2);
numberOfColorBands2 = 3;
xRAW = 10;   % position
yRAW = 10;
xGreen = 10;   % position
yGreen = 850;
xRed = 10;   % position
yRed = 430;
positionText = [1070 1460];

%% Speed function
ts = .3;
speed = zeros(countFinal,5);
speed(1,:)=[0 centerLocationRoI 0 0];

%% Create directory for result photo according the actual date and time
currDate = strrep(datestr(datetime), ':', '_');
currDate = currDate(1:17);
currTime = currDate(13:17);
if not(isfolder(currDate))
    mkdir(currDate);
end
pathfilename = ['./',currDate,'/vysledek_',currTime,'.csv']; % CSV file

%%  Declaration of CSV file

FileMesage = {'No.','time','red X','red Y','green X','green Y',...
    'RoI X','RoI Y', 'thetaR [rad]',...
    'thetaR [deg]','WP','dist2WP [pt]','dist2WP [m]','theta2WP [rad]','theta2WP [deg]'};

writecell(FileMesage,pathfilename,'WriteMode','append')

%% Main loop to navigate robot to the waypoints
for i = countInitial:countFinal
i
if (actualWP >= wayPointNo+1)     % &  (dist2WP <= 100)
  %% Serial port rutine - Stop Robot
    m = "|" + num2str(0) + ";" + num2str(0)+"/";
    writeline(s,m);
    n = s.NumBytesAvailable(); %Number of bytes available to be read, returned as a double.
    while s.NumBytesAvailable() == n
    end
    s.readline(); 
    break
end

[objectFrame, ts] = snapshot(cam);
imgRAW{i} = objectFrame;    % be saved later as an image%%  !! To save RAW image from Camera !!

%% Cut Region of Interest from whole image
origin2RoI = [centerLocationRoI(1,1)-sizeRoI(1,1)/2  centerLocationRoI(:,2)-sizeRoI(:,2)/2];
objectImageZeros = zeros(sizeRoI(:,1),sizeRoI(:,2),3,'uint8');     % black image in RGB color palette 
objectRegion = [centerLocationRoI(1:1)-sizeRoI(1:1)/2 centerLocationRoI(:,2)-sizeRoI(:,2)/2 (sizeRoI(:,1)) (sizeRoI(:,2))];
objectRegion = max(objectRegion,0) ;    % avoid negative value
objectImageCrop = imcrop(objectFrame,objectRegion);

tmp = size(objectImageCrop);
r1 = 1;
c1 = 1;
r2 = r1 + tmp(:,2,:) - 1;         % rows2
r2 = min([r2 tmp(:,2,:)]);
c2 = c1 + tmp(:,1,:) - 1;
c2 = min([c2, tmp(:,1,:)]);
objectImageZeros(1:(c2), 1:(r2),:) = objectImageCrop(1:(c2), 1:(r2),:);

objectImageRAW = objectImageZeros; 
%%  Maske two circles/markers
maskaGreen = maskGreenLab2(objectImageRAW);
maskaRed   = maskRedLab(objectImageRAW);
% centre of Gravity of binary image RED
% in seens that it is workong just for squar RiO 
grayImage = maskaRed;

sumgx = 0;
sumgy = 0;
sumg = 0;
for col = 1 : (sizeRoI(1,2))
    for row = 1 : (sizeRoI(1,1))
        gl = double(grayImage(row, col));
        sumg = sumg + gl;
        sumgx = sumgx + col * gl;
        sumgy = sumgy + row * gl;
    end
end

if sumg == 0
    sumg = Inf
    disp('Missing point')
    i
end

tmpx = sumgx / sumg;
tmpy = sumgy / sumg;
COGred =[tmpx,tmpy];

% Convert binary image to RGB + save each picture
labeledImage = 255 * repmat(uint8(grayImage), 1, 1, 3);
objectImageRed = insertMarker(labeledImage,COGred,'+','Color','red','Size',5);

%% 
% centre of Gravity of binary image GREEN
% in seens that it is workong just for squar RiO 

grayImage = maskaGreen;

sumgx = 0;
sumgy = 0;
sumg = 0;
for col = 1 : (sizeRoI(1,2))
    for row = 1 : (sizeRoI(1,1))
        gl = double(grayImage(row, col));
        sumg = sumg + gl;
        sumgx = sumgx + col * gl;
        sumgy = sumgy + row * gl;
    end
end

if sumg == 0
    sumg = Inf
    disp('Missing point')
    i
end

tmpx = sumgx / sumg;
tmpy = sumgy / sumg;
COGgreen =[tmpx,tmpy];

% Convert binary image to RGB + save each picture
labeledImage = 255 * repmat(uint8(grayImage), 1, 1, 3);
objectImageGreen = insertMarker(labeledImage,COGgreen,'+','Color','green','Size',5);

%% Positon of both markers
relativeOrigin = [objectRegion(1) objectRegion(2)];
positionRed = relativeOrigin + COGred;
positionGreen = relativeOrigin + COGgreen;

objectImage = insertMarker(objectFrame,positionRed,'+','Color','yellow','Size',5);
objectImage = insertMarker(objectImage,positionGreen,'x-mark','Color','yellow','Size',5);
%% Robot theta - orientation
% must be improved
thetaR = atan2((positionRed(:,2)-positionGreen(:,2)), (positionRed(:,1)-positionGreen(:,1)));

thetaRdeg = rad2deg(thetaR); % result in degree

%%
centerLocationRoI = (positionRed + positionGreen)/2;

objectImage = insertMarker(objectImage,centerLocationRoI,'circle','Color','red','Size',5);

%% Calculate robot relative position to waypoints

dist2WP = sqrt((wayPoint(actualWP,1)-centerLocationRoI(1,1))^2+(wayPoint(actualWP,2)-centerLocationRoI(1,2))^2);

dist2WPm = dist2WP*2.815/1800;

% PROBLEM
theta2WP = atan2((centerLocationRoI(1,2)-wayPoint(actualWP,2)), (centerLocationRoI(1,1)-wayPoint(actualWP,1)));
theta2WP = thetaR - theta2WP;

if theta2WP > pi
    theta2WP = -(2*pi-theta2WP);
end
if theta2WP < -pi
    theta2WP = (2*pi+theta2WP);
end

theta2WPdeg = rad2deg(theta2WP);
%% add waypoints to the final image
objectImage = insertMarker(objectImage,wayPoint,'x-mark','Color','magenta','Size',45);

%% Add text of d and theta
% Text description
text_str = ['WP: ' num2str(actualWP), '  delta: ' num2str(dist2WPm,'%0.3f'), ' m  theta2WP: ', num2str(theta2WPdeg,'%0.0f'), '°  theta2WP: ', num2str(theta2WP,'%0.3f'), 'rad' ];
box_color = ['black'];
objectImage = insertText(objectImage,positionText,text_str,'FontSize',32,'BoxColor',...
    box_color,'BoxOpacity',0.4,'TextColor','white');

%%  Waypoints labeling
distance = [-22 20];
position = wayPoint + distance;
objectImage = insertText(objectImage,position,text_strWP,'FontSize',32,'BoxColor',...
box_color,'BoxOpacity',0.4,'TextColor','black');

%%  Merge detailed photos into big picture 
% objectRegion
objectImage = insertShape(objectImage,'Rectangle',objectRegion,'Color','black','Opacity',0.4,'LineWidth',5);
% Green detail
% Determine the pasting boundaries.
r1 = yGreen;
c1 = xGreen;
r2 = r1 + rows2 - 1;
r2 = min([r2 rows]);
c2 = c1 + columns2 - 1;
c2 = min([c2, columns]);
objectImage(r1:r2, c1:c2,:) = objectImageGreen(1:(r2-r1+1), 1:(c2-c1+1),:);
objectRegion = [xGreen, yGreen, rows2, columns2];
objectImage = insertShape(objectImage,'Rectangle',objectRegion,'Color','green','Opacity',0.4,'LineWidth',5);

% Red detail
r1 = yRed;
c1 = xRed;
r2 = r1 + rows2 - 1;
r2 = min([r2 rows]);
c2 = c1 + columns2 - 1;
c2 = min([c2, columns]);
objectImage(r1:r2, c1:c2,:) = objectImageRed(1:(r2-r1+1), 1:(c2-c1+1),:);
objectRegion = [xRed, yRed, rows2, columns2];
objectImage = insertShape(objectImage,'Rectangle',objectRegion,'Color','red','Opacity',0.4,'LineWidth',5);

% RAW detail
r1 = yRAW;
c1 = xRAW;
r2 = r1 + rows2 - 1;
r2 = min([r2 rows]);
c2 = c1 + columns2 - 1;
c2 = min([c2, columns]);
objectImage(r1:r2, c1:c2,:) = objectImageRAW(1:(r2-r1+1), 1:(c2-c1+1),:);
objectRegion = [xRAW, yRAW, rows2, columns2];
objectImage = insertShape(objectImage,'Rectangle',objectRegion,'Color','black','Opacity',0.4,'LineWidth',5);

imgFinalMerge{i} = objectImage;

FileMesage = {i,ts,round(positionRed(:,1)),round(positionRed(:,2)),round(positionGreen(:,1)),round(positionGreen(:,2)),...
    round(centerLocationRoI(:,1)),round(centerLocationRoI(:,2)),thetaR,...
    round(thetaRdeg),actualWP,round(dist2WP),dist2WPm,theta2WP,round(theta2WPdeg)};

writecell(FileMesage,pathfilename,'WriteMode','append')
%% Speed function
speed(i+1,:) = 0;   % was ts

deltaVx = (speed(i,2) - centerLocationRoI(1,1));
deltaVy = (speed(i,3) - centerLocationRoI(1,2));

speed(i+1,:)=[0 centerLocationRoI deltaVx deltaVy];% deltaVx deltaVy]

centerLocationRoI(1,1) = round(centerLocationRoI(1,1) - deltaVx);
centerLocationRoI(1,2) = round(centerLocationRoI(1,2) - deltaVy);

if actualWP == 4 && dist2WP <= 100 
    actualWP = actualWP + 1  
end

if dist2WP <= 120        % treashold is 1 pt  = 1,56 mm
    actualWP = actualWP + 1  
end

if i > 1
    %% Serial port rutine
    m = "|" + num2str(dist2WPm) + ";" + num2str(theta2WP)+"/";
    writeline(s,m);
    n = s.NumBytesAvailable(); %Number of bytes available to be read, returned as a double.
    while s.NumBytesAvailable() == n
    end
    s.readline();

end

end


%% Serial port rutine - Stop Robot
m = "|" + num2str(0) + ";" + num2str(0)+"/";
writeline(s,m);
n = s.NumBytesAvailable(); %Number of bytes available to be read, returned as a double.
while s.NumBytesAvailable() == n
end
s.readline();


%% Save all previous images - Final product
for ii = countInitial:countFinal
       ii 
    filename = ['./',currDate,'/','picture' num2str(ii) '.png'];
     imwrite(imgFinalMerge{ii}, filename);
end

%% Save all previous images - source image
for ii = countInitial:countFinal
    ii;
    filenameRAW = ['./',currDate,'/','raw' num2str(ii) '.png'];
%      imwrite(imgRAW{ii}, filenameRAW);       %%  !! To save RAW image from Camera !! 
end
