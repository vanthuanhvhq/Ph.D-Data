% Analyses based on csv file
% Draw x,y robot position
% Draw theta robot position
%%
filename = ['./06-May-2022 15_27_analyze/picture13.png'];
info = imfinfo(filename);
xImage = info.Width;
yImage = info.Height;

filename = ['./06-May-2022 15_27_analyze/vysledek_15_27']; % CSV file
rawData = readmatrix(filename);
numPoints = height(rawData);
xImage = repmat( xImage, [1,numPoints] ).';
yImage = repmat( yImage, [1,numPoints] ).';

positionX = rawData(:,7);
positionY = yImage-rawData(:,8);
thetaR = rawData(:,10);   % deg
no = rawData(:,1);

%%
% figure
plot(positionX,positionY)
grid on;
ylabel('y axes (pt)')
xlabel('x axes (pt)')
xlim([0 info.Width])
ylim([0 info.Height])

%%
% Top two plots
tiledlayout(2,2)
nexttile
plot(positionX,positionY)
grid on;
ylabel('y axes [pt]')
xlabel('x axes [pt]')
xlim([0 info.Width])
ylim([0 2000])
% ylim([0 info.Height])
nexttile
plot(no,thetaR)
grid on;
ylabel('theta axes [degree]')
xlabel('sample [pt]')
xlim([0 100])
ylim([-180 180])
% Plot that spans
nexttile([1 2])
plot(positionX,positionY)