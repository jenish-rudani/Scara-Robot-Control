close all; clear all; clc;

%% Global Variables
Fname_generatedPosition  = "X_vs_Y.csv";
Fname_inputToolPositions = "XY_InputParameters.csv";
colors = linspace(1,1,5);

%% Reading Files
generatedPositionData = readtable(Fname_generatedPosition);
inputToolPositionsData = readtable(Fname_inputToolPositions);

%% Retrieving Data Points
X_Data = table2array(generatedPositionData(:,1));
Y_Data = table2array(generatedPositionData(:,2));

enteredInputX_Data = table2array(inputToolPositionsData(:,1));
enteredInputY_Data = table2array(inputToolPositionsData(:,2));

%% Set Fixed GCF
x0=0;
y0=0;
width=1920;
height=1080;
set(gcf,'position',[x0,y0,width,height])

%% Embed Input Tool Positions on X-Y Position Plot

textCell = arrayfun(@(x,y) sprintf('(%3.2f, %3.2f)',x,y),enteredInputX_Data,enteredInputY_Data,'un',0);
scatter(enteredInputX_Data,enteredInputY_Data,50,'o') % Plot Input Parameters scatter
xlabel("X")
ylabel("Y")

for ii = 1:numel(enteredInputX_Data) 
    text(enteredInputX_Data(ii)+5, enteredInputY_Data(ii)+2,textCell{ii},'FontSize',15) 
end

hold on
h = animatedline;
axis([-400,400,-400,400])

for k = 1:length(X_Data)
    addpoints(h,X_Data(k),Y_Data(k));
    pause(0.02)
end
drawnow % draw final frame



