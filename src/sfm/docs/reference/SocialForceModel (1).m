

% Computation of Social Force Model
% Author: Alejandro Acosta Montilla

%------------------------------------------------------------------------%
clear all
close all
clc

%------------------------------------------------------------------------%

% Definition of initial parameters for the whole problem
% Coordinates
xini=[9,2]; % Starting robot position
xgoal=[2 9]; % Goal position for the robot
xpini=[3,8]; % Starting person position
% SFM models
aobs=1; % SFM value a for the object model
dobs=1; % SFM value d for the object model
bobs=0.5; % SFM value b for the object model
ap=1.5; % SFM value a for the person model
dp=1.5; % SFM value d for the person model
bp=1.3; % SFM value b for the person model
k=3; % Contant for the force applied by the goal
% Speeds
vrini=[-1,1]; % Initial robot speed
vpini=[1,-1]; % Initial person speed
vgoal=[0, 0]; % Final robot speed at goal   
%Sample time
Tr=0.5; % In seconds

% Question a): Only one obstacle on the way

obs1size=[4,4]; % Top left coordinate for the squared obstacle
obs1length=4; % Length size of the squared obstacle
obs1angle=0; % Define the orientation of the object with rescpect to world
dvo1=[0, 0]; % Vector of the distance to the obstacle
do1=0; % Distance to the obstacle

vrd=vrini; % Initial value for the vector speed as the initial speed
fr=zeros(1,2); % Forces (accelerations, mass equal to 1) applied on the robot 
xr=xini; % Initial value for the vector of positions as initial position

% Calculate initial distance to the goal
distpos=xgoal-xr;
goaldist=round(norm(distpos)); % Value to store the distance to the goal (initially big)

% Start the loop until the goal is reached
while(goaldist~=0)

    % Check the distance to the obstacles
    [do1,dvo1]=CheckMinDist(obs1size,obs1length,obs1length,obs1angle,xr(end,:));
    
    % Check distance to the goal and update vgoal at each step with the unitary direction vector
    distpos=xgoal-xr(end,:);
    vgoal=norm(vrini)*distpos/norm(distpos);
    
    % Get the value of all forces in the problem
    fgoal=k*(vgoal-vrd(end,:)); % Calculate the forces done by the goal 
    fro=aobs*(exp((dobs-do1)/bobs))*(dvo1/do1); % Calculate the force done by the obstacle

    fr=[fr; fgoal+fro]; % Add the forces or accelerations to the vector for that purpose

    % Get next the speed for the robot in each axis vx and vy
    vrd=[vrd; vrd(end,1)+fr(end,1)*Tr vrd(end,2)+fr(end,2)*Tr]; 
    
    % Calculate the new robot position
    xr=[xr; xr(end,1)+vrd(end,1)*Tr xr(end,2)+vrd(end,2)*Tr]
    
    % Check distance to the goal at that step
    goaldist=round(norm(xr(end,:)-xgoal));
end

% Plots the results for the robot movement for this exercise
figure(5)
title(["Designed robot trajectory with SFM with 1 obstacle, time step= 0.5 s";"Obstacle parameters: a_o_b_s_ = 1; b_o_b_s_ = 0.5; d_o_b_s_ = 1"])

% Create a rectangle to place the obstacle on the plot
obs1=[obs1size; obs1size+[0,obs1length]; obs1size+[obs1length,obs1length]; obs1size+[obs1length,0];obs1size];
patch(obs1(:,1),obs1(:,2),'c');
hold on
grid on

% Plot the robot trajectory along all the time (x on y and y on x)
plot(xr(:,2),xr(:,1),"Marker","x","LineStyle","-","MarkerSize",8,"Color","g")
plot(xr(1,2),xr(1,1),"Marker","o","Color","k","MarkerSize",10,"MarkerFaceColor","k")
plot(xgoal(1,2),xgoal(1,1),"Marker","square","MarkerSize",10,"Color","r","MarkerFaceColor","r")
legend("Obstacle 1","SFM robot trajectory","Initial point","Goal","location","best")

% Set the size of the axis as desired
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
axis([0 10 0 10])
xticks(0:1:10); yticks(0:1:10);
xlabel("y coordinates")
ylabel("x coordinates")

% Question b): Three obstacles on the way

% Definition of obstacle 1 position
obs2size=[4,1]; % Top left coordinate for the squared obstacle
obs2length=2; % Length size of the squared obstacle
obs2angle=0; % Define the orientation of the object with rescpect to world
dvo2=[0, 0]; % Vector of the distance to the obstacle
do2=0; % Distance to the obstacle

% Definition of obstacle 2 position
obs3size=[2,3]; % Top left coordinate for the squared obstacle
obs3length=2; % Length size of the squared obstacle
obs3angle=0; % Define the orientation of the object with rescpect to world
dvo3=[0, 0]; % Vector of the distance to the obstacle
do3=0; % Distance to the obstacle

% Definition of obstacle 3 position
obs4size=[3,5]; % Top left coordinate for the squared obstacle
obs4length=2; % Length size of the squared obstacle
obs4angle=0; % Define the orientation of the object with rescpect to world
dvo4=[0, 0]; % Vector of the distance to the obstacle
do4=0; % Distance to the obstacle

vrd_2=vrini; % Initial value for the vector speed as the initial speed
fr_2=zeros(1,2); % Forces (accelerations, mass equal to 1) applied on the robot 
xr_2=xini; % Initial value for the vector of positions as initial position

% Calculate initial distance to the goal
distpos_2=xgoal-xr_2;
goaldist_2=round(norm(distpos_2)); % Value to store the distance to the goal (initially big)

% Start the loop until the goal is reached
while(goaldist_2~=0)

    % Check the distance to the 3 obstacles
    [do2,dvo2]=CheckMinDist(obs2size,obs2length,obs2length,obs2angle,xr_2(end,:));
    [do3,dvo3]=CheckMinDist(obs3size,obs3length,obs3length,obs3angle,xr_2(end,:));
    [do4,dvo4]=CheckMinDist(obs4size,obs4length,obs4length,obs4angle,xr_2(end,:));
    
    % Check distance to the goal and update vgoal at each step with the unitary direction vector
    distpos_2=xgoal-xr_2(end,:);
    vgoal_2=norm(vrini)*distpos_2/norm(distpos_2);
    
    % Get the value of all forces in the problem
    fgoal_2=k*(vgoal_2-vrd_2(end,:)); % Calculate the forces done by the goal 
    fro_2_1=aobs*(exp((dobs-do2)/bobs))*(dvo2/do2); % Calculate the force done by the obstacle
    fro_2_2=aobs*(exp((dobs-do3)/bobs))*(dvo3/do3); % Calculate the force done by the obstacle
    fro_2_3=aobs*(exp((dobs-do4)/bobs))*(dvo4/do4); % Calculate the force done by the obstacle

    fr_2=[fr_2; fgoal_2+fro_2_1+fro_2_2+fro_2_3]; % Add the forces or accelerations to the vector for that purpose

    % Get next the speed for the robot in each axis vx and vy
    vrd_2=[vrd_2; vrd_2(end,1)+fr_2(end,1)*Tr vrd_2(end,2)+fr_2(end,2)*Tr]; 
    
    % Calculate the new robot position
    xr_2=[xr_2; xr_2(end,1)+vrd_2(end,1)*Tr xr_2(end,2)+vrd_2(end,2)*Tr]
    
    % Check distance to the goal at that step
    goaldist_2=round(norm(xr_2(end,:)-xgoal));
end

% Plots the results for the robot movement for this exercise
figure(6)
title(["Designed robot trajectory with SFM with 3 obstacles, time step= 0.5 s";"Obstacle parameters: a_o_b_s_ = 1; b_o_b_s_ = 0.5; d_o_b_s_ = 1"])

% Create a rectangle to place obstacle 1 on the plot
obs2=[obs2size; obs2size+[0,obs2length]; obs2size+[obs2length,obs2length]; obs2size+[obs2length,0];obs2size];
patch(obs2(:,2),obs2(:,1),'c');
hold on
grid on

% Create a rectangle to place obstacle 2 on the plot
obs3=[obs3size; obs3size+[0,obs3length]; obs3size+[obs3length,obs3length]; obs3size+[obs3length,0];obs3size];
patch(obs3(:,2),obs3(:,1),'c');

% Create a rectangle to place obstacle 3 on the plot
obs4=[obs4size; obs4size+[0,obs4length]; obs4size+[obs4length,obs4length]; obs4size+[obs4length,0];obs4size];
patch(obs4(:,2),obs4(:,1),'c');

% Plot the robot trajectory along all the time (x on y and y on x)
plot(xr_2(:,2),xr_2(:,1),"Marker","x","LineStyle","-","MarkerSize",8,"Color","g")
plot(xr_2(1,2),xr_2(1,1),"Marker","o","Color","k","MarkerSize",10,"MarkerFaceColor","k")
plot(xgoal(1,2),xgoal(1,1),"Marker","square","MarkerSize",10,"Color","r","MarkerFaceColor","r")
legend("Obstacle 1","Obstacle 2","Obstacle 3","SFM robot trajectory","Initial point","Goal","location","best")

% Set the size of the axis as desired
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
axis([0 10 0 10])
xticks(0:1:10); yticks(0:1:10);
xlabel("y coordinates")
ylabel("x coordinates")

% Question c): Only one obstacle on the way and one person moving

obs5size=[6,1]; % Top left coordinate for the rectangle obstacle
obs5length=4; % Length size of the obstacle
obs5width=1; % Width of the obstacle
obs5angle=atan2(3,4); % Orientation of the object with respect to world
dvo5=[0, 0]; % Vector of the distance to the obstacle
do5=0; % Distance to the obstacle

vrd_3=vrini; % Initial value for the vector speed as the initial speed
fr_3=zeros(1,2); % Forces (accelerations, mass equal to 1) applied on the robot 
xr_3=xini; % Initial value for the vector of positions as initial position
xp=xpini; % Initial value for the vector of positions for the person
 
% Calculate initial distance to the goal
distpos_3=xgoal-xr_3;
goaldist_3=round(norm(distpos_3)); % Value to store the distance to the goal (initially big)

% Start the loop until the goal is reached
while(goaldist_3~=0)

    % Check the distance to the obstacles
    [do5,dvo5]=CheckMinDist(obs5size,obs5length,obs5width,obs5angle,xr_3(end,:));
    % Check the distance to the person
    dpers=(xr_3(end,:)-xp(end,:));
    
    % Check distance to the goal and update vgoal at each step with the unitary direction vector
    distpos_3=xgoal-xr_3(end,:);
    vgoal_3=norm(vrini)*distpos_3/norm(distpos_3);
    
    % Get the value of all forces in the problem
    fgoal_3=k*(vgoal_3-vrd_3(end,:)); % Calculate the forces done by the goal 
    fro_3_1=aobs*(exp((dobs-do5)/bobs))*(dvo5/do5); % Calculate the force done by the obstacle
    frp_3=ap*(exp((dp-norm(dpers))/bp))*(dpers/norm(dpers)); % Calculate the force done by the person
    
    fr_3=[fr_3; fgoal_3+fro_3_1+frp_3]; % Add the forces or accelerations to the vector for that purpose

    % Get next the speed for the robot in each axis vx and vy
    vrd_3=[vrd_3; vrd_3(end,1)+fr_3(end,1)*Tr vrd_3(end,2)+fr_3(end,2)*Tr]; 
    
    % Calculate the new robot position
    xr_3=[xr_3; xr_3(end,1)+vrd_3(end,1)*Tr xr_3(end,2)+vrd_3(end,2)*Tr]
    % Calculate new person position
    xp=[xp; xp(end,1)+vpini(1,1)*Tr xp(end,2)+vpini(1,2)*Tr];

    % Check distance to the goal at that step
    goaldist_3=round(norm(xr_3(end,:)-xgoal));
end

% Plots the results for the robot movement for this exercise
figure(7)
title(["Designed robot trajectory with SFM with 1 obstacle and 1 person, time step= 0.5 s";"Obstacle parameters: a_o_b_s_ = 1; b_o_b_s_ = 0.5; d_o_b_s_ = 1, person parameters: a_p_ = 1.5; b_p_ = 1.3; d_p_ = 1.5"])

% Create a rectangle to place the obstacle on the plot
obs5=[obs5size; obs5size+[-obs5length,obs5length-obs5width]; obs5size+[-obs5length+obs5width,obs5length]; obs5size+[obs5width,obs5width]; obs5size];
patch(obs5(:,2),obs5(:,1),'c');
hold on
grid on

% Plot the person trajectory along all the time (x on y and y on x)
plot(xp(:,2),xp(:,1),"Marker","diamond","LineStyle","--","MarkerSize",8,"Color","m")
plot(xp(1,2),xp(1,1),"Marker","o","Color","b","MarkerSize",10,"MarkerFaceColor","b")

% Plot the robot trajectory along all the time (x on y and y on x)
plot(xr_3(:,2),xr_3(:,1),"Marker","x","LineStyle","-","MarkerSize",8,"Color","g")
plot(xr_3(1,2),xr_3(1,1),"Marker","o","Color","k","MarkerSize",10,"MarkerFaceColor","k")
plot(xgoal(1,2),xgoal(1,1),"Marker","square","MarkerSize",10,"Color","r","MarkerFaceColor","r")

legend("Obstacle 1","Person trajectory","Starting person point","SFM robot trajectory","Initial point","Goal","location","best")

% Set the size of the axis as desired
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
axis([0 10 0 10])
xticks(0:1:10); yticks(0:1:10);
xlabel("y coordinates")
ylabel("x coordinates")


%------------------------------------------------------------------------%
%% Additional function to calculate the distance to different objects for exercise 2:
function [mindist,coords]=CheckMinDist(objcoord,objsize,objwidth,obsangle,xpos)
    mindist=100000000;
    coords=xpos;
    sizeob=0:0.01:(objsize/sin((pi/2)-obsangle));
    widob=0:0.01:(objwidth/sin((pi/2)-obsangle));
    % Create a rotation matrix upon the angle of the object
    if (obsangle ~= 0)
        A=rotz(90-obsangle*180/pi);
    else
         A=rotz(0); 
    end
    % Add the initial point of the object to the rotation matrix
    A(1,3)=objcoord(1,1); A(2,3)=objcoord(1,2);

    % Check the distance to all the points on the obstacle 
    for i=0:1:(objwidth/sin((pi/2)-obsangle)/0.01)
        for j=0:1:(objsize/sin((pi/2)-obsangle)/0.01)
            point=(A*([widob(i+1),sizeob(j+1),1])')'; % Calculate the point that is being rotated
            robdist=norm(xpos-point(1:2)); % Calculate distance to a point
            % Update only the value of the minimum distance if it is smaller
            if(robdist<mindist) 
                mindist=robdist;
                coords=xpos-point(1:2);
            end
        end
    end
end


