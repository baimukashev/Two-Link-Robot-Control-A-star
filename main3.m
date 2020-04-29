% Daulet Baimukashev
% Date: 23.09.18
% This program implements the A-star algorithm for two-link manipulator
% movement. The script is divided into 4 parts, as the assignment tasks.

close all; clear all; clc;
% enter here the start and goal theta1 and theta2 values in radians
start_theta = [1.1 -1.4];
goal_theta = [1.5 -0.5];

% PART1: create configuration space
N1 = 200; N2 = 200;
q1 = linspace(-pi/12, pi*13/12,N1);
q2 = linspace(-pi/2,pi/2,N2);

% create the data structure for nodes in grids of the C-space
% z(i,j) is the node in the C-space and it has the fields as obstacle,
% past_cost, heurestics, total cost and parent. 

% construct the Free and Obstacle Space
for i=1:1:N1 
    x1i = 0.50*cos(q1(i));
    y1i = 0.50*sin(q1(i));
    for j=1:1:N2
        x2i = x1i + 0.40*cos(q1(i) + q2(j));
        y2i = y1i + 0.40*sin(q1(i) + q2(j));
        % add constraints to 2 links
        % the 2 balls and wall 
        % also link 2 has 3 spheres(r=L2/6) that covers whole link length
        % at positions m = L/6, 3*L/6, 5*L/6
        if ((x2i-0.6)^2+(y2i-0.7)^2>0.04) && ((x2i+0.6)^2+(y2i-0.7)^2>0.04) && (y1i > -0.1) && (y2i>-0.1) ...
        &&(x1i+ (1/6)*0.40*cos(q1(i) + q2(j))-0.6)^2+(y1i+(1/6)*0.40*sin(q1(i) + q2(j))-0.7)^2>(0.2+0.40/6)^2 ...
        &&(x1i+ (1/2)*0.40*cos(q1(i) + q2(j))-0.6)^2+(y1i+(1/2)*0.40*sin(q1(i) + q2(j))-0.7)^2>(0.2+0.40/6)^2 ...
        &&(x1i+ (5/6)*0.40*cos(q1(i) + q2(j))-0.6)^2+(y1i+(5/6)*0.40*sin(q1(i) + q2(j))-0.7)^2>(0.2+0.40/6)^2 ...
        &&(x1i+ (1/6)*0.40*cos(q1(i) + q2(j))+0.6)^2+(y1i+(1/6)*0.40*sin(q1(i) + q2(j))-0.7)^2>(0.2+0.40/6)^2 ...
        &&(x1i+ (1/2)*0.40*cos(q1(i) + q2(j))+0.6)^2+(y1i+(1/2)*0.40*sin(q1(i) + q2(j))-0.7)^2>(0.2+0.40/6)^2 ...
        &&(x1i+ (5/6)*0.40*cos(q1(i) + q2(j))+0.6)^2+(y1i+(5/6)*0.40*sin(q1(i) + q2(j))-0.7)^2>(0.2+0.40/6)^2
            z(i,j).obs = 0;
        else
            z(i,j).obs = 1;
        end
        z(i,j).x = i;
        z(i,j).y = j;
        % these will be initialized later
        z(i,j).heurestics = 0;
        z(i,j).parent = [];
        z(i,j).past_cost = 0;
        z(i,j).est_cost = 0;
    end
end

for i=1:1:N1
    for j=1:1:N2
        Z(i,j) = z(j,i).obs;
    end
end

% Convert the input values of theta from radian to node addresses in C-space
[c index1] = min(abs(q1-start_theta(1)));
[c index2] = min(abs(q2-start_theta(2)));
[c index3] = min(abs(q1-goal_theta(1)));
[c index4] = min(abs(q2-goal_theta(2)));
start_node = z(index1,index2); 
goal_node =  z(index3,index4);

% check the if the inputs are valid
if start_theta(1) > pi*13/12 || start_theta(1) < -pi/12 ... 
        || start_theta(2) > pi/2 || start_theta(2) < -pi/2 
    disp('Error: Start point out of C-space ')
    return
end 
if goal_theta(1) > pi*13/12 || goal_theta(1) < -pi/12 ... 
        || goal_theta(2) > pi/2 || goal_theta(2) < -pi/2 
    disp('Enter: Goal point out of C-space')
    return
end 
if start_node.obs > 0
    disp('Error: Start point in obstacle space')
    return
end
if goal_node.obs > 0
    disp('Error: Goal point in obstacle space')
    return
end

% Plot the Frre space and Obstacle space
figure(1)
contourf(q1,q2,Z,1)
xlabel('theta 1, rad')
ylabel('theta 2, rad')
title('Representation of free(blue) and obstacle(yellow) space')

hold on
plot(q1(start_node.x),q2(start_node.y), 'r*');
hold on
plot(q1(goal_node.x),q2(goal_node.y), 'g*');
hold off
%% Graph search

% initialise the node fields
for i=1:1:N1
    for j=1:1:N2
        z(i,j).past_cost = 0;
        if i == start_node.x && j == start_node.y 
            z(i,j).past_cost = 0;
        else
        	z(i,j).past_cost = 10^6;
        end
        z(i,j).heurestics = sqrt((goal_node.x - z(i,j).x)^2 + (goal_node.y - z(i,j).y)^2);
        z(i,j).parent = [];
        z(i,j).est_cost = 0;
    end
end

open_list = [start_node];
closed_list= [];
count = 0;
success = 0;
all_nbr = [];

% A star loop
disp('A-star started')
while ~isempty(open_list)
    count = count + 1;
    size(open_list);
    % get the current node and remove it from OPEN
    current_node = open_list(1);
    open_list(1) = [];
    % add the current node to CLOSED
    closed_list = [closed_list; current_node];
    % if the current node is goal node, finish
    if current_node.x == goal_node.x && current_node.y == goal_node.y
        disp('Success. Shortest path found')
        break
    else
        % looking for neighbors
        for i=-1:1:1
            for j=-1:1:1
                if  i == 0 && j == 0 
                    % it is the current node
                    continue
                else
                    yn = current_node.y + j;
                    xn = current_node.x + i;
                    % check if nbr inside the C-space
                    if xn < N1-1 && yn < N2-1 && xn > 0 && yn > 0
                    nbr = z(xn,yn);
                    % check if nbr not in obstacle
                    if  z(xn,yn).obs <1
                        % check if neighbor not in the closed list
                        [nbr_in_closed, nc] = look_node(closed_list, nbr);
                        if nbr_in_closed == 0 
                            % cost from neighbor to current
                            cost_current_to_nbr = cost_current_nbr(current_node, nbr);
                            % compare the costs 
                            tentative_cost = current_node.past_cost + cost_current_to_nbr;
                            if  tentative_cost < nbr.past_cost
                                nbr.past_cost = tentative_cost ;
                                nbr.parent = [current_node];
                                nbr.est_cost = nbr.past_cost + nbr.heurestics;
                                % check if neighbor not in the open list
                                [nbr_in_open, nc2] = look_node(open_list, nbr);
                                if  nbr_in_open == 0
                                    open_list = [open_list; nbr];
                                    open_list = SortList(open_list);
                                    all_nbr = [all_nbr; nbr];
                                else
                                    if nbr.past_cost < open_list(nc2).past_cost
                                        open_list(nc2) = nbr;
                                        open_list = SortList(open_list);
                                        all_nbr = [all_nbr; nbr];
                                    end
                                end
                            end
                        end
                    end
                    end
                end
            end
        end
    end

end
%%

% Construct and plot the shortest path
temp = current_node;
path = [];
path0 = [];
while ~isempty(temp.parent)
    path = [path temp.parent];
    path0 = [path0; temp.x temp.y];
    temp = temp.parent;
end
path0 = [path0; start_node.x start_node.y];

figure(2)
contourf(q1,q2,Z,1);
xlabel('theta 1, rad');
ylabel('theta 2, rad');
title('Representation of the shortest path found by the A-star algorithm');

hold on
for j=length(all_nbr):-1:1
    h4 = plot(q1(all_nbr(j).x),q2(all_nbr(j).y), '*b', 'LineWidth', 0.5);
    hold on
end
h1 = plot(q1(start_node.x),q2(start_node.y), '*r', 'LineWidth', 2);
hold on
h2 = plot(q1(goal_node.x),q2(goal_node.y), 'g*','LineWidth', 2);
hold on
for i=length(path0):-1:1
    h3 = plot(q1(path0(i,1)),q2(path0(i,2)), '*k');
    pause(0.01);
    hold on
end
legend([h1 h2 h3 h4], 'Start node','Goal node','Shortest path', 'Visited neighbors');
hold off

%% Cubic via-point interpolation 

% set the time interval and velocity
Time = 2;
k = length(path0);
T = linspace(0, Time,k);
dT = Time/k;
B1 = []; B2 = []; dB1 = []; dB2 = []; qt=[]; qt2=[];
for i=1:k
    B1(i) = q1(path0(length(path0)+1-i,1));
    B2(i) = q2(path0(length(path0)+1-i,2));
end
for i=2:k
    slope1(i) = (B1(i)-B1(i-1))/dT;   
    slope2(i) = (B2(i)-B2(i-1))/dT;   
end
dB1(1) = 0;
dB1(k) = 0;
dB2(1) = 0;
dB2(k) = 0;
for i=2:k-1
    if slope1(i)*slope1(i+1)>0
        dB1(i) = (slope1(i)+slope1(i+1))/2;   
    else
        dB1(i) = 0;
    end
    if slope2(i)*slope2(i+1)>0
        dB2(i) = (slope2(i)+slope2(i+1))/2;   
    else
        dB2(i) = 0;
    end
end

% link 1
dt = dT;
for j=1:k-1
    aj0 = B1(j);
    aj1 = dB1(j);
    aj2 = (3*B1(j+1) - 3*B1(j) - 2*dB1(j)*dT- dB1(j+1)*dT)/(power(dT,2));
    aj3 = (2*B1(j) + (dB1(j+1) + dB1(j))*dT - 2*B1(j+1))/(power(dT,3));
    qt(j) = aj0+ aj1*(dt) + aj2*(power(dt,2))+ aj3*(power(dt,3));
    xlink = 0.50*cos(qt(j));
    ylink = 0.50*sin(qt(j));
end
qt(k)=B1(k);

% link 2
Ns = 1;
dt = dT/Ns;
for j=1:k-1
    aj0 = B2(j);
    aj1 = dB2(j);
    aj2 = (3*B2(j+1) - 3*B2(j) - 2*dB2(j)*dT- dB2(j+1)*dT)/(power(dT,2));
    aj3 = (2*B2(j) + (dB2(j+1) + dB2(j))*dT - 2*B2(j+1))/(power(dT,3));
    %for m = 1:10
    %    qt2((j-1)*Ns + m) = aj0+ aj1*((m)*dt) + aj2*(power((m)*dt,2)+ aj3*(power((m)*dt,3)));
    %end
    qt2(j) = aj0+ aj1*(dt) + aj2*(power(dt,2))+ aj3*(power(dt,3));
    xlink2 = 0.40*cos(qt(j)+qt2(j));
    ylink2 = 0.40*sin(qt(j)+qt2(j));
end
qt2(k)=B2(k);


% Plot Time evolution of angular speeds and positions')
 figure(3)
 subplot(2,2,1)
 plot( qt)
 xlabel('Time, s');
 ylabel('Position of link 1, rad');
 subplot(2,2,2)
 plot(  dB1)
 xlabel('Time, s');
 ylabel('Angular velocity of link 1, rad/s');
 subplot(2,2,3)
 plot(  qt2)
 xlabel('Time, s');
 ylabel('Position of link 2, rad');
 subplot(2,2,4)
 plot(  dB2)
 xlabel('Time, s');
 ylabel('Angular velocity of link 2, rad/s');
 hold off
%% Create the movie of link movement

figure(4)
axis([-1 1 -1 1]);
title('Link movement')
for i=1:length(path0)
    x1i = 0.50*cos(qt(i));
    y1i = 0.50*sin(qt(i));
    x2i = x1i + 0.40*cos(qt(i)+qt2(i));
    y2i = y1i + 0.40*sin(qt(i)+qt2(i));
    
    start_point_x = 0.50*cos(q1(path0(length(path0),1))) + ...
                    0.40*cos(q1(path0(length(path0),1)) + q2(path0(length(path0),2)));
    start_point_y = 0.50*sin(q1(path0(length(path0),1))) + ...
                    0.40*sin(q1(path0(length(path0),1)) + q2(path0(length(path0),2)));
    goal_point_x = 0.50*cos(q1(path0(1,1))) + 0.40*cos(q1(path0(1,1)) + q2(path0(1,2)));
    goal_point_y = 0.50*sin(q1(path0(1,1))) + 0.40*sin(q1(path0(1,1)) + q2(path0(1,2)));
    hold on
    plot(start_point_x,start_point_y, '*r');
    hold on
    plot(goal_point_x,goal_point_y, '*g');
    hold on
    plot([-1 1], [-0.1 -0.1], 'r');
    hold on
    draw_circle(-0.6,0.7,0.2);
    hold on
    draw_circle(0.6,0.7,0.2);
    hold on
    draw_line(x1i, y1i, x2i,y2i);
    pause(0.01);
end
hold off
