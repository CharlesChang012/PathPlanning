%
% PotentialFieldScript.m
%
clc; close all; clear all;
%% Generate some points

nrows = 400;
ncols = 600;

obstacle = false(nrows, ncols);

[x, y] = meshgrid (1:ncols, 1:nrows);

%% Generate some obstacle

obstacle (300:end, 100:250) = true;
obstacle (150:200, 400:500) = true;

t = ((x - 200).^2 + (y - 50).^2) < 50^2;
obstacle(t) = true;

t = ((x - 400).^2 + (y - 300).^2) < 100^2;
obstacle(t) = true;

%% Compute distance transform

d = bwdist(obstacle);

% Rescale and transform distances

d2 = (d/100) + 1;

d0 = 2;
nu = 800;

repulsive = nu*((1./d2 - 1/d0).^2);

repulsive (d2 > d0) = 0;  % when we move away d0 distance from the obstacle, the repulsive function will be 0.


%% Display repulsive potential

% figure;
% m = mesh (repulsive);
% m.FaceLighting = 'phong';
% axis equal;
% 
% title ('Repulsive Potential');

%% Compute attractive force

goal = [400, 50]; % User defined
start = [50, 350]; % User defined
xi = 1/700;

attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );

% figure;
% m = mesh (attractive);
% m.FaceLighting = 'phong';
% axis equal;
% 
% title ('Attractive Potential');

%% Display 2D configuration space

figure(1);
image(~obstacle);

hold on;
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
hold on
plot (start(1), start(2), 'b.', 'MarkerSize', 25)
hold off;


axis ([0 ncols 0 nrows]);
axis xy; % y value increase from bottom to up
axis on;

xlabel ('x');
ylabel ('y');

title ('Configuration Space');

%% Combine terms

f = attractive + repulsive;

figure(2);
m = mesh (f);
m.FaceLighting = 'phong';
axis equal;
xlabel ('x');
ylabel ('y');
title ('Total Potential');
%% PRM Method
% Build roadmap

nsamples = 200;
neighbors = 7;

roadmap = PRM (nsamples, neighbors, obstacle);

roadmap2 = AddNode2PRM (start', roadmap, neighbors, obstacle);
roadmap2 = AddNode2PRM (goal', roadmap2, neighbors, obstacle);

route_RPM = ShortestPathDijkstra(roadmap2.edges, roadmap2.edge_lengths, roadmap2.samples, nsamples+1, nsamples+2);
%% Plan route

route = GradientBasedPlanner (f, start, goal, 1000);
[route_Astar, numExpanded] = AStarGrid (obstacle, start, goal);

%% Plot route of two methods % TODO:
% Define colors
green = [62 150 81]./255;
red = [204 37 41]./255;
blue = [57 106 177]./255;

% Plot route
figure(6);
axis([0 nrows 0 ncols])
imshow(~obstacle);
hold on
% Draw line
Aroute_x = route_Astar(:,1);
Aroute_y = route_Astar(:,2);
line(Aroute_x, Aroute_y,'Color',green,'LineStyle',':','LineWidth',4)
hold on
RPMroute_x = route_RPM(:,1);
RPMroute_y= route_RPM(:,2);
line(RPMroute_x,RPMroute_y,'Color',green,'LineStyle','-', 'LineWidth',4)
hold on
route_x = route(:,1);
route_y= route(:,2);
line(route_x,route_y,'Color',green,'LineStyle','--', 'LineWidth',4)
hold on
% Plot Start and Goal position
plot(start(1), start(2),'.', "MarkerFaceColor", 'g', 'MarkerSize', 50)
hold on
plot(goal(1), goal(2), '.', "MarkerFaceColor", red, 'MarkerSize', 50)
% Figure settings
tt = title("Motion Planning in Configuration Space");
lgd = legend('A* search path', 'RPM path', 'Gradient-based path', 'Start', 'Goal', 'Location', 'southeast');
fontsize(tt, 16, 'pixels')
fontsize(lgd, 14, 'pixels')
xlabel('x')
ylabel('y')
axis xy;
axis on;
grid on;
hold off
%% Plot route in 3D gradient mesh
figure(5)
m = mesh (f);
axis equal;

% Plot Start and Goal position
% Start position
[sx, sy, sz] = sphere(20);

scale = 20;
sx = scale*sx;
sy = scale*sy;
sz = scale*(sz+1);

hold on;
s = mesh(sx, sy, sz);
s.FaceColor = green;
s.EdgeColor = 'none';
s.FaceLighting = 'phong';


s.XData = sx + start(1);
s.YData = sy + start(2);
s.ZData = sz + f(start(2), start(1));
% drawnow;

% Goal position
% [gx, gy, gz] = sphere(20);
% 
% scale = 20;
% gx = scale*gx;
% gy = scale*gy;
% gz = scale*(gz+1);
hold on
g = mesh(sx, sy, sz);
g.FaceColor = red;
g.EdgeColor = 'none';
g.FaceLighting = 'phong';

g.XData = sx + goal(1);
g.YData = sy + goal(2);
g.ZData = sz + f(goal(2), goal(1));
% drawnow;

% Route
hold on
P = round(route(1,:));
route_z = f(P(2), P(1));
for i = 2:size(route,1)
    P = round(route(i,:));
    z = f(P(2), P(1))+1;
    route_z = [route_z;z];
end
line(route_x,route_y,route_z,'Color','black','LineStyle','--','LineWidth',2.5);
tt = title({'Motion Planning in Configuration Space', 'with Gradient-based Method'});
lgd = legend('','Start', 'Goal', 'Gradient-based path', 'Location', 'southeast');
fontsize(tt, 16, 'pixels')
fontsize(lgd, 16, 'pixels')
xlabel('x')
ylabel('y')
hold off;
                        
% %% Plot the energy surface
% 
% figure;
% m = mesh (f);
% axis equal;
% 
% %% Plot ball sliding down hill
% 
% [sx, sy, sz] = sphere(20);
% 
% scale = 20;
% sx = scale*sx;
% sy = scale*sy;
% sz = scale*(sz+1);
% 
% hold on;
% p = mesh(sx, sy, sz);
% p.FaceColor = 'red';
% p.EdgeColor = 'none';
% p.FaceLighting = 'phong';
% hold off;
% 
% for i = 1:size(route,1)
%     P = round(route(i,:));
%     z = f(P(2), P(1));
% 
%     p.XData = sx + P(1);
%     p.YData = sy + P(2);
%     p.ZData = sz + f(P(2), P(1));
% 
%     drawnow;
% 
%     drawnow;
% 
% end
% 
% %% quiver plot
% [gx, gy] = gradient (-f);
% skip = 20;
% 
% figure;
% 
% xidx = 1:skip:ncols;
% yidx = 1:skip:nrows;
% 
% quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);
% 
% axis ([1 ncols 1 nrows]);
% 
% hold on;
% 
% ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
% pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
% p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
