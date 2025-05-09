clc;clear;close all;
%% 起始点，生成地图
mapsizex = 80;  % 地图
mapsizey = 80;
goal = [70, 70];
initial = [3, 3];
obsnum = 30;  %障碍物数量

[map,obs]=Map_generation1(mapsizex, mapsizey, obsnum);  % 地图
[map1,obs1]=Map_generation0(mapsizex, mapsizey, obsnum);  % 地图

figure;
hold on
scatter(obs(:, 1)', obs(:, 2)', 30, 'k', 'filled');
scatter(initial(1), initial(2), 100, 'r', 'o', 'filled');
scatter(goal(1), goal(2), 100, 'r', 'd', 'filled');
%% A*
width = 4; % 体积
tastar = tic;
path = Astar(map,initial,goal, width);
toc(tastar);
%% 人工势场
tapf = tic;
[apfpath, workout] = APF(initial, goal, obs);
toc(tapf);
%% RRT
Step = 2;%步长
pro = 0.1;%以终点作为P_rand的概率
trrt = tic;
[~] = RRT(map1, initial, goal, Step, pro);
trrt1 = toc(trrt);
%% 画图

hold on;
% A*拟合处理
% 样条插值
path = [initial;
    path];
t1 = 1:length(path(:, 1));
t_fine1 = linspace(1, length(path(:, 1)), 1000);
x_fit1 = spline(t1, path(:, 1), t_fine1);
y_fit1 = spline(t1, path(:, 2), t_fine1);
plot(x_fit1, y_fit1, 'b-', 'LineWidth', 1);

% 人工势场
plot(apfpath(:, 1), apfpath(:, 2), 'm-', 'LineWidth', 1);
% scatter(apfpath(:, 1), apfpath(:, 2), 10, 'm', 'filled');

hold off
