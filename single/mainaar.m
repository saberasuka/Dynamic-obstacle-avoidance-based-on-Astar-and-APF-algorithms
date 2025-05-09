clc;clear;close all;
%% ��ʼ�㣬���ɵ�ͼ
mapsizex = 80;  % ��ͼ
mapsizey = 80;
goal = [70, 70];
initial = [3, 3];
obsnum = 30;  %�ϰ�������

[map,obs]=Map_generation1(mapsizex, mapsizey, obsnum);  % ��ͼ
[map1,obs1]=Map_generation0(mapsizex, mapsizey, obsnum);  % ��ͼ

figure;
hold on
scatter(obs(:, 1)', obs(:, 2)', 30, 'k', 'filled');
scatter(initial(1), initial(2), 100, 'r', 'o', 'filled');
scatter(goal(1), goal(2), 100, 'r', 'd', 'filled');
%% A*
width = 4; % ���
tastar = tic;
path = Astar(map,initial,goal, width);
toc(tastar);
%% �˹��Ƴ�
tapf = tic;
[apfpath, workout] = APF(initial, goal, obs);
toc(tapf);
%% RRT
Step = 2;%����
pro = 0.1;%���յ���ΪP_rand�ĸ���
trrt = tic;
[~] = RRT(map1, initial, goal, Step, pro);
trrt1 = toc(trrt);
%% ��ͼ

hold on;
% A*��ϴ���
% ������ֵ
path = [initial;
    path];
t1 = 1:length(path(:, 1));
t_fine1 = linspace(1, length(path(:, 1)), 1000);
x_fit1 = spline(t1, path(:, 1), t_fine1);
y_fit1 = spline(t1, path(:, 2), t_fine1);
plot(x_fit1, y_fit1, 'b-', 'LineWidth', 1);

% �˹��Ƴ�
plot(apfpath(:, 1), apfpath(:, 2), 'm-', 'LineWidth', 1);
% scatter(apfpath(:, 1), apfpath(:, 2), 10, 'm', 'filled');

hold off
