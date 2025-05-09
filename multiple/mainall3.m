function mainall3()
clc;clear;close all;
%% 地图生成
% A*主路径，APF局部路径
mapsize = [100, 100];
global mapa
mapa = zeros(mapsize);
mapa(20:30, 15:30) = 1;
mapa(5:20, 60:90) = 1;
mapa(40:60, 40:55) = 1;
mapa(50:70, 70:80) = 1;
mapa(70:90, 10:30) = 1;

global obs
[row, col] = find(mapa == 1);
obs = [row, col];

global start_1;global goal_1;global start_2;
global goal_2;global start_3;global goal_3;
start_1 = [5, 5];
goal_1 = [90, 90];
start_2 = [5, 55];
goal_2 = [60, 10];
start_3 = [100, 25];
goal_3 = [40, 10];

%% 动态障碍物初始化
global dobs_num
dobs_num = 3; % 动态障碍物数量

dobs_angle1 = atan2(11, 13);
dobs_angle2 = atan2(5, 15);
dobs_angle3 = atan2(0, 15);
global dobs_unit;
global dt;
dt = 0.1;
dobs_unit = [dt * cos(dobs_angle1), -dt * sin(dobs_angle1);
    dt * cos(dobs_angle2), -dt * sin(dobs_angle2);
    dt * cos(dobs_angle3), -dt * sin(dobs_angle3);
    ];

%% A*
width = 4; % 体积
tastar = tic;
path_1 = Astar(mapa,start_1,goal_1, width);
path_2 = Astar(mapa,start_2,goal_2, width);
path_3 = Astar(mapa,start_3,goal_3, width);
disp('A*规划路径');
toc(tastar);

path_1 = [start_1;
    path_1];
path_2 = [start_2;
    path_2];
path_3 = [start_3;
    path_3];

%% 主循环
continuepath(path_1, path_2, path_3);

end

%% 路径连续显示
function continuepath(path_1, path_2, path_3)
    global obs
    global dt
    
    % 路径平滑
    cont_path_1 = smooth_path(path_1);
    cont_path_2 = smooth_path(path_2);
    cont_path_3 = smooth_path(path_3);
    
    % 初始化机器人状态
    % 三个参数 设置为数组[1; 2; 3]
    robot.pos = [cont_path_1(1,:);
        cont_path_2(1,:);
        cont_path_3(1,:);
        ];     % 初始位置
    robot.vel = [0; 0; 0];                  % 初始速度
    robot.theta = [0; 0; pi];                % 初始朝向
    
    dobs_pos = [22, 46; 50, 70; 70, 75]; % 初始位置
    
    global start_1;global start_2;global start_3;
    
    % 可视化初始化
    figure;
    hold on; axis equal; grid on;
    
    plot(cont_path_1(:,1), cont_path_1(:,2), 'b--', 'LineWidth', 1.5); % 连续路径
    plot(cont_path_2(:,1), cont_path_2(:,2), 'b--', 'LineWidth', 1.5);
    plot(cont_path_3(:,1), cont_path_3(:,2), 'b--', 'LineWidth', 1.5);
    
    scatter(obs(:, 1), obs(:, 2), 30, 'k', 'filled');
    % 圆形机器人
    h_robot_1 = rectangle('Position', [robot.pos(1, 1)-1, robot.pos(1, 2)-1, 2, 2],...
                       'Curvature', [1 1], 'FaceColor', 'r');           
    h_robot_2 = rectangle('Position', [robot.pos(2, 1)-1, robot.pos(2, 2)-1, 2, 2],...
                       'Curvature', [1 1], 'FaceColor', 'r');
    h_robot_3 = rectangle('Position', [robot.pos(3, 1)-1, robot.pos(3, 2)-1, 2, 2],...
                       'Curvature', [1 1], 'FaceColor', 'r');
    % 三个动态障碍物
    h_dobs1 = rectangle('Position', [dobs_pos(1, 1)-1, dobs_pos(1, 2)-1, 2, 2], 'FaceColor', 'k'); 
    h_dobs2 = rectangle('Position', [dobs_pos(2, 1)-1, dobs_pos(2, 2)-1, 2, 2], 'FaceColor', 'k'); 
    h_dobs3 = rectangle('Position', [dobs_pos(3, 1)-1, dobs_pos(3, 2)-1, 2, 2], 'FaceColor', 'k'); 
    
    % apf规划时计入的障碍物
    obs_in_range = [1, 1];
    h_apfobs = scatter(obs_in_range(:,1), obs_in_range(:,2), 'filled');
    
    % 绘制apf局部规划出的路线
    snew_local_path = [0, 0];
    h_apfpath_1 = plot(snew_local_path(:,1), snew_local_path(:,2), 'r', 'LineWidth', 1.5);       % 连续路径
    h_apfpath_2 = plot(snew_local_path(:,1), snew_local_path(:,2), 'r', 'LineWidth', 1.5);
    h_apfpath_3 = plot(snew_local_path(:,1), snew_local_path(:,2), 'r', 'LineWidth', 1.5);
    
    % 机器人实际路线
    robot_pos_c1 = start_1;
    h_robot_c1 = plot(robot_pos_c1(:,1), robot_pos_c1(:,2), 'color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
    robot_pos_c2 = start_2;
    h_robot_c2 = plot(robot_pos_c2(:,1), robot_pos_c2(:,2), 'color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
    robot_pos_c3 = start_3;
    h_robot_c3 = plot(robot_pos_c3(:,1), robot_pos_c3(:,2), 'color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
    
    % 查看cont_path
    h_cont_path_1 = plot(cont_path_1(:,1), cont_path_1(:,2), 'r', 'LineWidth', 1.5);       % 连续路径
    h_cont_path_2 = plot(cont_path_2(:,1), cont_path_2(:,2), 'r', 'LineWidth', 1.5);
    h_cont_path_3 = plot(cont_path_3(:,1), cont_path_3(:,2), 'r', 'LineWidth', 1.5);
    
    % 主控制循环
    current_idx = [1; 1; 1];
    run_time = tic;
    text_i = 1;
    
    distances_1 = [0];
    distances_2 = [0];
    distances_3 = [0];
    
    % 三个机器人全部完成任务后结束
    while current_idx(1) < size(cont_path_1, 1) || current_idx(2) < size(cont_path_2, 1) || current_idx(3) < size(cont_path_3, 1)
        % 更新动态障碍物位置
        dobs_pos = updateobs(dobs_pos);
        set(h_dobs1, 'Position', [dobs_pos(1, 1)-1, dobs_pos(1, 2)-1, 2, 2], 'FaceColor', 'k');
        set(h_dobs2, 'Position', [dobs_pos(2, 1)-1, dobs_pos(2, 2)-1, 2, 2], 'FaceColor', 'k'); 
        set(h_dobs3, 'Position', [dobs_pos(3, 1)-1, dobs_pos(3, 2)-1, 2, 2], 'FaceColor', 'k'); 
        % 运行时间
        run_time_1 = toc(run_time);
        
        % 判断各个机器人是否到达目标
        % 机器人一更新
        if current_idx(1) < size(cont_path_1, 1)
            rob1 = 1;
            [current_idx(1), robot_pos1, robot.vel(1), robot.theta(1), obs_in_range_1, cont_path_1, distances_1...
                ] = robotupdate(rob1, current_idx(1), cont_path_1, robot.pos, robot.vel(1),...
                robot.theta(1), dobs_pos, h_apfpath_1, distances_1);
        end
        
        % 机器人二更新
        if current_idx(2) < size(cont_path_2, 1)
            rob2 = 2;
            [current_idx(2), robot_pos2, robot.vel(2), robot.theta(2), obs_in_range_2, cont_path_2, distances_2...
                ] = robotupdate(rob2, current_idx(2), cont_path_2, robot.pos, robot.vel(2),...
                robot.theta(2), dobs_pos, h_apfpath_2, distances_2);
        end
        
        % 机器人三更新
        if current_idx(3) < size(cont_path_3, 1)
            rob3 = 3;
            [current_idx(3), robot_pos3, robot.vel(3), robot.theta(3), obs_in_range_3, cont_path_3, distances_3...
                ] = robotupdate(rob3, current_idx(3), cont_path_3, robot.pos, robot.vel(3),...
                robot.theta(3), dobs_pos, h_apfpath_3, distances_3);
        end
        
        % 更新cont_path
        set(h_cont_path_1, 'XData', cont_path_1(:,1), 'YData', cont_path_1(:,2));
        set(h_cont_path_2, 'XData', cont_path_2(:,1), 'YData', cont_path_2(:,2));
        set(h_cont_path_3, 'XData', cont_path_3(:,1), 'YData', cont_path_3(:,2));
        
        % 更新机器人实际走过的路径
        robot_pos_c1 = [robot_pos_c1; robot_pos1];
        set(h_robot_c1, 'XData', robot_pos_c1(:,1), 'YData', robot_pos_c1(:,2));
        robot_pos_c2 = [robot_pos_c2; robot_pos2];
        set(h_robot_c2, 'XData', robot_pos_c2(:,1), 'YData', robot_pos_c2(:,2));
        robot_pos_c3 = [robot_pos_c3; robot_pos3];
        set(h_robot_c3, 'XData', robot_pos_c3(:,1), 'YData', robot_pos_c3(:,2));
        
        robot.pos = [robot_pos1; robot_pos2; robot_pos3];
        
        obs_in_range = [obs_in_range_1; obs_in_range_2; obs_in_range_3];
        set(h_apfobs, 'XData', obs_in_range(:,1), 'YData', obs_in_range(:,2));
        uistack(h_apfobs, 'top');

        
       %% 动态路径重规划逻辑
        % 更新图形
        title(['运行时间：', num2str(run_time_1), '秒 ']);
        
        % 更新机器人图形位置
        set(h_robot_1, 'Position', [robot.pos(1, 1)-1, robot.pos(1, 2)-1, 2, 2]);
        set(h_robot_2, 'Position', [robot.pos(2, 1)-1, robot.pos(2, 2)-1, 2, 2]);
        set(h_robot_3, 'Position', [robot.pos(3, 1)-1, robot.pos(3, 2)-1, 2, 2]);

        % 刷新显示
        drawnow;
        pause(dt); % 模拟实时运行
    end
    disp('完成。');
end

%% 查找前瞻目标点 几何距离
function [target_idx] = findTargetPoint(path, current_pos, look_dist, start_idx)
    pathsize = size(path);
    distances = [];
    for i = start_idx: pathsize(1)
        differences_i = path(i, :) - current_pos; 
        distances_i = differences_i(1)^2 + differences_i(2)^2;
        distances = [distances; distances_i];
    end
    
    idx = find(distances >= look_dist, 1);
    if isempty(idx)
        target_idx = min(size(path,1), start_idx + 3);
    else
        target_idx = start_idx + idx - 1;
    end
end

%% 动态更新障碍物位置
function dobs_pos = updateobs(dobs_pos)
global dobs_unit;
dobs_pos = [
    dobs_pos(1, :) + dobs_unit(1, :);
    dobs_pos(2, :) + dobs_unit(2, :);
    dobs_pos(3, :) + dobs_unit(3, :);
    ];
% 边界碰撞检测（反弹）
if dobs_pos(1, 1) < 22 || dobs_pos(1, 1) > 35
    dobs_unit(1, :) = -dobs_unit(1, :);
end
if dobs_pos(2, 1) < 50 || dobs_pos(2, 1) > 65
    dobs_unit(2, :) = -dobs_unit(2, :);
end
if dobs_pos(3, 1) < 70 || dobs_pos(3, 1) > 85
    dobs_unit(3, :) = -dobs_unit(3, :);
end
end

%% 对系列坐标进行扩展
function [expanded_pos] = extension(pos)
rounded_pos = round(pos); % 四舍五入取整
expanded_pos = [];

for i = 1:size(rounded_pos, 1)
    x = rounded_pos(i, 1);
    y = rounded_pos(i, 2);
    for x_range = x-1 : x+1
        for y_range = y-1 : y+1
            expanded_pos = [expanded_pos;
                x_range, y_range];
        end
    end
end
end

%% 路径平滑函数（三次样条）
function smoothed_path = smooth_path(raw_path)
    t = 1:size(raw_path,1);
    ts = linspace(1, size(raw_path,1), 100); % 路径平滑参数调整
    spline_x = spline(t, raw_path(:,1)');
    spline_y = spline(t, raw_path(:,2)');
    smoothed_path = [ppval(spline_x, ts); ppval(spline_y, ts)]';
end

%% 动态障碍物检测函数
function [obs_in_range, rob_obs_1, rob_obs_2, rob_obs_3] = check_obstacles(robot_pos, check_obs, range)
    global mapa
    % 检测10×10区域内的障碍物
    x_range = [robot_pos(1)-range/2, robot_pos(1)+range/2];
    y_range = [robot_pos(2)-range/2, robot_pos(2)+range/2];
    obs_in_range = [];
    % 动态障碍物
    rob_obs_1 = 0;
    rob_obs_2 = 0;
    rob_obs_3 = 0;
    for i = 1: size(check_obs(:, 1))
        obstacles = check_obs(i, :);
        in_x = obstacles(:,1) >= x_range(1) & obstacles(:,1) <= x_range(2);
        in_y = obstacles(:,2) >= y_range(1) & obstacles(:,2) <= y_range(2);
        if in_x && in_y
            obs_in_range = [obs_in_range;
                obstacles];
            if (1 <= i) && (i <= 9)
                rob_obs_1 = 1;
            elseif (10 <= i) && (i <= 18)
                rob_obs_2 = 1;
            elseif (19 <= i) && (i <= 27)
                rob_obs_3 = 1;
            end
        end
    end
    % 地图静态障碍物
    map_left = round(robot_pos(1)-range/2);
    map_right = round(robot_pos(1)+range/2);
    map_up = round(robot_pos(2)+range/2);
    map_down = round(robot_pos(2)-range/2);
    if map_left < 1
        map_left = 1;
    end
    if map_right > size(mapa(1, :))
        map_right = size(mapa(1, :));
    end
    if map_down < 1
        map_down = 1;
    end
    if map_up > size(mapa(:, 1))
        map_up = size(mapa(:, 1));
    end
    
    [row, col] = find(mapa(map_left: map_right, map_down: map_up) == 1);
    obs = [row, col];
    obs(:, 1) = obs(:, 1) + map_left - 1;
    obs(:, 2) = obs(:, 2) + map_down - 1;

    obs_in_range = [obs_in_range; obs];
end

%% 集群机器人单体
function [current_idx, robot_pos_i, robot_vel, robot_theta, obs_in_range, cont_path, distances] = robotupdate(robi, current_idx,...
    cont_path, robot_pos, robot_vel, robot_theta, dobs_pos, h_apfpath, distances)
% 特别注意这里，输入robot_pos为数组，包含所有机器人位置，输出只有robot_pos_i一个数
% 速度规划参数 调整
max_speed = 2;        % 最大线速度 (m/s)
max_accel = 0.5;        % 最大加速度 (m/s)
global dt;
dt = 0.1;               % 控制周期 (s)
lookahead_dist = 2;   % 前瞻距离 (m)

% 收集除robi以外的的动态障碍物（包括其他机器人）自身位置
robot_pos_i = robot_pos(robi, :);
robot_pos_del = robot_pos;
robot_pos_del(robi, :) = [200, 200];
dobs_pos = [robot_pos_del; dobs_pos];
% 对障碍物进行扩展
check_obs = extension(dobs_pos);

local_plan_range = 20; % 参数调整 障碍物探测范围
[obs_in_range, rob_obs_1, rob_obs_2, ~] = check_obstacles(robot_pos_i, check_obs, local_plan_range);

% 两机器人相遇问题，权重：1>2>3
if rob_obs_1 == 1 && robi == 3 && sqrt((robot_pos(1, 1) - robot_pos(3, 1))^2 + (robot_pos(1, 2) - robot_pos(3, 2))^2) < 5
    robot_vel = 0;
    return
end
if rob_obs_2 == 1 && robi == 3 && sqrt((robot_pos(2, 1) - robot_pos(3, 1))^2 + (robot_pos(2, 2) - robot_pos(3, 2))^2) < 5
    robot_vel = 0;
    return
end
if rob_obs_1 == 1 && robi == 2 && sqrt((robot_pos(2, 1) - robot_pos(1, 1))^2 + (robot_pos(2, 2) - robot_pos(1, 2))^2) < 5
    robot_vel = 0;
    return
end

if ~isempty(obs_in_range)
    % 截取局部路径段（当前位置到前方若干个点）（调整）
    start_idx = max(1, current_idx);
    end_idx = min(size(cont_path,1), current_idx + 15);
    local_path = cont_path(start_idx:end_idx,:);
    if (local_path(1,1) - local_path(end,1))^2 + (local_path(1,2) - local_path(end,2))^2 < 2
        end_idx_lookdist = 50; % 调整
        distances = distances(current_idx: end, :);
        start_idx = max(1, current_idx);
        end_idx = find(distances >= end_idx_lookdist, 1);
        if isempty(end_idx)
            end_idx = min(size(cont_path,1), current_idx + 20);
        else
            end_idx = start_idx + end_idx - 1;
        end
        if end_idx - start_idx < 2
            end_idx = start_idx + 20;
        end
    end

    local_path = cont_path(start_idx:end_idx,:);
    % 使用APF进行局部路径优化
    [new_local_path] = APF(local_path(1,:), local_path(end,:), obs_in_range);
    % 平滑路径
    snew_local_path = smooth_path(new_local_path);
    
    set(h_apfpath, 'XData', snew_local_path(:,1), 'YData', snew_local_path(:,2));
    
    % 拼接新路径
    cont_path = [cont_path(1:start_idx - 1,:);
        snew_local_path;
        cont_path(end_idx + 1:end,:)];
end

% 获取前瞻目标点
[target_idx] = findTargetPoint(cont_path, robot_pos_i, lookahead_dist, current_idx);
% 计算期望速度方向
target_pos = cont_path(target_idx,:);
direction = target_pos - robot_pos_i;
desired_theta = atan2(direction(2), direction(1)); % 弧度

% 角速度控制
theta_vel = 1;
theta_error = desired_theta - robot_theta;
theta_error = mod(theta_error + pi, 2*pi) - pi; % 跳变问题，修复调整
omega = theta_vel * theta_error; % 角速度控制 调整

% 检测终点
remaining_dist = norm(cont_path(end,:) - robot_pos_i);
if remaining_dist < 1.0
    desired_vel = max_speed * (remaining_dist / 1.0); % 终点前减速
else
    desired_vel = max_speed;
end

% 线速度控制
accel = min(max_accel, (desired_vel - robot_vel)/dt);
robot_vel = robot_vel + accel * dt;

% 更新机器人状态
robot_theta = robot_theta + omega * dt;
robot_pos_i = robot_pos_i + robot_vel * dt * [cos(robot_theta), sin(robot_theta)];

distances = [];
for i = 1:size(cont_path(:, 1))
    differences_i = cont_path(i, :) - robot_pos_i; 
    distances_i = differences_i(1)^2 + differences_i(2)^2;
    distances = [distances; distances_i];
end
[~, current_idx] = min(distances);    % 找到最小平方距离的索引
end
