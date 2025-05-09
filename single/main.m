function main()
% A*主路径，APF局部路径
clc;clear;close all;

%% 地图生成
mapsize = [100, 100];
mapa = zeros(mapsize);

mapa(20:30, 15:30) = 1;
mapa(5:20, 60:90) = 1;
mapa(40:60, 40:55) = 1;
mapa(50:70, 70:80) = 1;
mapa(70:90, 10:30) = 1;

[row, col] = find(mapa == 1);
obs = [row, col];

initial = [5, 5];
goal = [90, 90];

%% 动态障碍物
% 动态障碍物初始化
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
path = Astar(mapa,initial,goal, width);
disp('A*规划路径');
toc(tastar);

path = [initial;
    path];

%% 主循环
continuepath(path, obs, mapa);

end

%% 主循环
function continuepath(waypoints, obs, mapa)
    % 注意调整前瞻值和障碍物检测范围
    
    % 三次样条插值
    t = 1:size(waypoints,1);  
    ts = linspace(1, size(waypoints,1), 100); 
    spline_x = spline(t, waypoints(:,1)');
    spline_y = spline(t, waypoints(:,2)');
    
    % 生成连续路径
    cont_path = [ppval(spline_x, ts); ppval(spline_y, ts)]';
    
    % 速度规划
    max_speed = 1.8;        % 最大线速度 (m/s)
    max_accel = 0.5;        % 最大加速度 (m/s?)
    global dt;
    dt = 0.1;               % 控制周期 (s)
    lookahead_dist = 1;   % 前瞻距离 (m)
    
    % 初始化机器人状态
    robot.pos = cont_path(1,:);     % 初始位置
    robot.vel = 0;                  % 初始速度
    robot.theta = 0;                % 初始朝向
    
    dobs_pos = [22, 46; 50, 70; 70, 75]; % 初始位置
    
    % 可视化初始化
    figure;
    hold on; axis equal; grid on;
    plot(cont_path(:,1), cont_path(:,2), 'b--', 'LineWidth', 1.5); % 连续路径
    scatter(obs(:, 1), obs(:, 2), 30, 'k', 'filled');
    h_robot = rectangle('Position', [robot.pos(1)-1, robot.pos(2)-1, 2, 2],...
                       'Curvature', [1 1], 'FaceColor', 'r'); % 圆形机器人
    h_heading = quiver(robot.pos(1), robot.pos(2), 1.5*cos(robot.theta), 1.5*sin(robot.theta),...
                'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 1); % 朝向箭头
    h_dobs1 = rectangle('Position', [dobs_pos(1, 1)-1, dobs_pos(1, 2)-1, 2, 2], 'FaceColor', 'k'); % 三个障碍物
    h_dobs2 = rectangle('Position', [dobs_pos(2, 1)-1, dobs_pos(2, 2)-1, 2, 2], 'FaceColor', 'k'); 
    h_dobs3 = rectangle('Position', [dobs_pos(3, 1)-1, dobs_pos(3, 2)-1, 2, 2], 'FaceColor', 'k'); 
    % 调整后的全局路径
    robot_pos_c1 = robot.pos;
    h_robot_c1 = plot(robot.pos(:,1), robot.pos(:,2), 'color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
    h_cont_path_1 = plot(cont_path(:,1), cont_path(:,2), 'g', 'LineWidth', 1.5); % 连续路径
    % 检测到的障碍物
    obs_in_range = [1, 1];
    h_apfobs = scatter(obs_in_range(:,1), obs_in_range(:,2), 'filled');
    % apf局部路径
    snew_local_path = [1, 1];
    h_apfpath = plot(snew_local_path(:,1), snew_local_path(:,2), 'r', 'LineWidth', 1.5); 
    
    % 控制循环
    current_idx = 1;
    run_time = tic;
    
    % 检测动态障碍物范围
    local_plan_range = 20;
    
    while current_idx < size(cont_path,1)
        
        % 更新动态障碍物位置
        dobs_pos = updateobs(dobs_pos);
        set(h_dobs1, 'Position', [dobs_pos(1, 1)-1, dobs_pos(1, 2)-1, 2, 2], 'FaceColor', 'k');
        set(h_dobs2, 'Position', [dobs_pos(2, 1)-1, dobs_pos(2, 2)-1, 2, 2], 'FaceColor', 'k'); 
        set(h_dobs3, 'Position', [dobs_pos(3, 1)-1, dobs_pos(3, 2)-1, 2, 2], 'FaceColor', 'k'); 
        
        % 更新地图上的障碍物
        ex_dobs_pos = extension(dobs_pos);
        
        % 检测附近障碍物
        [obs_in_range] = check_obstacles(robot.pos, ex_dobs_pos, local_plan_range, mapa);
        set(h_apfobs, 'XData', obs_in_range(:,1), 'YData', obs_in_range(:,2));
        uistack(h_apfobs, 'top');
        % 运行时间
        run_time_1 = toc(run_time);
        
        % 动态路径重规划逻辑
        if ~isempty(obs_in_range)
            % 截取局部路径段（当前位置到前方若干个点）（需调整）
            start_idx = max(1, current_idx - 5);
            end_idx = min(size(cont_path,1), current_idx + 15);
            local_path = cont_path(start_idx:end_idx,:);
            % 检测截取路段长度
            if (local_path(1,1) - local_path(end,1))^2 + (local_path(1,2) - local_path(end,2))^2 < 2
                end_idx_lookdist = 50; % 调整
                distances = distances(current_idx: end, :);
                start_idx = max(1, current_idx);
                end_idx = find(distances >= end_idx_lookdist, 1);
                if isempty(end_idx)
                    % target_idx = size(path,1);
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
            [new_local_path, ~] = APF(local_path(1,:), local_path(end,:), obs_in_range);
            % 平滑路径
            snew_local_path = smooth_path(new_local_path);
            
            set(h_apfpath, 'XData', snew_local_path(:,1), 'YData', snew_local_path(:,2));
            
            % 拼接新路径
            cont_path = [cont_path(1:start_idx - 1,:); 
                         snew_local_path;
                         cont_path(end_idx + 1:end,:)];
        end
        
        % 获取前瞻目标点
        [target_idx, ~] = findTargetPoint(cont_path, robot.pos, lookahead_dist, current_idx);
        
        % 计算期望速度方向
        target_pos = cont_path(target_idx,:);
        direction = target_pos - robot.pos;
        desired_theta = atan2(direction(2), direction(1)); % 弧度
        
        % 角速度控制
        theta_error = desired_theta - robot.theta;
        omega = 2.0 * theta_error; 
        
        % 检测终点
        remaining_dist = norm(cont_path(end,:) - robot.pos);
        if remaining_dist < 1.0
            desired_vel = max_speed * (remaining_dist / 1.0); % 终点前减速
        else
            desired_vel = max_speed;
        end
        
        % 线速度控制
        accel = min(max_accel, (desired_vel - robot.vel)/dt);
        robot.vel = robot.vel + accel * dt;
        
        % 更新机器人状态
        robot.theta = robot.theta + omega * dt;
        robot.pos = robot.pos + robot.vel * dt * [cos(robot.theta), sin(robot.theta)];

        % 更新图形
        title(['运行时间：', num2str(run_time_1), '秒 ']);
        set(h_robot, 'Position', [robot.pos(1)-1, robot.pos(2)-1, 2, 2]);
        set(h_heading, 'XData', robot.pos(1), 'YData', robot.pos(2),...
                      'UData', 1.5*cos(robot.theta), 'VData', 1.5*sin(robot.theta));   
        robot_pos_c1 = [robot_pos_c1; robot.pos];
        set(h_robot_c1, 'XData', robot_pos_c1(:,1), 'YData', robot_pos_c1(:,2));
        set(h_cont_path_1, 'XData', cont_path(:,1), 'YData', cont_path(:,2));
        
        % 更新前瞻点
        distances = [];
        for i = 1:size(cont_path(:, 1))
            differences_i = cont_path(i, :) - robot.pos;
            distances_i = differences_i(1)^2 + differences_i(2)^2;
            distances = [distances; distances_i];
        end
        [~, current_idx] = min(distances);    % 找到最小平方距离的索引

        % 刷新显示
        drawnow;
        pause(dt); % 模拟实时运行
    end
    disp('完成。');
end

%% 前瞻目标
function [target_idx, min_dist] = findTargetPoint(path, current_pos, look_dist, start_idx)
    pathsize = size(path);
    minar0 = path(start_idx, :) - current_pos;
    distances_i = sqrt(minar0(1)^2 + minar0(2)^2);
    distances = distances_i;
    for i = 1:(pathsize(1) - start_idx)
        minar = path(start_idx + i, :) - current_pos;
        distances_i = sqrt(minar(1)^2 + minar(2)^2);
        distances = [distances;distances_i];
    end
    
    idx = find(distances >= look_dist, 1);
    if isempty(idx)
        target_idx = size(path,1);
    else
        target_idx = start_idx + idx - 1;
    end
    min_dist = distances(idx);
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

%% 坐标扩展
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

%% 路径平滑处理
function smoothed_path = smooth_path(raw_path)
    t = 1:size(raw_path,1);
    ts = linspace(1, size(raw_path,1), 50);
    spline_x = spline(t, raw_path(:,1)');
    spline_y = spline(t, raw_path(:,2)');
    smoothed_path = [ppval(spline_x, ts); ppval(spline_y, ts)]';
end

%% 动态障碍物检测
function [obs_in_range] = check_obstacles(robot_pos, dobs_pos, range, mapa)
    % 检测10×10区域内的障碍物
    x_range = [robot_pos(1)-range/2, robot_pos(1)+range/2];
    y_range = [robot_pos(2)-range/2, robot_pos(2)+range/2];
    obs_in_range = [];
    for i = 1: size(dobs_pos(:, 1))
        obstacles = dobs_pos(i, :);
        in_x = obstacles(:,1) >= x_range(1) & obstacles(:,1) <= x_range(2);
        in_y = obstacles(:,2) >= y_range(1) & obstacles(:,2) <= y_range(2);
        if in_x && in_y
            obs_in_range = [obs_in_range;
                obstacles];
        end
    end
    
    if (robot_pos(1)-range/2) > 0.5 && (robot_pos(2)-range/2) > 0.5 
        [row, col] = find(mapa(round(robot_pos(1)-range/2): round(robot_pos(1)+range/2), ...
            round(robot_pos(2)-range/2): round(robot_pos(2)+range/2)) == 1);
        obs = [row, col];
        obs(:, 1) = obs(:, 1) + round(robot_pos(1)-range/2) - 1;
        obs(:, 2) = obs(:, 2) + round(robot_pos(2)-range/2) - 1;
    else
        [row, col] = find(mapa(1: round(robot_pos(1)+range/2), 1: round(robot_pos(2)+range/2)) == 1);
        obs = [row, col];
    end
    
    obs_in_range = [obs_in_range; obs];
end
