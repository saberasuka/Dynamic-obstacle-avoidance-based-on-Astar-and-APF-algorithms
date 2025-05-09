function continuepath(waypoints, obs)
    % A*生成的原始路径点（示例）
    % waypoints = [0 0; 1 2; 3 3; 5 5; 8 7; 10 10]; 
    % 注意调整前瞻值！
    
    % 1. 路径参数化（三次样条插值）
    t = 1:size(waypoints,1);       % 原始路径点参数
    ts = linspace(1, size(waypoints,1), 100); % 重采样参数
    
    % 分别对x,y坐标进行插值
    spline_x = spline(t, waypoints(:,1)');
    spline_y = spline(t, waypoints(:,2)');
    
    % 生成连续路径
    cont_path = [ppval(spline_x, ts); ppval(spline_y, ts)]';
    
    % 2. 速度规划参数
    max_speed = 1.0;        % 最大线速度 (m/s)
    max_accel = 0.5;        % 最大加速度 (m/s?)
    dt = 0.1;               % 控制周期 (s)
    lookahead_dist = 1;   % 前瞻距离 (m)
    
    % 初始化机器人状态
    robot.pos = cont_path(1,:);     % 初始位置
    robot.vel = 0;                  % 初始速度
    robot.theta = 0;                % 初始朝向
    
    % 可视化初始化
    figure;
    hold on; axis equal; grid on;
    plot(waypoints(:,1), waypoints(:,2), 'bo', 'MarkerFaceColor', 'b'); % 原始路径点
    plot(cont_path(:,1), cont_path(:,2), 'b--', 'LineWidth', 1.5);       % 连续路径
    scatter(obs(:, 1), obs(:, 2), 30, 'k', 'filled');
    h_robot = rectangle('Position', [robot.pos(1)-1, robot.pos(2)-1, 2, 2],...
                       'Curvature', [1 1], 'FaceColor', 'r');            % 圆形机器人
    h_heading = quiver(robot.pos(1), robot.pos(2), 1.5*cos(robot.theta), 1.5*sin(robot.theta),...
                'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 1);        % 朝向箭头
    
    % 主控制循环
    current_idx = 1;
    run_time = tic;
    text_i = 1;
    
    while current_idx < size(cont_path,1)
        
        % 获取前瞻目标点
        [target_idx, ~] = findTargetPoint(cont_path, robot.pos, lookahead_dist, current_idx);
        
        % 计算期望速度方向
        target_pos = cont_path(target_idx,:);
        direction = target_pos - robot.pos;
        desired_theta = atan2(direction(2), direction(1));
        
        % 速度控制（PID示例）
        theta_error = desired_theta - robot.theta;
        omega = 2.0 * theta_error; % 角速度控制
        
        % 线速度控制（带加速度限制）
        desired_vel = max_speed;
        accel = min(max_accel, (desired_vel - robot.vel)/dt);
        robot.vel = robot.vel + accel * dt;
        
        % 更新机器人状态
        robot.theta = robot.theta + omega * dt;
        robot.pos = robot.pos + robot.vel * dt * [cos(robot.theta), sin(robot.theta)];
        
        run_time_1 = toc(run_time);
        title(['运行时间：', num2str(run_time_1), '秒 ']);
        
        % 更新图形
        
        if text_i == 10
            time_text = num2str(run_time_1);
            scatter(robot.pos(1), robot.pos(2), 15, 'k', 'filled');
            text(robot.pos(1), robot.pos(2), time_text);
            text_i = 1;
        else
            text_i = text_i + 1;
        end
        
        set(h_robot, 'Position', [robot.pos(1)-1, robot.pos(2)-1, 2, 2]);
        set(h_heading, 'XData', robot.pos(1), 'YData', robot.pos(2),...
                      'UData', 1.5*cos(robot.theta), 'VData', 1.5*sin(robot.theta));
        
                  % 推进路径索引
                  start_idx = 1;
                  pathsize = size(cont_path);
                  minar0 = cont_path(start_idx, :) - robot.pos;
                  distances_i = sqrt(minar0(1)^2 + minar0(2)^2);
                  distances = [distances_i];
                  % distances = vecnorm(path(start_idx,:) - current_pos, 2, 2);
                  for i = 1:(pathsize(1) - start_idx)
                      minar = cont_path(start_idx + i, :) - robot.pos;
                      distances_i = sqrt(minar(1)^2 + minar(2)^2);
                      distances = [distances;distances_i];
                  end

        [~, current_idx] = min(distances);
        
        % 刷新显示
        drawnow;
        pause(dt); % 模拟实时运行
    end
    disp('连续路径跟踪完成！');
end

% 辅助函数：查找前瞻目标点
function [target_idx, min_dist] = findTargetPoint(path, current_pos, look_dist, start_idx)
    % distances = vecnorm(path(start_idx:end,:) - current_pos, 2, 2);
    pathsize = size(path);
    minar0 = path(start_idx, :) - current_pos;
    distances_i = sqrt(minar0(1)^2 + minar0(2)^2);
    distances = [distances_i];
    % distances = vecnorm(path(start_idx,:) - current_pos, 2, 2);
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
