function [path] = Astar (map, initial, goal, width)
% A*主算法

map = consider_size(map, width); % 机器人体积

msize = size(map);
h = [];  % 计算每个单元格的启发式值。估计为目标和单元格之间的直线距离
for i = 1:msize(1)
    for j = 1:msize(2)
        if map(i,j) == 1
            % h(i,j) = Inf;
            h(i,j) = 100 * sqrt(2);
        else
            h(i,j) = sqrt((i-goal(1))^2+(j-goal(2))^2);  % 欧几里得距离
        end
    end
end

open = [initial(1), initial(2), h(initial(1), initial(2)), -1, -1];
closed=[];
[~, n] = min(open(:,3));
% 初始节点
current = open(n,:);
% 逐步搜索
while ~isequal(current(1:2),goal)
    % 扩充周围节点,除上一点和现在点
    process = expand_node(map, current, h, msize(1), msize(2));
    closed=[closed;
        current];
    % 删去现节点
    open = open([1:n-1, n+1:end], :);
    if isempty(process)==0
        for i=1:length(process(:, 1))
            n_open = idxget(open(:,1:2), process(i,1:2));
            if n_open ~= 0 
                if(process(i,3) < open(n_open, 3))
                    open(n_open,:) = process(i,:);
                end
            else
                n_close = idxget(closed(:,1:2), process(i,1:2));
                if n_close == 0
                    open = [open; process(i,:)];
                else
                    if (process(i,3) < closed(n_close,3))
                        closed(n_close, :) = process(i, :);
                    end
                end
            end
        end
    end
    [~, n] = min(open(:,3));
    current = open(n,:);
end
% 整理路径
closed = [closed ;current];
node = current;
path = current(1:2);
while ~isequal(node(4:5), initial)
    path = [node(4:5); path];
    node = closed(idxget(closed(:,1:2), node(4:5)),:);
end
fprintf('A*路径搜索完成\n')
end
