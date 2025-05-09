function [path] = Astar (map, initial, goal, width)
% A*���㷨

map = consider_size(map, width); % ���������

msize = size(map);
h = [];  % ����ÿ����Ԫ�������ʽֵ������ΪĿ��͵�Ԫ��֮���ֱ�߾���
for i = 1:msize(1)
    for j = 1:msize(2)
        if map(i,j) == 1
            % h(i,j) = Inf;
            h(i,j) = 100 * sqrt(2);
        else
            h(i,j) = sqrt((i-goal(1))^2+(j-goal(2))^2);  % ŷ����þ���
        end
    end
end

open = [initial(1), initial(2), h(initial(1), initial(2)), -1, -1];
closed=[];
[~, n] = min(open(:,3));
% ��ʼ�ڵ�
current = open(n,:);
% ������
while ~isequal(current(1:2),goal)
    % ������Χ�ڵ�,����һ������ڵ�
    process = expand_node(map, current, h, msize(1), msize(2));
    closed=[closed;
        current];
    % ɾȥ�ֽڵ�
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
% ����·��
closed = [closed ;current];
node = current;
path = current(1:2);
while ~isequal(node(4:5), initial)
    path = [node(4:5); path];
    node = closed(idxget(closed(:,1:2), node(4:5)),:);
end
fprintf('A*·���������\n')
end
