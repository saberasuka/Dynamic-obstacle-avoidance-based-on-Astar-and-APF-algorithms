function path = RRT(map, initial, goal, Step, pro)

   %把终点作为路径的最后一个节点
    path = goal;
    %计算地图的宽度和长度
    [H,W]=size(map);
    
    %树节点列表，最开始只有起始点
    Tree_List = initial;
    
    %最开始把起始点作为新节点P_new
    P_new  = initial;
    
    %把起始点的父节点置位自己，即其父节点在树节点列表的索引为1
    f_index_list = 1;
    
    %计算新节点到终点的距离
    distance = norm(P_new - goal);
    while distance > Step %当新节点到终点的距离大于步长便一直进行树的生长
        %在地图上随机生成一个点P_rand,
        if rand()< pro   %一定概率朝着结束位置搜索
            P_rand = goal;
        else  
            P_rand = [rand()*W rand()*H];
        end
        
        %在树节点列表中找到离P_rand最近的节点作为P_near节点
        diff = repmat(P_rand,length(Tree_List(:,1)),1)-Tree_List;         %计算P_rand节点与树节点列表中所有的节点的X轴和Y轴的距离
        [~,index] =min(sqrt(diff(:,1).^2+diff(:,2).^2));                   %找P_rand节点与树节点列表中距离最近的节点的索引

        %以树节点列表中与P_rand节点最近的节点作为P_near节点
        P_near = Tree_List(index,:);

        %计算P_near和P_rand连线的角度
        line_angle = atan2(P_rand(2)-P_near(2),P_rand(1)-P_near(1));
        
        %计算新节点的坐标值生成新节点P_new
        x = P_near(1) + Step*cos(line_angle);
        y = P_near(2) + Step*sin(line_angle);
        P_new = [x y];

        %检测P_near和P_new连线之间是否存在障碍物
        isobs = check_obs(map,P_near,P_new);

        %若P_near和P_new连线之间不存在障碍物
        if isobs == 1
             diff = repmat(P_new,length(Tree_List(:,1)),1)-Tree_List;         %判断该路径是否已搜索过，如果已搜索过，则重新搜索
             if min(sqrt(diff(:,1).^2+diff(:,2).^2))<sqrt(Step)
                continue;
             end
            
            %新节点P_new的父节点为P_near，所以其父节点在树节点列表中的索引就是P_near在树节点列表中的索引
            f_index_list = [f_index_list;index];
        
            %把新节点P_new加入到树列表中
            Tree_List = [Tree_List;P_new];
            
            %画出P_near和P_new的连接线 修改 画图
            x = [P_near(1),P_new(1)]; % 线段的x坐标
            y = [P_near(2),P_new(2)]; % 线段的y坐标
            plot(x, y, '-b*', 'LineWidth', 1); % 画线段
            hold on; 

            
            %计算新节点P_new到终点的距离
            distance = norm(P_new-goal);
        else 
            %若P_near和P_new连线之间存在障碍物则重新生成P_rand
            continue
        end
    end
    
    %把终点加入到树节点列表，其父节点为最新的P_new节点
    E_f_index = length(Tree_List(:,1));
    Tree_List = [Tree_List;goal];
    f_index_list = [f_index_list;E_f_index];
    
    %画出最后一个P_new和终点的连接线 修改 画图
    x = [Tree_List(E_f_index,1),goal(1)]; % 线段的x坐标
    y = [Tree_List(E_f_index,2),goal(2)]; % 线段的y坐标
    plot(x, y, '-b*', 'LineWidth', 1); % 画线段
    hold on;
    
    %到此随机树已经生成完毕，接下来就是搜索路径了
    %最开始把终点作为当前节点
    current_node = goal;
    
    %终点的父节点即父节点索引列表的最后一个元素
    f_index = f_index_list(end);
    while f_index ~= 1
        %找到当前节点的父节点
        father_node = Tree_List(f_index,:);
        %把父节点插入到路径列表的最前面
        path = [father_node;path];
        %父节点变成当前节点
        current_node = father_node;
        %找到当前节点在树节点列表中的索引
        
%         disp(['Tree_List 尺寸: ', num2str(size(Tree_List))]);
%         disp(['current_node 尺寸: ', num2str(size(current_node))]);
        temp_index = find(ismember(Tree_List, current_node, 'rows'));
        %temp_index = find(Tree_List == current_node);
        current_index = temp_index(1);
        %在父节点列表中找到父节点索引
        f_index = f_index_list(current_index);
    end
    
    %把起始点加入到路径最前面
    path = [initial;path];
    
    %画出路径连接线 修改 画图
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2); % 画线段
    hold on;
    
    return
end

% 检测地图中两个节点之间是否存在障碍物
function isobs = check_obs(map,P1,P2)

    % 判断两点之间是否存在障碍物的标志
    isobs = 1;
    
    %计算地图的宽度和长度
    [H,W]=size(map);
    
    %计算两个节点之间的距离
    d = norm(P1-P2);
    
    %计算两个节点之间连线的角度
    line_angle = atan2(P2(2)-P1(2),P2(1)-P1(1));
    
    %判断从P1到P2的连线每一个点是否在障碍物上
    for r = 0:d
        x = floor(P1(1) + r*cos(line_angle));
        y = floor(P1(2) + r*sin(line_angle));
        
        %先判断该点是否溢出边界，溢出边界则视为存在障碍物
        if x > 0 && x < W && y > 0 && y < H
            %判断该点是否在障碍物上
            if map(y,x) == 0 %该点在障碍物上，判断这两个节点之间存在障碍物，直接返回
                isobs = 0;
                return
            end
        else
            isobs = 0;
            return
        end
    end
end

