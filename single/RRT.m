function path = RRT(map, initial, goal, Step, pro)

   %���յ���Ϊ·�������һ���ڵ�
    path = goal;
    %�����ͼ�Ŀ�Ⱥͳ���
    [H,W]=size(map);
    
    %���ڵ��б��ʼֻ����ʼ��
    Tree_List = initial;
    
    %�ʼ����ʼ����Ϊ�½ڵ�P_new
    P_new  = initial;
    
    %����ʼ��ĸ��ڵ���λ�Լ������丸�ڵ������ڵ��б������Ϊ1
    f_index_list = 1;
    
    %�����½ڵ㵽�յ�ľ���
    distance = norm(P_new - goal);
    while distance > Step %���½ڵ㵽�յ�ľ�����ڲ�����һֱ������������
        %�ڵ�ͼ���������һ����P_rand,
        if rand()< pro   %һ�����ʳ��Ž���λ������
            P_rand = goal;
        else  
            P_rand = [rand()*W rand()*H];
        end
        
        %�����ڵ��б����ҵ���P_rand����Ľڵ���ΪP_near�ڵ�
        diff = repmat(P_rand,length(Tree_List(:,1)),1)-Tree_List;         %����P_rand�ڵ������ڵ��б������еĽڵ��X���Y��ľ���
        [~,index] =min(sqrt(diff(:,1).^2+diff(:,2).^2));                   %��P_rand�ڵ������ڵ��б��о�������Ľڵ������

        %�����ڵ��б�����P_rand�ڵ�����Ľڵ���ΪP_near�ڵ�
        P_near = Tree_List(index,:);

        %����P_near��P_rand���ߵĽǶ�
        line_angle = atan2(P_rand(2)-P_near(2),P_rand(1)-P_near(1));
        
        %�����½ڵ������ֵ�����½ڵ�P_new
        x = P_near(1) + Step*cos(line_angle);
        y = P_near(2) + Step*sin(line_angle);
        P_new = [x y];

        %���P_near��P_new����֮���Ƿ�����ϰ���
        isobs = check_obs(map,P_near,P_new);

        %��P_near��P_new����֮�䲻�����ϰ���
        if isobs == 1
             diff = repmat(P_new,length(Tree_List(:,1)),1)-Tree_List;         %�жϸ�·���Ƿ����������������������������������
             if min(sqrt(diff(:,1).^2+diff(:,2).^2))<sqrt(Step)
                continue;
             end
            
            %�½ڵ�P_new�ĸ��ڵ�ΪP_near�������丸�ڵ������ڵ��б��е���������P_near�����ڵ��б��е�����
            f_index_list = [f_index_list;index];
        
            %���½ڵ�P_new���뵽���б���
            Tree_List = [Tree_List;P_new];
            
            %����P_near��P_new�������� �޸� ��ͼ
            x = [P_near(1),P_new(1)]; % �߶ε�x����
            y = [P_near(2),P_new(2)]; % �߶ε�y����
            plot(x, y, '-b*', 'LineWidth', 1); % ���߶�
            hold on; 

            
            %�����½ڵ�P_new���յ�ľ���
            distance = norm(P_new-goal);
        else 
            %��P_near��P_new����֮������ϰ�������������P_rand
            continue
        end
    end
    
    %���յ���뵽���ڵ��б��丸�ڵ�Ϊ���µ�P_new�ڵ�
    E_f_index = length(Tree_List(:,1));
    Tree_List = [Tree_List;goal];
    f_index_list = [f_index_list;E_f_index];
    
    %�������һ��P_new���յ�������� �޸� ��ͼ
    x = [Tree_List(E_f_index,1),goal(1)]; % �߶ε�x����
    y = [Tree_List(E_f_index,2),goal(2)]; % �߶ε�y����
    plot(x, y, '-b*', 'LineWidth', 1); % ���߶�
    hold on;
    
    %����������Ѿ�������ϣ���������������·����
    %�ʼ���յ���Ϊ��ǰ�ڵ�
    current_node = goal;
    
    %�յ�ĸ��ڵ㼴���ڵ������б�����һ��Ԫ��
    f_index = f_index_list(end);
    while f_index ~= 1
        %�ҵ���ǰ�ڵ�ĸ��ڵ�
        father_node = Tree_List(f_index,:);
        %�Ѹ��ڵ���뵽·���б����ǰ��
        path = [father_node;path];
        %���ڵ��ɵ�ǰ�ڵ�
        current_node = father_node;
        %�ҵ���ǰ�ڵ������ڵ��б��е�����
        
%         disp(['Tree_List �ߴ�: ', num2str(size(Tree_List))]);
%         disp(['current_node �ߴ�: ', num2str(size(current_node))]);
        temp_index = find(ismember(Tree_List, current_node, 'rows'));
        %temp_index = find(Tree_List == current_node);
        current_index = temp_index(1);
        %�ڸ��ڵ��б����ҵ����ڵ�����
        f_index = f_index_list(current_index);
    end
    
    %����ʼ����뵽·����ǰ��
    path = [initial;path];
    
    %����·�������� �޸� ��ͼ
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2); % ���߶�
    hold on;
    
    return
end

% ����ͼ�������ڵ�֮���Ƿ�����ϰ���
function isobs = check_obs(map,P1,P2)

    % �ж�����֮���Ƿ�����ϰ���ı�־
    isobs = 1;
    
    %�����ͼ�Ŀ�Ⱥͳ���
    [H,W]=size(map);
    
    %���������ڵ�֮��ľ���
    d = norm(P1-P2);
    
    %���������ڵ�֮�����ߵĽǶ�
    line_angle = atan2(P2(2)-P1(2),P2(1)-P1(1));
    
    %�жϴ�P1��P2������ÿһ�����Ƿ����ϰ�����
    for r = 0:d
        x = floor(P1(1) + r*cos(line_angle));
        y = floor(P1(2) + r*sin(line_angle));
        
        %���жϸõ��Ƿ�����߽磬����߽�����Ϊ�����ϰ���
        if x > 0 && x < W && y > 0 && y < H
            %�жϸõ��Ƿ����ϰ�����
            if map(y,x) == 0 %�õ����ϰ����ϣ��ж��������ڵ�֮������ϰ��ֱ�ӷ���
                isobs = 0;
                return
            end
        else
            isobs = 0;
            return
        end
    end
end

