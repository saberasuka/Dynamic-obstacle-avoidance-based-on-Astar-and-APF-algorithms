function [process]=expand_node (map, parent, h, V, H)
process=[];
% Ѱ����Χ�ڵ�
for i=-1:1
    for j=-1:1
        % б��
        if j~=0 && i~=0 && parent(1)+i>0 && parent(1)+i<=V && parent(2)+j>0 && parent(2)+j<=H
            if map(parent(1)+i, parent(2)+j) == 0
                c=parent(3) + sqrt(2) + h(parent(1)+i, parent(2)+j) - h(parent(1), parent(2));
                if ((parent(1)+i) ~= parent(4) || (parent(2)+j) ~= parent(5))
                    process=[process; [parent(1)+i, parent(2)+j, c, parent(1),parent(2)]];
                end
            end
            % ����
        elseif (j==0 || i==0) && j~=i && parent(1)+i>0 && parent(1)+i<=V && parent(2)+j>0 && parent(2)+j<=H
            if map(parent(1)+i, parent(2)+j) == 0
                c=parent(3) + 1 + h(parent(1)+i, parent(2)+j) - h(parent(1), parent(2));
                if ((parent(1)+i)~=parent(4)|| (parent(2)+j)~=parent(5))
                    process=[process; [parent(1)+i, parent(2)+j, c, parent(1),parent(2)]];
                end
            end
        end
    end
end
end
