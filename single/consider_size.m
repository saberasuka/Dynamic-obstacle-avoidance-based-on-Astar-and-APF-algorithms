function [inflatedm] = consider_size(map, width)
% ���������
    [i, j] = size(map);
    % �� w ת��Ϊż����ȷ�����ͶԳƣ�
    width = ceil(width/2) * 2;
    inflatedm = map;
    
    for ii = width/2+1 : i-width/2
        for jj = width/2+1 : j-width/2
            % ���ԭ��ͼ��λ�����ϰ���
            if map(ii, jj) == 1
                % ��ֹ��խͨ������
                obsarray = map(ii + width/2 : ii - width/2, jj + width/2 : jj - width/2);
                obsarray(width/2 + 1, width/2 + 1) = 0;
                if sum(obsarray(:) == 1) == 0
                    % �ڵ�ͼ�б����Χ����Ϊ�ϰ���
                    inflatedm(ii-width/2 : ii+width/2, jj-width/2 : jj+width/2) = 1;
                end
            end
        end
    end
end
