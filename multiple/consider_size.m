function [inflatedm] = consider_size(map, width)
% 机器人体积
    [i, j] = size(map);
    % 将 w 转换为偶数（确保膨胀对称）
    width = ceil(width/2) * 2;
    inflatedm = map;
    
    for ii = width/2+1 : i-width/2
        for jj = width/2+1 : j-width/2
            % 如果原地图该位置是障碍物
            if map(ii, jj) == 1
                % 防止较窄通道阻塞
                obsarray = map(ii + width/2 : ii - width/2, jj + width/2 : jj - width/2);
                obsarray(width/2 + 1, width/2 + 1) = 0;
                if sum(obsarray(:) == 1) == 0
                    % 在地图中标记周围区域为障碍物
                    inflatedm(ii-width/2 : ii+width/2, jj-width/2 : jj+width/2) = 1;
                end
            end
        end
    end
end
