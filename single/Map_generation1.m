function [map,obs]= Map_generation1(mapsizex, mapsizey, obsnum)
% 地图生成
map=zeros(mapsizex, mapsizey);
sizemap=[mapsizex, mapsizey];

for i=1:obsnum
    map(ceil(rand*sizemap(1)) ,ceil(rand*sizemap(2)))=1; % 修改
end

% 障碍物位置
[row, col] = find(map == 1); % 修改
obs = [row, col];
sizeobs = size(obs);
if sizeobs(1) < obsnum
    Obs_complement = obsnum - sizeobs(1);
    for ioc = 1:Obs_complement
        obs(sizeobs(1) + ioc, :) = obs(sizeobs(1), :);
    end
end


