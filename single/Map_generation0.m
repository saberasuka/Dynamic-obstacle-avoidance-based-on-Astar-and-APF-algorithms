function [map,obs]= Map_generation0(mapsizex, mapsizey, obsnum)
% ��ͼ����
map=ones(mapsizex, mapsizey);
sizemap=[mapsizex, mapsizey];

for i=1:obsnum
    map(ceil(rand*sizemap(1)) ,ceil(rand*sizemap(2)))=0; % �޸�
end

% �ϰ���λ��
[row, col] = find(map == 0); % �޸�
obs = [row, col];
sizeobs = size(obs);
if sizeobs(1) < obsnum
    Obs_complement = obsnum - sizeobs(1);
    for ioc = 1:Obs_complement
        obs(sizeobs(1) + ioc, :) = obs(sizeobs(1), :);
    end
end


