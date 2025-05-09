function [apfpath, workout] = APF(initial, goal, obs)

startX = initial(1);  % 出发点位置
startY = initial(2);
desX = goal(1);  % 终点位置
desY = goal(2);

sizeobs = size(obs);
obsnum = sizeobs(1);
%% 超参数设置
% 斥力作用范围，跟随地图大小更改             
    Kaat = 0.5;                     % 引力尺度因子
    Krep = 0.5;                     % 斥力尺度因子
    P0 = 25;                        % 斥力作用范围
    StepRate = 0.1;                 % 步长
    Epoch = 2000;                   % 最大迭代次数
    de = 20;                        % 引力距离因子
%% 算法主体
CountFlag = 0;
apfpath = initial;
while(1)
   [Fattx,Fatty] = Attractive(startX,desX,startY,desY,Kaat,de); 
   Frepx = zeros(1, obsnum); 
   Frepy = zeros(1, obsnum);
   for i = 1:obsnum
       [Frepx(1,i),Frepy(1,i)] = Repulsive(startX,startY,obs(i,1),obs(i,2),desX,desY,Krep,P0);
   end

   Fxsum = Fattx + sum(Frepx);
   Fysum = Fatty + sum(Frepy);

   startX = startX + StepRate*Fxsum;
   startY = startY + StepRate*Fysum;

   apfpath = [apfpath;
       startX, startY];  % 人工势场 路径

   if(abs(startX-goal(1)) < 1 && abs(startY-goal(2))< 1 )
       % fprintf('人工势场路径搜索完成\n')
       workout = 1;
       break;
   end

   CountFlag = CountFlag + 1;
   if(CountFlag >= Epoch)
       % fprintf('超时，人工势场路径搜索完成\n')
       workout = 0;
       break;
   end    
end
end