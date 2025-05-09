function [apfpath, workout] = APF(initial, goal, obs)

startX = initial(1);  % ������λ��
startY = initial(2);
desX = goal(1);  % �յ�λ��
desY = goal(2);

sizeobs = size(obs);
obsnum = sizeobs(1);
%% ����������
% �������÷�Χ�������ͼ��С����             
    Kaat = 0.5;                     % �����߶�����
    Krep = 0.5;                     % �����߶�����
    P0 = 25;                        % �������÷�Χ
    StepRate = 0.1;                 % ����
    Epoch = 2000;                   % ����������
    de = 20;                        % ������������
%% �㷨����
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
       startX, startY];  % �˹��Ƴ� ·��

   if(abs(startX-goal(1)) < 1 && abs(startY-goal(2))< 1 )
       % fprintf('�˹��Ƴ�·���������\n')
       workout = 1;
       break;
   end

   CountFlag = CountFlag + 1;
   if(CountFlag >= Epoch)
       % fprintf('��ʱ���˹��Ƴ�·���������\n')
       workout = 0;
       break;
   end    
end
end