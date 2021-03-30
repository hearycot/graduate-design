%% 1.���ɵ�ͼ
close all;clc;clear;
% ȷ����ͼ��С,m��n����4*x+5�Ĺ���
m = 4*randi([4,10],1)+5;
n = 4*randi([4,10],1)+5;
[Map,Map_Spoint,Map_Epoint] = Map_Generate(m,n);
% ������ͼ
h1 = plot(Map_Spoint(1),Map_Spoint(2),'gO');
hold on;
h2 = plot(Map_Epoint(1),Map_Epoint(2),'rO');
title('Route planing with A* algorithms');
axis([-2 m+2 -2 n+2]);
for i = 1:m
    for j = 1:n
        if (Map(i,j) == -inf)
            h3 = plot(i,j,'k.');
        end
    end
end
plot(Map_Spoint(1),Map_Spoint(2),'b+');
plot(Map_Epoint(1),Map_Epoint(2),'b+');
%% 2.A*�㷨
Routing = Astar(Map,Map_Spoint,Map_Epoint,m,n);
i = 1;
while(1)
    h4 = plot(Routing(1,i),Routing(2,i),'b+');
    i = i+1;
    if(Routing(1,i) == Map_Spoint(1)&&Routing(2,i) == Map_Spoint(2))  %·����������,��ͼ����,����ѭ��
        plot(Routing(1,i),Routing(2,i),'b+');
        break;
    end
end
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing');
%% 3.·���Ż�
Smooth_path = Routing_Smoothing(Routing);
%% 4.������ӻ�
figure;
plot(Routing(1,:),Routing(2,:),'b');
axis([-2 m+2 -2 n+2]);
title('Route planing');
figure;
plot(Smooth_path(1,:),Smooth_path(2,:),'r');
axis([-2 m+2 -2 n+2]);
title('Route planing after smoothing');    