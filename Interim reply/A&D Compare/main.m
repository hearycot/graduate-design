%% 1.���ɵ�ͼ
close all;clc;clear;
% ȷ����ͼ��С,m��n����4*x+5�Ĺ���
% m = 4*randi([4,10],1)+5;
% n = 4*randi([4,10],1)+5;
m = 45;n = 45;
%m = 30;n = 30;
%[Map,Map_Spoint,Map_Epoint] = Map_Generate(m,n);
%save complex_map_7 Map Map_Spoint Map_Epoint;
load complex_map;
%% 2.���������㷨�õ���·��
Routing_Astar_Diagonal = Astar_Diagonal(Map,Map_Spoint,Map_Epoint,m,n);
% ������ͼ
h1 = plot(Map_Spoint(1),Map_Spoint(2),'gO');
hold on;
h2 = plot(Map_Epoint(1),Map_Epoint(2),'rO');
title('Route planing with Astar_Diagonal algorithms');
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
i = 1;
while(1)
    h4 = plot(Routing_Astar_Diagonal(1,i),Routing_Astar_Diagonal(2,i),'b+');
    i = i+1;
    if(Routing_Astar_Diagonal(1,i) == Map_Spoint(1)&&Routing_Astar_Diagonal(2,i) == Map_Spoint(2))  %·����������,��ͼ����,����ѭ��
        plot(Routing_Astar_Diagonal(1,i),Routing_Astar_Diagonal(2,i),'b+');
        break;
    end
end
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing_Astar_Diagonal','Location','NorthEastOutside');
figure;
Routing_Astar_Euclid = Astar_Euclid(Map,Map_Spoint,Map_Epoint,m,n);
% ������ͼ
h1 = plot(Map_Spoint(1),Map_Spoint(2),'gO');
hold on;
h2 = plot(Map_Epoint(1),Map_Epoint(2),'rO');
title('Route planing with Astar_Diagonal algorithms');
axis([-2 m+2 -2 n+2]);
for i = 1:m
    for j = 1:n
        if (Map(i,j) == -inf)
            h3 = plot(i,j,'k.');
        end
    end
end
plot(Map_Spoint(1),Map_Spoint(2),'r+');
plot(Map_Epoint(1),Map_Epoint(2),'r+');
j = 1;
while(1)
    h4 = plot(Routing_Astar_Euclid(1,j),Routing_Astar_Euclid(2,j),'r+');
    j = j+1;
    if(Routing_Astar_Euclid(1,j) == Map_Spoint(1)&&Routing_Astar_Euclid(2,j) == Map_Spoint(2))  %·����������,��ͼ����,����ѭ��
        plot(Routing_Astar_Euclid(1,j),Routing_Astar_Euclid(2,j),'r+');
        break;
    end
end
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing_Astar_Euclid','Location','NorthEastOutside');
figure;
Routing_Dstar = Dstar(Map,Map_Spoint,Map_Epoint,m,n);
% ������ͼ
h1 = plot(Map_Spoint(1),Map_Spoint(2),'gO');
hold on;
h2 = plot(Map_Epoint(1),Map_Epoint(2),'rO');
title('Route planing with Dstar algorithms');
axis([-2 m+2 -2 n+2]);
for i = 1:m
    for j = 1:n
        if (Map(i,j) == -inf)
            h3 = plot(i,j,'k.');
        end
    end
end
plot(Map_Spoint(1),Map_Spoint(2),'g+');
plot(Map_Epoint(1),Map_Epoint(2),'g+');
k = 1;
while(1)
    h4 = plot(Routing_Dstar(1,k),Routing_Dstar(2,k),'g+');
    k = k+1;
    if(Routing_Dstar(1,k) == Map_Spoint(1)&&Routing_Dstar(2,k) == Map_Spoint(2))  %·����������,��ͼ����,����ѭ��
        plot(Routing_Dstar(1,k),Routing_Dstar(2,k),'g+');
        break;
    end
end
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing_Astar_Euclid','Location','NorthEastOutside');
%% 3.·���Ż�
Smooth_path_Astar_Diagonal = Routing_Smoothing(Routing_Astar_Diagonal);
Smooth_path_Astar_Euclid= Routing_Smoothing(Routing_Astar_Euclid);
Smooth_path_Dstar= Routing_Smoothing(Routing_Dstar);
%% 4.������ӻ�
figure;
plot(Routing_Astar_Diagonal(1,:),Routing_Astar_Diagonal(2,:),'b');
axis([-2 m+2 -2 n+2]);
title('Astar_Diagonal Route planing');
figure;
plot(Routing_Astar_Euclid(1,:),Routing_Astar_Euclid(2,:),'r');
axis([-2 m+2 -2 n+2]);
title('Astar_Euclid Route planing');
figure;
plot(Routing_Dstar(1,:),Routing_Dstar(2,:),'g');
axis([-2 m+2 -2 n+2]);
title('Dstar Route planing');
figure;
plot(Smooth_path_Astar_Diagonal(1,:),Smooth_path_Astar_Diagonal(2,:),'b');
hold on;
plot(Smooth_path_Astar_Euclid(1,:),Smooth_path_Astar_Euclid(2,:),'r');
hold on;
plot(Smooth_path_Dstar(1,:),Smooth_path_Dstar(2,:),'g');
axis([-2 m+2 -2 n+2]);
title('All Route planing after smoothing');    