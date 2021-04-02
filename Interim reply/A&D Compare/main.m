%% 1.生成地图
close all;clc;clear;
% 确定地图大小,m和n满足4*x+5的规律
% m = 4*randi([4,10],1)+5;
% n = 4*randi([4,10],1)+5;
m = 45;n = 45;
[Map,Map_Spoint,Map_Epoint] = Map_Generate(m,n);
save complex_map_6 Map Map_Spoint Map_Epoint;
%load complex_map;
% 画出地图
h1 = plot(Map_Spoint(1),Map_Spoint(2),'gO');
hold on;
h2 = plot(Map_Epoint(1),Map_Epoint(2),'rO');
title('Route planing with A*/D algorithms');
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
%% 2.画出三种算法得到的路径
Routing_Astar_Diagonal = Astar_Diagonal(Map,Map_Spoint,Map_Epoint,m,n);
Routing_Astar_Euclid = Astar_Euclid(Map,Map_Spoint,Map_Epoint,m,n);
Routing_Dstar = Dstar(Map,Map_Spoint,Map_Epoint,m,n);
i = 1;
while(1)
    h4 = plot(Routing_Astar_Diagonal(1,i),Routing_Astar_Diagonal(2,i),'b+');
    i = i+1;
    if(Routing_Astar_Diagonal(1,i) == Map_Spoint(1)&&Routing_Astar_Diagonal(2,i) == Map_Spoint(2))  %路径点就是起点,画图结束,跳出循环
        plot(Routing_Astar_Diagonal(1,i),Routing_Astar_Diagonal(2,i),'b+');
        break;
    end
end
hold on;
j = 1;
while(1)
    h5 = plot(Routing_Astar_Euclid(1,j),Routing_Astar_Euclid(2,j),'r+');
    j = j+1;
    if(Routing_Astar_Euclid(1,j) == Map_Spoint(1)&&Routing_Astar_Euclid(2,j) == Map_Spoint(2))  %路径点就是起点,画图结束,跳出循环
        plot(Routing_Astar_Euclid(1,j),Routing_Astar_Euclid(2,j),'r+');
        break;
    end
end
hold on;
k = 1;
while(1)
    h6 = plot(Routing_Dstar(1,k),Routing_Dstar(2,k),'g+');
    k = k+1;
    if(Routing_Dstar(1,k) == Map_Spoint(1)&&Routing_Dstar(2,k) == Map_Spoint(2))  %路径点就是起点,画图结束,跳出循环
        plot(Routing_Dstar(1,k),Routing_Dstar(2,k),'g+');
        break;
    end
end
legend([h1,h2,h3,h4,h5,h6],'Start Point','End Point','Obstacle','Routing_Astar_Diagonal','Routing_Astar_Euclid','Routing_Dstar','Location','NorthEastOutside');
%% 3.路径优化
Smooth_path_Astar_Diagonal = Routing_Smoothing(Routing_Astar_Diagonal);
Smooth_path_Astar_Euclid= Routing_Smoothing(Routing_Astar_Euclid);
Smooth_path_Dstar= Routing_Smoothing(Routing_Dstar);
%% 4.结果可视化
figure;
plot(Routing_Astar_Diagonal(1,:),Routing_Astar_Diagonal(2,:),'b');
hold on;
plot(Routing_Astar_Euclid(1,:),Routing_Astar_Euclid(2,:),'r');
hold on;
plot(Routing_Dstar(1,:),Routing_Dstar(2,:),'g');
axis([-2 m+2 -2 n+2]);
title('Route planing');
figure;
plot(Smooth_path_Astar_Diagonal(1,:),Smooth_path_Astar_Diagonal(2,:),'b');
hold on;
plot(Smooth_path_Astar_Euclid(1,:),Smooth_path_Astar_Euclid(2,:),'r');
hold on;
plot(Smooth_path_Dstar(1,:),Smooth_path_Dstar(2,:),'g');
axis([-2 m+2 -2 n+2]);
title('Route planing after smoothing');    