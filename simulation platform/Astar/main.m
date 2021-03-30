%% 1.生成地图
close all;clc;clear;
% 确定地图大小,m和n满足4*x+5的规律
m = 4*randi([4,10],1)+5;
n = 4*randi([4,10],1)+5;
[Map,Map_Spoint,Map_Epoint] = Map_Generate(m,n);
% 画出地图
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
%% 2.A*算法
Routing = Astar(Map,Map_Spoint,Map_Epoint,m,n);
i = 1;
while(1)
    h4 = plot(Routing(1,i),Routing(2,i),'b+');
    i = i+1;
    if(Routing(1,i) == Map_Spoint(1)&&Routing(2,i) == Map_Spoint(2))  %路径点就是起点,画图结束,跳出循环
        plot(Routing(1,i),Routing(2,i),'b+');
        break;
    end
end
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing');
%% 3.路径优化
Smooth_path = Routing_Smoothing(Routing);
%% 4.结果可视化
figure;
plot(Routing(1,:),Routing(2,:),'b');
axis([-2 m+2 -2 n+2]);
title('Route planing');
figure;
plot(Smooth_path(1,:),Smooth_path(2,:),'r');
axis([-2 m+2 -2 n+2]);
title('Route planing after smoothing');    