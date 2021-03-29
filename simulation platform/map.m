%% 利用随机函数生成地图
%% 1. 初始化参数
close all;clc;clear;
% 地图大小
m = 29;
n = 29;
% 构建地图
% -inf表示不可到达的障碍物点,inf代表可到达空间
% 构建边界
for i = 2:m-1
    for j = 2:n-1
        Matrix(i,j) = inf;
    end
end   
for j = 1:n
    Matrix(1,j) = -inf;
    Matrix(29,j) = -inf;
end
for i = 1:m
    Matrix(i,1) = -inf;
    Matrix(i,29) = -inf;
end
% 构建障碍物
% 切割地图,在每个图单元生成障碍物
count = 1;
i = 1;j = 1;
length = 5;
temp_watch = [];
while(count ~= 7*7)
    type = randi(2);
    coordinate_x = i+2;
    coordinate_y = j+2;
    temp_watch = [temp_watch;coordinate_x coordinate_y length];
    if type == 1
        % 生成横向障碍物
        for k = 1:length
            Matrix(coordinate_x+k-3,coordinate_y) = -inf;
        end
    else 
        % 生成纵向障碍物
        for k = 1:length
            Matrix(coordinate_x,coordinate_y+k-3) = -inf;
        end
    end
    count = count+1;
    i = i+4;
    if i >= 29
        i = 1;
        j = j+4;
    end
end
% 固定起点和终点坐标
% Spoint_x = 2;Spoint_y = 2;
% Epoint_x = 28;Epoint_y = 28;
% 随机起点和终点坐标
Spoint_x =0;Spoint_y =0;
Epoint_x =0;Epoint_y =0;
while ((Spoint_x == Epoint_x)&&(Spoint_y == Epoint_y)||Matrix(Spoint_x,Spoint_y) == -inf||Matrix(Epoint_x,Epoint_y) == -inf)
    Spoint_x = randi([2,28],1);
    Spoint_y = randi([2,28],1);
    Epoint_x = randi([2,28],1);
    Epoint_y = randi([2,28],1);   
end
Spoint = [Spoint_x Spoint_y];
Epoint = [Epoint_x Epoint_y];
% 显示地图
h1 = plot(Spoint(1),Spoint(2),'gO');
hold on
h2 = plot(Epoint(1),Epoint(2),'rO');
title('Route planing with A* algorithms');
axis([0 m+2 0 n+2]);
for i = 1:m
    for j = 1:n
        if (Matrix(i,j) == -inf)
            h3 = plot(i,j,'k.');
        end
    end
end

%% 3. A*算法搜索路径
%%寻路
Matrix(Spoint(1),Spoint(2))=0;
Matrix(Epoint(1),Epoint(2))=inf;
G=Matrix;
F=Matrix;
openlist=Matrix;
closelist=Matrix;
parentx=Matrix;
parenty=Matrix;
openlist(Spoint(1),Spoint(2)) =0;
%closelist(Epoint(1),Epoint(2))=inf;
%subplot(2,2,1);
plot(Epoint(1),Epoint(2),'b+');
%subplot(2,2,1);
plot(Spoint(1),Spoint(2),'b+');
while(1)
    num=inf;
    for p=1:m
        for q=1:n
            % 这里是不是可以加一个break？找到起始点就跳出循环，避免无用步骤，默认只有一个起点
            % open表为0，表示可以作为拓展对象，close表为1，表示已经走过了，不需要再作为下一次起点
            if(openlist(p,q)==0&&closelist(p,q)~=1)
                Outpoint=[p,q];
                % 操作集F大于0，并且小于上一集操作集时，才存入nextpoint，否则略过
                if(F(p,q)>=0&&num>F(p,q))
                    num=F(p,q);
                    Nextpoint=[p,q];
                end
            end
        end
    end
    closelist(Nextpoint(1),Nextpoint(2))=1;
    for i = 1:3
        for j = 1:3
            k = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
            if(i==2&&j==2|closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)==1)
                continue;
            elseif (k == -inf)
                % 下面一句是否可以删掉？
                % 无路可走，自己的状态集不发生改变
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j) = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
                closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=1;
            elseif (k == inf)
                distance=((i-2)^2+(j-2)^2)^0.5;
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1),Nextpoint(2))+distance;
                openlist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=0;
                % H=((Nextpoint(1)-a2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;%欧几里德距离启发函数
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较复杂的对角线启发函数
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较简单的对角线函数
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)+H;
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
            else distance=((i-2)^2+(j-2)^2)^0.5;
                if(k>(distance+G(Nextpoint(1),Nextpoint(2))))
                    k=distance+G(Nextpoint(1),Nextpoint(2));
                    % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;  %欧几里德距离启发函数
                    H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较复杂的对角线启发函数
                    H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                    H=sqrt(2)*10*H_diagonal+10*(H_straight-2*H_diagonal);
                    % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较简单的对角线函数
                    F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=k+H;
                    parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                    parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
                end
            end
            if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
                parentx(Epoint(1),Epoint(2))=Nextpoint(1);
                parenty(Epoint(1),Epoint(2))=Nextpoint(2);
                break;
            end
        end
        if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
            parentx(Epoint(1),Epoint(2))=Nextpoint(1);
            parenty(Epoint(1),Epoint(2))=Nextpoint(2);
            break;
        end
    end
    if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
        parentx(Epoint(1),Epoint(2))=Nextpoint(1);
        parenty(Epoint(1),Epoint(2))=Nextpoint(2);
        break;
    end
end
P=[];
s=1;
while(1)
    if(num==inf)
        break;
    end
    %subplot(2,2,1);
    h4 = plot(Epoint(1),Epoint(2),'b+');
    P(s,:)=Epoint;
    s=s+1;
    % pause(1);
    % 这里的xx似乎可以删掉？
    xx=Epoint(1);
    % 取出父亲节点
    Epoint(1)=parentx(Epoint(1),Epoint(2));
    Epoint(2)=parenty(xx,Epoint(2));
    if(parentx(Epoint(1),Epoint(2))==Spoint(1)&&parenty(Epoint(1),Epoint(2))==Spoint(2))
        %subplot(2,2,1);
        plot(Epoint(1),Epoint(2),'b+');
        P(s,:)=Epoint;
        break;
    end
end
P(s+1,:)=Spoint;
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing');
%% 4. 路径优化
%将得到的折现曲线拟合成光滑的曲线
P=P';
a=[];
b=[];
a=P(1,:);
% 这里应该少了一句b取第二行元素
b=P(2,:);
figure
%subplot(2,2,3);
plot(a,b);
axis([0,n+3,0,n+3]);
title('Route planing');
values = spcrv([[a(1) a a(end)];[b(1) b b(end)]],3);
%% 5. 结果可视化
figure
%subplot(2,2,4);
plot(values(1,:),values(2,:),'r');
axis([0,m+3,0,m+3]);
title('Route planing after smoothing');    