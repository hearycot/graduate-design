function Routing = Dstar(Map,Spoint,Epoint,m,n)
    % 寻路D算法
    Map(Spoint(1),Spoint(2)) = 0;
    Map(Epoint(1),Epoint(2)) = inf;
    openlist = Map;
    closelist = Map;
    G_set = Map;
    parent_x = Map;
    parent_y = Map;
    while(1)
        for p = 1:m
            for q = 1:n
                if (openlist(p,q) == 0&&closelist(p,q) == 0)   % openlist为0,代表可到达;colselist为2,代表是障碍物，无需考察;colselist为1，代表是需要考察的点 
                    Nextpoint=[p,q];
                    break;
                end
            end
            if (openlist(p,q) == 0&&closelist(p,q) == 0)   % openlist为0,代表可到达;colselist为2,代表是障碍物，无需考察;colselist为1，代表是需要考察的点 
                Nextpoint=[p,q];
                break;
            end
        end 
        closelist(Nextpoint(1),Nextpoint(2)) = 1; 
        for i = 1:3
            for j = 1:3
                k = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2);
                if (i == 2&&j == 2)
                    continue
                elseif (k == -inf)
                    closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 2;
                elseif (k == inf)
                    distance = ((i-2)^2+(j-2)^2)^0.5;  % 欧几里得距离
                    G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1),Nextpoint(2))+distance;  % 更新搜索点的状态集
                    openlist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;
                    parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                    parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % 记录每个搜索点的父亲节点，存入两个矩阵中
                    closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;   % 将考察点的周围点置为可考察的状态
                else
                    % 搜索点的状态集并非为inf/-inf，而是一个值，代表这一点已经有了一个状态，需要和新的状态比较，取较小值
                    distance = ((i-2)^2+(j-2)^2)^0.5; % 欧几里得距离
                    if(k > (G_set(Nextpoint(1),Nextpoint(2))+distance))
                        k = G_set(Nextpoint(1),Nextpoint(2))+distance;
                        G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = k;  % 更新状态集
                        parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                        parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % 记录每个搜索点的父亲节点，存入两个矩阵中
                        closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;   % 将考察点的周围点置为可考察的状态
                    end
                end
                    if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)))  %搜索点为终点跳出循环
                        parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                        parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %记录父亲节点,跳出循环
                        break;
                    end
            end
                if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)))  %搜索点为终点跳出循环
                    parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                    parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %记录父亲节点,跳出循环
                    break;
                end
        end
           if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)))  %搜索点为终点或者cost(操作集代价无穷大)为inf,跳出循环
                parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %记录父亲节点,跳出循环
                break;
           end
    end
    Path = [];  %创建路径矩阵
    Path(1,:) = [Epoint(1) Epoint(2)];
    k = 2;
    while(1)
    temp_G_set_Pathpoint = G_set(Epoint(1),Epoint(2));
    for i = 1:3
        for j = 1:3
            if (i ~= 2||j ~= 2)
                if ((G_set(Epoint(1)+i-2,Epoint(2)+j-2)) >= 0&&(G_set(Epoint(1)+i-2,Epoint(2)+j-2) <= temp_G_set_Pathpoint))
                    temp_G_set_Pathpoint = G_set(Epoint(1)+i-2,Epoint(2)+j-2);  
                    Path(k,:) = [Epoint(1)+i-2 Epoint(2)+j-2];
                end
            end
        end
    end
    Epoint = Path(k,:);
    k = k+1;
    if(Epoint(1) == Spoint(1)&&Epoint(2) == Spoint(2))  % 父亲节点就是起点,跳出循环
        break;
    end
    end
    % 返回相关参数
    Path = Path';
    Routing = Path; 
end