function Routing = Astar_Diagonal(Map,Spoint,Epoint,m,n)
    % 寻路A*算法
    Map(Spoint(1),Spoint(2)) = 0;
    Map(Epoint(1),Epoint(2)) = inf;
    G_set = Map;
    F_set = Map;
    openlist = Map;
    closelist = Map;
    parent_x = Map;
    parent_y = Map;
    openlist(Spoint(1),Spoint(2)) = 0;
    while(1)
        cost = inf;
        for p = 1:m
            for q = 1:n
                if (openlist(p,q) == 0&&closelist(p,q) ~= 1)  % openlist为0,代表可到达;colselist不为1,代表未考察过 
                    % Outpoint = [p,q];
                    if (F_set(p,q) >= 0&&cost > F_set(p,q))
                        cost = F_set(p,q);
                        Nextpoint=[p,q];
                    end
                end
            end
        end 
        closelist(Nextpoint(1),Nextpoint(2)) = 1;
        for i = 1:3
            for j = 1:3
                k = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2);
                if (i == 2&&j == 2||closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) == 1)
                    continue
                elseif (k == -inf)
                    closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 1;
                elseif (k == inf)
                    distance = ((i-2)^2+(j-2)^2)^0.5;  % 欧几里得距离
                    G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1),Nextpoint(2))+distance;  % 更新搜索点的状态集
                    openlist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;
                    % 欧几里得距离启发函数
                    % H = ((Nextpoint(1)+i-2-Epoint(1))^2+(Nextpoint(2)+j-2-Epoint(2))^2)^0.5;
                    % 对角线启发函数
                    % 设定横向移动耗散值为D,对角线移动耗散值为sqrt(2)*D，取D=1
                    H_diagonal = min(abs(Nextpoint(1)+i-2-Epoint(1)),abs(Nextpoint(2)+j-2-Epoint(2)));  % 沿对角线移动的步数
                    H_straight = abs(Nextpoint(1)+i-2-Epoint(1))+abs(Nextpoint(2)+j-2-Epoint(2));       % 曼哈顿距离
                    H = sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                    F_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2)+H; % 更新操作集，操作集等于状态集加上启发函数代价值
                    parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                    parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % 记录每个搜索点的父亲节点，存入两个矩阵中
                else
                    % 搜索点的状态集并非为inf/-inf，而是一个值，代表这一点已经有了一个状态，需要和新的状态比较，取较小值
                    distance = ((i-2)^2+(j-2)^2)^0.5; % 欧几里得距离
                    if(k > (G_set(Nextpoint(1),Nextpoint(2))+distance))
                        k = G_set(Nextpoint(1),Nextpoint(2))+distance;
                        % 欧几里得距离启发函数
                        % H = ((Nextpoint(1)+i-2-Epoint(1))^2+(Nextpoint(2)+j-2-Epoint(2))^2)^0.5;
                        % 对角线启发函数
                        H_diagonal = min(abs(Nextpoint(1)+i-2-Epoint(1)),abs(Nextpoint(2)+j-2-Epoint(2)));
                        H_straight = abs(Nextpoint(1)+i-2-Epoint(1))+abs(Nextpoint(2)+j-2-Epoint(2));
                        H = sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                        F_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2)+H; % 更新操作集，操作集等于状态集加上启发函数代价值
                        parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                        parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % 记录每个搜索点的父亲节点，存入两个矩阵中
                    end
                end
                    if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  % 搜索点为终点或者cost(操作集代价无穷大)为inf,跳出循环
                        parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                        parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  % 记录父亲节点,跳出循环
                        break;
                    end
            end
                if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  % 搜索点为终点或者cost(操作集代价无穷大)为inf,跳出循环
                    parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                    parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  % 记录父亲节点,跳出循环
                    break;
                end
        end
           if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  % 搜索点为终点或者cost(操作集代价无穷大)为inf,跳出循环
                parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  % 记录父亲节点,跳出循环
                break;
           end
    end
    Path = [];  % 创建路径矩阵
    i = 1;
    while(1)
        if (cost == inf)  % cost=inf,跳出循环
            break;
        end
        Path(i,:) = Epoint;
        i = i+1;
        temp_Epoint_x = Epoint(1);
        Epoint(1) = parent_x(Epoint(1),Epoint(2));
        Epoint(2) = parent_y(temp_Epoint_x,Epoint(2));
        if(parent_x(Epoint(1),Epoint(2)) == Spoint(1)&&parent_y(Epoint(1),Epoint(2)) == Spoint(2))  % 父亲节点就是起点，跳出循环
            Path(i,:) = Epoint;
            break;
        end
    end
    Path(i+1,:) = Spoint;  % 记录跳出循环的点
    Path = Path';
    % 返回相关参数
    Routing = Path; 
end