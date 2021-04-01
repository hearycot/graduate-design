function [Map,Map_Spoint,Map_Epoint] = Map_Generate(m,n)
    for i = 2:m-1
        for j = 2:n-1
            Matrix(i,j) = inf;
        end
    end   
    for j = 1:n
        Matrix(1,j) = -inf;
        Matrix(m,j) = -inf;
    end
    for i = 1:m
        Matrix(i,1) = -inf;
        Matrix(i,n) = -inf;
    end
    % 构建障碍物
    % 切割地图为5*5的图单元,在每个图单元生成障碍物
    count = 1;
    i = 1;j = 1;
    length = 5;
    % temp_watch = [];
    while(count ~= (((m-5)/4+1)*((n-5)/4+1)))
        type = randi(2);
        coordinate_x = i+2;
        coordinate_y = j+2;
        % temp_watch = [temp_watch;coordinate_x coordinate_y length];
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
        if i >= m
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
        Spoint_x = randi([2,m-1],1);
        Spoint_y = randi([2,n-1],1);
        Epoint_x = randi([2,m-1],1);
        Epoint_y = randi([2,n-1],1);   
    end
    Spoint = [Spoint_x Spoint_y];
    Epoint = [Epoint_x Epoint_y];
    % 返回相关参数
    Map = Matrix;
    Map_Spoint = Spoint;
    Map_Epoint = Epoint;
end