%% ��������������ɵ�ͼ
%% 1. ��ʼ������
close all;clc;clear;
% ��ͼ��С
m = 29;
n = 29;
% ������ͼ
% -inf��ʾ���ɵ�����ϰ����,inf����ɵ���ռ�
% �����߽�
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
% �����ϰ���
% �и��ͼ,��ÿ��ͼ��Ԫ�����ϰ���
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
        % ���ɺ����ϰ���
        for k = 1:length
            Matrix(coordinate_x+k-3,coordinate_y) = -inf;
        end
    else 
        % ���������ϰ���
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
% �̶������յ�����
% Spoint_x = 2;Spoint_y = 2;
% Epoint_x = 28;Epoint_y = 28;
% ��������յ�����
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
% ��ʾ��ͼ
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
plot(Epoint(1),Epoint(2),'b+');
plot(Spoint(1),Spoint(2),'b+');
%% 2. A*�㷨����·��
% Ѱ·A*�㷨
Matrix(Spoint(1),Spoint(2)) = 0;
Matrix(Epoint(1),Epoint(2)) = inf;
G_set = Matrix;
F_set = Matrix;
openlist = Matrix;
closelist = Matrix;
parent_x = Matrix;
parent_y = Matrix;
openlist(Spoint(1),Spoint(2)) = 0;
while(1)
    cost = inf;
    for p = 1:m
        for q = 1:n
            if (openlist(p,q) == 0&&closelist(p,q) ~= 1)
                Outpoint = [p,q];
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
                closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) == 1;
            elseif (k == inf)
                distance = ((i-2)^2+(j-2)^2)^0.5;  % ŷ����þ���
                G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1),Nextpoint(2)) + distance;  % �����������״̬��
                openlist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;
                % ŷ����þ�����������
                % H = ((Nextpoint(1)+i-2-Epoint(1))^2+(Nextpoint(2)+j-2-Epoint(2))^2)^0.5;
                % �Խ�����������
                % �趨�����ƶ���ɢֵΪD,�Խ����ƶ���ɢֵΪsqrt(2)*D��ȡD=1
                H_diagonal = min(abs(Nextpoint(1)+i-2-Epoint(1)),abs(Nextpoint(2)+j-2-Epoint(2)));  % �ضԽ����ƶ��Ĳ���
                H_straight = abs(Nextpoint(1)+i-2-Epoint(1))+abs(Nextpoint(2)+j-2-Epoint(2));       % �����پ���
                H = sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                F_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2)+H; % ���²�����������������״̬������������������ֵ
                parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % ��¼ÿ��������ĸ��׽ڵ㣬��������������
            else
                % �������״̬������Ϊinf/-inf������һ��ֵ��������һ���Ѿ�����һ��״̬����Ҫ���µ�״̬�Ƚϣ�ȡ��Сֵ
                distance = ((i-2)^2+(j-2)^2)^0.5; % ŷ����þ���
                if(k > (G_set(Nextpoint(1),Nextpoint(2))+distance))
                    k = G_set(Nextpoint(1),Nextpoint(2))+distance;
                    % �Խ�����������
                    H_diagonal = min(abs(Nextpoint(1)+i-2-Epoint(1)),abs(Nextpoint(2)+j-2-Epoint(2)));
                    H_straight = abs(Nextpoint(1)+i-2-Epoint(1))+abs(Nextpoint(2)+j-2-Epoint(2));
                    H = sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                    F_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2)+H; % ���²�����������������״̬������������������ֵ
                    parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                    parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % ��¼ÿ��������ĸ��׽ڵ㣬��������������
                end
            end
                if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  %������Ϊ�յ����cost(���������������)Ϊinf,����ѭ��
                    parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                    parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %��¼���׽ڵ�,����ѭ��
                    break;
                end
        end
            if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)
                parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %��¼���׽ڵ�,����ѭ��
                break;
            end
    end
       if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)
            parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
            parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %��¼���׽ڵ�,����ѭ��
            break;
       end
end
Path = [];  %����·������
i = 1;
while(1)
    if (cost == inf)  %cost=inf,����ѭ��
        break;
    end
    h4 = plot(Epoint(1),Epoint(2),'b+');
    Path(i,:) = Epoint;
    i = i+1;
    temp_Epoint_x = Epoint(1);
    Epoint(1) = parent_x(Epoint(1),Epoint(2));
    Epoint(2) = parent_y(temp_Epoint_x,Epoint(2));
    if(parent_x(Epoint(1),Epoint(2)) == Spoint(1)&&parent_y(Epoint(1),Epoint(2)) == Spoint(2))  %���׽ڵ�����յ㣬ֱ�ӻ�ͼ����ѭ��
        plot(Epoint(1),Epoint(2),'b+');
        Path(i,:) = Epoint;
        break;
    end
end
Path(i+1,:) = Spoint;  %��¼����ѭ���ĵ�
legend([h1,h2,h3,h4],'Start Point','End Point','Obstacle','Routing');    
%% 3. ·���Ż�
% ���õ�������������ϳɹ⻬������
Path = Path';
Path_x = [];
Path_y = [];
Path_x = Path(1,:);
Path_y = Path(2,:);
figure
plot(Path_x,Path_y);
axis([0 m+2 0 n+2]);
title('Route planing');
values = spcrv([[Path_x(1) Path_x Path_x(end)];[Path_y(1) Path_y Path_y(end)]],3);
%% 4. ������ӻ�
figure
plot(values(1,:),values(2,:),'r');
axis([0 m+2 0 n+2]);
title('Route planing after smoothing');    