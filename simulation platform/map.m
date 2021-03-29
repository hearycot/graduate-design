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

%% 3. A*�㷨����·��
%%Ѱ·
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
            % �����ǲ��ǿ��Լ�һ��break���ҵ���ʼ�������ѭ�����������ò��裬Ĭ��ֻ��һ�����
            % open��Ϊ0����ʾ������Ϊ��չ����close��Ϊ1����ʾ�Ѿ��߹��ˣ�����Ҫ����Ϊ��һ�����
            if(openlist(p,q)==0&&closelist(p,q)~=1)
                Outpoint=[p,q];
                % ������F����0������С����һ��������ʱ���Ŵ���nextpoint�������Թ�
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
                % ����һ���Ƿ����ɾ����
                % ��·���ߣ��Լ���״̬���������ı�
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j) = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
                closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=1;
            elseif (k == inf)
                distance=((i-2)^2+(j-2)^2)^0.5;
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1),Nextpoint(2))+distance;
                openlist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=0;
                % H=((Nextpoint(1)-a2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;%ŷ����¾�����������
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϸ��ӵĶԽ�����������
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϼ򵥵ĶԽ��ߺ���
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)+H;
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
            else distance=((i-2)^2+(j-2)^2)^0.5;
                if(k>(distance+G(Nextpoint(1),Nextpoint(2))))
                    k=distance+G(Nextpoint(1),Nextpoint(2));
                    % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;  %ŷ����¾�����������
                    H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϸ��ӵĶԽ�����������
                    H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                    H=sqrt(2)*10*H_diagonal+10*(H_straight-2*H_diagonal);
                    % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%�Ƚϼ򵥵ĶԽ��ߺ���
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
    % �����xx�ƺ�����ɾ����
    xx=Epoint(1);
    % ȡ�����׽ڵ�
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
%% 4. ·���Ż�
%���õ�������������ϳɹ⻬������
P=P';
a=[];
b=[];
a=P(1,:);
% ����Ӧ������һ��bȡ�ڶ���Ԫ��
b=P(2,:);
figure
%subplot(2,2,3);
plot(a,b);
axis([0,n+3,0,n+3]);
title('Route planing');
values = spcrv([[a(1) a a(end)];[b(1) b b(end)]],3);
%% 5. ������ӻ�
figure
%subplot(2,2,4);
plot(values(1,:),values(2,:),'r');
axis([0,m+3,0,m+3]);
title('Route planing after smoothing');    