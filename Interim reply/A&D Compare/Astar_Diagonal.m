function Routing = Astar_Diagonal(Map,Spoint,Epoint,m,n)
    % Ѱ·A*�㷨
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
                if (openlist(p,q) == 0&&closelist(p,q) ~= 1)  % openlistΪ0,����ɵ���;colselist��Ϊ1,����δ����� 
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
                    distance = ((i-2)^2+(j-2)^2)^0.5;  % ŷ����þ���
                    G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1),Nextpoint(2))+distance;  % �����������״̬��
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
                        % ŷ����þ�����������
                        % H = ((Nextpoint(1)+i-2-Epoint(1))^2+(Nextpoint(2)+j-2-Epoint(2))^2)^0.5;
                        % �Խ�����������
                        H_diagonal = min(abs(Nextpoint(1)+i-2-Epoint(1)),abs(Nextpoint(2)+j-2-Epoint(2)));
                        H_straight = abs(Nextpoint(1)+i-2-Epoint(1))+abs(Nextpoint(2)+j-2-Epoint(2));
                        H = sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                        F_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2)+H; % ���²�����������������״̬������������������ֵ
                        parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                        parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % ��¼ÿ��������ĸ��׽ڵ㣬��������������
                    end
                end
                    if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  % ������Ϊ�յ����cost(���������������)Ϊinf,����ѭ��
                        parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                        parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  % ��¼���׽ڵ�,����ѭ��
                        break;
                    end
            end
                if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  % ������Ϊ�յ����cost(���������������)Ϊinf,����ѭ��
                    parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                    parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  % ��¼���׽ڵ�,����ѭ��
                    break;
                end
        end
           if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)) || cost == inf)  % ������Ϊ�յ����cost(���������������)Ϊinf,����ѭ��
                parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  % ��¼���׽ڵ�,����ѭ��
                break;
           end
    end
    Path = [];  % ����·������
    i = 1;
    while(1)
        if (cost == inf)  % cost=inf,����ѭ��
            break;
        end
        Path(i,:) = Epoint;
        i = i+1;
        temp_Epoint_x = Epoint(1);
        Epoint(1) = parent_x(Epoint(1),Epoint(2));
        Epoint(2) = parent_y(temp_Epoint_x,Epoint(2));
        if(parent_x(Epoint(1),Epoint(2)) == Spoint(1)&&parent_y(Epoint(1),Epoint(2)) == Spoint(2))  % ���׽ڵ������㣬����ѭ��
            Path(i,:) = Epoint;
            break;
        end
    end
    Path(i+1,:) = Spoint;  % ��¼����ѭ���ĵ�
    Path = Path';
    % ������ز���
    Routing = Path; 
end