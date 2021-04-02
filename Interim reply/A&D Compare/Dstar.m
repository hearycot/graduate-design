function Routing = Dstar(Map,Spoint,Epoint,m,n)
    % Ѱ·D�㷨
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
                if (openlist(p,q) == 0&&closelist(p,q) == 0)   % openlistΪ0,�����ɵ���;colselistΪ2,�������ϰ�����迼��;colselistΪ1����������Ҫ����ĵ� 
                    Nextpoint=[p,q];
                    break;
                end
            end
            if (openlist(p,q) == 0&&closelist(p,q) == 0)   % openlistΪ0,�����ɵ���;colselistΪ2,�������ϰ�����迼��;colselistΪ1����������Ҫ����ĵ� 
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
                    distance = ((i-2)^2+(j-2)^2)^0.5;  % ŷ����þ���
                    G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = G_set(Nextpoint(1),Nextpoint(2))+distance;  % �����������״̬��
                    openlist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;
                    parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                    parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % ��¼ÿ��������ĸ��׽ڵ㣬��������������
                    closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;   % ����������Χ����Ϊ�ɿ����״̬
                else
                    % �������״̬������Ϊinf/-inf������һ��ֵ��������һ���Ѿ�����һ��״̬����Ҫ���µ�״̬�Ƚϣ�ȡ��Сֵ
                    distance = ((i-2)^2+(j-2)^2)^0.5; % ŷ����þ���
                    if(k > (G_set(Nextpoint(1),Nextpoint(2))+distance))
                        k = G_set(Nextpoint(1),Nextpoint(2))+distance;
                        G_set(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = k;  % ����״̬��
                        parent_x(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(1); 
                        parent_y(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = Nextpoint(2);  % ��¼ÿ��������ĸ��׽ڵ㣬��������������
                        closelist(Nextpoint(1)+i-2,Nextpoint(2)+j-2) = 0;   % ����������Χ����Ϊ�ɿ����״̬
                    end
                end
                    if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)))  %������Ϊ�յ�����ѭ��
                        parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                        parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %��¼���׽ڵ�,����ѭ��
                        break;
                    end
            end
                if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)))  %������Ϊ�յ�����ѭ��
                    parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                    parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %��¼���׽ڵ�,����ѭ��
                    break;
                end
        end
           if(((Nextpoint(1)+i-2)==Epoint(1)&&(Nextpoint(2)+j-2) == Epoint(2)))  %������Ϊ�յ����cost(���������������)Ϊinf,����ѭ��
                parent_x(Epoint(1),Epoint(2))=Nextpoint(1);
                parent_y(Epoint(1),Epoint(2))=Nextpoint(2);  %��¼���׽ڵ�,����ѭ��
                break;
           end
    end
    Path = [];  %����·������
    k = 1;
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
    if(Epoint(1) == Spoint(1)&&Epoint(2) == Spoint(2))  % ���׽ڵ�������,����ѭ��
        break;
    end
    end
    % ������ز���
    Path = Path';
    Routing = Path; 
end