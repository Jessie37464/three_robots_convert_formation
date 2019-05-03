function robot_formation_main
%   ������
%   ˵����
%-------------------------
%   ���룺
%   PID����------����������
%   Ӳ������------����Ӳ��
%   A------ͨ������ͼ���ڽӾ���
%   Sim_duration------����ʱ��
%   r------���Ͱ뾶
%   con_v------������ɺ������˶��ٶ�
%   dT------����������
%   status------���ζ�Ӧ���
%   n------С����Ŀ
%   IX------��ʼλ��
%   �����
%   DX------С�������˶��ٶ�            
            clear all;
            close all;
            clc;

 %�������
            status=1;            
            car_n=3;
            
%���Զ������
            Sim_duration =24; 
            dT = 0.1;
            r=400;
            con_v = [100;0];

%PID ����                            
            Kp = 5;
            Ki = 0.08;
            Kd = 0.1;  
%Ӳ�� ����                             
            L=9.925;    %�������е㵽����/���ֵĳ���
            R=3.25;     %���ӵİ뾶
            l = 1.5;
            max_rpm = 130;
            
%��ʼ�������� Multi_Vehicle_Main
            tEnd =Sim_duration;    %����ʱ��15
            nSim = tEnd/dT;        %��ʱ��0��15��Ϊ���Ϊ0.1��150�Σ�����15-0��/0.1
            timeS=(0:(nSim-1))*dT; %0��14.9   ǰһ�̵�ʱ��
            time=(0:nSim)*dT;      %0��15     ��һ�̵�ʱ��
          
            Ix_1 = [450; 0];   %����ʼλ��        
            Ix_3 = [300; 0];
            Ix_2 = [150; 0];  %����ʼλ��
          
            IX = [Ix_1  Ix_2  Ix_3]; %��2*7�ľ���ÿһ�д���һ������㣨x,y��
            
            for i = 1:car_n         %����1����2����ʼλ��
                IP(1:2,i) = IX(1:2,i);
            end
 
%��ʼ��[Xd(:,:,k)]=Guidance(r,status);          
            Xd=zeros(2,car_n,length(timeS));  %��Ӧlength(timeS)����1�ͳ�2��λ�ã�x��y��
            
%DX(:,:,k)= Path_Planner( X(:,:,k),[],A1,A2,A3,time(k),Xd(:,:,k),con_v);;
            X=zeros(2,car_n,length(time));
            X(:,:,1)=IP;                    %[0;80]
            A1 = [0  0  0  1  0;...         %���˽ṹ1
                  1  0  0  0  0;...
                  0  1  0  0  0;...
                  0  1  0  0  0;...
                  0  0  0  1  0];
            A2 = [0  0  0  0  0;...          %���˽ṹ2
                  1  0  0  0  0;...
                  0  1  0  0  0;...
                  0  1  0  0  0;...
                  0  0  0  1  0];
                for i = 1:car_n               %���˽ṹ3�����Ż������������仯
                    for j = 1:car_n
                        if j == (i-1)
                           A3(i,j)=1;
                        elseif i==1&j==car_n
                            A3(i,j)=1;
                        else
                            A3(i,j)=0;
                        end
                     end
                end
            
            DX=zeros(2,car_n,length(timeS));
            
%X(:,:,k+1) = RK4(@Path_Planner, X(:,:,k), dT, 0,A1,A2,A3,time(k),Xd(:,:,k),con_v);
%���������һ��΢�ַ�����ֵ�������㷨
%��һ�������������õ�����ѧ�ڳ�ֵ������������ϵͳ״̬�仯

%��ʼ��[vr(:,:,k),vl(:,:,k)]=New_Dynamics(x,y,theta,X(:,:,k+1),L,R,l,max_rpm); 
            x = IP(1,:)';   %[0;1200]    �����ĳ�ʼxλ��
            y = IP(2,:)';   %[800;-800]   �����ĳ�ʼyλ��
            theta = zeros(1,car_n);%[0 0]  �����ĳ�ʼ�Ƕ�
            E_k = zeros(1,car_n);
            e_k_1 = zeros(1,car_n);
            vr = zeros(1,car_n);  %[0 0]  �����ĳ�ʼ�����ٶ�
            vl = zeros(1,car_n);  %[0 0]  �����ĳ�ʼ�����ٶ�
          
%��ʼ��[x,y,theta,position_new(:,:,k)] = sensor(x,y,theta,vr(:,:,k),vl(:,:,k),dT,L,R);            
            position_new = zeros(3,car_n,length(timeS));  

 
            for k = 1:80
   
                [Xd(:,:,k)]=Guidance(r,status,car_n);
                X(:,:,k+1) = RK4(@Path_Planner, X(:,:,k), dT, 0,A1,A2,A3,time(k),Xd(:,:,k),con_v,status,car_n);
                DX(:,:,k)= Path_Planner( X(:,:,k),[],A1,A2,A3,time(k),Xd(:,:,k),con_v,status,car_n);
                 X(:,:,k+1)              
                for i = 1:100
                    [vr,vl,E_k,e_k_1]=PID(x,y,theta,X(:,:,k+1),dT,L,R,l,max_rpm,Kp,Ki,Kd,E_k,e_k_1,car_n);
                    [x,y,theta,position_new(:,:,k)] = sensor(x,y,theta,vr,vl,dT,L,car_n);
                    i = i+1;               
                end
               plotS_I(X,position_new,theta,time,car_n,k); 
            end
            
             for k = 81:160
                status=2;
                [Xd(:,:,k)]=Guidance(r,status,car_n);
                X(:,:,k+1) = RK4(@Path_Planner, X(:,:,k), dT, 0,A1,A2,A3,time(k),Xd(:,:,k),con_v,status,car_n);
                DX(:,:,k)= Path_Planner( X(:,:,k),[],A1,A2,A3,time(k),Xd(:,:,k),con_v,status,car_n);
                 X(:,:,k+1)              
                for i = 1:100
                    [vr,vl,E_k,e_k_1]=PID(x,y,theta,X(:,:,k+1),dT,L,R,l,max_rpm,Kp,Ki,Kd,E_k,e_k_1,car_n);
                    [x,y,theta,position_new(:,:,k)] = sensor(x,y,theta,vr,vl,dT,L,car_n);
                    i = i+1;               
                end
               plotS_I(X,position_new,theta,time,car_n,k); 
             end
            
            for k = 161:240
                status=3;
                [Xd(:,:,k)]=Guidance(r,status,car_n);
                X(:,:,k+1) = RK4(@Path_Planner, X(:,:,k), dT, 0,A1,A2,A3,time(k),Xd(:,:,k),con_v,status,car_n);
                DX(:,:,k)= Path_Planner( X(:,:,k),[],A1,A2,A3,time(k),Xd(:,:,k),con_v,status,car_n);
                 X(:,:,k+1)              
                for i = 1:100
                    [vr,vl,E_k,e_k_1]=PID(x,y,theta,X(:,:,k+1),dT,L,R,l,max_rpm,Kp,Ki,Kd,E_k,e_k_1,car_n);
                    [x,y,theta,position_new(:,:,k)] = sensor(x,y,theta,vr,vl,dT,L,car_n);
                    i = i+1;               
                end
               plotS_I(X,position_new,theta,time,car_n,k); 
            end
            
            
            
             plotS_I2(position_new,time,car_n);
            save Sim_data.mat X DX time nSim dT Sim_duration  ;
   
end




%% ���·���滮�� 
  function DX = Path_Planner(X,tau,A1,A2,A3,t,Xd,con_v,status,car_n)
%   ·���滮���������ɶ�С����ӵ��˶�·��
%   �㷨������һ��ϵͳһ�����㷨
%-------------------------
%   ���룺
%   X------����С��λ��״̬
%   tau----����������
%   A------ͨ������ͼ���ڽӾ���
%   t------�����л�ʱ��
%   Xd------��ƹ���
%   con_v------������ɺ������˶��ٶ�
%   �����
%   DX------С�������˶��ٶ�

             switch status
% ------------------------------------------------------     
             case {1,2,3,4}
%��i�������˶�����Ϊ�䵱ǰ����λ�����ٽ�����֮��ľ����ȥ������Ӧ�о���      
             for i=1:car_n
                 for j=1:car_n
                 us(:,j,i)=-A3(j,i)*(X(:,j)-X(:,i)-(Xd(:,j)-Xd(:,i)));
                 end
             end
             u=sum(us,3); %
% ----------------------------------------------------------- 
            case 5
             for i=1:car_n
                 for j=1:car_n
                 us(:,j,i)=-A1(j,i)*(X(:,j)-X(:,i)-(Xd(:,j)-Xd(:,i)));
                 end
             end
             u=sum(us,3); 
             
             end
             
             for i =1:car_n
                DX(1:2,i) = u(:,i)+con_v;%С�������˶��ٶ�=���+������ɺ������˶��ٶ�
             end
end

%% �������
function [Xd]=Guidance(r,status,car_n)
%   ����������������ɶ�С����ӵ��˶�����
%   ˵�����趨�������ʱ����С������ڹ��������ĵ�λ��
%-------------------------
%   ���룺
%   r------���Ͱ뾶
%   status------�����л��붨
%   �����
%   Xd------С����Թ�������λ��
            switch status
   
            case 1
                
            d1=[0.5*r*cos((4-3)*2*pi/6);0.5*r*sin((4-3)*2*pi/6)];
            d2=[0.5*r*cos((2-3)*2*pi/6);0.5*r*sin((2-3)*2*pi/6)];
            d3=[r*cos((3-3)*2*pi/3);r*sin((3-3)*2*pi/3)];
           
         
            d=[d1 d2 d3 ];
% ----------------------------------------------------------- ������            
  
            case 2
            for i = 1:(car_n)
                d(1:2,i)=[0;r*(car_n-i)/car_n];
            end
% ----------------------------------------------------------- ��
            case 3
            for i = 1:(car_n)
                d(1:2,i)=[(r)*(car_n-i)/car_n;0];
            end                
% ----------------------------------------------------------- ��
            end
            
            Xd=d;

end



%% PID������
function [vr,vl,E_k,e_k_1]=PID(x,y,theta,X,dT,L,R,l,max_rpm,Kp,Ki,Kd,E_k,e_k_1,car_n)
%   �����ɡ�������С���˶��ٶ�
%   ˵���������ֳ�ģ��ת��Ϊ�ʵ�ģ�ͽ�һ��ת��Ϊ����ģ��
%-------------------------
%   ���룺
%   x------��ǰx������
%   y------��ǰy������
%   theta------��ǰ��ͷ��x�����ɼн�
%   X------Ŀ�������[x;y]
%   dT------ʱ����
%   Ӳ������
%   PID ����
%   �����
%   vl------�����ٶ�
%   vr------�����ٶ�
%   �ۻ���

            
            for i = 1:car_n
% ����С����Ŀ�������
                g_x = X(1,i) - x(i)
                g_y = X(2,i) - y(i)
                
                for j = (i+1):(car_n+1)
                    x(car_n+1) = 0;
                    y(car_n+1) = 0;
                    if (sqrt((x(i)-x(j))^2+(y(i)-y(j))^2) < 60)
                        ao_x = x(i)-x(j);
                        ao_y = y(i)-y(j);
                        aph =0.5;
                        g_x = aph*g_x+(1-aph)*ao_x;%�û�������Ŀ���ľ��룬�����������˵ľ����Ȩ��
                        g_y = aph*g_y+(1-aph)*ao_y;
                        break                       
                    end
                end
            
%����С�����ٶ�
                v = sqrt(g_y^2+g_x^2)
                
% ����С����ǰ������x��н�            
                phi = atan2(sin(theta(i)),cos(theta(i)));
                
% ����С������������x��н�  
                theta_g = atan2(g_y,g_x);    

                e_k = theta_g-phi ;        
                e_k = atan2(sin(e_k),cos(e_k));
  
 % ����Kp
                e_P = e_k;
            
 % ����Ki
                e_I = E_k(i) + e_k*dT;
                     
 % ����Kd
                e_D = (e_k-e_k_1(i))/dT;    
              
 %����С�����ٶ�
                w = Kp*e_P + Ki*e_I + Kd*e_D
            
%���������
                E_k(i) = e_I;
                e_k_1(i) = e_k;
            

% ����֧��ת�ٷ�Χ���Ƚ�
                max_vel = max_rpm*2*pi/60;
            
                min_rpm = 1;
                min_vel = min_rpm*2*pi/60;
            
                vel_max = max_vel;
                vel_min = min_vel;
            
                if (abs(v) > 0)
                    % 1. Limit v,w to be possible in the range [vel_min, vel_max]
                    % (avoid stalling or exceeding motor limits)
                    v_lim = max(min(abs(v), (R/2)*(2*vel_max)), (R/2)*(2*vel_min));
                    w_lim = max(min(abs(w), (R/L)*(vel_max-vel_min)), 0);
                
                    % 2. Compute the desired curvature of the robot's motion
                    vel_r_d = (2*v_lim+w_lim*L)/(2*R);
                    vel_l_d = (2*v_lim-w_lim*L)/(2*R);
                
                    % 3. Find the max and min vel_r/vel_l
                    vel_rl_max = max(vel_r_d, vel_l_d);
                    vel_rl_min = min(vel_r_d, vel_l_d);
                
                    % 4. Shift vel_r and vel_l if they exceed max/min vel
                    if (vel_rl_max > vel_max)
                        vel_r = vel_r_d - (vel_rl_max-vel_max);
                        vel_l = vel_l_d - (vel_rl_max-vel_max);
                    elseif (vel_rl_min < vel_min)
                        vel_r = vel_r_d + (vel_min-vel_rl_min);
                        vel_l = vel_l_d + (vel_min-vel_rl_min);
                    else
                        vel_r = vel_r_d;
                        vel_l = vel_l_d;
                    end
                
                    % 5. Fix signs (Always either both positive or negative)

                    v_shift = R/2*(vel_r+vel_l);
                    w_shift = R/L*(vel_r-vel_l);               
                    v = sign(v)*v_shift;
                    w = sign(w)*w_shift;
                
                else
                    % Robot is stationary, so we can either not rotate, or
                    % rotate with some minimum/maximum angular velocity
                    w_min = R/L*(2*vel_min);
                    w_max = R/L*(2*vel_max);
                    
                    if abs(w) > w_min
                        w = sign(w)*max(min(abs(w), w_max), w_min);
                    else
                        w = 0;
                    end

                end

%�ʵ�ģ��ת��Ϊ���ģ��            
                vr(i) = (2*v+w*L)/(2*R);
                vl(i) = (2*v-w*L)/(2*R);
                 
            end

end

%% ������
function [x,y,theta,position_new] = sensor(x,y,theta,vr,vl,dT,L,car_n)
%   ������������������ת�ٻ��ʵ��λ��
%   ˵���������ֳ�ģ��ת��Ϊ�ʵ�ģ�ͽ�һ��ת��Ϊ����ģ��
%-------------------------
%   ���룺
%   x------ǰһʱ��x������
%   y------ǰһʱ��y������
%   theta------ǰһʱ�̳�ͷ��x�����ɼн�
%   vr------����ת��
%   vl------����ת��
%   dT------ʱ����
%   Ӳ������
%   �����
%   x------��ǰx������
%   y------��ǰy������
%   theta------��ǰ��ͷ��x�����ɼн�
           
%������������Ӧֵ            
            for i = 1:car_n
                
                position_new(1,i) = x(i);
                position_new(2,i) = y(i);
                position_new(3,i) = theta(i);
%����λ��������λ��                        
                d_right = vr(i)*dT;
                d_left = vl(i)*dT;

%����λ�ñ仯����Ƕȱ仯��
                d_center = (d_right + d_left)/2;
                theta_dt = (d_right - d_left)/L;
            
%x y ������仯��
                x_dt = d_center*cos(theta(i));
                y_dt = d_center*sin(theta(i));

%��ǰ x y theta                        
                theta(:,i) = theta(:,i) + theta_dt;
                theta(:,i) = atan2(sin(theta(:,i)),cos(theta(:,i)));
                x(i) = x(i) + x_dt;
                y(i) = y(i) + y_dt;  

            end

end
 
%% ��ͼ�붯��
function plotS_I(X,position_new,theta,time,car_n,k)
%   ��ͼ�붯��������С�����˶�����ͼ�λ�չʾ
%   ˵��������С������·����ʵ��·����λ��
%-------------------------
%   ���룺
%   X------��ǰʱ����������ֵ
%   position_new------��ǰʱ��ʵ������ֵ
%   theta------��ǰʱ��С����x�����ɼн�
%   time------����ʱ��
%   �����
%   figure��2��------С������·����ʵ��·����λ��

%% ·���Ŀռ��ʾ
% ���ڻ�������·��

% ���ڻ���С��λ��
                car = zeros(2,4,car_n);
               
                for n = 1:car_n           
                    phi = position_new(3,n,k) ;

                    r = 60;
                    d0=[r*cos(phi);r*sin(phi)];
                    d1=[r*cos(phi+5*pi/6);r*sin(phi+5*pi/6)];
                    d2=[r*cos(phi-5*pi/6);r*sin(phi-5*pi/6)];
                 
                    
     
                    car(:,:,n) = [d0 d1 d2 d0];
                    for i = 1:4  
              
                        car(:,i,n) = car(:,i,n)+position_new(1:2,n,k);
             
                    end
                end

                figure(2);
                
                switch car_n
                
                    case 3
                        x1 = X(:,1,1:k);
                        x2 = X(:,2,1:k);
                        x3 = X(:,3,1:k); 
                        t1 = position_new(:,1,1:k);
                        t2 = position_new(:,2,1:k);
                        t3 = position_new(:,3,1:k);
                        plot(x1(1,:),x1(2,:),'k-.',x2(1,:),x2(2,:),'b-.',x3(1,:),x3(2,:),'r-.',...
                        t1(1,:),t1(2,:),'b-',t2(1,:),t2(2,:),'b-',t3(1,:),t3(2,:),'b-',...
                        car(1,:,1),car(2,:,1),'b-',car(1,:,2),car(2,:,2),'b-',car(1,:,3),car(2,:,3),'b-');
                        axis([0 3000 -400 400]);
                        title('���������л��������˽ṹ�ı�ӿ���')
                        xlabel('\fontsize{14}\fontname{Times New Roman}x (m)'),ylabel('\fontsize{14}\fontname{Times New Roman}\ity {\rm(m)}')
                end
                
                drawnow;
                grid off; 
                
 
end



function plotS_I2(position_new,time,car_n)
%   ��ͼ�붯��������С�����˶�����ͼ�λ�չʾ
%   ˵��������С��λ��ʱ��ͼ��
%-------------------------
%   ���룺
%   position_new------��ǰʱ��ʵ������ֵ
%   time------����ʱ��
%   �����
%   figure��1��------�ٶ�ʱ��ͼ��λ��ʱ��ͼ��

%% ·����ʱ���ʾ
% ���ڻ���ʵ��·�� 
Dtime=time(1:end-1);
tEnd=time(end);

    figure(1);
 
    switch car_n
      
    case 3
        t1 = position_new(:,1,:);
        t2 = position_new(:,2,:);
        t3 = position_new(:,3,:);
        
        subplot(3,1,1)
        Hp{1}(:,1)=plot(Dtime,t1(1,:),'k-',Dtime,t2(1,:),'b:',Dtime,t3(1,:),'r-.');
        grid on
        title('PID Controller'),xlabel('\fontsize{14}\fontname{Times New Roman}time (s)'),ylabel('\fontsize{14}\fontname{Times New Roman}\itx {\rm(m)}') 
        Ht1(1)=gca;
        set(Ht1(1),'XLim',[0 tEnd],'XTick',0:1:tEnd);
   
        subplot(3,1,2)
        Hp{1}(:,2)=plot(Dtime,t1(2,:),'k-',Dtime,t2(2,:),'b:',Dtime,t3(2,:),'r-.');
        grid on
        xlabel('\fontsize{14}\fontname{Times New Roman}time (s)'),ylabel('\fontsize{14}\fontname{Times New Roman}\ity {\rm(m)}')
        Ht1(2)=gca;
        set(Ht1(2),'XLim',[0 tEnd],'XTick',0:1:tEnd);

        subplot(3,1,3)
        Hp{1}(:,3)=plot(Dtime,t1(3,:),'k-',Dtime,t2(3,:),'b:',Dtime,t3(3,:),'r-.');
        grid on
        xlabel('\fontsize{14}\fontname{Times New Roman}time (s)'),ylabel('\fontsize{14}\fontname{Times New Roman}\ittheta {\rm(rad)}')
        Ht1(3)=gca;
        set(Ht1(3),'XLim',[0 tEnd],'XTick',0:1:tEnd);
        
  
    end


end
