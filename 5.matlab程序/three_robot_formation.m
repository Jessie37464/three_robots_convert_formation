function robot_formation_main
%   主函数
%   说明：
%-------------------------
%   输入：
%   PID参数------调整控制律
%   硬件参数------调整硬件
%   A------通信拓扑图的邻接矩阵
%   Sim_duration------仿真时间
%   r------构型半径
%   con_v------构型完成后整体运动速度
%   dT------积分器步长
%   status------队形对应编号
%   n------小车数目
%   IX------初始位置
%   输出：
%   DX------小车期望运动速度            
            clear all;
            close all;
            clc;

 %输入变量
            status=1;            
            car_n=3;
            
%可自定义参数
            Sim_duration =24; 
            dT = 0.1;
            r=400;
            con_v = [100;0];

%PID 参数                            
            Kp = 5;
            Ki = 0.08;
            Kd = 0.1;  
%硬件 参数                             
            L=9.925;    %左右轮中点到左轮/右轮的长度
            R=3.25;     %轮子的半径
            l = 1.5;
            max_rpm = 130;
            
%初始化主函数 Multi_Vehicle_Main
            tEnd =Sim_duration;    %仿真时间15
            nSim = tEnd/dT;        %将时间0到15分为间隔为0.1的150段，即（15-0）/0.1
            timeS=(0:(nSim-1))*dT; %0到14.9   前一刻的时间
            time=(0:nSim)*dT;      %0到15     后一刻的时间
          
            Ix_1 = [450; 0];   %车起始位置        
            Ix_3 = [300; 0];
            Ix_2 = [150; 0];  %车起始位置
          
            IX = [Ix_1  Ix_2  Ix_3]; %是2*7的矩阵，每一列代表一个坐标点（x,y）
            
            for i = 1:car_n         %给车1、车2赋初始位置
                IP(1:2,i) = IX(1:2,i);
            end
 
%初始化[Xd(:,:,k)]=Guidance(r,status);          
            Xd=zeros(2,car_n,length(timeS));  %对应length(timeS)个车1和车2的位置（x，y）
            
%DX(:,:,k)= Path_Planner( X(:,:,k),[],A1,A2,A3,time(k),Xd(:,:,k),con_v);;
            X=zeros(2,car_n,length(time));
            X(:,:,1)=IP;                    %[0;80]
            A1 = [0  0  0  1  0;...         %拓扑结构1
                  1  0  0  0  0;...
                  0  1  0  0  0;...
                  0  1  0  0  0;...
                  0  0  0  1  0];
            A2 = [0  0  0  0  0;...          %拓扑结构2
                  1  0  0  0  0;...
                  0  1  0  0  0;...
                  0  1  0  0  0;...
                  0  0  0  1  0];
                for i = 1:car_n               %拓扑结构3，随着机器人数量而变化
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
%龙格库塔是一个微分方程数值解的求解算法
%是一个积分器用来得到动力学在初值和输入作用下系统状态变化

%初始化[vr(:,:,k),vl(:,:,k)]=New_Dynamics(x,y,theta,X(:,:,k+1),L,R,l,max_rpm); 
            x = IP(1,:)';   %[0;1200]    两车的初始x位置
            y = IP(2,:)';   %[800;-800]   两车的初始y位置
            theta = zeros(1,car_n);%[0 0]  两车的初始角度
            E_k = zeros(1,car_n);
            e_k_1 = zeros(1,car_n);
            vr = zeros(1,car_n);  %[0 0]  两车的初始右轮速度
            vl = zeros(1,car_n);  %[0 0]  两车的初始左轮速度
          
%初始化[x,y,theta,position_new(:,:,k)] = sensor(x,y,theta,vr(:,:,k),vl(:,:,k),dT,L,R);            
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




%% 编队路径规划器 
  function DX = Path_Planner(X,tau,A1,A2,A3,t,Xd,con_v,status,car_n)
%   路径规划器——生成多小车编队的运动路径
%   算法：线性一阶系统一致性算法
%-------------------------
%   输入：
%   X------所有小车位置状态
%   tau----积分器步长
%   A------通信拓扑图的邻接矩阵
%   t------拓扑切换时刻
%   Xd------设计构型
%   con_v------构型完成后整体运动速度
%   输出：
%   DX------小车期望运动速度

             switch status
% ------------------------------------------------------     
             case {1,2,3,4}
%第i辆车的运动方向为其当前期望位置与临近车辆之间的距离减去构型上应有距离      
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
                DX(1:2,i) = u(:,i)+con_v;%小车期望运动速度=误差+构型完成后，整体运动速度
             end
end

%% 构型设计
function [Xd]=Guidance(r,status,car_n)
%   构型设计器——生成多小车编队的运动构型
%   说明：设定构型完成时各个小车相对于构型中中心的位置
%-------------------------
%   输入：
%   r------构型半径
%   status------拓扑切换想定
%   输出：
%   Xd------小车相对构型中心位置
            switch status
   
            case 1
                
            d1=[0.5*r*cos((4-3)*2*pi/6);0.5*r*sin((4-3)*2*pi/6)];
            d2=[0.5*r*cos((2-3)*2*pi/6);0.5*r*sin((2-3)*2*pi/6)];
            d3=[r*cos((3-3)*2*pi/3);r*sin((3-3)*2*pi/3)];
           
         
            d=[d1 d2 d3 ];
% ----------------------------------------------------------- 三角形            
  
            case 2
            for i = 1:(car_n)
                d(1:2,i)=[0;r*(car_n-i)/car_n];
            end
% ----------------------------------------------------------- 列
            case 3
            for i = 1:(car_n)
                d(1:2,i)=[(r)*(car_n-i)/car_n;0];
            end                
% ----------------------------------------------------------- 行
            end
            
            Xd=d;

end



%% PID控制律
function [vr,vl,E_k,e_k_1]=PID(x,y,theta,X,dT,L,R,l,max_rpm,Kp,Ki,Kd,E_k,e_k_1,car_n)
%   控制律——生成小车运动速度
%   说明：将独轮车模型转换为质点模型进一步转换为差速模型
%-------------------------
%   输入：
%   x------当前x轴坐标
%   y------当前y轴坐标
%   theta------当前车头与x轴所成夹角
%   X------目标点坐标[x;y]
%   dT------时间间隔
%   硬件参数
%   PID 参数
%   输出：
%   vl------左轮速度
%   vr------右轮速度
%   累积量

            
            for i = 1:car_n
% 计算小车与目标点间距离
                g_x = X(1,i) - x(i)
                g_y = X(2,i) - y(i)
                
                for j = (i+1):(car_n+1)
                    x(car_n+1) = 0;
                    y(car_n+1) = 0;
                    if (sqrt((x(i)-x(j))^2+(y(i)-y(j))^2) < 60)
                        ao_x = x(i)-x(j);
                        ao_y = y(i)-y(j);
                        aph =0.5;
                        g_x = aph*g_x+(1-aph)*ao_x;%该机器人与目标点的距离，与其他机器人的距离的权重
                        g_y = aph*g_y+(1-aph)*ao_y;
                        break                       
                    end
                end
            
%计算小车线速度
                v = sqrt(g_y^2+g_x^2)
                
% 计算小车当前朝向与x轴夹角            
                phi = atan2(sin(theta(i)),cos(theta(i)));
                
% 计算小车期望朝向与x轴夹角  
                theta_g = atan2(g_y,g_x);    

                e_k = theta_g-phi ;        
                e_k = atan2(sin(e_k),cos(e_k));
  
 % 计算Kp
                e_P = e_k;
            
 % 计算Ki
                e_I = E_k(i) + e_k*dT;
                     
 % 计算Kd
                e_D = (e_k-e_k_1(i))/dT;    
              
 %计算小车角速度
                w = Kp*e_P + Ki*e_I + Kd*e_D
            
%储存误差量
                E_k(i) = e_I;
                e_k_1(i) = e_k;
            

% 与电机支持转速范围做比较
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

%质点模型转换为差分模型            
                vr(i) = (2*v+w*L)/(2*R);
                vl(i) = (2*v-w*L)/(2*R);
                 
            end

end

%% 传感器
function [x,y,theta,position_new] = sensor(x,y,theta,vr,vl,dT,L,car_n)
%   传感器——测量车轮转速获得实际位置
%   说明：将独轮车模型转换为质点模型进一步转换为差速模型
%-------------------------
%   输入：
%   x------前一时刻x轴坐标
%   y------前一时刻y轴坐标
%   theta------前一时刻车头与x轴所成夹角
%   vr------右轮转速
%   vl------左轮转速
%   dT------时间间隔
%   硬件参数
%   输出：
%   x------当前x轴坐标
%   y------当前y轴坐标
%   theta------当前车头与x轴所成夹角
           
%计算五辆车对应值            
            for i = 1:car_n
                
                position_new(1,i) = x(i);
                position_new(2,i) = y(i);
                position_new(3,i) = theta(i);
%左轮位移与右轮位移                        
                d_right = vr(i)*dT;
                d_left = vl(i)*dT;

%质心位置变化量与角度变化量
                d_center = (d_right + d_left)/2;
                theta_dt = (d_right - d_left)/L;
            
%x y 轴坐标变化量
                x_dt = d_center*cos(theta(i));
                y_dt = d_center*sin(theta(i));

%当前 x y theta                        
                theta(:,i) = theta(:,i) + theta_dt;
                theta(:,i) = atan2(sin(theta(:,i)),cos(theta(:,i)));
                x(i) = x(i) + x_dt;
                y(i) = y(i) + y_dt;  

            end

end
 
%% 绘图与动画
function plotS_I(X,position_new,theta,time,car_n,k)
%   绘图与动画——对小车的运动进行图形化展示
%   说明：画出小车期望路径、实际路径与位姿
%-------------------------
%   输入：
%   X------当前时刻期望坐标值
%   position_new------当前时刻实际坐标值
%   theta------当前时刻小车与x轴所成夹角
%   time------仿真时间
%   输出：
%   figure（2）------小车期望路径、实际路径与位姿

%% 路径的空间表示
% 用于绘制期望路径

% 用于绘制小车位姿
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
                        title('三机器人切换三种拓扑结构的编队控制')
                        xlabel('\fontsize{14}\fontname{Times New Roman}x (m)'),ylabel('\fontsize{14}\fontname{Times New Roman}\ity {\rm(m)}')
                end
                
                drawnow;
                grid off; 
                
 
end



function plotS_I2(position_new,time,car_n)
%   绘图与动画——对小车的运动进行图形化展示
%   说明：画出小车位置时间图像
%-------------------------
%   输入：
%   position_new------当前时刻实际坐标值
%   time------仿真时间
%   输出：
%   figure（1）------速度时间图像及位置时间图像

%% 路径的时间表示
% 用于绘制实际路径 
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
