function [] = ekf_localization_new()
 
% Homework for ekf localization
% Modified by YH on 09/09/2019, thanks to the original open source
% Any questions please contact: zjuyinhuan@gmail.com

    close all;
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PxEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0.1 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
%     convQ=?
    Q=diag([0.1 0.1 degreeToRadian(1)]).^2;
    
    % Covariance Matrix for observation
%     convR=?
    R=diag([1.5 1.5 degreeToRadian(3)]).^2;
    
    % Other Intial
    % ?

    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        % ?
        % 已修改 应该没问题
        xPred = doMotion(xEkf, u);
        F=jacobF(xPred, u);
        PPred= F*PxEkf*F' + Q;
        
        % Update
        % xEkf=?
        % 已修改 应该没问题
        H=jacobH(xPred);
        y = z - doObservation(xPred);
        S = H*PPred*H' + R;
        K = PPred*H'*inv(S);
        xEkf = xPred + K*y;
        PxEkf = (eye(size(xEkf,1)) - K*H)*PPred;
        
        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 


end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    %?
    %已修改 应该没问题
        F = [1 0 0
        0 1 0
        0 0 1];
    v = sqrt(u(1)*u(1)+u(2)*u(2));
    B = [
        dt*cos(x(3)) 0 0
        dt*sin(x(3)) 0 0
        0 0 dt];
 
    x= F*x+B*[v,v,u(3)]';
end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    %?
    %已修改 应该有问题
    v = sqrt(u(1)*u(1)+u(2)*u(2));
    jF=[
    1 0 0
    0 1 0
    -dt*v*sin(x(1)) 1+dt*v*cos(x(1)) 1];

end

%Observation Model
function z = doObservation(x)
    %?
    H = [1 0 0
        0 1 0
        0 0 1];
 
    z=H*x;
 end

%Jacobian of Observation Model
function jH = jacobH(x)
    %?
    jH =[1 0 0
        0 1 0
        0 0 1];
end

% finally plot the results
function []=finalPlot(estimation)
    figure;
    
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
    
    % calculate error
    % ?
    x_true=estimation.xTruth(size(estimation.xTruth(:,1)),1);
    y_true=estimation.xTruth(size(estimation.xTruth(:,2)),2);
    fprintf("终点真实值：      ");
    fprintf("x=%f,y=%f\n",x_true(1),y_true(1));
    
    x_ekf=estimation.xEkf(size(estimation.xEkf(:,1)),1);
    y_ekf=estimation.xEkf(size(estimation.xEkf(:,2)),2);
    fprintf("终点EFK定位结果： ");
    fprintf("x=%f,y=%f\n",x_ekf(1),y_ekf(1));
    
    x_xodom=estimation.xOdom(size(estimation.xOdom(:,1)),1);
    y_xodom=estimation.xOdom(size(estimation.xOdom(:,2)),2);
    fprintf("终点纯里程计累积：");
    fprintf("x=%f,y=%f\n\n",x_xodom(1),y_xodom(1));
    
    error_ekf=sqrt((x_true(1)-x_ekf(1))^2+(y_true(1)-y_ekf(1))^2);
    error_xodom=sqrt((x_true(1)-x_xodom(1))^2+(y_true(1)-y_xodom(1))^2);
    fprintf("终点纯里程计累积误差为：");
    disp(error_xodom);
    fprintf("终点EKF定位误差为：");
    disp(error_ekf);
    
    number=size(estimation.xTruth(:,1));
    error_ekf_sum=0;
    error_xodom_sum=0;
    for i=1:1:number(1)
        x_true=estimation.xTruth(i,1);
        y_true=estimation.xTruth(i,2);
        x_ekf=estimation.xEkf(i,1);
     	y_ekf=estimation.xEkf(i,2);
        x_xodom=estimation.xOdom(i,1);
        y_xodom=estimation.xOdom(i,2);
        error_ekf_sum=error_ekf_sum+sqrt((x_true(1)-x_ekf(1))^2+(y_true(1)-y_ekf(1))^2);
        error_xodom_sum=error_xodom_sum+sqrt((x_true(1)-x_xodom(1))^2+(y_true(1)-y_xodom(1))^2);
        
    end
    fprintf("纯里程计累积平均误差为：");
    disp(error_xodom_sum/number(1));
    fprintf("EKF平均定位误差为：");
    disp(error_ekf_sum/number(1));
end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end
