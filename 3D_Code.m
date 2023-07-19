% 3-D position estimation filter
clc;
clear all;
True_position=zeros(3,1);
True_position(:,1)=[1 1 1]';
True_velocity(:,1)=[0 0 0]';
True_acceleration(:,1)=[0 0 0]';
measured_position(:,1)=[0 0 0]';
Time_period=0.2;
time=0:Time_period:1000;
observation_matrix=[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
for k=2:1:length(time)
    True_position(:,k)=True_position(:,k-1)+True_velocity(:,k-1)*Time_period+(1/2)*True_acceleration(:,k-1)*Time_period^2;
    measured_position(:,k)=True_position(:,k)+5*randn(3,1);
    True_velocity(:,k)=True_velocity(:,k-1)+True_acceleration(:,k-1)*Time_period;
    measured_velocity(:,k)=True_velocity(:,k)+10*randn(3,1);
    True_acceleration(:,k)=randn(3,1)/10;
end
A=[1 0 0 Time_period 0 0; 0 1 0 0 Time_period 0; 0 0 1 0 0 Time_period; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]; % State transition matrix
B=[(Time_period^2)/2 0 0; 0 (Time_period^2)/2 0; 0 0 (Time_period^2)/2; Time_period 0 0; 0 Time_period 0; 0 0 Time_period];
Q = [0.005 0 0 0 0 0; 0 0.005 0 0 0 0;0 0 0.0005 0 0 0; 0 0 0 0.0003 0 0; 0 0 0 0 0.00001 0; 0 0 0 0 0 0.0002]; % process Covariance
R = [0.05 0 0;0 0.05 0; 0 0 0.05];  % noise covarainace
P = zeros(size(Q)); % initial error covariance
Xe=[0 0 0 0 0 0]'; % initialization of Estimated values
for k=1:length(True_position)
    if(k==1)
    x1 = A*Xe(:,k) + B*(randn(3,1)/100);  % Update equations
    P1 = A*P*A' + Q;
    else
    x1 = A*Xe(:,k-1) + B*(randn(3,1)/100);
    P1 = A*P*A' + Q;
    end
    K = P1*observation_matrix'*inv(observation_matrix*P1*observation_matrix' + R); %Measurement equations
    Xe(:,k) = x1+K*(measured_position(:,k) - observation_matrix*x1);    
    P = (eye(size(P)) - K*observation_matrix)*P1;
end
[x,y]=size(Xe);
for i=1:3
    for j=1:y
        X(i,j)=Xe(i,j);
    end
end
figure,
u=plot3(True_position(1,(1:10:end)),True_position(2,(1:10:end)),True_position(3,(1:10:end)),'g','LineWidth',1); hold on
v=plot3(X(1,(1:10:end)),X(2,(1:10:end)),X(3,(1:10:end)),'b','LineWidth',1);hold on
w=plot3(measured_position(1,(1:10:end)),measured_position(2,(1:10:end)),measured_position(3,(1:10:end)),'r','LineWidth',1);
legend([u,w,v],'True 3-D position','noisy','Estimates 3-D Position')