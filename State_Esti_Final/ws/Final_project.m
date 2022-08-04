%% Clear and close everything
clear; close all; clc;

% Load HW data
load('IMU_B5_data.mat')
load('Opti_B5_data.mat')
load('Vipose_B5_data.mat')
load('Campose_B5_data.mat')
time_steps = length(Vipose_data.time);

%% 0. Start by entering your last name
lastName = "kosaraju";
disp("EMCH-792 State_esti_FinalProject, " + lastName)
IMU_data.time = IMU_data.time - IMU_data.time(1);
Vipose_data.time = Vipose_data.time - Vipose_data.time(1);
opti_data.time= opti_data.time - opti_data.time(1);
dt=Vipose_data.time(2);

%%%%%%%%%%%%%%%%%%
%Extract T from H_O_m
optiT=zeros([3,time_steps]);
 for i = 2:time_steps
     optiT(1,i)=opti_data.x(find(opti_data.time <= Vipose_data.time(i),1,'last'));
     optiT(2,i)=opti_data.y(find(opti_data.time <= Vipose_data.time(i),1,'last'));
     optiT(3,i)=opti_data.z(find(opti_data.time <= Vipose_data.time(i),1,'last'));
 end

%Extract Euler
optiq=zeros([4,time_steps]);
 for i = 2:time_steps
     optiq(1,i)=opti_data.qw(find(opti_data.time <= Vipose_data.time(i),1,'last'));
     optiq(2,i)=opti_data.qx(find(opti_data.time <= Vipose_data.time(i),1,'last'));
     optiq(3,i)=opti_data.qy(find(opti_data.time <= Vipose_data.time(i),1,'last'));
     optiq(4,i)=opti_data.qz(find(opti_data.time <= Vipose_data.time(i),1,'last'));
 end
%%
opti_eu = zeros(4,time_steps);
opti_eul= zeros(3,time_steps);
for i = 1: time_steps
    opti_eu(:,i) = [optiq(1,i), optiq(2,i), optiq(3,i), optiq(4,i)];
    opti_eul(:,i) = quat2eul(opti_eu(:,i)');   
end



% % Initialize system constants

%For y_est input fn
camq=zeros([4,time_steps]);
 for i = 2:time_steps
     camq(1,:)=cam_data.qw;
     camq(2,:)=cam_data.qx;
     camq(3,:)=cam_data.qy;
     camq(4,:)=cam_data.qz;
 end
 camT=zeros([3,time_steps]);
 for i = 2:time_steps
     camT(1,:)=cam_data.x;
     camT(2,:)=cam_data.y;
     camT(3,:)=cam_data.z;
 end

%Rotation MAtrix to extract From Camera data
for t=1:time_steps
    q(:,t)=[camq(1,t), camq(2,t), camq(3,t), camq(4,t)];   
end
qut=q';
eul=quat2eul(qut);    
eul_tta=eul(:,3);
eul_phi=eul(:,2);
eul_shi=eul(:,1);

 y = zeros([6, time_steps]);
 y(1,:) = Vipose_data.x;
 y(2,:) = Vipose_data.y;
 y(3,:) = Vipose_data.z;
 qt = [Vipose_data.qw, Vipose_data.qx, Vipose_data.qy, Vipose_data.qz];
 quttoeul= quat2eul(qt);
 y(4,:)= quttoeul(:,3);
 y(5,:)= quttoeul(:,2);
 y(6,:)= quttoeul(:,1);


X_true = zeros([9,time_steps]);
X_true(1,:) = optiT(1,:);
X_true(2,:) = optiT(2,:);
X_true(3,:) = optiT(3,:);
X_true(7,:) = opti_eul(3,:);
X_true(8,:) = opti_eul(1,:);
X_true(9,:) = opti_eul(2,:);


%%Inputs
ax=IMU_data.ax;     %%Inputs 
ay=IMU_data.ay; 
az=IMU_data.az;
ttax= IMU_data.ttax;
phiy= IMU_data.shiz;
shiz= IMU_data.phiy;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finding Variance of y1,y2,y3 resp  Measurment Noise
y1=Vipose_data.x;
y2=Vipose_data.y;
y3=Vipose_data.z;
%%%%%%%%%%
y1_std= std(y1);
y2_std= std(y2);
y3_std= std(y3);
y7_std= std(eul_tta);
y8_std= std(eul_phi);
y9_std= std(eul_shi);
%%%%%%%%%%
V_variance1=y1_std^2;  %square of x1_std
V_variance2=y2_std^2;
V_variance3=y3_std^2;
V_variance7=y7_std^2;
V_variance8=y8_std^2;
V_variance9=y9_std^2;
% Initialize R and Q matrices

Q = diag([0,0,0,0.1, 0.1, 0.1, 0.001, 0.001, 0.01]);
R = zeros([6,6]);
R(1,1)=V_variance1;
R(2,2)=V_variance2;
R(3,3)=V_variance3;
R(4,4)=V_variance7;
R(5,5)=V_variance8;
R(6,6)=V_variance9;


% 2. Unscented Kalman Filter
%%%%%%%
%Initializing the UKF specific variables
X_est_UKF = zeros([9, time_steps]);
P_UKF = diag([1, 1, 1, 1, 1, 1, 1, 1, 1]);
P_store_UKF = zeros([81, time_steps]);
P_store_UKF(:, 1) = P_UKF(:);

% For sigma points
n = size(X_est_UKF, 1);
m = size(y, 1);

x_hat = zeros(n, 2*n);
s_hat = zeros(n, 2*n);
y_hat = zeros(m, 2*n);
% We already have the true state and measurements from the EKF
for i = 2:time_steps
    % Get L matrix (L*L' = P)
    L = chol(P_UKF)';
    
    %1a.% Get sigma points
    x_hat(:, 1) = X_est_UKF(:, i-1) + sqrt(n) * L(:, 1);
    x_hat(:, 2) = X_est_UKF(:, i-1) - sqrt(n) * L(:, 1);
    x_hat(:, 3) = X_est_UKF(:, i-1) + sqrt(n) * L(:, 2);
    x_hat(:, 4) = X_est_UKF(:, i-1) - sqrt(n) * L(:, 2);
    x_hat(:, 5) = X_est_UKF(:, i-1) + sqrt(n) * L(:, 3);
    x_hat(:, 6) = X_est_UKF(:, i-1) - sqrt(n) * L(:, 3);
    x_hat(:, 7) = X_est_UKF(:, i-1) + sqrt(n) * L(:, 4);
    x_hat(:, 8) = X_est_UKF(:, i-1) - sqrt(n) * L(:, 4);
    x_hat(:, 9) = X_est_UKF(:, i-1) + sqrt(n) * L(:, 5);
    x_hat(:, 10) = X_est_UKF(:, i-1) - sqrt(n) * L(:,5);
    x_hat(:, 11) = X_est_UKF(:, i-1) + sqrt(n) * L(:,6);
    x_hat(:, 12) = X_est_UKF(:, i-1) - sqrt(n) * L(:,6);
    x_hat(:, 13) = X_est_UKF(:, i-1) + sqrt(n) * L(:,7);
    x_hat(:, 14) = X_est_UKF(:, i-1) - sqrt(n) * L(:,7);
    x_hat(:, 15) = X_est_UKF(:, i-1) + sqrt(n) * L(:,8);
    x_hat(:, 16) = X_est_UKF(:, i-1) - sqrt(n) * L(:,8);
    x_hat(:, 17) = X_est_UKF(:, i-1) + sqrt(n) * L(:,9);
    x_hat(:, 18) = X_est_UKF(:, i-1) - sqrt(n) * L(:,9);

    
    %1b.% Propagate sigma points
    for j = 1 : size(x_hat,2)
        s_hat(:, j) = system_f(x_hat(:, j), dt, ax(j), ay(j), az(j), ttax(j), phiy(j), shiz(j)); %
    end
    
    %1c.% Update estimated state
    s_mean = sum(s_hat,2) / (2 * n);
    X_est_UKF(:, i) = s_mean;   
    
    %2% Update the error covariance matrix
    P_UKF = Q;
    for k = 1 : size(x_hat, 2)
        ds = (s_hat(:, k)-s_mean);
        P_UKF = P_UKF + ((ds*ds') / (2 * n));
    end
    
    % Get new L matrix (L*L' = P)
    L = chol(P_UKF)';
    
    % Get new sigma points
    x_hat(:, 1) = X_est_UKF(:, i) + sqrt(n) * L(:, 1);
    x_hat(:, 2) = X_est_UKF(:, i) - sqrt(n) * L(:, 1);
    x_hat(:, 3) = X_est_UKF(:, i) + sqrt(n) * L(:, 2);
    x_hat(:, 4) = X_est_UKF(:, i) - sqrt(n) * L(:, 2);
    x_hat(:, 5) = X_est_UKF(:, i) + sqrt(n) * L(:, 3);
    x_hat(:, 6) = X_est_UKF(:, i) - sqrt(n) * L(:, 3);
    x_hat(:, 7) = X_est_UKF(:, i) + sqrt(n) * L(:, 4);
    x_hat(:, 8) = X_est_UKF(:, i) - sqrt(n) * L(:, 4);
    x_hat(:, 9) = X_est_UKF(:, i) + sqrt(n) * L(:, 5);
    x_hat(:, 10) = X_est_UKF(:,i) - sqrt(n) * L(:, 5);
    x_hat(:, 11) = X_est_UKF(:,i) + sqrt(n) * L(:, 6);
    x_hat(:, 12) = X_est_UKF(:,i) - sqrt(n) * L(:, 6);
    x_hat(:, 13) = X_est_UKF(:,i) + sqrt(n) * L(:, 7);
    x_hat(:, 14) = X_est_UKF(:,i) - sqrt(n) * L(:, 7);
    x_hat(:, 15) = X_est_UKF(:,i) + sqrt(n) * L(:, 8);
    x_hat(:, 16) = X_est_UKF(:,i) - sqrt(n) * L(:, 8);
    x_hat(:, 17) = X_est_UKF(:,i) + sqrt(n) * L(:, 9);
    x_hat(:, 18) = X_est_UKF(:,i) - sqrt(n) * L(:, 9);
   
    %3a.% Run sigma points through measurement function
    for j = 1 : size(x_hat, 2)
        y_hat(:,j) = measurement_h(x_hat(:, j));
    end
    
    %3b.% Get measurement mean
    y_mean = sum(y_hat, 2) / (2 * n);
    
    %4a.% Compute measurement covariance matrix
    P_y = R;
    for w = 1 : size(y_hat, 2)
        dy = y_hat(:,w) - y_mean;
        P_y = P_y + (dy*dy')  / (2 * n);
    end
    
    %4b.% Compute cross covariance
    P_xy = 0;
    for j = 1 : size(x_hat, 2)
        dx = x_hat(:, j) - X_est_UKF(:, i);
        dy = y_hat(:,j) - y_mean;
        P_xy = P_xy + (dx*dy');
    end
    P_xy = P_xy / (2 * n);
        
    %4c.% Compute Kalman gain
    K = P_xy/P_y;
    
    % Get expected measurement
    y_exp(:,i) = y_mean;    
    
    %5.% Correct state estimate
    X_est_UKF(:, i) = X_est_UKF(:, i) + K*(y(:, i) - y_exp(:,i));
    
    %6.% Update the error covariance matrix
    P_UKF = P_UKF - K*P_y*K';
    P_store_UKF(:, i) = P_UKF(:);
    display(i)
end

%% 3. Plot the filters results
figure
hold on
plot(X_true(1, :), 'k-')
plot(X_est_UKF(1, :), 'b-')
plot(y(1, :), '')
hold off
legend('X1 True', 'X1 UKF Estimate','y','Location','best'); 
title('X1');
ylim([-3, 2])
xlabel('Time steps');

figure
hold on
plot(X_true(2, :), 'k-')
plot(X_est_UKF(2, :), 'b-')
plot(y(2, :), '')
hold off
legend('X2 True', 'X2 UKF Estimate', 'y','Location','best'); 
title('X2');
ylim([-3, 2])
xlabel('Time steps');

figure
hold on
plot(X_true(3, :), 'k-')
plot(X_est_UKF(3, :), 'b-')
plot(y(3, :), '')
hold off
legend('X3 True', 'X3 UKF Estimate','y','Location','best');
title('X3');
ylim([-3, 2])
xlabel('Time steps');

figure
hold on
plot(X_true(7, :), 'k-')
plot(X_est_UKF(7, :), 'b-')
plot(y(4, :), '')
hold off
legend('X7 True', 'X7 UKF Estimate','y','Location','best');
title('X7');
ylim([-3, 2])
xlabel('Time steps');

figure
hold on
plot(X_true(8, :), 'k-')
plot(X_est_UKF(8, :), 'b-')
plot(y(5, :), '')
hold off
legend('X8 True', 'X8 UKF Estimate','y','Location','best');
title('X8');
ylim([-3, 2.6])
xlabel('Time steps');

figure
hold on
plot(X_true(9, :), 'k-')
plot(X_est_UKF(9, :), 'b-')
plot(y(6, :), '')
hold off
legend('X9 True', 'X9 UKF Estimate','y','Location','best');
title('X9');
ylim([-3, 2])
xlabel('Time steps');

%% 4. Plot the filters error levels and estimated error bounds
%%%%%%%%%%%%%%%%%%%%%%%%%Error of filters%%%%%%%%%%%%%%%%%%%%%%%%
std_X1_UKF = sqrt(P_store_UKF(1, :));
std_X2_UKF = sqrt(P_store_UKF(11, :));
std_X3_UKF = sqrt(P_store_UKF(21, :));
std_X7_UKF = sqrt(P_store_UKF(61, :));
std_X8_UKF = sqrt(P_store_UKF(71, :));
std_X9_UKF = sqrt(P_store_UKF(81, :));

figure
hold on
plot(X_true(1, :) - X_est_UKF(1, :), 'b-')
plot(3*std_X1_UKF, 'r--')
plot(-3*std_X1_UKF, 'r--')
hold off
legend('UKF Error', 'UKF 3std', 'Location','best'); 
title('X1 estimation error');
ylabel('Estimation error');
xlabel('Time steps');


figure
hold on
plot(X_true(2, :) - X_est_UKF(2, :), 'b-')
plot(3*std_X2_UKF, 'r--')
plot(-3*std_X2_UKF, 'r--')
hold off
legend('UKF Error', 'UKF 3std', 'Location','best'); 
title('X2 estimation error');
ylabel('Estimation error');
xlabel('Time steps');

figure
hold on
plot(X_true(3, :) - X_est_UKF(3, :), 'b-')
plot(3*std_X3_UKF, 'r--')
plot(-3*std_X3_UKF, 'r--')
hold off
legend('UKF Error', 'UKF 3std', 'Location','best'); 
title('X3 estimation error');
ylabel('Estimation error');
xlabel('Time steps');

figure
hold on
plot(X_true(7, :) - X_est_UKF(7, :), 'b-')
plot(3*std_X7_UKF, 'r--')
plot(-3*std_X7_UKF, 'r--')
hold off
legend('UKF Error', 'UKF 3std', 'Location','best'); 
title('X7 estimation error');
ylabel('Estimation error');
xlabel('Time steps');

figure
hold on
plot(X_true(8, :) - X_est_UKF(8, :), 'b-')
plot(3*std_X8_UKF, 'r--')
plot(-3*std_X8_UKF, 'r--')
hold off
legend('UKF Error', 'UKF 3std', 'Location','best'); 
title('X8 estimation error');
ylabel('Estimation error');
xlabel('Time steps');

figure
hold on
plot(X_true(9, :) - X_est_UKF(9, :), 'b-')
plot(3*std_X9_UKF, 'r--')
plot(-3*std_X9_UKF, 'r--')
hold off
legend('UKF Error', 'UKF 3std', 'Location','best'); 
title('X9 estimation error');
ylabel('Estimation error');
xlabel('Time steps');

%% Any functions need to go under here

function y_est = measurement_h(X)
url=[X(9), X(8), X(7)];
R = eul2rotm(url,'ZYX');
T = [X(1); X(2); X(3)];
H_C_m = [R T; 0, 0, 0, 1];  %Marker in camera's(Base) Frame
H_D_c = [0.0, 0.0, 1.0, 0.1; -1.0, 0.0, 0.0, 0.025; 0.0, -1.0, 0.0, -0.085; 0, 0, 0, 1];
H_D_m = mtimes(H_C_m, H_D_c);  %Marker in Drones Frame
R_Dm = zeros(3,3);
R_Dm(1,1) = H_D_m(1,1);
R_Dm(1,2) = H_D_m(1,2);
R_Dm(1,3) = H_D_m(1,3);
R_Dm(2,1) = H_D_m(2,1);
R_Dm(2,2) = H_D_m(2,2);
R_Dm(2,3) = H_D_m(2,3);
R_Dm(3,1) = H_D_m(3,1);
R_Dm(3,2) = H_D_m(3,2);
R_Dm(3,3) = H_D_m(3,3);
rpy = rotm2eul(R_Dm, 'ZYX');
y_est= zeros(6,1);
y_est(1,1)= H_D_m(1,4);
y_est(2,1)= H_D_m(2,4);
y_est(3,1)= H_D_m(3,4);
y_est(4,1)= rpy(3);
y_est(5,1)= rpy(2);
y_est(6,1)= rpy(1);
end


function X_p = system_f(X, dt,ax,ay,az,ttax,phiy,shiz) %
% X = [x_1;, x_2; x_3....]
X_p(1, 1) = X(1)+X(4)*dt;
X_p(2, 1) = X(2)+X(5)*dt;
X_p(3, 1) = X(3)+X(6)*dt;
X_p(4, 1) = X(4)+ax*dt; 
X_p(5, 1) = X(5)+ay*dt; 
X_p(6, 1) = X(6)+az*dt;
X_p(7, 1) = X(7)+ttax*dt;
X_p(8, 1) = X(8)+phiy*dt;
X_p(9, 1) = X(9)+shiz*dt;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%