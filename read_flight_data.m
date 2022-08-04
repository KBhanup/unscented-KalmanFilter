%% Clear everything
clear; close all; clc;

bag_file = 'flight_logs_nmpc_flt_2022-07-29-11-15-56_Dep14.bag';
rot_data = rosbag(bag_file);
rosbag info 'flight_logs_nmpc_flt_2022-07-29-11-15-56_Dep14.bag';
%rosbag info 'Feb14.bag'

%%%%%
%%******************
% Rc_inputs
rc_bag = select(rot_data, 'Topic', '/mavros/rc/in');
rc_bag_struct = readMessages(rc_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %/rc/in
rc_data = table();
rc_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), rc_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, rc_bag_struct);
start_t= rc_data.time(1);
rc_data.time = rc_data.time - start_t;

rc_data.roll = cellfun(@(m) double(m.Channels(3,1)), rc_bag_struct);
rc_data.pitch = cellfun(@(m) double(m.Channels(4,1)), rc_bag_struct);
rc_data.yaw = cellfun(@(m) double(m.Channels(2,1)), rc_bag_struct);
rc_data.thrust = cellfun(@(m) double(m.Channels(1,1)), rc_bag_struct);

%rc_data.arm = cellfun(@(m) double(m.Channels(5,1)), rc_bag_struct);
rc_data.cntlr = cellfun(@(m) double(m.Channels(11,1)), rc_bag_struct);
rc_data.offb = cellfun(@(m) double(m.Channels(5,1)), rc_bag_struct);




% %%%%%%%%%%%%%%%%%%
% lOCAL_odom-pose
Lodom_bag = select(rot_data, 'Topic', '/mavros/local_position/odom');
Lodom_bag_struct = readMessages(Lodom_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %StagArray
Lodom_data = table();
Lodom_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), Lodom_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, Lodom_bag_struct);
Lodom_data.time = Lodom_data.time - start_t;

Lodom_data.x = cellfun(@(m) double(m.Pose.Pose.Position.X), Lodom_bag_struct);
Lodom_data.y = cellfun(@(m) double(m.Pose.Pose.Position.Y), Lodom_bag_struct);
Lodom_data.z = cellfun(@(m) double(m.Pose.Pose.Position.Z), Lodom_bag_struct);

Lodom_data.qw = cellfun(@(m) double(m.Pose.Pose.Orientation.W), Lodom_bag_struct);
Lodom_data.qx = cellfun(@(m) double(m.Pose.Pose.Orientation.X), Lodom_bag_struct);
Lodom_data.qy = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), Lodom_bag_struct);
Lodom_data.qz = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), Lodom_bag_struct);
% %%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%
% Mocap-pose
Mocap_bag = select(rot_data, 'Topic', '/mavros/vision_pose/pose');
Mocap_bag_struct = readMessages(Mocap_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %StagArray
Mocap_data = table();
Mocap_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), Mocap_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, Mocap_bag_struct);
Mocap_data.time = Mocap_data.time - start_t;

Mocap_data.x = cellfun(@(m) double(m.Pose.Position.X), Mocap_bag_struct);
Mocap_data.y = cellfun(@(m) double(m.Pose.Position.Y), Mocap_bag_struct);
Mocap_data.z = cellfun(@(m) double(m.Pose.Position.Z), Mocap_bag_struct);

Mocap_data.qw = cellfun(@(m) double(m.Pose.Orientation.W), Mocap_bag_struct);
Mocap_data.qx = cellfun(@(m) double(m.Pose.Orientation.X), Mocap_bag_struct);
Mocap_data.qy = cellfun(@(m) double(m.Pose.Orientation.Y), Mocap_bag_struct);
Mocap_data.qz = cellfun(@(m) double(m.Pose.Orientation.Z), Mocap_bag_struct);


%%%%%%%%%%%%%%%%%%
%%%Deploed points
Deployed_bag = select(rot_data, 'Topic', '/deployed_setpoint');
Deployed_bag_struct = readMessages(Deployed_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %/drone_state
Deployed_data = table();
Deployed_data.Markerx = cellfun(@(m) double(m.X), Deployed_bag_struct);
Deployed_data.Markery = cellfun(@(m) double(m.Y), Deployed_bag_struct);
Deployed_data.Markerz = cellfun(@(m) double(m.Z), Deployed_bag_struct);
% %%%%%%%%%%%%%%%%%%
% Mocap-pose
Local_bag = select(rot_data, 'Topic', '/mavros/local_position/odom');
Local_bag_struct = readMessages(Local_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %StagArray
Local_data = table();
Local_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), Local_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, Local_bag_struct);
Local_data.time = Local_data.time - start_t;

Local_data.x = cellfun(@(m) double(m.Pose.Pose.Position.X), Local_bag_struct);
Local_data.y = cellfun(@(m) double(m.Pose.Pose.Position.Y), Local_bag_struct);
Local_data.z = cellfun(@(m) double(m.Pose.Pose.Position.Z), Local_bag_struct);
% %%%%%%%%%%%%%%%%%%
%Drone_Trajectory
DrnTrj_bag = select(rot_data, 'Topic', '/drone_trajectory');
DrnTrj_bag_struct = readMessages(DrnTrj_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %StagArray
DrnTrj_data = table();
DrnTrj_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), DrnTrj_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9,DrnTrj_bag_struct);
DrnTrj_data.time = DrnTrj_data.time - start_t;

DrnTrj_data.x = cellfun(@(m) double(m.Trajectory_.Position.X), DrnTrj_bag_struct);
DrnTrj_data.y = cellfun(@(m) double(m.Trajectory_.Position.Y), DrnTrj_bag_struct);
DrnTrj_data.z = cellfun(@(m) double(m.Trajectory_.Position.Z), DrnTrj_bag_struct);

%DrnTrj_data.qw = cellfun(@(m) double(m.Trajectory_.Orientation.W), DrnTrj_bag_struct);
DrnTrj_data.qx = cellfun(@(m) double(m.Trajectory_.Orientation.X), DrnTrj_bag_struct);
DrnTrj_data.qy = cellfun(@(m) double(m.Trajectory_.Orientation.Y), DrnTrj_bag_struct);
DrnTrj_data.qz = cellfun(@(m) double(m.Trajectory_.Orientation.Z), DrnTrj_bag_struct);
% %%%%%%%%%%%%%%%%%%


%%%%%%%rosout
% %%%%%%%%%%%%%%%%%%
% %Drone_Trajectory
% rosout_bag = select(rot_data, 'Topic', '/rosout');
% rosout_bag_struct = readMessages(rosout_bag, 'DataFormat', 'struct');
% 
% % bag_struct{1,1} %StagArray
% rosout_data = table();
% rosout_data.time = ...
%     cellfun(@(m) double(m.Header.Stamp.Sec), rosout_bag_struct) + ...
%     cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9,rosout_bag_struct);
% rosout_data.time = rosout_data.time - start_t;
% 
% rosout_data.x = cellfun(@(m) double(m.Msg), rosout_bag_struct, 'UniformOutput',false);
% rosout_data.y = cellfun(@(m) cell(m.Topics), rosout_bag_struct, 'UniformOutput',false);

% Rosout
bag_select = select(rot_data, 'Topic', '/rosout');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');
rosout_data = table();
rosout_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);
rosout_data.time = rosout_data.time - start_t;
rosout_data.node = cellfun(@(m) string(m.Name), bag_struct);
rosout_data.msg = cellfun(@(m) string(m.Msg), bag_struct);
rosout_data.level = cellfun(@(m) double(m.Level), bag_struct);

%%%%%%%%%


% %%%%%%%%%%%%%%%%%%
%Setpoint_raw/local PositionTarget
Setploc_bag = select(rot_data, 'Topic', '/mavros/setpoint_raw/local');
Setploc_bag_struct = readMessages(Setploc_bag, 'DataFormat', 'struct');

% bag_struct{1,1}
Setploc_data = table();
Setploc_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), Setploc_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9,Setploc_bag_struct);
Setploc_data.time =Setploc_data.time - start_t;

Setploc_data.x = cellfun(@(m) double(m.Position.X), Setploc_bag_struct);
Setploc_data.y = cellfun(@(m) double(m.Position.Y), Setploc_bag_struct);
Setploc_data.z = cellfun(@(m) double(m.Position.Z), Setploc_bag_struct);

%%********
%Setpoint_raw/attitude
Setpatti_bag = select(rot_data, 'Topic', '/mavros/setpoint_raw/attitude');
Setpatti_bag_struct = readMessages(Setpatti_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %StagArray
Setpatti_data = table();
Setpatti_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), Setpatti_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9,Setpatti_bag_struct);
Setpatti_data.time =Setpatti_data.time - start_t;

Setpatti_data.qx = cellfun(@(m) double(m.Orientation.X), Setpatti_bag_struct);
Setpatti_data.qy = cellfun(@(m) double(m.Orientation.Y), Setpatti_bag_struct);
Setpatti_data.qz = cellfun(@(m) double(m.Orientation.Z), Setpatti_bag_struct);
Setpatti_data.qw = cellfun(@(m) double(m.Orientation.W), Setpatti_bag_struct);
Setpatti_data.Thrust = cellfun(@(m) double(m.Thrust), Setpatti_bag_struct);


%%%%%%%%%%%%
% Marker-pose wrt Drone
Drone_select = select(rot_data, 'Topic', '/marker/pose');
Drone_struct = readMessages(Drone_select, 'DataFormat', 'struct');
% bag_struct{1,1} %StagArray
Drone_data = table();
Drone_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), Drone_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, Drone_struct);
Drone_data.time = Drone_data.time - start_t;

Drone_data.x = cellfun(@(m) double(m.Pose.Position.X), Drone_struct);
Drone_data.y = cellfun(@(m) double(m.Pose.Position.Z), Drone_struct);
Drone_data.z = cellfun(@(m) double(m.Pose.Position.Y), Drone_struct);
%[-3.711696,-1.365)-3.332]
Drone_data.qw = cellfun(@(m) double(m.Pose.Orientation.W), Drone_struct);
Drone_data.qx = cellfun(@(m) double(m.Pose.Orientation.X), Drone_struct);
Drone_data.qy = cellfun(@(m) double(m.Pose.Orientation.Z), Drone_struct);
Drone_data.qz = cellfun(@(m) double(m.Pose.Orientation.Y), Drone_struct);

% %%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%
% % Marker-pose
% cam_select = select(rot_data, 'Topic', '/stag_ros/markers');
% cam_struct = readMessages(cam_select, 'DataFormat', 'struct');
% % bag_struct{1,1} %StagArray
% cam_data = table();
% cam_data.time = ...
%     cellfun(@(m) double(m.StagArray.Header.Stamp.Sec), cam_struct) + ...
%     cellfun(@(m) double(m.StagArray.Header.Stamp.Nsec)*1e-9, cam_struct);
% cam_data.time = cam_data.time - cam_data.time(1);
% 
% cam_data.x = cellfun(@(m) double(m.StagArray.Pose.Position.X), cam_struct);
% cam_data.y = cellfun(@(m) double(m.StagArray.Pose.Position.Z), cam_struct);
% cam_data.z = cellfun(@(m) double(m.StagArray.Pose.Position.Y), cam_struct);
% %[-3.711696,-1.365)-3.332]
% cam_data.qw = cellfun(@(m) double(m.StagArray.Pose.Orientation.W), cam_struct);
% cam_data.qx = cellfun(@(m) double(m.StagArray.Pose.Orientation.X), cam_struct);
% cam_data.qy = cellfun(@(m) double(m.StagArray.Pose.Orientation.Z), cam_struct);
% cam_data.qz = cellfun(@(m) double(m.StagArray.Pose.Orientation.Y), cam_struct);

% %%%%%%%%%%%%%%%%%%
% Drone_State
drnst_bag = select(rot_data, 'Topic', '/drone_state');
drnst_bag_struct = readMessages(drnst_bag, 'DataFormat', 'struct');

% bag_struct{1,1} %/drone_state
drnst_data = table();
drnst_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), drnst_bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, drnst_bag_struct);
drnst_data.time = drnst_data.time -start_t;

drnst_data.x = cellfun(@(m) double(m.Pose.Position.X), drnst_bag_struct);
drnst_data.y = cellfun(@(m) double(m.Pose.Position.Y), drnst_bag_struct);
drnst_data.z = cellfun(@(m) double(m.Pose.Position.Z), drnst_bag_struct);

drnst_data.qw = cellfun(@(m) double(m.Pose.Orientation.W), drnst_bag_struct);
drnst_data.qx = cellfun(@(m) double(m.Pose.Orientation.X), drnst_bag_struct);
drnst_data.qy = cellfun(@(m) double(m.Pose.Orientation.Y), drnst_bag_struct);
drnst_data.qz = cellfun(@(m) double(m.Pose.Orientation.Z), drnst_bag_struct);

drnst_data.Vx = cellfun(@(m) double(m.Velocity.X), drnst_bag_struct);
drnst_data.Vy = cellfun(@(m) double(m.Velocity.Y), drnst_bag_struct);
drnst_data.Vz = cellfun(@(m) double(m.Velocity.Z), drnst_bag_struct);

drnst_data.Dbx = cellfun(@(m) double(m.Disturbances.X), drnst_bag_struct);
drnst_data.Dby = cellfun(@(m) double(m.Disturbances.Y), drnst_bag_struct);
drnst_data.Dbz = cellfun(@(m) double(m.Disturbances.Z), drnst_bag_struct);

drnst_data.MarkerFound = cellfun(@(m) double(m.MarkerFound.Data), drnst_bag_struct);
drnst_data.Markerx = cellfun(@(m) double(m.MarkerPose.Position.X), drnst_bag_struct);
drnst_data.Markery = cellfun(@(m) double(m.MarkerPose.Position.Y), drnst_bag_struct);
drnst_data.Markerz = cellfun(@(m) double(m.MarkerPose.Position.Z), drnst_bag_struct);


%%%How to select i? from the resp distb and thrustcmd pick the row number
% %%%start and end of the time you want to sample between
% for i=1342:2681
%     mnDbz(i-1341)= drnst_data.Dbz(i); 
%     
% end
% 
% for i=144:644
%     mnThrst(i-143)= Setpatti_data.Thrust(i);
% end
% % 
% % % for i=818: 2427
% % %     mnDbz(i-817)= drnst_data.Dbz(i); 
% % %     
% % % end
% % % 
% % % for i=110:710
% % %     mnThrst(i-109)= Setpatti_data.Thrust(i);
% % % end
% mnDbz = mnDbz';
% mnThrst =mnThrst';
% meanofDbz = mean(mnDbz);
% meanofThrst = mean(mnThrst); 


% %%Position
% figure
% subplot(3,1,1)
% hold on
% plot(Mocap_data.time, Mocap_data.x, 'r-', 'LineWidth', 4)
% plot(drnst_data.time, drnst_data.x, 'b.', 'LineWidth', 1)
% hold off
% legend('Mocap.X', 'drnst.x','Location','southwest');
% title('PoseX');
% ylim([-3, 2])
% %xlim([0,186.2])
% xlabel('Time steps');
% hold off
% 
% subplot(3,1,2)
% hold on
% plot(Mocap_data.time, Mocap_data.y, 'r-', 'LineWidth', 4)
% plot(drnst_data.time, drnst_data.y, 'b.', 'LineWidth', 1)
% hold off
% legend('Mocap.Y', 'drnst.Y','Location','southwest');
% title('PoseY');
% ylim([-3, 2])
% %xlim([0,186.2])
% xlabel('Time steps');
% hold off
% 
% subplot(3,1,3)
% hold on
% plot(Mocap_data.time, Mocap_data.z, 'r-', 'LineWidth', 4)
% plot(drnst_data.time, drnst_data.z, 'b.', 'LineWidth', 1)
% hold off
% legend('Mocap.Z', 'drnst.Z','Location','southwest');
% title('PoseZ');
% %ylim([-3, 2])
% %xlim([0,186.2])
% xlabel('Time steps');
% hold off
% %%%


%%Drone_Trajectory
figure
subplot(3,1,1)
hold on
plot(Mocap_data.time, Mocap_data.x, 'r-', 'LineWidth', 4)
plot(drnst_data.time, drnst_data.x, 'b.', 'LineWidth', 1)
%plot(Local_data.time, Local_data.x, 'k.')
plot(drnst_data.time, drnst_data.Markerx, 'g-', 'LineWidth', 2)
plot(DrnTrj_data.time, DrnTrj_data.x, 'k.', 'LineWidth', 2)
plot([36.09, 36.09], [-2, 0], '--g', 'LineWidth', 1);
plot([69.44, 69.44], [-2, 0], '--g', 'LineWidth', 1);
yline(-1.6,'--k', 'LineWidth', 1 );
%plot(drnst_data.time, 1, '--g', 'LineWidth','LineWidth', 3)
hold off
legend('Mocap.X', 'drnst.x','Location','northeast');
title('PoseX');
ylim([-2, 2])
%xlim([120, 160])
xlabel('Time steps');
hold off

subplot(3,1,2)
hold on
plot(Mocap_data.time, Mocap_data.y, 'r-', 'LineWidth', 4)
plot(drnst_data.time, drnst_data.y, 'b.', 'LineWidth', 1)
%plot(Local_data.time, Local_data.y, 'k.')
plot(drnst_data.time, drnst_data.Markery, 'g-', 'LineWidth', 2)
plot(DrnTrj_data.time, DrnTrj_data.y, 'k.', 'LineWidth', 2)
plot([36.09, 36.09], [-2, 0], '--g', 'LineWidth', 1);
plot([69.44, 69.44], [-2, 0], '--g', 'LineWidth', 1);
yline(0,'--k', 'LineWidth', 1 );
hold off
legend('Mocap.Y', 'drnst.Y','Location','northeast');
title('PoseY');
ylim([-2, 2])
%xlim([120, 160])
xlabel('Time steps');
hold off

subplot(3,1,3)
hold on
plot(Mocap_data.time, Mocap_data.z, 'r-', 'LineWidth', 4)
plot(drnst_data.time, drnst_data.z, 'b.', 'LineWidth', 1)
%plot(Local_data.time, Local_data.z, 'k.')
plot(drnst_data.time, drnst_data.Markerz, 'g-', 'LineWidth', 2)
plot(DrnTrj_data.time, DrnTrj_data.z, 'k.', 'LineWidth', 2)
plot([36.09, 36.09], [0, 2], '--g', 'LineWidth', 1);
plot([69.44, 69.44], [0, 2], '--g', 'LineWidth', 1);
yline(2.11,'--k', 'LineWidth', 1 );
hold off
legend('Mocap.Z', 'drnst.Z','Location','northeast');
title('PoseZ');
ylim([0, 4])
%xlim([120, 160])
xlabel('Time steps');
hold off

%%%%

%%%%%%%%%%%%%%%%%
%Distrubances
figure
subplot(3,1,1)
hold on
plot(drnst_data.time, drnst_data.Dbx, 'r.', 'LineWidth', 2)
plot([36.09, 36.09], [-2, 0], '--g', 'LineWidth', 1);
plot([69.44, 69.44], [-2, 0], '--g', 'LineWidth', 1);
yline(0,'--k', 'LineWidth', 1 );
hold off
legend('DbX','Location','northeast');
title('DistbX');
ylim([-3, 2])
%xlim([120, 160])
xlabel('Time steps');
hold off

subplot(3,1,2)
hold on
plot(drnst_data.time, drnst_data.Dby, 'r.', 'LineWidth', 2)
plot([36.09, 36.09], [-2, 0], '--g', 'LineWidth', 1);
plot([69.44, 69.44], [-2, 0], '--g', 'LineWidth', 1);
yline(0,'--k', 'LineWidth', 1 );
hold off
legend('DbY','Location','northeast');
title('DistbY');
ylim([-3, 2])
%xlim([120, 160])
xlabel('Time steps');
hold off

subplot(3,1,3)
hold on
plot(drnst_data.time, drnst_data.Dbz, 'r.', 'LineWidth', 2)
plot([36.09, 36.09], [-2, 0], '--g', 'LineWidth', 1);
plot([69.44, 69.44], [-2, 0], '--g', 'LineWidth', 1);
yline(0,'--k', 'LineWidth', 1 );
hold off
legend('DbZ','Location','northeast');
title('DistbZ');
ylim([-3, 2])
%xlim([120, 160])
xlabel('Time steps');
hold off
%%%%%%%%
figure
hold on
plot(rc_data.time, rc_data.offb, 'b.', 'LineWidth', 2)
hold off
legend('offb', 'Location','northeast');
title('Offb');
%xlim([0,187])
xlabel('Time steps');
hold off



%%%%%%%%%%%%%%%%%%
