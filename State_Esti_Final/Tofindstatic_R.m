load('Vipose_B3_data.mat')



Vipose_data.time = Vipose_data.time - Vipose_data.time(1);

y = zeros([6, 2325]);
 y(1,:) = Vipose_data.x;
 y(2,:) = Vipose_data.y;
 y(3,:) = Vipose_data.z;
 qt = [Vipose_data.qw, Vipose_data.qx, Vipose_data.qy, Vipose_data.qz];
 quttoeul= quat2eul(qt);
 y(4,:)= quttoeul(:,3);
 y(5,:)= quttoeul(:,2);
 y(6,:)= quttoeul(:,1);



%Finding Variance of y1,y2,y3 resp  Measurment Noise
y1=Vipose_data.x;
y2=Vipose_data.y;
y3= Vipose_data.z;
%%%%%%%%%%
y1_std= std(y1);
y2_std= std(y2);
y3_std= std(y3);
y7_std= std(eul_tta);
y8_std= std(eul_phi);
y9_std= std(eul_shi);
%%%%%%%%%
V_variance1=y1_std^2  %square of x1_std
V_variance2=y2_std^2
V_variance3=y3_std^2
V_variance7=y7_std^2
V_variance8=y8_std^2
V_variance9=y9_std^2