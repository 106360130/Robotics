close all;
clear all;

%T_input = [0.7936 -0 -0.9848 0; 0.8529 0.5 0.1504 0.3252; 0.4924 -0.866 0.0868 -0.158; 0 0 0 1] 
T_input = [-0.7071 -0 -0.7071 0; 0.5 0.7071 -0.5 0.37; 0.5 -0.7071 -0.5 0.26; 0 0 0 1]
theta_one_sol = kinematics_one_sol(T_input)


A = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1];
B = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1];
C = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1];

sampling_rate = 0.002;
t1 = 0 : sampling_rate : 0.3;
t2 = 0.3 : sampling_rate : 0.7;
t3 = 0.7 : sampling_rate : 1;

T = 1;
t_acc = 0.2;

%等速運動
q_t1 = linear_portion(A, B, T, t1);
q_t3 = linear_portion(B, C, T, t3);
%等速運動

A_new = q_t1(:, :, end);
C_new = q_t3(:, :, 1);

position_t1 = noap_postion(q_t1);
position_t3 = noap_postion(q_t3);

%加速運動
q_t2 = accelerated_portion(A_new, B, C_new, T, t2, t_acc);
%加速運動
position_t2 = noap_postion(q_t2);


position = [position_t1; position_t2; position_t3];

t = [t1 t2 t3];
figure();
subplot(3, 1, 1);plot(t, position(:,1));
title("X");
subplot(3, 1, 2);plot(t, position(:,2));
title("Y");
subplot(3, 1, 3);plot(t, position(:,3));
title("Z");

figure;
plot3(position(:,1), position(:,2), position(:,3));


