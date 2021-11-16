clear all;
close all;

theta = [90, 99, -119, -10, 10, 0];
T = kinematics_forward(theta)


euler_angles = euler_angle(T)




a = [0.12, 0.25, 0.26, 0, 0, 0];
d = [0, 0, 0, 0, 0, 0];
alpha=[-90, 0, 0, -90, 90, 0];

i = 6;
T_result = T_coe(d(i), a(i), alpha(i))

theta_inverse_1 = kinematics_inverse_1(T)*(180/pi)
theta_inverse_2 = kinematics_inverse_2(T)*(180/pi)
theta_inverse_3 = kinematics_inverse_3(T)*(180/pi)
theta_inverse_4 = kinematics_inverse_4(T)*(180/pi)
theta_inverse_5 = kinematics_inverse_5(T)*(180/pi)
theta_inverse_6 = kinematics_inverse_6(T)*(180/pi)
theta_inverse_7 = kinematics_inverse_7(T)*(180/pi)
theta_inverse_8 = kinematics_inverse_8(T)*(180/pi)