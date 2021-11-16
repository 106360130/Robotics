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

theta_inverse = kinematics_inverse(T)*(180/pi)