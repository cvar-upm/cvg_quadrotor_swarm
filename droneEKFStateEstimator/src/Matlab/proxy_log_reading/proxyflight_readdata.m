%% Add nacho proxy log_tools to path
addpath(genpath('./log_tools_lib'));

%% load data
clear all, close all, clc;

file_path = 'PreTest01.txt';
file_path = 'events_video.txt';
% Altitude01.txt

[navdata data navdata_timestamp0] = load_navdata(file_path);
[cmddata] = load_cmd(file_path, navdata_timestamp0, data);
[ekfdata] = load_EKF_data(file_path, navdata_timestamp0, data);
[controllerdata] = load_controller_data(file_path, navdata_timestamp0, data);

%% plot data
close all;

i_pos = strcmp(controllerdata.mode,'position');
i_spd = strcmp(controllerdata.mode,'speed');
i_tcpos = strcmp(controllerdata.mode,'tc_position');
i_tcstr = strcmp(controllerdata.mode,'tc_straight');
i_tctrn = strcmp(controllerdata.mode,'tc_turn');

figure(1)
subplot(2,1,1)
plot( cmddata.t, cmddata.pitchc*24);
hold all
plot( navdata.t, navdata.pitch);
plot( ekfdata.t, ekfdata.pitch*180/pi);
grid on;
ylabel('pitch[deg]');
xlabel('t   [seg]');
% legend('command','EKF')
subplot(2,1,2)
plot( cmddata.t, cmddata.rollc*24);
hold all
plot( navdata.t, navdata.roll);
plot( ekfdata.t, ekfdata.roll*180/pi);
grid on;
ylabel('roll[deg]');
xlabel('t   [seg]');
legend('cmd','navdata','EKF')

figure(2)
subplot(2,1,1)
plot( controllerdata.t, controllerdata.vxc);
hold all
plot( controllerdata.t, controllerdata.vxfi);
plot( ekfdata.t, ekfdata.vx);
grid on;
ylabel('vx[m/s]');
xlabel('t [seg]');
% legend('command','EKF')
subplot(2,1,2)
plot( controllerdata.t, controllerdata.vyc);
hold all
plot( controllerdata.t, controllerdata.vyfi);
plot( ekfdata.t, ekfdata.vy);
legend('cmd','cmdfi','EKF')
grid on;
ylabel('vx[m/s]');
xlabel('t [seg]');

figure(3)
subplot(3,1,1)
plot( controllerdata.t, controllerdata.xc);
hold all
plot( ekfdata.t, ekfdata.x);
grid on;
ylabel('X [m]');
xlabel('t [seg]');
% legend('command','EKF')
subplot(3,1,2)
plot( controllerdata.t, controllerdata.yc);
hold all
plot( ekfdata.t, ekfdata.y);
legend('command','EKF')
grid on;
ylabel('Y [m]');
xlabel('t [seg]');
subplot(3,1,3)
plot( controllerdata.t, controllerdata.zc);
hold all
plot( ekfdata.t, ekfdata.altitude);
plot( navdata.t, navdata.altitude);
legend('command','EKF')
grid on;
ylabel('Z [m]');
xlabel('t [seg]');

figure(4)
plot( controllerdata.xc, controllerdata.yc);
hold all
plot( ekfdata.x, ekfdata.y);
grid on;
ylabel('X [m]');
xlabel('Y [m]');
legend('command','EKF')
axis equal;

figure(5)
scatter( controllerdata.t, i_pos);
hold all
scatter( controllerdata.t, i_spd);
scatter( controllerdata.t, i_tcpos);
scatter( controllerdata.t, i_tcstr);
scatter( controllerdata.t, i_tctrn);
ylim([0 2])
legend('pos','spd','tcpos','tcstr','tctrn');

figure(6)
% Altitude controller debug
subplot(2,1,1)
plot( controllerdata.t, controllerdata.zc);
hold all
plot( ekfdata.t, ekfdata.altitude);
plot( navdata.t, navdata.altitude);
legend('command','EKF')
grid on;
ylabel('Z [m]');
xlabel('t [seg]');
subplot(2,1,2)
plot( cmddata.t, cmddata.daltc);
legend('command')
grid on;
ylabel('dZ/dt [m]');
xlabel('t [seg]');

figure(7)
plot3( controllerdata.xc, controllerdata.yc , controllerdata.zc);
hold all
plot3( ekfdata.x, ekfdata.y, ekfdata.altitude);
legend('command','EKF')
grid on;
xlabel('X [m]');
ylabel('Y [m]');
ylabel('Z [m]');
axis equal

Ts = 0.06;
t  = min(ekfdata.t):Ts:max(ekfdata.t);
ekfx = interp1(ekfdata.t, ekfdata.x, t);
ctrlxc = interp1( controllerdata.t, controllerdata.xc, t);
ekfy = interp1(ekfdata.t, ekfdata.y, t);
ctrlyc = interp1( controllerdata.t, controllerdata.yc, t);
ekfz = interp1(ekfdata.t, ekfdata.altitude, t);
ctrlzc = interp1( controllerdata.t, controllerdata.zc, t);

figure(8)
subplot(3,1,1)
plot( t, ekfx - ctrlxc);
ylim([-0.5 +.5]);
ylabel('eX [m]');
xlabel('t [seg]');
grid on
subplot(3,1,2)
plot( t, ekfy - ctrlyc);
ylim([-0.5 +.5]);
ylabel('eY [m]');
xlabel('t [seg]');
grid on
subplot(3,1,3)
plot( t, ekfz - ctrlzc);
ylim([-0.5 +.5]);
ylabel('eZ [m]');
xlabel('t [seg]');
grid on




%% Remove nacho proxy log_tools from path
rmpath(genpath('./log_tools_lib'));
