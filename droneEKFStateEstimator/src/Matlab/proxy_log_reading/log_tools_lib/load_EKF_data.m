function [ ekfdata ] = load_EKF_data( file_path, navdata_timestamp0, data)
% Function on Nacho log_tools to load the EKF estimations
%   Detailed explanation goes here

if ( not(isa(data,'cell')) )
    disp(['Loading file "' file_path '"...']);
    data = txt2cellm(file_path, [' ' ':']);
else
    disp(['Data has been provided as argument to the load_EKF_data function.']);
end

disp('Recovering signals:');
disp('[Pitch]');
[ekfdata.timestamp, ekfdata.pitch] = log2signals(data, 'EKF', 'pitch', navdata_timestamp0);
% i = setdiff(1:size(ekfdata.timestamp, 2), find(strcmp(ekfdata.comm, 'error') == 1));
i = 1:length(ekfdata.timestamp);
% ekfdata.t = ekfdata.timestamp(i) - ekfdata.timestamp(i(1));
ekfdata.t = ekfdata.timestamp(i);
ekfdata.pitch = ekfdata.pitch(i);
disp('[Roll]');
[ekfdata.timestamp, ekfdata.roll] = log2signals(data, 'EKF', 'roll', navdata_timestamp0);
ekfdata.roll = ekfdata.roll(i);
disp('[dYaw]');
[ekfdata.timestamp, ekfdata.dyaw] = log2signals(data, 'EKF', 'dYaw', navdata_timestamp0);
ekfdata.dyaw = ekfdata.dyaw(i);
disp('[Yaw]');
[ekfdata.timestamp, ekfdata.yaw] = log2signals(data, 'EKF', 'Yaw', navdata_timestamp0);
ekfdata.yaw = ekfdata.yaw(i);
disp('[dZ]');
[ekfdata.timestamp, ekfdata.dz] = log2signals(data, 'EKF', 'dZ', navdata_timestamp0);
ekfdata.dz = ekfdata.dz(i);
disp('[Z]');
[ekfdata.timestamp, ekfdata.altitude] = log2signals(data, 'EKF', 'Z', navdata_timestamp0);
ekfdata.altitude = ekfdata.altitude(i);
disp('[X]');
[ekfdata.timestamp, ekfdata.x] = log2signals(data, 'EKF', 'X', navdata_timestamp0);
ekfdata.x = ekfdata.x(i);
disp('[Y]');
[ekfdata.timestamp, ekfdata.y] = log2signals(data, 'EKF', 'Y', navdata_timestamp0);
ekfdata.y = ekfdata.y(i);
disp('[Vx]');
[ekfdata.timestamp, ekfdata.vx] = log2signals(data, 'EKF', 'Vx', navdata_timestamp0);
ekfdata.vx = ekfdata.vx(i);
disp('[Vy]');
[ekfdata.timestamp, ekfdata.vy] = log2signals(data, 'EKF', 'Vy', navdata_timestamp0);
ekfdata.vy = ekfdata.vy(i);
disp('[Vxm]');
[ekfdata.timestamp, ekfdata.vxm] = log2signals(data, 'EKF', 'Vxm', navdata_timestamp0);
ekfdata.vxm = ekfdata.vxm(i);
disp('[Vym]');
[ekfdata.timestamp, ekfdata.vym] = log2signals(data, 'EKF', 'Vym', navdata_timestamp0);
ekfdata.vym = ekfdata.vym(i);

end

