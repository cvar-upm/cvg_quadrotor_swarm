function [ controllerdata ] = load_controller_data( file_path, navdata_timestamp0, data)
%Function on Nacho log_tools to load the controller commands
%   Detailed explanation goes here

if ( not(isa(data,'cell')) )
    disp(['Loading file "' file_path '"...']);
    data = txt2cellm(file_path, [' ' ':']);
else
    disp(['Data has been provided as argument to the load_controller_data function.']);
end

disp('Recovering signals:');
disp('[Control mode]');
[controllerdata.timestamp, controllerdata.mode] = log2signals(data, 'Control', 'mode', navdata_timestamp0, 0, 1);
% i = setdiff(1:size(controllerdata.timestamp, 2), find(strcmp(controllerdata.comm, 'error') == 1));
i = 1:length(controllerdata.timestamp);
% controllerdata.t = controllerdata.timestamp(i) - controllerdata.timestamp(i(1));
controllerdata.t = controllerdata.timestamp(i);
disp('[xc]');
[controllerdata.timestamp, controllerdata.xc] = log2signals(data, 'Control', 'x', navdata_timestamp0);
controllerdata.xc = controllerdata.xc(i);
disp('[yc]');
[controllerdata.timestamp, controllerdata.yc] = log2signals(data, 'Control', 'y', navdata_timestamp0);
controllerdata.yc = controllerdata.yc(i);
disp('[zc]');
[controllerdata.timestamp, controllerdata.zc] = log2signals(data, 'Control', 'z', navdata_timestamp0);
controllerdata.zc = controllerdata.zc(i);
disp('[vxc]');
[controllerdata.timestamp, controllerdata.vxc] = log2signals(data, 'Control', 'vx', navdata_timestamp0);
controllerdata.vxc = controllerdata.vxc(i);
disp('[vyc]');
[controllerdata.timestamp, controllerdata.vyc] = log2signals(data, 'Control', 'vy', navdata_timestamp0);
controllerdata.vyc = controllerdata.vyc(i);
disp('[vxfi]');
[controllerdata.timestamp, controllerdata.vxfi] = log2signals(data, 'Control', 'vxfi', navdata_timestamp0);
controllerdata.vxfi = controllerdata.vxfi(i);
disp('[vyfi]');
[controllerdata.timestamp, controllerdata.vyfi] = log2signals(data, 'Control', 'vyfi', navdata_timestamp0);
controllerdata.vyfi = controllerdata.vyfi(i);
disp('[vzfi]');
[controllerdata.timestamp, controllerdata.vzfi] = log2signals(data, 'Control', 'vzfi', navdata_timestamp0);
controllerdata.vzfi = controllerdata.vzfi(i);
disp('[rollfi]');
disp('[pitchfi]');
try
    [controllerdata.timestamp, controllerdata.rollfi] = log2signals(data, 'Control', 'roll', navdata_timestamp0);
    [controllerdata.timestamp, controllerdata.pitchfi] = log2signals(data, 'Control', 'pitch', navdata_timestamp0);
catch exception
    try
         [controllerdata.timestamp, controllerdata.rollfi] = log2signals(data, 'Control', 'rollfi', navdata_timestamp0);
        [controllerdata.timestamp, controllerdata.pitchfi] = log2signals(data, 'Control', 'pitchfi', navdata_timestamp0);
    catch exception
    end
end
controllerdata.rollfi = controllerdata.rollfi(i);
controllerdata.pitchfi = controllerdata.pitchfi(i);
disp('[yawc]');
[controllerdata.timestamp, controllerdata.yawc] = log2signals(data, 'Control', 'yaw', navdata_timestamp0);
controllerdata.yawc = controllerdata.yawc(i);
disp('[dyawfi]');
[controllerdata.timestamp, controllerdata.dyawfi] = log2signals(data, 'Control', 'dyawfi', navdata_timestamp0);
controllerdata.dyawfi = controllerdata.dyawfi(i);

end

