function [ cmddata ] = load_cmd( file_path, navdata_timestamp0, data)
% Function on Nacho log_tools to load the commands sent to the multirotor
%   Detailed explanation goes here

if ( not(isa(data,'cell')) )
    disp(['Loading file "' file_path '"...']);
    data = txt2cellm(file_path, [' ' ':']);
else
    disp(['Data has been provided as argument to the load_cmd function.']);
end

% 1356083480544348 [cmd] mode:MOVE yawSpeed:-0.054506 pitch:-0.010991 roll:-0.033206 altSpeed:0.000000

disp('Recovering signals:');
disp('[Proxy mode]');
[cmddata.timestamp, cmddata.comm] = log2signals(data, 'cmd', 'mode', navdata_timestamp0, 0, 1);
% i = setdiff(1:size(cmddata.timestamp, 2), find(strcmp(cmddata.comm, 'error') == 1));
i = 1:length(cmddata.timestamp);
% cmddata.t = cmddata.timestamp(i) - cmddata.timestamp(i(1));
cmddata.t = cmddata.timestamp(i);
disp('[yawSpeedc]');
[cmddata.timestamp, cmddata.dyawc] = log2signals(data, 'cmd', 'yawSpeed', navdata_timestamp0);
cmddata.dyawc = cmddata.dyawc(i);
disp('[pitchc]');
[cmddata.timestamp, cmddata.pitchc] = log2signals(data, 'cmd', 'pitch', navdata_timestamp0);
cmddata.pitchc = cmddata.pitchc(i);
disp('[rollc]');
[cmddata.timestamp, cmddata.rollc] = log2signals(data, 'cmd', 'roll', navdata_timestamp0);
cmddata.rollc = cmddata.rollc(i);
disp('[altSpeedc]');
[cmddata.timestamp, cmddata.daltc] = log2signals(data, 'cmd', 'altSpeed', navdata_timestamp0);
cmddata.daltc = cmddata.daltc(i);
end

