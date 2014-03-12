function [navdata data navdata_timestamp0] = load_navdata(file_path)

% cd lib;

disp(['Loading file "' file_path '"...']);
data = txt2cellm(file_path, [' ' ':']);
disp('Recovering signals:');
disp('[Comm. state]');
[navdata.timestamp, navdata.comm, first_time] = log2signals(data, 'info', 'comm', -1, 0, 1);
i = setdiff(1:size(navdata.timestamp, 2), find(strcmp(navdata.comm, 'error') == 1));
navdata.t = navdata.timestamp(i) - navdata.timestamp(i(1));
navdata_timestamp0 = first_time;
disp(strcat('navdata_timestamp0 = ', num2str(navdata_timestamp0)));
disp('[Mode]');
[navdata.timestamp, navdata.mode] = log2signals(data, 'info', 'mode', navdata_timestamp0, 0, 1);
navdata.mode = navdata.mode(i);
disp('[Native mode]');
[navdata.timestamp, navdata.native_mode] = log2signals(data, 'info', 'nat.mode', navdata_timestamp0);
navdata.native_mode = navdata.native_mode(i);
disp('[Battery]');
[navdata.timestamp, navdata.battery] = log2signals(data, 'info', 'batt', navdata_timestamp0);
navdata.battery = navdata.battery(i);
disp('[Yaw]');
[navdata.timestamp, navdata.yaw] = log2signals(data, 'info', 'yaw', navdata_timestamp0);
navdata.yaw = navdata.yaw(i);
disp('[Pitch]');
[navdata.timestamp, navdata.pitch] = log2signals(data, 'info', 'pitch', navdata_timestamp0);
navdata.pitch = navdata.pitch(i);
disp('[Roll]');
[navdata.timestamp, navdata.roll] = log2signals(data, 'info', 'roll', navdata_timestamp0);
navdata.roll = navdata.roll(i);
disp('[Altitude]');
[navdata.timestamp, navdata.altitude] = log2signals(data, 'info', 'z', navdata_timestamp0);
navdata.altitude = -navdata.altitude(i);
disp('[Forward speed]');
[navdata.timestamp, navdata.vx] = log2signals(data, 'info', 'Vx', navdata_timestamp0);
navdata.vx = navdata.vx(i);
disp('[Lateral speed]');
[navdata.timestamp, navdata.vy] = log2signals(data, 'info', 'Vy', navdata_timestamp0);
navdata.vy = navdata.vy(i);
disp('[Yaw rate]');
[navdata.timestamp, navdata.vyaw] = log2signals(data, 'info', 'Vyaw', navdata_timestamp0);
navdata.vyaw = navdata.vyaw(i);

% cd ..

end

