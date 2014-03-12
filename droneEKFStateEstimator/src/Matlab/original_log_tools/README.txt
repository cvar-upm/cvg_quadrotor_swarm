------------------------------------------------
Available functions for MAVwork log manipulation
------------------------------------------------

navdata = load_navdata(path_to_file)
    Description
        This function loads the navigation data from a MAVwork log file into a structure
    Input
        path_to_file: Absolute or relative path to file
    Output
        navdata: Structure with navigation data
            [Members of navdata structure]
            timestamp: Internal proxy timestamp (microseconds). The timebase is unknown. For plots, better use 't'.
            comm: State of the communication with the drone at every timestamp.
            t: Sample time. The initial value is 0.
            mode:  Mode at every sample time. Valid modes are: LANDED, TAKINGOFF, HOVERING, FLYING, LANDING
            native_mode: Internal MAV code to describe its status.
            battery: Battery level; 1=full, 0=empty.
            yaw: Rotation around the MAV local z axis in degrees. The MAV z axis points to the ground from the MAV center.
            pith: Rotation around the MAV local y axis in degrees. The MAV y axis points to the right from the MAVs center.
            roll: Rotation around the MAV local x axis in degrees. The MAV x axis points forward from the MAV center.
            altitude: MAV altitude in meters.
            vx: Forward speed in m/s (along MAV local x axis). Forward is positive, backward is negative.
            vy: Latersal speed in m/s (along MAV local y axis). Right is positive, left is negative.
            vyaw: Yaw rate in degrees/s around the MAV local z axis.

trajectory = trajectory_from_navdata(navdata)
    Description
        This function reconstructs the the MAV trajectory from the altitude data and the trapezoidal integration of forward and lateral speeds.
    Input
        navdata: Structure obtained from a call to load_navdata()
    Output
        trajectory: MAV trajectory reconstruction.
            [Members trajectory structure]
            t: Sample time. It is equivalent to navdata.t
            pos: 3D position at each t. It is a 3xN matrix, where N is the number of samples in the trajectory.

plot_3d_trajectory(trajectory)
    Description
        This function draws the MAV trajectory through a plot3().
    Input
        trajectory: Structure obtained from a call to trajectory_from_navdata()

--------------
Examples
--------------

% This loads a log file as a structure
data = load_navdata('events.txt');

% This plots pitch (in blue) and forward speed (in green) in single graph
figure;
plot(data.t, data.pitch, 'b');
hold on;
plot(data.t, data.vx, 'g');

% This plots the MAV trajectory
trajectory = trajectory_from_navdata(data);
figure;
plot_3d_trajectory(trajectory);

% Show the moments (in seconds) when the communication with the drone was lost or recovered
comm_ok = find(strcmp(data.comm, 'ok') == 1);
comm_ok_to_error_transitions = find(diff(strcmp(data.comm, 'error')) == 1);
comm_lost_times = data.timestamp(comm_ok_to_error_transitions) - data.timestamp(comm_ok(1))
comm_error_to_ok_transitions = find(diff(strcmp(data.comm, 'ok')) == 1);
comm_recovered_times = data.timestamp(comm_error_to_ok_transitions + 1) - data.timestamp(comm_ok(1))
