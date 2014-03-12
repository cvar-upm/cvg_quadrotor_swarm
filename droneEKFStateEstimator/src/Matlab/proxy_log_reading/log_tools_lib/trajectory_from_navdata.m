function trajectory = trajectory_from_navdata(navdata)

cd lib
[trajectory.t, pos] = pos_from_vel(navdata.t, navdata.vx, navdata.vy, navdata.yaw, navdata.pitch, navdata.roll);
trajectory.pos = pos; 
trajectory.pos(3, :) = navdata.altitude;
cd ..

end

