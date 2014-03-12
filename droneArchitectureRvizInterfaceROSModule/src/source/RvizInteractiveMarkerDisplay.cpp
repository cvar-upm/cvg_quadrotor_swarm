#include "RvizInteractiveMarkerDisplay.h"

using namespace visualization_msgs;
using namespace interactive_markers;

//Menu Initialize
MenuHandler menu_handler;
MenuHandler::EntryHandle h_first_entry;
MenuHandler::EntryHandle h_mode_last=2;
MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Drone Selection" );

//Server Initialize
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

//Marker objects
visualization_msgs::Marker obstacles;
visualization_msgs::Marker walls;
visualization_msgs::Marker line_strip;
visualization_msgs::Marker point;

//Initialize Markers
int ini = 0;
int iniMP = 0;
int iniO = 0;


//-------------------------Create Markers(Box / Sphere)------------------------------------------------//


Marker makeBox( InteractiveMarker &msg )
{

    Marker marker;
    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;

}

Marker makeSphere(InteractiveMarker &msg )
{

    Marker marker;
    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale * 0.4;
    marker.scale.y = msg.scale * 0.4;
    marker.scale.z = msg.scale * 0.4;
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    return marker;

}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{

    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();

}

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";             //Para cambiar la orientacion del GRF aqui se puede!
    int_marker.scale = 1;

    return int_marker;

}

void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
    h_mode_last = feedback->menu_entry_id;
    std::string namesss=feedback->control_name;
    ROS_INFO("Name: %s", namesss.c_str());
    ROS_INFO("last %i", h_mode_last);
    menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );
    menu_handler.reApply( *server );
    server->applyChanges();

}


//--------------------------Create Menu/Interactive Markers---------------------------------------------//


void DroneRvizDisplay::initMenu(int i)
{

    std::ostringstream s;

    s << "Drone " << i;

    h_mode_last = menu_handler.insert( sub_menu_handle, s.str(), &modeCb );
    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
    h_mode_last = 2;
    menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );  //try
    //

    ROS_INFO("INIT");
    //obstacles.ns="zero";


}

void DroneRvizDisplay::makeAxesMenu(std::string name)
{

    InteractiveMarker int_marker = makeEmptyMarker();
    int_marker.name = name;
    int_marker.description = name;
    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;
    int_marker.controls.push_back(control);
    server->insert(int_marker);

}

void DroneRvizDisplay::makeMovingMarker( const tf::Vector3& position , std::string name, std::string frame_id_in)
{

    std::string frame_id = frame_id_in;
    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;



    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    ROS_INFO("%s", name.c_str());

    int_marker.name = name;
    int_marker.description = name;
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;
    control.markers.push_back( makeSphere(int_marker) );
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    ROS_INFO("Created Marker");

    static tf::TransformBroadcaster br;
    tf::Transform t;
    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(0.0,-10.0,0.0));
    t.setRotation(tf::createQuaternionFromRPY(0.0,0.0,0.0));
    br.sendTransform(tf::StampedTransform(t, time, "base_link",frame_id_in));

}


//--------------------------SERVER FUNCTIONS-------------------------------------------------------------//


void DroneRvizDisplay::ServerReset()
{

    server.reset();

    return;

}

int DroneRvizDisplay::ActiveDroneId()
{

    return h_mode_last-1;

}

void DroneRvizDisplay::ServerResetNew()
{

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    return;

}

void DroneRvizDisplay::MenuHandlerApply()
{

    menu_handler.apply(*server, "GRF");

}

void DroneRvizDisplay::ServerApplyChanges()
{

    server->applyChanges();

    return;

}


//---------------------------SUBSCRIBER CALLBACK FUNCTIONS--------------------------------------------------//


void DroneRvizDisplay::PoseCallback(const droneMsgsROS::dronePose &pose_euler, int idDrone)
{

    static tf::TransformBroadcaster br;
    tf::Transform t;
    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(double(pose_euler.x),double(pose_euler.y),double(pose_euler.z)));
    t.setRotation(tf::createQuaternionFromRPY(pose_euler.roll, pose_euler.pitch, pose_euler.yaw));
    br.sendTransform(tf::StampedTransform(t, time, "base_link",std::string("Moving ")+std::to_string(idDrone)));
    std::string frame=std::string("Moving ")+std::to_string(idDrone);

    //ROS_INFO("%s",frame.c_str());


    return;

}

void DroneRvizDisplay::ObstaclesPubCallback(const droneMsgsROS::obstaclesTwoDim obstacles, int idDrone)
{

    if (iniO == idDrone)
    {
        DroneRvizDisplay::ObstaclesPubCallbackAdd(obstacles, "Obstacles Drone"+std::to_string(idDrone));
    }
    else if (iniO==0)
    {
        DroneRvizDisplay::ObstaclesPubCallbackAdd(obstacles, "Obstacles Drone"+std::to_string(idDrone));
        iniO=idDrone;
    }
    else if (iniO != idDrone && iniO != 0)
    {
        DroneRvizDisplay::ObstaclesPubCallbackDelete(obstacles, "Obstacles Drone"+std::to_string(iniO));
        DroneRvizDisplay::ObstaclesPubCallbackAdd(obstacles, "Obstacles Drone"+std::to_string(idDrone));
        iniO=idDrone;

    }

}

void DroneRvizDisplay::TrajectoryPubCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone)
{

    if (ini == idDrone)
    {
        DroneRvizDisplay::TrajectoryPubCallbackAdd(trajectory, idDrone, "Trajectory Drone"+std::to_string(idDrone));
    }
    else if (ini==0)
    {
        DroneRvizDisplay::TrajectoryPubCallbackAdd(trajectory, idDrone, "Trajectory Drone"+std::to_string(idDrone));
        ini=idDrone;
    }
    else if (ini != idDrone && ini != 0)
    {
        DroneRvizDisplay::TrajectoryPubCallbackDelete(trajectory, ini, "Trajectory Drone"+std::to_string(ini));
        DroneRvizDisplay::TrajectoryPubCallbackAdd(trajectory, idDrone, "Trajectory Drone"+std::to_string(idDrone));
        ini=idDrone;

    }

}

void DroneRvizDisplay::MissionPointPubCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone)
{

    if (iniMP == idDrone)
    {
        DroneRvizDisplay::MissionPointPubCallbackAdd(missionPoint, idDrone, "Mission Point Drone"+std::to_string(idDrone));
    }
    else if (iniMP==0)
    {
        DroneRvizDisplay::MissionPointPubCallbackAdd(missionPoint, idDrone, "Mission Point Drone"+std::to_string(idDrone));
        iniMP=idDrone;
    }
    else if (iniMP != idDrone && iniMP != 0)
    {
        DroneRvizDisplay::MissionPointPubCallbackDelete(missionPoint, iniMP, "Mission Point Drone"+std::to_string(iniMP));
        DroneRvizDisplay::MissionPointPubCallbackAdd(missionPoint, idDrone, "Mission Point Drone"+std::to_string(idDrone));
        iniMP=idDrone;

    }

}


//----------------------------ADD/DELETE MARKERS CALLBACKS--------------------------------------------------//


void DroneRvizDisplay::ObstaclesPubCallbackAdd(const droneMsgsROS::obstaclesTwoDim Drone_obstacles, std::string name)
{

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obstacles.header.frame_id = "/base_link";
    obstacles.header.stamp = ros::Time::now();
    walls.header.frame_id = "/base_link";
    walls.header.stamp = ros::Time::now();

    // Set the namespace and id for this obstacles.  This serves to create a unique ID
    // Any obstacles sent with the same namespace and id will overwrite the old one
    obstacles.ns=name;
    walls.ns = name;
    // Set the obstacles type.
    obstacles.type = visualization_msgs::Marker::CYLINDER;
    walls.type = visualization_msgs::Marker::CUBE;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    obstacles.pose.orientation.x = 0.0;
    obstacles.pose.orientation.y = 0.0;
    obstacles.pose.orientation.z = 0.0;
    obstacles.pose.orientation.w = 1.0;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    walls.pose.orientation.x = 0.0;
    walls.pose.orientation.y = 0.0;
    walls.pose.orientation.z = 0.0;
    walls.pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    obstacles.color.r = 0.0f;
    obstacles.color.g = 0.0f;
    obstacles.color.b = 0.8f;
    obstacles.color.a = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    walls.color.r = 0.0f;
    walls.color.g = 0.0f;
    walls.color.b = 0.8f;
    walls.color.a = 0.75;

    // Marker Lifetime
    obstacles.lifetime = ros::Duration();

    // Marker Lifetime
    walls.lifetime = ros::Duration();

    //Obstacle Iterator


    for ( auto it : Drone_obstacles.poles )
    {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_O = it.id;
        x1 = it.centerX;
        y = it.centerY;
        rx = it.radiusX;
        ry = it.radiusY;
        yaw = it.yawAngle;

        //Unique Obstacle ID
        obstacles.id = id_O;

        // Set the obstacles action
        obstacles.action = visualization_msgs::Marker::ADD;

        //Set Obstacle Pose
        obstacles.pose.position.x = x1;
        obstacles.pose.position.y = y;
        obstacles.pose.position.z = 1.0;

        //Set Obstacle Scale
        obstacles.scale.x = rx;
        obstacles.scale.y = ry;
        obstacles.scale.z = 2.0;

        // Publish the obstacles
        obstacles_pub.publish(obstacles);

    }

    //Walls Iterator


    for ( auto it : Drone_obstacles.walls )
    {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_2 = it.id;
        x2 = it.centerX;
        y2 = it.centerY;
        sx = it.sizeX;
        sy = it.sizeY;
        yaw2 = it.yawAngle;

        //Unique Obstacle ID
        walls.id = id_2;

        // Set the obstacles action
        walls.action = visualization_msgs::Marker::ADD;

        //Set Obstacle Pose
        walls.pose.position.x = x2;
        walls.pose.position.y = y2;
        walls.pose.position.z = 1.0;

        //Set Obstacle Scale
        walls.scale.x = sx;
        walls.scale.y = sy;
        walls.scale.z = 2.0;

        // Publish the obstacles
        obstacles_pub.publish(walls);

    }

}

void DroneRvizDisplay::ObstaclesPubCallbackDelete(const droneMsgsROS::obstaclesTwoDim Drone_obstacles, std::string name)
{

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obstacles.header.frame_id = "/base_link";
    obstacles.header.stamp = ros::Time::now();
    walls.header.frame_id = "/base_link";
    walls.header.stamp = ros::Time::now();

    // Set the namespace and id for this obstacles.  This serves to create a unique ID
    // Any obstacles sent with the same namespace and id will overwrite the old one
    obstacles.ns=name;
    walls.ns = name;
    // Set the obstacles type.
    obstacles.type = visualization_msgs::Marker::CYLINDER;
    walls.type = visualization_msgs::Marker::CUBE;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    obstacles.pose.orientation.x = 0.0;
    obstacles.pose.orientation.y = 0.0;
    obstacles.pose.orientation.z = 0.0;
    obstacles.pose.orientation.w = 1.0;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    walls.pose.orientation.x = 0.0;
    walls.pose.orientation.y = 0.0;
    walls.pose.orientation.z = 0.0;
    walls.pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    obstacles.color.r = 0.0f;
    obstacles.color.g = 0.0f;
    obstacles.color.b = 0.8f;
    obstacles.color.a = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    walls.color.r = 0.0f;
    walls.color.g = 0.0f;
    walls.color.b = 0.8f;
    walls.color.a = 0.7;

    // Marker Lifetime
    obstacles.lifetime = ros::Duration();

    // Marker Lifetime
    walls.lifetime = ros::Duration();

    //Obstacle Iterator


    for ( auto it : Drone_obstacles.poles )
    {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_O = it.id;
        x1 = it.centerX;
        y = it.centerY;
        rx = it.radiusX;
        ry = it.radiusY;
        yaw = it.yawAngle;

        //Unique Obstacle ID
        obstacles.id = id_O;

        // Set the obstacles action
        obstacles.action = visualization_msgs::Marker::DELETE;

        //Set Obstacle Pose
        obstacles.pose.position.x = x1;
        obstacles.pose.position.y = y;
        obstacles.pose.position.z = 1.0;

        //Set Obstacle Scale
        obstacles.scale.x = rx;
        obstacles.scale.y = ry;
        obstacles.scale.z = 2.0;

        // Publish the obstacles
        obstacles_pub.publish(obstacles);

    }

    //Walls Iterator


    for ( auto it : Drone_obstacles.walls )
    {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_2 = it.id;
        x2 = it.centerX;
        y2 = it.centerY;
        sx = it.sizeX;
        sy = it.sizeY;
        yaw2 = it.yawAngle;

        //Unique Obstacle ID
        walls.id = id_2;

        // Set the obstacles action
        walls.action = visualization_msgs::Marker::DELETE;

        //Set Obstacle Pose
        walls.pose.position.x = x2;
        walls.pose.position.y = y2;
        walls.pose.position.z = 1.0;

        //Set Obstacle Scale
        walls.scale.x = sx;
        walls.scale.y = sy;
        walls.scale.z = 2.0;

        // Publish the obstacles
        obstacles_pub.publish(walls);

    }

}

void DroneRvizDisplay::TrajectoryPubCallbackAdd(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone, std::string name)
{

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = name;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;


    line_strip.id = idDrone;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.03;

    // Line strip is green
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    int size=trajectory.droneTrajectory.size();

    if (trajectory.droneTrajectory.size()>0)
    {
        for (int i=0; i < size; i++)
        {

            geometry_msgs::Point p;
            p.x = trajectory.droneTrajectory[i].x;
            p.y = trajectory.droneTrajectory[i].y;
            p.z = trajectory.droneTrajectory[i].z;

            line_strip.points.push_back(p);

        }

    }
    else if (trajectory.droneTrajectory.size()==0)
    {

        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;

        line_strip.points.push_back(p);

    }
    //ros::NodeHandle n;

    trajectory_pub = n.advertise<visualization_msgs::Marker>("visualization_trajectory", 10);
    trajectory_pub.publish(line_strip);

}

void DroneRvizDisplay::TrajectoryPubCallbackDelete(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone, std::string name)
{

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = name;
    line_strip.action = visualization_msgs::Marker::DELETE;
    line_strip.pose.orientation.w = 1.0;


    line_strip.id = idDrone;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.03;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    int size=trajectory.droneTrajectory.size();

    if (trajectory.droneTrajectory.size()>0)
    {
        for (int i=0; i < size; i++)
        {

            geometry_msgs::Point p;
            p.x = trajectory.droneTrajectory[i].x;
            p.y = trajectory.droneTrajectory[i].y;
            p.z = trajectory.droneTrajectory[i].z;

            line_strip.points.push_back(p);

        }

    }
    else if (trajectory.droneTrajectory.size()==0)
    {

        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;

        line_strip.points.push_back(p);

    }
    //ros::NodeHandle n;

    trajectory_pub = n.advertise<visualization_msgs::Marker>("visualization_trajectory", 10);
    trajectory_pub.publish(line_strip);


}

void DroneRvizDisplay::MissionPointPubCallbackAdd(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone, std::string name)
{


    point.header.frame_id = "/base_link";
    point.header.stamp = ros::Time();
    point.ns = name;
    point.id = idDrone;
    point.type = visualization_msgs::Marker::SPHERE;
    point.action = visualization_msgs::Marker::ADD;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    if (missionPoint.droneTrajectory.size()>0)
    {
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;

        point.pose.position.x = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].x;
        point.pose.position.y = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].y;
        point.pose.position.z = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].z;

    }
    else if (missionPoint.droneTrajectory.size()==0)
    {
        point.scale.x = 0.0;
        point.scale.y = 0.0;
        point.scale.z = 0.0;

        point.pose.position.x = 0.0;
        point.pose.position.y = 0.0;
        point.pose.position.z = 0.0;

    }

    point.color.a = 1.0;
    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;

    mission_pub = n.advertise<visualization_msgs::Marker>( "visualization_missionPoint", 100);
    mission_pub.publish( point );

}

void DroneRvizDisplay::MissionPointPubCallbackDelete(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone, std::string name)
{

    visualization_msgs::Marker point;
    point.header.frame_id = "/base_link";
    point.header.stamp = ros::Time();
    point.ns = name;
    point.id = idDrone;
    point.type = visualization_msgs::Marker::SPHERE;
    point.action = visualization_msgs::Marker::DELETE;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    if (missionPoint.droneTrajectory.size()>0)
    {
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;

        point.pose.position.x = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].x;
        point.pose.position.y = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].y;
        point.pose.position.z = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].z;

    }
    else if (missionPoint.droneTrajectory.size()==0)
    {
        point.scale.x = 0.0;
        point.scale.y = 0.0;
        point.scale.z = 0.0;

        point.pose.position.x = 0.0;
        point.pose.position.y = 0.0;
        point.pose.position.z = 0.0;

    }

    point.color.a = 1.0;
    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;

    mission_pub = n.advertise<visualization_msgs::Marker>( "visualization_missionPoint", 100);
    mission_pub.publish( point );

}




