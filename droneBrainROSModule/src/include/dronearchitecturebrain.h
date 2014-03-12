#ifndef DRONEARCHITECTUREBRAIN_H
#define DRONEARCHITECTUREBRAIN_H

// ROS
#include "ros/ros.h"

// C++ standar libraries
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <set>
#include <algorithm>
#include <sstream>

// DroneModule parent class
#include "droneModuleROS.h"

// droneMsgsROS
#include "droneMsgsROS/droneNavData.h"
#include "droneMsgsROS/societyPose.h"
#include "droneMsgsROS/droneInfo.h"

// Topic and Rosservice names
#include "communication_definition.h"
#include "std_msgs/Int32.h"

#include "thisswarmagentinterface.h"
#include "otherswarmagentlistener.h"
#include "brainstatemachine.h"

#define FREQ_ARCHITECTURE_BRAIN  20.0
#define TEST_WITH_AUTONOMOUS_BRAIN

// Things monitored about "this" drone
// - reception of "bool isStarted" from all "this"' drone modules allows to know if they are working (not that we can do anything otherwise...)

// - subscription to "this"' droneNavData allows to now if the WiFi is working ok, also:
//      [*] to make the drone Hover and switch off the controller.
//              Then, switch on the controller and send the command again.
//              We have to determine the localization modules behaviour under this event...
//              Ideally the drone should start hovering automatically upon WiFi disconnection...
//      [*] to inform back the MissionPlanner about drone status during take-off and landing tasks
// - Connection with the mission planner to inform about mission status
// - publish and reception of Integer from "/societyBroadcast" topic, reception of new
// - it allows to start societyMember instances to monitor each new drone
// - when it is not received from a drone for more than 5 seconds this drone is considered to have been switched off

// - Things monitored about the other drones
//      [class SocietyMember or SwarmOtherAgentListener]
//      [*] EstimatedPose (when the "other" drone's WiFi connection, I believe this is no longer received)
//      [*] isStarted from DroneArchitectureBrain (allows to know if the other drone is still working)
//              Note that this capability is already provided by the "/societyBroadcast" integer reception.
//              We have to decide which is better, I prefer to make this on the ""


// [Task] [DONE] Programar añadir esqueleto basico de un droneModule a DroneArchitectureBrain
// [Task] [DONE] Add module_names to communication_definition.h, buscar nombres y cambiarlos dentro del codigo. Comprobar rosmake.
// [Task] [DONE, added in droneMsgsROS/src/include/communication_definition.h] Add module_names enum para uso interno en dronearchitecturebrain... por ahora no requeremos que se vea en todos sitios
// [Task] [DONE] Program DroneModuleInterface, if asked it reports basic state status of the concerned module
//              - service_subscribers to start/reset/stop drone_modules
//              - automatic module monitoring by means of the isStarted topic (a function isStarted() has to be called periodically)
//              - the constructor requires a string with the module_name (¿Maybe an enum too?).
//                  Lo bueno de añadir un enum, es que permite mandar comandos de start/reset/stop sin comparar strings...
// [Task] [DONE] ThisSwarmAgentInterface tiene DroneModuleInterface(s) y los monitoriza
//              - automatic monitoring of all DroneModules, including desired behaviour upon DroneModule fail
//              - lo puedo programar directamente en DroneArchitectureBrain, el problema es que entonces no es nada modular...
//              - los enum que definen los modulos permiten recibir ordenes a traves de una interfaz/function call unica
//              - monitoriza el droneNavData del drone para conocer el estado de la WiFi.
//              - (a function isStartedModule( ModuleNames::name module_name_enum) has to be called to know the state of the components)
//              - (a function isWifiOk() has to be called to know the state of the components)
//              - Añadir take-off... (flying-modes) interface
// [Task] [DONE, already done by Module (DroneModule.h)] Tener droneId, del drone de este laptop
// [Task] [DONE] OtherSwarmAgentListener (o societyMember)
//              - recibe EstimatedPose
//              - monitoriza si el otro drone es desconectado o si está caido a traves de su /droneX/DroneArchitectureBrain/isStarted
//              - (a function isOnline() has to be called periodically)
// [Task] [DONE] Add "/societyBroadcast" topic broadcast and subscription, en el callback se crean los societyMembers
// [Task] [DONE] Publish "/societyPose"
// [Task] [DONE] Add an [ncurses] interface and/or logging capability to this module (to see how the state is being monitored)
// [Task] [DONE] Ir haciendo funcion run() que ejecuta las function call de monitorizacion de las clases anteriores
// [Task] Add satisfactory behaviour regarding WiFi disconnection to SwarmThisAgentInterface
//          (lo de hacer hovering, guardar el mission point, y todo eso).
// [Task] Añadir interfaz con el MissionPlanner (aun por definir por Jose).
//          Hay que añadir un dronemsg mensaje con enum-like constants para evitar añadir funciones de interfaz
// [Task] Si me da tiempo, leer el paper de Ivan para intentar relacionarlo con el punto (a) mandado a Sri sobre ACC14


class DroneArchitectureBrain : public DroneModule {

// DroneModule related functions
// protected:
//    int idDrone; // see Module class
public:
    DroneArchitectureBrain();
    ~DroneArchitectureBrain();
    void init();
    void open(ros::NodeHandle & nIn, std::string moduleName);
    void close();
    bool run();

public:
    ThisSwarmAgentInterface this_drone_interface;

// "/societyBroadcast" and "/societyPose" topics stuff
private:
    ros::Publisher  isInTheSystemPubl; // publishes whether the drone has to be considered as an obstacle by other drones
    ros::Publisher  societyBroadcastPubl; // publishes this->idDrone
    ros::Subscriber societyBroadcastSubs; // subscription, what other drones are there in the swarm?
    void societyBroadcastCallback(const std_msgs::Int32::ConstPtr &msg);
public:
    std::list<OtherSwarmAgentListener> societyMembers;
    std::vector<int> societyIds;
private:
// "societyPose" publishing
    ros::Publisher  societyPosePubl;
    droneMsgsROS::societyPose societyPose_msg;
    int  counter_other_agents;
    void publishSocietyPose();
public: // to allow to printw stuff
    BrainStateMachine brain_state_machine;
};
#endif // DroneArchitectureBrain_H
