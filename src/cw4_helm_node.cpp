// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "asv_msgs/HeadingHold.h"
#include "asv_msgs/HeadingStamped.h"
#include "asv_msgs/BasicPositionStamped.h"
#include "asv_msgs//VehicleStatus.h"
#include "asv_srvs/VehicleState.h"
#include "asv_srvs/PilotControl.h"
#include "asv_msgs/AISContact.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "marine_msgs/NavEulerStamped.h"
#include "marine_msgs/Heartbeat.h"
#include "marine_msgs/Contact.h"
#include "marine_msgs/Helm.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "std_msgs/Int32.h"

ros::Publisher asv_helm_pub;
ros::Publisher asv_inhibit_pub;
ros::Publisher position_pub;
ros::Publisher heading_pub;
ros::Publisher speed_pub;
ros::Publisher speed_modulation_pub;
ros::Publisher heartbeat_pub;
ros::Publisher contact_pub;


double heading;
double rudder;
double throttle;
ros::Time last_time;
bool joystick_override = false;

double last_boat_heading;

double desired_speed;
ros::Time desired_speed_time;
double desired_heading;
ros::Time desired_heading_time;

float obstacle_distance;
float speed_modulation;

std::string piloting_mode;
int current_line = -1;


void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    throttle = msg->twist.linear.x/2.75;
    rudder = -msg->twist.angular.z;
    
    last_time = msg->header.stamp;
}

void helmCallback(const marine_msgs::Helm::ConstPtr& msg)
{
    throttle = msg->throttle;
    rudder = msg->rudder;
    
    last_time = msg->header.stamp;
}


void positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& inmsg)
{
    geographic_msgs::GeoPointStamped gps;
    gps.header = inmsg->header;
    gps.position.latitude = inmsg->basic_position.position.latitude;
    gps.position.longitude = inmsg->basic_position.position.longitude;
    position_pub.publish(gps);
    
    geometry_msgs::TwistStamped ts;
    ts.header = inmsg->header;
    ts.twist.linear.x = inmsg->basic_position.sog;
    speed_pub.publish(ts);
}

void headingCallback(const asv_msgs::HeadingStamped::ConstPtr& msg)
{
    last_boat_heading = msg->heading.heading;
    marine_msgs::NavEulerStamped nes;
    nes.header = msg->header;
    nes.orientation.heading = msg->heading.heading*180.0/M_PI;
    heading_pub.publish(nes);
}

void obstacleDistanceCallback(const std_msgs::Float32::ConstPtr& inmsg)
{
    float stop_distance = 25.0;
    float start_slowing_down_distance = 50.0;

    obstacle_distance = inmsg->data;
    if(obstacle_distance < 0 || obstacle_distance > start_slowing_down_distance)
        speed_modulation = 1.0;
    else if (obstacle_distance < stop_distance)
        speed_modulation = 0.0;
    else
        speed_modulation = (obstacle_distance-stop_distance)/(start_slowing_down_distance-stop_distance);
    std_msgs::Float32 sm;
    sm.data = speed_modulation;
    speed_modulation_pub.publish(sm);
}

void desiredSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
{
    desired_speed = inmsg->twist.linear.x;
    desired_speed_time = inmsg->header.stamp;
}

void desiredHeadingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    desired_heading = inmsg->orientation.heading;
    desired_heading_time = inmsg->header.stamp;
}

void sendHeadingHold(const ros::TimerEvent event)
{
    asv_msgs::HeadingHold asvMsg;
    bool doDesired = true;
    if (!last_time.isZero())
    {
        if(event.last_real-last_time>ros::Duration(.5))
        {
            throttle = 0.0;
            rudder = 0.0;
        }
        else
            doDesired = false;
    }

    ros::Duration delta_t = event.current_real-event.last_real;
    heading = last_boat_heading + rudder; //*delta_t.toSec();
    heading = fmod(heading,M_PI*2.0);
    if(heading < 0.0)
        heading += M_PI*2.0;
    
    asvMsg.heading.heading = heading;
    if(doDesired)
    {
        joystick_override = false;
        asvMsg.thrust.type = asv_msgs::Thrust::THRUST_SPEED;
    }
    else
    {
        joystick_override = true;
        asvMsg.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
    }
    //asvMsg.thrust.type = asv_msgs::Thrust::THRUST_SPEED;
    asvMsg.thrust.value = throttle*100.0;
    asvMsg.header.stamp = event.current_real;
    if(doDesired)
    {
        if (event.current_real - desired_heading_time < ros::Duration(.5) && event.current_real - desired_speed_time < ros::Duration(.5))
        {
            asvMsg.header.stamp = desired_heading_time;
            asvMsg.heading.heading = desired_heading*M_PI/180.0;
            asvMsg.thrust.value = desired_speed*speed_modulation;
        }
        //else
          //  std::cerr << "asv_helm: desired values timeout: " << event.current_real << ", " << desired_heading_time << ", " << desired_speed_time << std::endl;
    }
    asv_helm_pub.publish(asvMsg);
}

void helmModeCallback(const std_msgs::String::ConstPtr& inmsg)
{
    if (piloting_mode == "standby" && inmsg->data != "standby")
    {
        asv_srvs::VehicleState vs;
        vs.request.desired_state = asv_srvs::VehicleStateRequest::VP_STATE_ACTIVE;
        ros::service::call("/control/vehicle/state",vs);
        
        asv_srvs::PilotControl pc;
        pc.request.control_request = true;
        ros::service::call("/control/vehicle/pilot",pc);
        
        std_msgs::Bool inhibit;
        inhibit.data = false;
        asv_inhibit_pub.publish(inhibit);
    }
    if (inmsg->data == "standby")
    {
        //asv_srvs::PilotControl pc;
        //pc.request.control_request = false;
        //ros::service::call("/control/vehicle/pilot",pc);

        std_msgs::Bool inhibit;
        inhibit.data = true;
        asv_inhibit_pub.publish(inhibit);
    }
        
    piloting_mode = inmsg->data;
}

void currentLineCallback(const std_msgs::Int32::ConstPtr& inmsg)
{
    current_line = inmsg->data;
}

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}

void aisContactCallback(const asv_msgs::AISContact::ConstPtr& inmsg)
{
    marine_msgs::Contact c;
    c.contact_source = marine_msgs::Contact::CONTACT_SOURCE_AIS;
    
    c.header = inmsg->header;
    c.mmsi = inmsg->mmsi;
    c.name = inmsg->name;
    c.callsign = inmsg->callsign;
    
    c.position.latitude = inmsg->position.latitude;
    c.position.longitude = inmsg->position.longitude;
    
    c.cog = inmsg->cog;
    c.sog = inmsg->sog;
    c.heading = inmsg->heading;
    
    c.dimension_to_stbd = inmsg->dimension_to_stbd;
    c.dimension_to_port = inmsg->dimension_to_port;
    c.dimension_to_bow = inmsg->dimension_to_bow;
    c.dimension_to_stern = inmsg->dimension_to_stern;

    contact_pub.publish(c);
}

void vehicleSatusCallback(const asv_msgs::VehicleStatus::ConstPtr& inmsg)
{
    marine_msgs::Heartbeat hb;
    hb.header.stamp = ros::Time::now();

    marine_msgs::KeyValue kv;

    kv.key = "piloting_mode";
    kv.value = piloting_mode;
    hb.values.push_back(kv);

    kv.key = "js_override";
    kv.value = boolToString(joystick_override);
    hb.values.push_back(kv);
    
    kv.key = "current_line";
    std::stringstream ss;
    ss << current_line;
    kv.value = ss.str();
    hb.values.push_back(kv);

    kv.key = "state";
    switch(inmsg->vehicle_state)
    {
        case asv_msgs::VehicleStatus::VP_STATE_RESET:
            kv.value = "reset";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_INITIAL:
            kv.value = "initial";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_CONFIG:
            kv.value = "config";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_ARMED:
            kv.value = "armed";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_PAUSE:
            kv.value = "pause";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_ACTIVE:
            kv.value = "active";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_RECOVER:
            kv.value = "recover";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_MANNED:
            kv.value = "manned";
            break;
        case asv_msgs::VehicleStatus::VP_STATE_EMERGENCY:
            kv.value = "emergency";
            break;
        default:
            kv.value = "unknown";
    }
    hb.values.push_back(kv);
    
    
    kv.key = "state_reason";
    kv.value = inmsg->vehicle_state_reason;
    hb.values.push_back(kv);
    
    kv.key = "pilot_control";
    kv.value =inmsg->pilot_control;
    hb.values.push_back(kv);
    
    kv.key = "ros_pilot_mode";
    switch(inmsg->ros_pilot_mode)
    {
        case asv_msgs::VehicleStatus::PILOT_NOT_IN_COMMAND:
            kv.value = "not in command";
            break;
        case asv_msgs::VehicleStatus::PILOT_INACTIVE:
            kv.value = "inactive";
            break;
        case asv_msgs::VehicleStatus::PILOT_INHIBITED:
            kv.value = "inhibited";
            break;
        case asv_msgs::VehicleStatus::PILOT_DIRECT_DRIVE:
            kv.value = "direct drive";
            break;
        case asv_msgs::VehicleStatus::PILOT_HEADING_HOLD:
            kv.value = "heading hold";
            break;
        case asv_msgs::VehicleStatus::PILOT_SEEK_POSITION:
            kv.value = "seek position";
            break;
        case asv_msgs::VehicleStatus::PILOT_TRACK_FOLLOW:
            kv.value = "track follow";
            break;
        case asv_msgs::VehicleStatus::PILOT_VIRTUAL_DRIVE:
            kv.value = "virtual drive";
            break;
        default:
            kv.value = "unknown";
    }
    hb.values.push_back(kv);
    
    heartbeat_pub.publish(hb);
}



int main(int argc, char **argv)
{
    heading = 0.0;
    throttle = 0.0;
    rudder = 0.0;
    last_boat_heading = 0.0;
    obstacle_distance = -1.0;
    speed_modulation = 1.0;
    piloting_mode = "standby";
    
    ros::init(argc, argv, "cw4_helm");
    ros::NodeHandle n;
    
    asv_helm_pub = n.advertise<asv_msgs::HeadingHold>("/control/drive/heading_hold",1);
    asv_inhibit_pub = n.advertise<std_msgs::Bool>("/control/drive/inhibit",1,true);
    heading_pub = n.advertise<marine_msgs::NavEulerStamped>("/heading",1);
    position_pub = n.advertise<geographic_msgs::GeoPointStamped>("/position",1);
    speed_pub = n.advertise<geometry_msgs::TwistStamped>("/sog",1);
    speed_modulation_pub = n.advertise<std_msgs::Float32>("/speed_modulation",1);
    heartbeat_pub = n.advertise<marine_msgs::Heartbeat>("/heartbeat", 10);
    contact_pub = n.advertise<marine_msgs::Contact>("/contact",10);

    ros::Subscriber asv_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    ros::Subscriber asv_position_sub = n.subscribe("/sensor/vehicle/position",10,positionCallback);
    ros::Subscriber asv_heading_sub = n.subscribe("/sensor/vehicle/heading",5,headingCallback);
    ros::Subscriber piloting_mode_sub = n.subscribe("/project11/piloting_mode",10,helmModeCallback);
    ros::Subscriber dspeed_sub = n.subscribe("/project11/desired_speed",10,desiredSpeedCallback);
    ros::Subscriber dheading_sub = n.subscribe("/project11/desired_heading",10,desiredHeadingCallback);
    ros::Subscriber obstacle_distance_sub =  n.subscribe("/obstacle_distance",10,obstacleDistanceCallback);
    ros::Subscriber vehicle_state_sub =  n.subscribe("/vehicle_status",10,vehicleSatusCallback);
    ros::Subscriber current_line_sub = n.subscribe("/project11/mission_manager/current_line",10,currentLineCallback);
    ros::Subscriber ais_contact_sub = n.subscribe("/sensor/ais/contact",10,aisContactCallback);
    ros::Subscriber helm_sub = n.subscribe("/helm",10,helmCallback);

    ros::Timer timer = n.createTimer(ros::Duration(0.1),sendHeadingHold);
    
    ros::spin();
    return 0;
}
