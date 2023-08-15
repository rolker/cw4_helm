// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "asv_msgs/HeadingHold.h"
#include "asv_msgs/HeadingStamped.h"
#include "asv_msgs/BasicPositionStamped.h"
#include "asv_msgs/VehicleStatus.h"
#include "asv_msgs/VirtualDrive.h"
#include "asv_srvs/VehicleState.h"
#include "asv_srvs/PilotControl.h"
#include "asv_msgs/AISContact.h"
#include "c_worker_4_msgs/EngineFeedback.h"

#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project11_msgs/Heartbeat.h"
#include "project11_msgs/Contact.h"
#include "project11_msgs/Helm.h"
#include "project11/pid.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_msgs/DiagnosticArray.h"


ros::Publisher asv_helm_pub;
ros::Publisher asv_inhibit_pub;
ros::Publisher asv_virtual_drive_pub;

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;

ros::Publisher heartbeat_pub;
ros::Publisher contact_pub;

ros::Publisher diagnostic_pub;

double heading;
double rudder;
double throttle;
ros::Time last_helm_time;

double last_boat_heading;

bool standby;

nav_msgs::Odometry latest_odometry;
project11::PID pid;
double max_speed = 2.75;
double max_yaw_speed = 0.5;

void helmCallback(const project11_msgs::Helm::ConstPtr& msg)
{
    throttle = msg->throttle;
    rudder = msg->rudder;
    
    last_helm_time = msg->header.stamp;
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    throttle = msg->twist.linear.x/max_speed;
    if(msg->header.stamp - latest_odometry.header.stamp < ros::Duration(1.0))
    {
        pid.setPoint(msg->twist.linear.x);
        throttle = pid.update(latest_odometry.twist.twist.linear.x, latest_odometry.header.stamp);
    }
    else
        ROS_WARN_STREAM_THROTTLE(2.0,"No recent odometry for use with throttle PID");
    throttle = std::max(-1.0, std::min(1.0, throttle));
    rudder = -msg->twist.angular.z/max_yaw_speed;
    rudder = std::max(-1.0, std::min(1.0, rudder));
    last_helm_time = msg->header.stamp;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  latest_odometry = *msg;
}

void positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& inmsg)
{
  sensor_msgs::NavSatFix nsf;
  nsf.header = inmsg->header;
  nsf.latitude = inmsg->basic_position.position.latitude;
  nsf.longitude = inmsg->basic_position.position.longitude;
  nsf.altitude = std::numeric_limits<double>::quiet_NaN();
  
  position_pub.publish(nsf);
    
  geometry_msgs::TwistStamped ts;
  ts.header = inmsg->header;
  double yaw = M_PI_2-inmsg->basic_position.cog;
  
  ts.twist.linear.x = inmsg->basic_position.sog*cos(yaw);
  velocity_pub.publish(ts);
}

void headingCallback(const asv_msgs::HeadingStamped::ConstPtr& msg)
{
  last_boat_heading = msg->heading.heading;
  sensor_msgs::Imu imu;
  imu.header = msg->header;
  if(imu.header.stamp.sec == 0)
    imu.header.stamp = ros::Time::now();
  tf2::Quaternion q;
  q.setRPY(0,0,M_PI_2-last_boat_heading);
  tf2::convert(q, imu.orientation);
  imu.angular_velocity_covariance[0] = -1;
  imu.linear_acceleration_covariance[0] = -1;
  orientation_pub.publish(imu);
}

void sendVirtualDrive(const ros::TimerEvent event)
{
    asv_msgs::VirtualDrive driveMessage;
    if (!last_helm_time.isZero()&&event.last_real-last_helm_time>ros::Duration(.5))
    {
        throttle = 0.0;
        rudder = 0.0;
    }

    driveMessage.steerage = rudder*100.0;
    driveMessage.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
    driveMessage.thrust.value = throttle*100.0;
    driveMessage.header.stamp = event.current_real;

    asv_virtual_drive_pub.publish(driveMessage);

}

void sendHeadingHold(const ros::TimerEvent event)
{
    asv_msgs::HeadingHold asvMsg;
    if (!last_helm_time.isZero()&&event.last_real-last_helm_time>ros::Duration(.5))
    {
        throttle = 0.0;
        rudder = 0.0;
    }

    ros::Duration delta_t = event.current_real-event.last_real;
    heading = last_boat_heading + rudder; //*delta_t.toSec();
    heading = fmod(heading,M_PI*2.0);
    if(heading < 0.0)
        heading += M_PI*2.0;
    
    asvMsg.heading.heading = heading;
    asvMsg.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
    asvMsg.thrust.value = throttle*100.0;
    asvMsg.header.stamp = event.current_real;
    asv_helm_pub.publish(asvMsg);
}

void standbyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if(standby && !msg->data)
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
  standby = msg->data;
  if (msg->data)
  {
      asv_srvs::VehicleState vs;
      vs.request.desired_state = asv_srvs::VehicleStateRequest::VP_STATE_PAUSE;
      ros::service::call("/control/vehicle/state",vs);
      //asv_srvs::PilotControl pc;
      //pc.request.control_request = false;
      //ros::service::call("/control/vehicle/pilot",pc);

      std_msgs::Bool inhibit;
      inhibit.data = true;
      asv_inhibit_pub.publish(inhibit);
  }
        
}

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}

void aisContactCallback(const asv_msgs::AISContact::ConstPtr& inmsg)
{
    project11_msgs::Contact c;
    c.contact_source = project11_msgs::Contact::CONTACT_SOURCE_AIS;
    
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
    project11_msgs::Heartbeat hb;
    
    // hb.header.stamp =  inmsg->header.stamp;
    // timestamp from message seems unreliable
    hb.header.stamp = ros::Time::now();

    project11_msgs::KeyValue kv;

    kv.key = "project11_standby";
    kv.value = boolToString(standby);
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

void engineStatusCallback(const c_worker_4_msgs::EngineFeedback::ConstPtr &msg)
{
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = msg->header.stamp;

    diagnostic_msgs::DiagnosticStatus dstatus;
    dstatus.level = dstatus.OK;
    dstatus.name = "cw4/engine";
    dstatus.message = "";

    const std::string states[] = {"unknown", "isolated", "ignition_on", "glowing", "cranking", "verify_running", "running", "stopping"};

    diagnostic_msgs::KeyValue kv;
    kv.key = "engine_state";

    if (msg->engine_state >= c_worker_4_msgs::EngineFeedback::ENGINE_STATE_UNKNOWN && msg->engine_state <= c_worker_4_msgs::EngineFeedback::ENGINE_STATE_STOPPING)
    {
      kv.value = states[msg->engine_state];
      if(msg->engine_state == c_worker_4_msgs::EngineFeedback::ENGINE_STATE_UNKNOWN)
      {
        if(dstatus.level < dstatus.WARN)
          dstatus.level = dstatus.WARN;
        dstatus.message += "unknown state ";
      }

    }
    else
    {
        kv.value = "invalid: " + std::to_string(msg->engine_state);
        if(dstatus.level < dstatus.WARN)
          dstatus.level = dstatus.WARN;
        dstatus.message += "invalid state ";
    }

    dstatus.values.push_back(kv);

    const std::string req_states[] = {"isolate", "run"};
    kv.key = "engine_request";

    if (msg->engine_request >= c_worker_4_msgs::EngineFeedback::ENGINE_STATE_REQ_ISOLATE && msg->engine_request <= c_worker_4_msgs::EngineFeedback::ENGINE_STATE_REQ_RUN)
        kv.value = req_states[msg->engine_request];
    else
    {
        kv.value = "invalid: " + std::to_string(msg->engine_request);
        if(dstatus.level < dstatus.WARN)
          dstatus.level = dstatus.WARN;
        dstatus.message += "invalid state ";
    }
    dstatus.values.push_back(kv);

    kv.key = "altenator_current";
    kv.value = std::to_string(msg->alternator_current)+" (amps)";
    dstatus.values.push_back(kv);

    kv.key = "coolant_temperature";
    kv.value = std::to_string(msg->coolant_temperature)+" (Celsius)";
    dstatus.values.push_back(kv);

    kv.key = "oil_pressure";
    kv.value = std::to_string(msg->oil_pressure)+" (Pascals)";
    dstatus.values.push_back(kv);

    kv.key = "engine_speed";
    kv.value = std::to_string(msg->engine_speed)+" (rpm)";
    dstatus.values.push_back(kv);

    kv.key = "hours";
    kv.value = std::to_string(msg->hours);
    dstatus.values.push_back(kv);

    kv.key = "starter_voltage";
    kv.value = std::to_string(msg->starter_voltage)+" (Volts)";
    dstatus.values.push_back(kv);

    kv.key = "primary_fuel_level";
    kv.value = std::to_string(msg->primary_fuel_level)+"%";
    dstatus.values.push_back(kv);

    kv.key = "secondary_fuel_level";
    kv.value = std::to_string(msg->secondary_fuel_level)+"%";
    dstatus.values.push_back(kv);

    kv.key = "exhaust_temperature";
    kv.value = std::to_string(msg->exhaust_temperature)+" (Celsius)";
    dstatus.values.push_back(kv);

    diag_array.status.push_back(dstatus);

    diagnostic_pub.publish(diag_array);
}

int main(int argc, char **argv)
{
    heading = 0.0;
    throttle = 0.0;
    rudder = 0.0;
    last_boat_heading = 0.0;
    standby = true;
    
    ros::init(argc, argv, "cw4_helm");
    ros::NodeHandle n;
    
    pid.configure(ros::NodeHandle("~/pid"));

    asv_helm_pub = n.advertise<asv_msgs::HeadingHold>("/control/drive/heading_hold",1);
    asv_inhibit_pub = n.advertise<std_msgs::Bool>("/control/drive/inhibit",1,true);
    asv_virtual_drive_pub = n.advertise<asv_msgs::VirtualDrive>("/control/drive/virtual",1);
    
    orientation_pub = n.advertise<sensor_msgs::Imu>("sensors/oem/orientation",1);
    position_pub = n.advertise<sensor_msgs::NavSatFix>("sensors/oem/position",1);
    velocity_pub = n.advertise<geometry_msgs::TwistStamped>("sensors/oem/velocity",1);
    heartbeat_pub = n.advertise<project11_msgs::Heartbeat>("project11/status/helm", 10);
    contact_pub = n.advertise<project11_msgs::Contact>("sensors/ais/contact",10);
    diagnostic_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",10);

    ros::Subscriber asv_position_sub = n.subscribe("/sensor/vehicle/position",10,positionCallback);
    ros::Subscriber asv_heading_sub = n.subscribe("/sensor/vehicle/heading",5,headingCallback);
    ros::Subscriber stanby_sub = n.subscribe("project11/piloting_mode/standby/active", 5, standbyCallback);

    ros::Subscriber vehicle_state_sub =  n.subscribe("/vehicle_status",10,vehicleSatusCallback);
    ros::Subscriber ais_contact_sub = n.subscribe("/sensor/ais/contact",10,aisContactCallback);
    ros::Subscriber helm_sub = n.subscribe("project11/control/helm",10,helmCallback);
    ros::Subscriber twist_sub = n.subscribe("project11/control/cmd_vel",10,twistCallback);

    ros::Subscriber odom_sub = n.subscribe("project11/odom", 5, odometryCallback);

    ros::Subscriber engine_status_sub = n.subscribe("/sensor/vehicle/engine",10,engineStatusCallback);

    //ros::Timer timer = n.createTimer(ros::Duration(0.1),sendHeadingHold);
    ros::Timer timer = n.createTimer(ros::Duration(0.1),sendVirtualDrive);

    ros::spin();
    return 0;
}
