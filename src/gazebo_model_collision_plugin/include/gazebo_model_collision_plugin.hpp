// Copyright (c) 2019 Steven Macenski

#ifndef _GAZEBO_COLLISION_PLUGIN_H_
#define _GAZEBO_COLLISION_PLUGIN_H_

#include <string>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_model_collision_plugin/Contact.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo
{

class CollisionPlugin : public SensorPlugin
{

public:
  CollisionPlugin();

  virtual ~CollisionPlugin();

  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  virtual void OnUpdate();
  sensors::ContactSensorPtr parentSensor;
  event::ConnectionPtr updateConnection;
  GazeboRosPtr gazeboRos;
  ros::Publisher contactPub;
};

}

#endif
