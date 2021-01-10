// Copyright (c) 2019 Steven Macenski

#include "../include/gazebo_model_collision_plugin.hpp"
#include <iostream>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CollisionPlugin)

/*****************************************************************************/
CollisionPlugin::CollisionPlugin() : SensorPlugin()
/*****************************************************************************/
{
}

/*****************************************************************************/
CollisionPlugin::~CollisionPlugin()
/*****************************************************************************/
{
}

/*****************************************************************************/
void CollisionPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
/*****************************************************************************/
{
  ROS_INFO("CollisionPlugin: Starting contact sensor.");
  std::string topic;
  double rate;
  gazeboRos = GazeboRosPtr(new GazeboRos(_sensor, _sdf, "ContactSensor"));
  gazeboRos->isInitialized();
  gazeboRos->getParameter<std::string> (topic, "topicName", "/gazebo/base_collision" );
  gazeboRos->getParameter<double> (rate, "updateRate", 10.0 );
  contactPub = gazeboRos->node()->advertise<gazebo_model_collision_plugin::Contact>(topic, 5);

  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "CollisionPlugin requires a ContactSensor.\n";
    return;
  }

  this->parentSensor->SetUpdateRate(rate);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&CollisionPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/*****************************************************************************/
void CollisionPlugin::OnUpdate()
/*****************************************************************************/
{
  gazebo_model_collision_plugin::Contact msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = this->parentSensor->ParentName();
  msg.header.frame_id = msg.header.frame_id.substr(msg.header.frame_id.find("::") + 2);
  std::vector<std::string> objs_hit;

  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::string obj_name = contacts.contact(i).collision2();
    while (true)
    {
      
      if (obj_name.find(std::string("::")) != std::string::npos)
      {
        obj_name = obj_name.substr(0, obj_name.find("::"));
      }
      else
      {
        break;
      }
    }
    
    if (obj_name == "ground" || obj_name == "sun")
    {
      continue;
    }

    if (std::find(objs_hit.begin(), objs_hit.end(), obj_name) == objs_hit.end())
    {
      objs_hit.push_back(obj_name);
    }
  }

  if (objs_hit.size() > 0 || contactPub.getNumSubscribers() < 1)
  {
    msg.objects_hit = objs_hit;
    contactPub.publish(msg);    
  }

  return;
}
