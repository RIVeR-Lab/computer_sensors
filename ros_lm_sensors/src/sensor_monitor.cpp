#include <sensors/sensors.h>
#include <string>
#include <sensor_msgs/Temperature.h>
#include <ros_lm_sensors/sensor_chip.h>
#include <ros/ros.h>
#include <vector>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/foreach.hpp>

#define define_and_get_param(type, var_name, param_name, default_value) \
	type var_name(default_value);					\
	get_param(var_name, param_name)

template<class T> static inline bool get_param(T& var, std::string param_name){
  if(!ros::param::get(param_name, var)){
    ROS_WARN_STREAM("Parameter <"<<param_name<<"> not set. Using default value '"<<var<<"'");
    return false;
  }
  return true;
}

std::vector<SensorChipPtr> sensor_chips_;

void enumerate_sensors(){
  sensor_chips_.clear();
  
  sensors_chip_name const *chip_name;
  int number = 0;
  while ((chip_name = sensors_get_detected_chips(NULL, &number)) != NULL){
    sensor_chips_.push_back(SensorChipPtr(new SensorChip(chip_name)));
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_monitor");
  ros::NodeHandle nh;

  define_and_get_param(int, diagnostic_rate, "~diagnostic_period", 1);//updater retrieves this value, do this to display warning if not set
  define_and_get_param(std::string, temperature_topic, "~temperature_topic", "temperature");
  define_and_get_param(std::string, computer_frame, "~computer_frame", "");


  ros::Publisher temperaturePublisher = nh.advertise<sensor_msgs::Temperature>(temperature_topic, 100);

  sensors_cleanup();
  sensors_init(NULL);

  enumerate_sensors();

  diagnostic_updater::Updater updater;
  
  if(computer_frame.empty())
    updater.setHardwareID("none");
  else
    updater.setHardwareID(computer_frame);


  BOOST_FOREACH(SensorChipPtr sensor_chip, sensor_chips_){
    BOOST_FOREACH(ComputerSensorPtr sensor, sensor_chip->features_){
      updater.add(sensor->getSensorName(), boost::bind(&ComputerSensor::publishValues, sensor, computer_frame, temperaturePublisher, _1));
    }
  }


  while(ros::ok()){
    updater.update();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  sensors_cleanup();
}

