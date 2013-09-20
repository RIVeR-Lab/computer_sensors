#include <stdlib.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <ros_lm_sensors/sensor_chip.h>
#include <ros_lm_sensors/fan_sensor.h>
#include <ros_lm_sensors/temp_sensor.h>
#include <ros_lm_sensors/other_sensor.h>

#define NAME_BUFFER_SIZE 50

SensorChip::SensorChip(sensors_chip_name const *chip_name):internal_name_(chip_name){
  char name_buffer[NAME_BUFFER_SIZE];
  sensors_snprintf_chip_name(name_buffer, NAME_BUFFER_SIZE, internal_name_);
  name_ = name_buffer;

  ROS_INFO("Found Sensor: %s", getName().c_str());
  enumerate_features();
}
void SensorChip::enumerate_features(){
  features_.clear();

  sensors_feature const *feature;
  int number = 0;

  while ((feature = sensors_get_features(internal_name_, &number)) != NULL) {
    sensors_feature_type type = feature->type;
    switch(type){
    case SENSORS_FEATURE_FAN:
      features_.push_back(SensorChipFeaturePtr(new FanSensor(*this, feature)));
      break;
    case SENSORS_FEATURE_TEMP:
      features_.push_back(SensorChipFeaturePtr(new TempSensor(*this, feature)));
      break;
    default:
      features_.push_back(SensorChipFeaturePtr(new OtherSensor(*this, feature)));
      break;
    }
  }
}



SensorChipFeature::SensorChipFeature(SensorChip& chip, sensors_feature const *feature):chip_(chip), feature_(feature){
  name_ = feature_->name;
  char* label_c_str = sensors_get_label(chip_.internal_name_, feature_);
  label_ = label_c_str;
  free(label_c_str);
  sensor_name_ = getChipName()+"/"+getName();


  ROS_INFO("\tFound Feature: %s(%s)[%d]", getLabel().c_str(), getName().c_str(), feature_->type);

  enumerate_subfeatures();
}
void SensorChipFeature::enumerate_subfeatures(){
  sensors_subfeature const *subfeature;
  int number = 0;

  while ((subfeature = sensors_get_all_subfeatures(chip_.internal_name_, feature_, &number)) != NULL) {
    sub_features_.push_back(SensorChipSubFeaturePtr(new SensorChipSubFeature(*this, subfeature)));
  }
}
SensorChipSubFeaturePtr SensorChipFeature::getSubFeatureByType(sensors_subfeature_type type){
  BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
    if(subfeature->getType()==type)
      return subfeature;
  }
  return SensorChipSubFeaturePtr();
}



SensorChipSubFeature::SensorChipSubFeature(SensorChipFeature& feature, sensors_subfeature const *subfeature):feature_(feature), subfeature_(subfeature){
  name_ = subfeature_->name;

  ROS_INFO("\t\tFound Sub-Feature: %s[%d] = %f", getName().c_str(), subfeature_->type, getValue());
}
std::string SensorChipSubFeature::getName(){
  return name_;
}
double SensorChipSubFeature::getValue(){
  double value;
  sensors_get_value(feature_.chip_.internal_name_, subfeature_->number, &value);
  return value;
}
sensors_subfeature_type SensorChipSubFeature::getType(){
  return subfeature_->type;
}
