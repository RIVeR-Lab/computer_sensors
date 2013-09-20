#ifndef TEMP_SENSOR_H_
#define TEMP_SENSOR_H_

class TempSensor;

#include <stdlib.h>
#include <sensors/sensors.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/Temperature.h>
#include "ros_lm_sensors/sensor_chip.h"

class TempSensor : public SensorChipFeature{
 public:
  TempSensor(SensorChip& chip, sensors_feature const *feature):SensorChipFeature(chip, feature){}
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat){
    SensorChipSubFeaturePtr temp = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_INPUT);
    SensorChipSubFeaturePtr max_temp = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_MAX);
    SensorChipSubFeaturePtr temp_crit = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT);
    SensorChipSubFeaturePtr temp_crit_alarm = getSubFeatureByType(SENSORS_SUBFEATURE_TEMP_CRIT_ALARM);
    if(temp){
      double temp_val = temp->getValue();
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "TEMP OK (%f C)", temp_val);
      stat.add("Temperature", temp_val);

      sensor_msgs::Temperature temp_msg;
      temp_msg.variance = 0;
      temp_msg.temperature = temp_val;
      temp_msg.header.frame_id = frame_base+"/"+getSensorName();
      temp_msg.header.stamp = ros::Time::now();
      temperaturePublisher.publish(temp_msg);

      if(max_temp && max_temp->getValue()!=0)
	stat.add("Max Temperature", max_temp->getValue());
      if(temp_crit && temp_crit->getValue()!=0)
	stat.add("Temperature Critical", temp_crit->getValue());
      if(temp_crit_alarm && temp_crit_alarm->getValue()!=0)
	stat.add("Temperature Critical Alarm", temp_crit_alarm->getValue());
    }
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NO TEMP Input!!!");
  };

};


#endif
