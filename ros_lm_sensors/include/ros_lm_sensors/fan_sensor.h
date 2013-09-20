#ifndef FAN_SENSOR_H_
#define FAN_SENSOR_H_

class FanSensor;

#include <stdlib.h>
#include <sensors/sensors.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "ros_lm_sensors/sensor_chip.h"


class FanSensor : public SensorChipFeature{
 public:
  FanSensor(SensorChip& chip, sensors_feature const *feature):SensorChipFeature(chip, feature){}
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat){
    SensorChipSubFeaturePtr speed = getSubFeatureByType(SENSORS_SUBFEATURE_FAN_INPUT);
    if(speed){
      double speed_val = speed->getValue();
      if(speed_val==0)
	stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Fan stalled or not connected");
      else
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "FAN OK (%f RPM)", speed_val);
      stat.add("Fan Speed", speed_val);
    }
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NO FAN Input!!!");
  };
};

#endif
