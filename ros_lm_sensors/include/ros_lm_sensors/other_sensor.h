#ifndef OTHER_SENSOR_H_
#define OTHER_SENSOR_H_

class OtherSensor;

#include <stdlib.h>
#include <sensors/sensors.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "ros_lm_sensors/sensor_chip.h"

class OtherSensor : public SensorChipFeature{
 public:
  OtherSensor(SensorChip& chip, sensors_feature const *feature):SensorChipFeature(chip, feature){}
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat){
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Unkown sensor type");
    BOOST_FOREACH(SensorChipSubFeaturePtr subfeature, sub_features_){
      stat.add(subfeature->getName(), subfeature->getValue());
    }
  };
};


#endif
