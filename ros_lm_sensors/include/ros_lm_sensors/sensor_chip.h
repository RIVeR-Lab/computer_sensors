#ifndef SENSOR_CHIP_H_
#define SENSOR_CHIP_H_

#include <stdlib.h>
#include <sensors/sensors.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include "ros_lm_sensors/computer_sensor.h"

class SensorChipSubFeature;
class SensorChipFeature;
class SensorChip;

typedef boost::shared_ptr<SensorChip> SensorChipPtr;
typedef boost::shared_ptr<SensorChipFeature> SensorChipFeaturePtr;
typedef boost::shared_ptr<SensorChipSubFeature> SensorChipSubFeaturePtr;


class SensorChip{
private:
  std::string name_;
  sensors_chip_name const *internal_name_;
  void enumerate_features();
public:
  std::vector<SensorChipFeaturePtr> features_;
  SensorChip(sensors_chip_name const *chip_name);
  std::string getName(){return name_;}

  friend class SensorChipFeature;
  friend class SensorChipSubFeature;
};


class SensorChipFeature : public ComputerSensor{
private:
  std::string name_;
  std::string label_;
  std::string sensor_name_;
  SensorChip& chip_;
  sensors_feature const *feature_;
  void enumerate_subfeatures();
public:
  std::vector<SensorChipSubFeaturePtr> sub_features_;
  SensorChipFeature(SensorChip& chip, sensors_feature const *feature);
  SensorChipSubFeaturePtr getSubFeatureByType(sensors_subfeature_type type);

  std::string getName(){return name_;};
  std::string getLabel(){return label_;};
  virtual std::string getSensorName(){return sensor_name_;};
  virtual std::string getSensorLabel(){return getLabel();};
  sensors_feature_type getType(){return feature_->type;};
  std::string getChipName(){return chip_.getName();};

  friend class SensorChipSubFeature;
};


class SensorChipSubFeature{
private:
  std::string name_;
  SensorChipFeature& feature_;
  sensors_subfeature const *subfeature_;
public:
  SensorChipSubFeature(SensorChipFeature& feature, sensors_subfeature const *subfeature);
  std::string getName();
  sensors_subfeature_type getType();
  double getValue();
};


#endif
