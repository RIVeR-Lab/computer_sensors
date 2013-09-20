#ifndef COMPUTER_SENSOR_H_
#define COMPUTER_SENSOR_H

#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

class ComputerSensor;

typedef boost::shared_ptr<ComputerSensor> ComputerSensorPtr;

class ComputerSensor {
 public:
  virtual std::string getSensorName() = 0;
  virtual std::string getSensorLabel() = 0;
  virtual void publishValues(std::string frame_base, ros::Publisher temperaturePublisher, diagnostic_updater::DiagnosticStatusWrapper &stat) = 0;
};

#endif
