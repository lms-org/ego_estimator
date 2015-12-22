#ifndef EGO_ESTIMATOR_H
#define EGO_ESTIMATOR_H

#include <lms/module.h>
#include <sensor_utils/sensor.h>

/**
 * @brief LMS module ego_estimator
 **/
class EgoEstimator : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;

protected:
    lms::ReadDataChannel<sensor_utils::SensorContainer> sensors;
};

#endif // EGO_ESTIMATOR_H
