#ifndef EGO_ESTIMATOR_H
#define EGO_ESTIMATOR_H

#include <lms/module.h>
#include <sensor_utils/sensor.h>

#include <kalman/UnscentedKalmanFilter.hpp>
#include "filter/measurement_model.h"
#include "filter/system_model.h"

/**
 * @brief LMS module ego_estimator
 **/
class EgoEstimator : public lms::Module {
public:
    typedef float T;
    typedef Kalman::UnscentedKalmanFilter<State <T>> Filter;
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;

protected:
    void computeMeasurement();
    void computeFilterStep();

protected:
    lms::ReadDataChannel<sensor_utils::SensorContainer> sensors;

    Control<T> u;
    Measurement<T> z;

    SystemModel<T> sys; // System model
    MeasurementModel<T> mm; // Measurement model

    Filter filter;

    lms::Time lastTimestamp;
    lms::Time currentTimestamp;
    bool firstRun;
};

#endif // EGO_ESTIMATOR_H
