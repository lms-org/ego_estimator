#ifndef EGO_ESTIMATOR_H
#define EGO_ESTIMATOR_H

#include <fstream>

#include <lms/module.h>
#include <sensor_utils/sensor.h>
#include <street_environment/car.h>

#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include "filter/measurement_model.h"
#include "filter/system_model.h"

/**
 * @brief LMS module ego_estimator
 **/
class EgoEstimator : public lms::Module {
public:
    typedef float T;

    typedef CTRA::State<T> State;
    typedef CTRA::Control<T> Control;
    typedef CTRA::Measurement<T> Measurement;

    typedef CTRA::MeasurementModel<T> MeasurementModel;
    typedef CTRA::SystemModel<T> SystemModel;
    typedef Kalman::ExtendedKalmanFilter<State> Filter;
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;

protected:
    void computeTimeStamp();
    void computeMeasurement();
    void computeFilterStep();
    void initFilter();
    void updateCarState();
protected:
    lms::ReadDataChannel<sensor_utils::SensorContainer> sensors;
    lms::WriteDataChannel<street_environment::CarCommand> car;

    sensor_utils::SensorHasUpdate sensorHasUpdate;
    Control u;
    Measurement z;

    SystemModel sys;
    MeasurementModel mm;

    Filter filter;

    lms::Time lastTimestamp;
    lms::Time currentTimestamp;
    lms::Time previousDelta;

    std::ofstream stateLog;
    std::ofstream measurementLog;
    bool stateLogEnabled;
};

#endif // EGO_ESTIMATOR_H
