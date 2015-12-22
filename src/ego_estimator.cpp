#include "ego_estimator.h"

bool EgoEstimator::initialize() {
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    return true;
}

bool EgoEstimator::deinitialize() {
    return true;
}

bool EgoEstimator::cycle() {
    return true;
}
