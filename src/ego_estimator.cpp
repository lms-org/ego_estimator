#include "ego_estimator.h"

#include <sensor_utils/imu.h>
#include <sensor_utils/odometer.h>

bool EgoEstimator::initialize() {
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    return true;
}

bool EgoEstimator::deinitialize() {
    return true;
}

bool EgoEstimator::cycle() {
    if(firstRun)
    {
        lastTimestamp = lms::Time::ZERO;
        firstRun = false;

        // Init kalman
        State<T> s;
        s.setZero();
        filter.init(s);

        // TODO: set proper covariance
        Kalman::Covariance<State<T>> cov;
        cov.setIdentity();
        filter.setCovariance(cov);

        return true;
    }

    currentTimestamp = lms::Time::ZERO;

    // Compute Measurement Update
    computeMeasurement();

    // Compute kalman filter step
    computeFilterStep();

    // Update timestamp
    lastTimestamp = currentTimestamp;

    // Add current state estimate with timestamp to interpolation values
    // TODO

    return true;
}

void EgoEstimator::computeMeasurement()
{
    T v = 0;
    T ax = 0;
    T ay = 0;
    T omega = 0;
    size_t numVelocitySources = 0;

    if(!sensors->hasSensor("IMU")) {
        logger.error("imu") << "MISSING IMU SENSOR!";

        // FALLBACK TO STEERING ANGLES FOR TURN RATE
        // TODO!!!
    } else {
        auto imu = sensors->sensor<sensor_utils::IMU>("IMU");
        currentTimestamp = imu->timestamp();

        // TODO: Set Covariances
        omega = imu->gyroscope.z();
        ax    = imu->accelerometer.x();
        ay    = imu->accelerometer.y();
    }

    if(!sensors->hasSensor("HALL")) {
        logger.error("hall") << "MISSING HALL SENSOR!";
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
        currentTimestamp = hall->timestamp();

        // TODO: Set Covariances
        v += hall->velocity.x();
        numVelocitySources++;
    }

    /*
    // TODO: enable mouse sensors -> check if quality is high enough to be considered valid
    if(!sensors->hasSensor("MOUSE_FRONT")) {
        //logger.error("mouse_front") << "MISSING MOUSE_FRONT SENSOR!";
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("MOUSE_FRONT");

        // TODO: Set Covariances
        v += hall->velocity.x();
        numVelocitySources++;
    }

    if(!sensors->hasSensor("MOUSE_REAR")) {
        //logger.error("mouse_front") << "MISSING MOUSE_FRONT SENSOR!";
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("MOUSE_REAR");

        // TODO: Set Covariances
        v += hall->velocity.x();
        numVelocitySources++;
    }
     */

    if( numVelocitySources > 0 ) {
        v /= numVelocitySources;
    } else {
        // Set covariacne to very high
        // TODO
    }

    // Set actual measurement vector
    z.v() = v;
    z.ax() = ax;
    z.ay() = ay;
    z.omega() = omega;
}

void EgoEstimator::computeFilterStep()
{
    // time since UKF was last called (parameter, masked as control input)
    auto delta = ( currentTimestamp - lastTimestamp );
    u.dt() = delta.toFloat();

    // predict state for current time-step using the kalman filter
    filter.predict(sys, u);
    // perform measurement update
    filter.update(mm, z);
}
