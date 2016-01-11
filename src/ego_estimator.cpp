#include "ego_estimator.h"

#include <sensor_utils/imu.h>
#include <sensor_utils/odometer.h>

#define GRAVITY T(9.81)

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

        Kalman::Covariance<State<T>> cov;
        cov.setZero();
        cov(State<T>::X, State<T>::X) = config().get<T>("sys_var_x", 1);
        cov(State<T>::Y, State<T>::Y) = config().get<T>("sys_var_y", 1);
        cov(State<T>::A, State<T>::A) = config().get<T>("sys_var_a", 1);
        cov(State<T>::V, State<T>::V) = config().get<T>("sys_var_v", 1);
        cov(State<T>::THETA, State<T>::THETA) = config().get<T>("sys_var_theta", 1);
        cov(State<T>::OMEGA, State<T>::OMEGA) = config().get<T>("sys_var_omega", 1);
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

    T axVar = 0;
    T ayVar = 0;
    T vVar = 0;
    T omegaVar = 0;

    if(!sensors->hasSensor("IMU")) {
        logger.error("imu") << "MISSING IMU SENSOR!";

        // FALLBACK TO STEERING ANGLES FOR TURN RATE
        // TODO!!!
    } else {
        auto imu = sensors->sensor<sensor_utils::IMU>("IMU");
        currentTimestamp = imu->timestamp();

        omega = imu->gyroscope.z();
        ax    = GRAVITY*imu->accelerometer.x();
        ay    = GRAVITY*imu->accelerometer.y();

        omegaVar = imu->gyroscopeCovariance.zz();
        axVar    = GRAVITY*GRAVITY*imu->accelerometerCovariance.xx();
        ayVar    = GRAVITY*GRAVITY*imu->accelerometerCovariance.yy();
    }

    if(!sensors->hasSensor("HALL")) {
        logger.error("hall") << "MISSING HALL SENSOR!";
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
        currentTimestamp = hall->timestamp();

        // TODO: Set Covariances
        v += hall->velocity.x();
        numVelocitySources++;

        vVar = hall->velocityCovariance.xx();
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

    // Set measurement covariances
    Kalman::Covariance< Measurement<T> > cov;
    cov.setZero();
    cov(Measurement<T>::AX,    Measurement<T>::AX)    = axVar;
    cov(Measurement<T>::AY,    Measurement<T>::AY)    = ayVar;
    cov(Measurement<T>::V,     Measurement<T>::V)     = vVar;
    cov(Measurement<T>::OMEGA, Measurement<T>::OMEGA) = omegaVar;
    mm.setCovariance(cov);

    logger.debug("measurementVector") << std::endl << z;
    logger.debug("measurementCovariance") << std::endl << mm.getCovariance();
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

    logger.debug("stateEstimate") << std::endl << filter.getState();
    logger.debug("stateCovariance") << std::endl << filter.getCovariance();
}
