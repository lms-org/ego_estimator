#include "ego_estimator.h"

#include <sensor_utils/imu.h>
#include <sensor_utils/odometer.h>

#define GRAVITY T(9.81)

bool EgoEstimator::initialize() {
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");

    // Init Kalman Filter
    initFilter();
    return true;
}

bool EgoEstimator::deinitialize() {
    return true;
}

bool EgoEstimator::cycle() {
    // Compute Measurement Update
    computeMeasurement();

    // Compute Kalman filter step
    computeFilterStep();

    // Update timestamp
    lastTimestamp = currentTimestamp;

    // Add current state estimate with timestamp to interpolation values
    // TODO

    return true;
}

void EgoEstimator::initFilter()
{
    lastTimestamp = lms::Time::ZERO;
    currentTimestamp = lms::Time::ZERO;

    // Init kalman
    State<T> s;
    s.setZero();
    filter.init(s);

    // Set initial state covariance
    Kalman::Covariance<State<T>> stateCov;
    stateCov.setZero();
    stateCov(State::X, State::X)          = config().get<T>("filter_init_var_x", 1);
    stateCov(State::Y, State::Y)          = config().get<T>("filter_init_var_y", 1);
    stateCov(State::A, State::A)          = config().get<T>("filter_init_var_a", 1);
    stateCov(State::V, State::V)          = config().get<T>("filter_init_var_v", 1);
    stateCov(State::THETA, State::THETA)  = config().get<T>("filter_init_var_theta", 1);
    stateCov(State::OMEGA, State::OMEGA)  = config().get<T>("filter_init_var_omega", 1);
    filter.setCovariance(stateCov);

    // Set process noise covariance
    Kalman::Covariance<State<T>> cov;
    cov.setZero();
    cov(State::X, State::X)           = config().get<T>("sys_var_x", 1);
    cov(State::Y, State::Y)           = config().get<T>("sys_var_y", 1);
    cov(State::A, State::A)           = config().get<T>("sys_var_a", 1);
    cov(State::V, State::V)           = config().get<T>("sys_var_v", 1);
    cov(State::THETA, State::THETA)   = config().get<T>("sys_var_theta", 1);
    cov(State::OMEGA, State::OMEGA)   = config().get<T>("sys_var_omega", 1);
    sys.setCovariance(cov);
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
        auto imuTimestamp = imu->timestamp();

        if( imuTimestamp <= lastTimestamp ) {
            logger.warn("imu") << "No IMU measurement in current timestep";
        }

        if( imuTimestamp >= currentTimestamp ) {
            currentTimestamp = imuTimestamp;
        }

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

        auto hallTimestamp = hall->timestamp();
        if( hallTimestamp <= lastTimestamp ) {
            logger.warn("hall") << "No Hall measurement in current timestep";
        }

        if( hallTimestamp >= currentTimestamp ) {
            currentTimestamp = hallTimestamp;
        }

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
    if( currentTimestamp == lms::Time::ZERO || lastTimestamp == lms::Time::ZERO ) {
        // No valid timestamps for prediction step
        // -> ignore
        return;
    }

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
