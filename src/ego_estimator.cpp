#include "ego_estimator.h"

#include <sensor_utils/imu.h>
#include <sensor_utils/odometer.h>

#define GRAVITY T(9.81)

bool EgoEstimator::initialize() {
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    car = writeChannel<sensor_utils::Car>("CAR");

    // Init Kalman Filter
    initFilter();

    // Debug logging
    if( config().get<bool>("state_log",false) )
    {
        if(!isEnableSave()) {
            logger.error() << "Command line option --enable-save was not specified";
            return false;
        }

        stateLogEnabled = true;
        stateLog.open( saveLogDir("ego_estimator") + "/state_log.csv" );
        stateLog << "timestamp,x,y,theta,v,omega,a" << std::endl;

        measurementLog.open( saveLogDir("ego_estimator") + "/measurement_log.csv" );
        measurementLog << "timestamp,ax,ay,v,omega" << std::endl;
    } else {
        stateLogEnabled = false;
    }

    return true;
}

bool EgoEstimator::deinitialize() {
    if(stateLogEnabled) {
        stateLog.close();
    }
    return true;
}

bool EgoEstimator::cycle() {
    //compute the timestamp
    computeTimeStamp();
    // Compute Measurement Update
    computeMeasurement();

    // Compute Kalman filter step
    computeFilterStep();

    // Update car state
    updateCarState();

    // Add current state estimate with timestamp to interpolation values
    // TODO

    return true;
}

void EgoEstimator::initFilter()
{
    lastTimestamp = lms::Time::ZERO;
    currentTimestamp = lms::Time::ZERO;

    // Init kalman
    State s;
    s.setZero();
    filter.init(s);

    // Set initial state covariance
    Kalman::Covariance<State> stateCov;
    stateCov.setZero();
    stateCov(State::X, State::X)          = config().get<T>("filter_init_var_x", 1);
    stateCov(State::Y, State::Y)          = config().get<T>("filter_init_var_y", 1);
    stateCov(State::A, State::A)          = config().get<T>("filter_init_var_a", 1);
    stateCov(State::V, State::V)          = config().get<T>("filter_init_var_v", 1);
    stateCov(State::THETA, State::THETA)  = config().get<T>("filter_init_var_theta", 1);
    stateCov(State::OMEGA, State::OMEGA)  = config().get<T>("filter_init_var_omega", 1);
    filter.setCovariance(stateCov);

    // Set process noise covariance
    Kalman::Covariance<State> cov;
    cov.setZero();
    cov(State::X, State::X)           = config().get<T>("sys_var_x", 1);
    cov(State::Y, State::Y)           = config().get<T>("sys_var_y", 1);
    cov(State::A, State::A)           = config().get<T>("sys_var_a", 1);
    cov(State::V, State::V)           = config().get<T>("sys_var_v", 1);
    cov(State::THETA, State::THETA)   = config().get<T>("sys_var_theta", 1);
    cov(State::OMEGA, State::OMEGA)   = config().get<T>("sys_var_omega", 1);
    sys.setCovariance(cov);
}

void EgoEstimator::computeTimeStamp(){
    //set current one to old one
    lastTimestamp = currentTimestamp;
    bool invalid = true;
    if(sensors->hasSensor("IMU")){
        auto imu = sensors->sensor<sensor_utils::IMU>("IMU");
        auto imuTimestamp = imu->timestamp();
        if( imuTimestamp >= currentTimestamp ) {
            currentTimestamp = imuTimestamp;
        }
        invalid = false;
    }
    if(sensors->hasSensor("HALL")){
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
        auto hallTimestamp = hall->timestamp();
        if( hallTimestamp >= currentTimestamp ) {
            currentTimestamp = hallTimestamp;
        }
        invalid = false;
    }
    if(invalid){
        //timestamps setzen, die elektronik schickt keine richtigen Daten
        currentTimestamp = currentTimestamp+ lms::Time::fromMillis(config().get<int>("defaultStepInMs",10));
    }
}

void EgoEstimator::computeMeasurement(){
    T v = 0;
    T ax = 0;
    T ay = 0;
    T omega = 0;

    T axVar = 0;
    T ayVar = 0;
    T vVar = 0;
    T omegaVar = 0;

    if(!sensors->hasSensor("HALL")) {
        logger.warn("hall") << "MISSING HALL SENSOR!";
        v = car->targetSpeed();
        vVar = config().get<float>("backup_vVar",1);
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");

        auto hallTimestamp = hall->timestamp();
        if( hallTimestamp <= lastTimestamp ) {
            logger.warn("hall") << "No Hall measurement in current timestep";
            //TODO error handling (may call !hasSensor code)
        }
        v = hall->velocity.x();
        vVar = hall->velocityCovariance.xx();
    }

    if(!sensors->hasSensor("IMU")) {
        logger.warn("imu") << "MISSING IMU SENSOR!";
        // FALLBACK TO STEERING ANGLES FOR TURN RATE
        float steeringFront = car->steeringFront();
        float steeringRear = car->steeringRear();
        float radstand = config().get<float>("radstand",0.26);
        float dt = (currentTimestamp-lastTimestamp).toFloat(); //TODO
        float distance = v*dt;
        float angle = distance/radstand*sin(steeringFront-steeringRear)/cos(steeringRear);
        omega = angle/dt;
        omegaVar = config().get<float>("backup_omegaVar",1); //TODO set val
        //we don't acc
        axVar = 1;
        ayVar = 1;
    } else {
        auto imu = sensors->sensor<sensor_utils::IMU>("IMU");
        auto imuTimestamp = imu->timestamp();

        if( imuTimestamp <= lastTimestamp ) {
            logger.warn("imu") << "No IMU measurement in current timestep";
            //TODO error handling (may call !hasSensor code)
        }

        // TODO: remove acceleration orientation hack
        omega = imu->gyroscope.z();
        ax    = GRAVITY*imu->accelerometer.x();
        ay    = GRAVITY*imu->accelerometer.y();

        omegaVar = imu->gyroscopeCovariance.zz();
        axVar    = GRAVITY*GRAVITY*imu->accelerometerCovariance.xx();
        ayVar    = GRAVITY*GRAVITY*imu->accelerometerCovariance.yy();
    }

    /*
    // TODO: enable mouse sensors -> check if quality is high enough to be considered valid
    if(!sensors->hasSensor("MOUSE_FRONT")) {
        //logger.error("mouse_front") << "MISSING MOUSE_FRONT SENSOR!";
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("MOUSE_FRONT");

        // TODO: Set Covariances
        vMotion = hall->velocity.x();
    }

    if(!sensors->hasSensor("MOUSE_REAR")) {
        //logger.error("mouse_front") << "MISSING MOUSE_FRONT SENSOR!";
    } else {
        auto hall = sensors->sensor<sensor_utils::Odometer>("MOUSE_REAR");

        // TODO: Set Covariances
        vMotion = hall->velocity.x();
    }
     */

    // Set actual measurement vector
    z.v() = v;
    z.ax() = ax;
    z.ay() = ay;
    z.omega() = omega;

    // Set measurement covariances
    Kalman::Covariance< Measurement > cov;
    cov.setZero();
    cov(Measurement::AX,    Measurement::AX)    = axVar;
    cov(Measurement::AY,    Measurement::AY)    = ayVar;
    cov(Measurement::V,     Measurement::V)     = vVar;
    cov(Measurement::OMEGA, Measurement::OMEGA) = omegaVar;
    mm.setCovariance(cov);

    logger.debug("measurementVector") << std::endl << z;
    logger.debug("measurementCovariance") << std::endl << mm.getCovariance();

    if(stateLogEnabled) {
        measurementLog << currentTimestamp.micros() << ","
                        << ax << ","
                        << ay << ","
                        << v << ","
                        << omega << std::endl;
    }
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
    if(delta < lms::Time::ZERO) {
        logger.error("time") << "JUMPING BACKWARDS IN TIME!"
                             << " last = " << lastTimestamp
                             << " current = " << currentTimestamp
                             << " delta = " << delta;
        return;
    }
    u.dt() = delta.toFloat();

    logger.debug("delta") << u.dt();

    auto previousState = filter.getState();

    // predict state for current time-step using the kalman filter
    filter.predict(sys, u);
    // perform measurement update
    filter.update(mm, z);

    const auto& state = filter.getState();

    logger.debug("stateEstimate") << std::endl << state;
    logger.debug("stateCovariance") << std::endl << filter.getCovariance();

    if(stateLogEnabled) {
        stateLog << currentTimestamp.micros() << ","
                 << state.x() << ","
                 << state.y() << ","
                 << state.theta() << ","
                 << state.v() << ","
                 << state.omega() << ","
                 << state.a()
                 << std::endl;
    }
}

void EgoEstimator::updateCarState()
{
    const auto& state = filter.getState();
    auto viewDir =lms::math::vertex2f(std::cos(state.theta()), std::sin(state.theta()));
    car->updatePosition(lms::math::vertex2f(state.x(), state.y()), viewDir);
    car->updateVelocity(state.v(), viewDir);
    car->updateTurnRate(state.omega());
}
