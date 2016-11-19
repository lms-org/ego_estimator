#include "ego_estimator.h"

#include <sensor_utils/imu.h>
#include <sensor_utils/odometer.h>
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"

#define GRAVITY T(9.81)

bool EgoEstimator::initialize() {
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    car = writeChannel<street_environment::Car>("CAR");

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

    previousDelta = lms::Time::ZERO;

    //Add some sensorUpdate...
    sensorHasUpdate.addSensor(sensor_utils::Sensor("IMU"));
    sensorHasUpdate.addSensor(sensor_utils::Sensor("HALL"));
    return true;
}

bool EgoEstimator::deinitialize() {
    if(stateLogEnabled) {
        stateLog.close();
    }
    return true;
}

bool EgoEstimator::cycle() {

    lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> phoenixService = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");

    if(phoenixService.isValid()){
        if(phoenixService->rcStateChanged() || phoenixService->driveModeChanged()){
            //reset kalman
            initFilter();
            //reset car
            car->updatePosition(lms::math::vertex2f(0, 0), lms::math::vertex2f(1,0));
            car->updateVelocity(0, lms::math::vertex2f(1,0));
            car->updateTurnRate(0);
        }
    }else{
        logger.warn("cycle")<<" No pheonix-service available";
    }
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

    if(!sensorHasUpdate.hasUpdate(*sensors->sensor<sensor_utils::Sensor>("HALL"))) {
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

    if(!sensorHasUpdate.hasUpdate(*sensors->sensor<sensor_utils::Sensor>("IMU"))) {
        logger.warn("imu") << "MISSING IMU SENSOR!";
        // FALLBACK TO STEERING ANGLES FOR TURN RATE
        float steeringFront = car->steeringFront();
        float steeringRear = car->steeringRear();
        float radstand = config().get<float>("radstand",0.26);
        const float dt = (currentTimestamp-lastTimestamp).toFloat(); //TODO
        float distance = v*dt;
        float angle = distance/radstand*sin(steeringFront-steeringRear)/cos(steeringRear);
        omega = angle/dt;
        omegaVar = config().get<float>("backup_omegaVar",0.025); //TODO set val
        //we don't acc
        axVar = 0.5;
        ayVar = 1.5;
        ax = 0;
        ay = omega*v;

    } else {
        auto imu = sensors->sensor<sensor_utils::IMU>("IMU");
        // TODO: remove acceleration orientation hack
        omega = imu->gyroscope.z();
        ax    = GRAVITY*imu->accelerometer.x();
        ay    = GRAVITY*imu->accelerometer.y();

        omegaVar = imu->gyroscopeCovariance.zz();
        axVar    = GRAVITY*GRAVITY*imu->accelerometerCovariance.xx();
        ayVar    = GRAVITY*GRAVITY*imu->accelerometerCovariance.yy();
    }

    // Set actual measurement vector
    //TODO fail, if v gets smaller, the filter fails!
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

    logger.debug("measurementVector") << z;
    logger.debug("measurementCovariance") << mm.getCovariance();
    //Error checking
    if(omega != omega){
        throw std::runtime_error("omega is NAN");
    }
    if(ay != ay){
        throw std::runtime_error("ay is NAN");
    }
    if(ax != ax){
        throw std::runtime_error("ax is NAN");
    }
    if(v != v){
        throw std::runtime_error("v is NAN");
    }

    if(stateLogEnabled) {
        measurementLog << currentTimestamp.micros() << ","
                        << ax << ","
                        << ay << ","
                        << v << ","
                        << omega << std::endl;
    }
}

void EgoEstimator::computeFilterStep(){
    if( currentTimestamp == lms::Time::ZERO || lastTimestamp == lms::Time::ZERO ) {
        // No valid timestamps for prediction step
        // -> ignore
        return;
    }


    // time since UKF was last called (parameter, masked as control input)
    auto currentDelta = ( currentTimestamp - lastTimestamp );
    if(currentDelta == lms::Time::ZERO) {
        // Just predict using previous delta
        logger.error("time") << "No Sensor Data "
            << " last = " << lastTimestamp
            << " current = " << currentTimestamp;

    } else if( currentDelta < lms::Time::ZERO) {
        logger.error("time") << "JUMPING BACKWARDS IN TIME!"
            << " last = " << lastTimestamp
            << " current = " << currentTimestamp
            << " delta = " << currentDelta;
        return;
    }

    logger.debug("delta") << "current: " << currentDelta << " previous: " << previousDelta;

    if(currentDelta == lms::Time::ZERO) {
        u.dt() = previousDelta.toFloat();

        // Prediction only
        // predict state for current time-step using the kalman filter
        filter.predict(sys, u);
    } else {
        u.dt() = currentDelta.toFloat();

        // Prediction + update
        // predict state for current time-step using the kalman filter
        filter.predict(sys, u);
        // perform measurement update
        filter.update(mm, z);

        // Update previous delta
        previousDelta = currentDelta;
    }

    const auto& state = filter.getState();

    logger.debug("newState") << state;
    logger.debug("stateCovariance") <<filter.getCovariance();

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

//float oldX;
void EgoEstimator::updateCarState(){
    const auto& state = filter.getState();
    /*
    float diff = state.x()-oldX;
    if(diff < 0){
        logger.error("DISS-vor")<<oldX;
        logger.error("DISS-jetzt") << currentTimestamp.micros() << ","
                 << state.x() << ","
                 << state.y() << ","
                 << state.theta() << ","
                 << state.v() << ","
                 << state.omega() << ","
                 << state.a()
                 << std::endl;
    }
    oldX  =state.x();
    */
    auto viewDir =lms::math::vertex2f(std::cos(state.theta()), std::sin(state.theta()));
    car->updatePosition(lms::math::vertex2f(state.x(), state.y()), viewDir);
    car->updateVelocity(state.v(), viewDir);
    car->updateTurnRate(state.omega());
}
