#ifndef CAR_TRACKER_FILTER_SYSTEM_MODEL_H
#define CAR_TRACKER_FILTER_SYSTEM_MODEL_H

#include <kalman/SystemModel.hpp>

namespace CTRA {

/**
 * @brief System state vector-type for CTRA motion model
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)

    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Orientation
    static constexpr size_t THETA = 2;
    //! X velocity
    static constexpr size_t V = 3;
    //! Y velocity
    static constexpr size_t A = 4;
    //! Angular velocity
    static constexpr size_t OMEGA = 5;

    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T theta()   const { return (*this)[ THETA ]; }
    T v()      const { return (*this)[ V ]; }
    T a()      const { return (*this)[ A ]; }
    T omega()   const { return (*this)[ OMEGA ]; }

    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& theta()  { return (*this)[ THETA ]; }
    T& v()     { return (*this)[ V ]; }
    T& a()     { return (*this)[ A ]; }
    T& omega()  { return (*this)[ OMEGA ]; }
};

/**
 * @brief System control-input vector-type for CTRA motion model
 *
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)

    //! time since filter was last called
    static constexpr size_t DT = 0;

    T dt()      const { return (*this)[ DT ]; }
    T& dt()     { return (*this)[ DT ]; }
};

/**
 * @brief System model CTRA motion
 *
 * This is the system model defining how our car moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Control type shortcut definition
    typedef Control<T> C;

    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        auto th = x.theta();
        auto v = x.v();
        auto a = x.a();
        auto om = x.omega();
        auto dT = u.dt();

        if (std::abs(om) < T(0.01))
        {
            x_.x()      = x.x() + T(0.5)*dT*(2*v+a*dT)*cos(th);
            x_.y()      = x.y() + T(0.5)*dT*(2*v+a*dT)*sin(th);
        }
        else
        {
            x_.x()      = x.x() + 1/(om*om)*((v*om+a*om*dT)*sin(th+om*dT) + a*cos(th+om*dT) - v*om*sin(th) - a*cos(th));
            x_.y()      = x.y() + 1/(om*om)*((-v*om-a*om*dT)*cos(th+om*dT) + a*sin(th+om*dT) + v*om*cos(th) - a*sin(th));
        }

        x_.theta()  = th + om*dT;
        x_.v()      = v + a*dT;
        x_.a()      = a + 0;
        x_.omega()  = om + 0;

        // Return transitioned state vector
        return x_;
    }
};

} // namespace CTRA

#endif
