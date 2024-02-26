#ifndef KALMAN_VISCOELASTIC_ESTIMATION_SYSTEMMODEL_HPP_
#define KALMAN_VISCOELASTIC_ESTIMATION_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanExperimentsFT
{
namespace Estimation
{

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(State, T, 4)
    
    //! Penetration inside the soft tissue
    static constexpr size_t POSITION = 0;
    //! Velocity of penetration inside the soft tissue
    static constexpr size_t VELOCITY = 1;
    //! Elasticity of the soft tissue
    static constexpr size_t ELASTICITY = 2;
    //! Viscosity of the soft tissue
    static constexpr size_t VISCOSITY = 3;
    
    T x1()      const { return (*this)[ POSITION ]; }
    T x2()      const { return (*this)[ VELOCITY ]; }
    T x3()      const { return (*this)[ ELASTICITY ]; }
    T x4()      const { return (*this)[ VISCOSITY ]; }
    
    T& x1()         { return (*this)[ POSITION ]; }
    T& x2()         { return (*this)[ VELOCITY ]; }
    T& x3()         { return (*this)[ ELASTICITY ]; }
    T& x4()         { return (*this)[ VISCOSITY ]; }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
    public:
    KALMAN_VECTOR(Control, T, 2)
    
    //! Force on the FT sensor
    static constexpr size_t FORCE = 0;
    
    T u()       const { return (*this)[ FORCE ]; }
    
    T& u()      { return (*this)[ FORCE ]; }
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExperimentsFT::Estimation::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExperimentsFT::Estimation::Control<T> C;
    
    //! Constructor
    SystemModel( const double dT, const double m)
    {   
        // Set model parameters
        this->dT = dT;
        this->m = m;
    }
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
        double dT = this->dT;
        double m = this->m;
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity
        /*Dynamics of the model:
        x = [
            x(1) + dT * x(2);
            x(2) - dT/(m0+m)*( -k0*xd - c0*xdp + x(1)*(k0+x(3)) + x(2)*(c0+x(4)) ) + w;
            x(3);
            x(4); 
        ];*/
        x_.x1() = x.x1() + x.x2()*dT;
        x_.x2() = x.x2() - dT/(m)*( -u.u() + x.x1()*(x.x3()) + x.x2()*(x.x4()) );
        x_.x3() = x.x3();
        x_.x4() = x.x4();
        
        // Return transitioned state vector
        return x_;
    }

    protected:

    double dT;
    double m;
    
    void updateJacobians( const S& x, const C& u )
    {   
        m = this->m;
        dT = this->dT;

        this->F.setIdentity();
        // partial derivative of x.x1() w.r.t. x.x2()
        this->F( S::POSITION, S::VELOCITY ) = dT;
        // partial derivative of x.x2() w.r.t. x.x1()
        this->F( S::VELOCITY, S::POSITION ) = -(dT * (x.x3())) / (m);
        // partial derivative of x.x2() w.r.t. x.x2()
        this->F( S::VELOCITY, S::VELOCITY ) = 1 - (dT * (x.x4())) / (m);
        // partial derivative of x.x2() w.r.t. x.x3()
        this->F( S::VELOCITY, S::ELASTICITY ) = -(dT * x.x1()) / (m);
        // partial derivative of x.x2() w.r.t. x.x4()
        this->F( S::VELOCITY, S::VISCOSITY ) = -(dT * x.x2() )/ (m);


        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setZero();
        this->W( S::POSITION, S::POSITION ) = 1.0;
        this->W( S::VELOCITY, S::VELOCITY ) = 1.0;
        
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif