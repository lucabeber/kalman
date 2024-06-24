#ifndef KALMAN_VISCOELASTIC_ESTIMATION_SYSTEMMODEL_HPP_
#define KALMAN_VISCOELASTIC_ESTIMATION_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanExperimentsNoFTNL
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
class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)
    
    //! Penetration inside the soft tissue
    static constexpr size_t POSITION = 0;
    //! Velocity of penetration inside the soft tissue
    static constexpr size_t VELOCITY = 1;
    //! Elasticity of the soft tissue
    static constexpr size_t ELASTICITY = 2;
    //! Viscosity of the soft tissue
    static constexpr size_t VISCOSITY = 3;
    //! Change in elasticity of the soft tissue
    static constexpr size_t ELASTICITY_CHANGE = 4;
    //! Change in viscosity of the soft tissue
    static constexpr size_t VISCOSITY_CHANGE = 5;
    
    T x1()      const { return (*this)[ POSITION ]; }
    T x2()      const { return (*this)[ VELOCITY ]; }
    T x3()      const { return (*this)[ ELASTICITY ]; }
    T x4()      const { return (*this)[ VISCOSITY ]; }
    T x5()      const { return (*this)[ ELASTICITY_CHANGE ]; }
    T x6()      const { return (*this)[ VISCOSITY_CHANGE ]; }
    
    T& x1()         { return (*this)[ POSITION ]; }
    T& x2()         { return (*this)[ VELOCITY ]; }
    T& x3()         { return (*this)[ ELASTICITY ]; }
    T& x4()         { return (*this)[ VISCOSITY ]; }
    T& x5()         { return (*this)[ ELASTICITY_CHANGE ]; }
    T& x6()         { return (*this)[ VISCOSITY_CHANGE ]; }
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
class Control : public Kalman::Vector<T, 1>
{
    public:
    KALMAN_VECTOR(Control, T, 1)
    
    //! Desired penetration inside the soft tissue
    static constexpr size_t X_TILDE = 0;
    
    T u()       const { return (*this)[ X_TILDE ]; }
    
    T& u()      { return (*this)[ X_TILDE ]; }
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
	typedef KalmanExperimentsNoFTNL::Estimation::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExperimentsNoFTNL::Estimation::Control<T> C;
    
    //! Constructor
    SystemModel( const double dT, const double m, const double k0, const double c0 )
    {   
        // Set model parameters
        this->dT = dT;
        this->m = m;
        this->k0 = k0;
        this->c0 = c0;
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
        double k0 = this->k0;
        double c0 = this->c0;
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity
        /*Dynamics of the model:
        x = [
            x(1) + dT * x(2);
            x(2) - dT/(ls_param2(3))*( -f + x(1)^(1.5)*(x(3)) + x(2)*(x(4))*x(1)^(0.5) );
            x(3);
            x(4); 
        ];
        */
        x_.x1() = x.x1() + x.x2()*dT;
        x_.x2() = x.x2() - dT/(m)*( -u.u() + pow(abs(x.x1()),1.5)*(x.x3()) + x.x2()*pow(abs(x.x1()),0.5)*(x.x4()));
        x_.x3() = x.x3() + x.x5()*dT;
        x_.x4() = x.x4() + x.x6()*dT;
        x_.x5() = x.x5();
        x_.x6() = x.x6();
        
        // Return transitioned state vector
        return x_;
    }

    protected:

    double dT;
    double m;
    double k0;
    double c0;
    
    void updateJacobians( const S& x, const C& u )
    {   
        m = this->m;
        k0 = this->k0;
        c0 = this->c0;
        dT = this->dT;

        /*
        jacobian of the state transition w.r.t. the state 
        (1	dT	0	0
        -((dT (3 x3 x1+x4 x2))/(2 (m+m0) Sqrt[x1]))	1-(dT (c0+x4 Sqrt[x1]))/(m+m0)	-((dT x1^(3/2))/(m+m0))	-((dT Sqrt[x1] x2)/(m+m0))
        0	0	1	0
        0	0	0	1)
        */

        this->F.setIdentity();
        // partial derivative of x.x1() w.r.t. x.x2()
        this->F( S::POSITION, S::VELOCITY ) = dT;
        // partial derivative of x.x2() w.r.t. x.x1()
        this->F( S::VELOCITY, S::POSITION ) = -((dT * ( 3 * x.x3() * x.x1() + x.x4() * x.x2() ))/(2 * (m) * sqrt(abs(x.x1()))));
        // partial derivative of x.x2() w.r.t. x.x2()
        this->F( S::VELOCITY, S::VELOCITY ) = 1 - ((dT * (x.x4() * sqrt(abs(x.x1()))))/(m));
        // partial derivative of x.x2() w.r.t. x.x3()
        this->F( S::VELOCITY, S::ELASTICITY ) = -((dT * pow(abs(x.x1()), 3/2))/(m));
        // partial derivative of x.x2() w.r.t. x.x4()
        this->F( S::VELOCITY, S::VISCOSITY ) = -((dT * sqrt(abs(x.x1())) * x.x2())/(m));
        // partial derivative of x.x3() w.r.t. x.x5()
        this->F( S::ELASTICITY, S::ELASTICITY_CHANGE ) = dT;
        // partial derivative of x.x4() w.r.t. x.x6()
        this->F( S::VISCOSITY, S::VISCOSITY_CHANGE ) = dT;


        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setZero();
        this->W( S::POSITION, S::POSITION ) = 1.0;
        this->W( S::VELOCITY, S::VELOCITY ) = 1.0;
        
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif