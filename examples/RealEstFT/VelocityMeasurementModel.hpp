#ifndef KALMAN_VISCOELASTIC_ESTIMATION_VELOCITYMEASUREMENTMODEL_HPP_
#define KALMAN_VISCOELASTIC_ESTIMATION_VELOCITYMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExperimentsFT
{
namespace Estimation
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class VelocityMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(VelocityMeasurement, T, 1)
    
    //! Distance to landmark 1
    static constexpr size_t VELOCITY = 0;
    
    T v()       const { return (*this)[ VELOCITY ]; }
    
    T& v()      { return (*this)[ VELOCITY ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class VelocityMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VelocityMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  KalmanExperimentsFT::Estimation::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExperimentsFT::Estimation::VelocityMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    VelocityMeasurementModel(){

        this->V.setIdentity();
        
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        measurement.v() = x.x2();
        
        return measurement;
    }

    protected:
        void updateJacobians( const S& x )
        {
        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();
        
        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H( M::VELOCITY, S::VELOCITY ) = 1;
        }
};

} // namespace Robot
} // namespace KalmanExamples

#endif