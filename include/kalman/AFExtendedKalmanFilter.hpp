#ifndef KALMAN_AFEXTENDEDKALMANFILTER_HPP_
#define KALMAN_AFEXTENDEDKALMANFILTER_HPP_

#include "KalmanFilterBase.hpp"
#include "StandardFilterBase.hpp"
#include "LinearizedSystemModel.hpp"
#include "LinearizedMeasurementModel.hpp"
#include <iostream>

namespace Kalman {
    
    /**
     * @brief Extended Kalman Filter (EKF)
     * 
     * This implementation is based upon [An Introduction to the Kalman Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
     * by Greg Welch and Gary Bishop.
     *
     * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
     */
    template<class StateType>
    class AFExtendedKalmanFilter : public KalmanFilterBase<StateType>,
                                 public StandardFilterBase<StateType>
    {
    public:
        //! Kalman Filter base type
        typedef KalmanFilterBase<StateType> KalmanBase;
        //! Standard Filter base type
        typedef StandardFilterBase<StateType> StandardBase;
        
        //! Numeric Scalar Type inherited from base
        using typename KalmanBase::T;
        
        //! State Type inherited from base
        using typename KalmanBase::State;
        
        //! Linearized Measurement Model Type
        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = LinearizedMeasurementModel<State, Measurement, CovarianceBase>;
        
        //! Linearized System Model Type
        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = LinearizedSystemModel<State, Control, CovarianceBase>;
        
    protected:
        //! Kalman Gain Matrix Type
        template<class Measurement>
        using KalmanGain = Kalman::KalmanGain<State, Measurement>;
        
    protected:
        //! State Estimate
        using KalmanBase::x;
        //! State Covariance Matrix
        using StandardBase::P;

        Vector<T, State::ColsAtCompileTime> y;
        
    public:
        /**
         * @brief Constructor
         */
        AFExtendedKalmanFilter()
        {
            // Setup state and covariance
            P.setIdentity();
        }
        
        /**
         * @brief Perform filter prediction step using system model and no control input (i.e. \f$ u = 0 \f$)
         *
         * @param [in] s The System model
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( SystemModelType<Control, CovarianceBase>& s )
        {
            // predict state (without control)
            Control u;
            u.setZero();
            return predict( s, u );
        }
        
        /**
         * @brief Perform filter prediction step using control input \f$u\f$ and corresponding system model
         *
         * @param [in] s The System model
         * @param [in] u The Control input vector
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( SystemModelType<Control, CovarianceBase>& s, const Control& u )
        {
            s.updateJacobians( x, u );
            
            // predict state
            x = s.f(x, u);
            
            // predict covariance
            P  = ( s.F * P * s.F.transpose() ) + ( s.W * s.getCovariance() * s.W.transpose() );
            P *= alpha; // Apply fading factor to the state covariance
            // return state prediction
            return this->getState();
        }
        
        /**
         * @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
         *
         * @param [in] m The Measurement model
         * @param [in] z The measurement vector
         * @return The updated state estimate
         */
        template<class Measurement, template<class> class CovarianceBase>
        const State& update( MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
        {
            m.updateJacobians( x );
            
            // COMPUTE KALMAN GAIN
            // compute innovation covariance
            Covariance<Measurement> S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );
            
            // compute kalman gain
            KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();
            
            // UPDATE STATE ESTIMATE AND COVARIANCE
            // Update state using computed kalman gain and innovation
            y = z - m.h( x );
            x += K * ( y );
            
            // Update covariance
            P -= K * m.H * P;
            
            updateFadingFactor();
            // return updated state estimate
            return this->getState();
        }
    private:
        // Example implementation of fading factor adjustment
        void updateFadingFactor() {
            // Simple heuristic: adjust alpha based on the magnitude of the innovation
            double innovationMagnitude = y.norm();
            // std::cout << "Innovation magnitude: " << innovationMagnitude << std::endl;
            if (innovationMagnitude > threshold) {
                alpha = std::min(maxAlpha, alpha + alphaIncrement);
            } else {
                alpha = std::max(minAlpha, alpha - alphaDecrement);
            }
            // std::cout << "Fading factor: " << alpha << std::endl;
        }

        double threshold = 1.5; // Threshold for innovation magnitude
        double maxAlpha = 1.001; // Maximum allowed fading factor
        double minAlpha = 1.0; // Minimum allowed fading factor
        double alphaIncrement = 0.0001; // Increment step for increasing alpha
        double alphaDecrement = 0.00005; // Decrement step for decreasing alpha
        double alpha = 1.0; // Initial fading factor
    };
}

#endif