#ifndef KALMAN_AFUNSCENTEDKALMANFILTER_HPP_
#define KALMAN_AFUNSCENTEDKALMANFILTER_HPP_

#include "UnscentedKalmanFilterBase.hpp"
#include "StandardFilterBase.hpp"
#include <iostream>

namespace Kalman {
    
    template<class StateType>
    class AdaptiveFadingUnscentedKalmanFilter : public UnscentedKalmanFilterBase<StateType>,
                                                public StandardFilterBase<StateType>
    {
    public:
        typedef UnscentedKalmanFilterBase<StateType> UnscentedBase;
        typedef StandardFilterBase<StateType> StandardBase;
        
        using typename UnscentedBase::T;
        using typename UnscentedBase::State;
        
        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = typename UnscentedBase::template MeasurementModelType<Measurement, CovarianceBase>;

        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = typename UnscentedBase::template SystemModelType<Control, CovarianceBase>;
        
    protected:
        using UnscentedBase::SigmaPointCount;
        
        template<class Type>
        using SigmaPoints = typename UnscentedBase::template SigmaPoints<Type>;
        
        template<class Measurement>
        using KalmanGain = Kalman::KalmanGain<State, Measurement>;
        
    protected:
        using UnscentedBase::x;
        using StandardBase::P;
        using UnscentedBase::sigmaStatePoints;
        
        // Protected typedefs
        //! The number of sigma points (depending on state dimensionality)
        static constexpr int MesCount = 1;
        
        //! Vector containg the sigma scaling weights
        typedef Vector<T, MesCount> MeasError;
        
        //! Matrix type containing the sigma state or measurement points
        typedef SquareMatrix<T, MesCount> SMat;
        

    protected:
        // Member variables
        // T alpha_f;
        SMat S_k;
        T rho;
        MeasError v_k;

    public:
        AdaptiveFadingUnscentedKalmanFilter(T alpha = T(1), T beta = T(2), T kappa = T(0), T alpha_f = T(1.001))
            : UnscentedKalmanFilterBase<StateType>(alpha, beta, kappa), alpha_f(alpha_f)
        {
            P.setIdentity();
            S_k *= 0;
            rho = 1e-3;
        }
        
        template<class Control, template<class> class CovarianceBase>
        const State& predict( const SystemModelType<Control, CovarianceBase>& s )
        {
            Control u;
            u.setZero();
            return predict( s, u );
        }
        
        template<class Control, template<class> class CovarianceBase>
        const State& predict( const SystemModelType<Control, CovarianceBase>& s, const Control& u )
        {
            if(!computeSigmaPoints())
            {
                assert(false);
            }
            
            x = this->template computeStatePrediction<Control, CovarianceBase>(s, u);
            
            computeCovarianceFromSigmaPoints(x, sigmaStatePoints, s.getCovariance(), P);
            
            P *= alpha_f; // Apply fading factor to the state covariance
            
            return this->getState();
        }
        
        template<class Measurement, template<class> class CovarianceBase>
        const State& update( const MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
        {
            SigmaPoints<Measurement> sigmaMeasurementPoints;
            
            Measurement y = this->template computeMeasurementPrediction<Measurement, CovarianceBase>(m, sigmaMeasurementPoints);

            v_k = z - y;
            
            Covariance<Measurement> P_yy;
            computeCovarianceFromSigmaPoints(y, sigmaMeasurementPoints, m.getCovariance(), P_yy);
            
            updateFadingFactor();

            KalmanGain<Measurement> K;
            computeKalmanGain(y, sigmaMeasurementPoints, P_yy, K);
            
            x += K * (v_k);
            
            updateStateCovariance<Measurement>(K, P_yy);
            
            return this->getState();
        }
        
    protected:
        /**
         * @brief Compute sigma points from current state estimate and state covariance
         * 
         * @note This covers equations (6) and (9) of Algorithm 2.1 in the Paper
         */
        bool computeSigmaPoints()
        {
            // Get square root of covariance
            CovarianceSquareRoot<State> llt;
            llt.compute(P);
            if(llt.info() != Eigen::Success)
            {
                return false;
            }
            
            SquareMatrix<T, State::RowsAtCompileTime> _S = llt.matrixL().toDenseMatrix();
            
            // Set left "block" (first column)
            sigmaStatePoints.template leftCols<1>() = x;
            // Set center block with x + gamma * S
            sigmaStatePoints.template block<State::RowsAtCompileTime, State::RowsAtCompileTime>(0,1)
                    = ( this->gamma * _S).colwise() + x;
            // Set right block with x - gamma * S
            sigmaStatePoints.template rightCols<State::RowsAtCompileTime>()
                    = (-this->gamma * _S).colwise() + x;
            
            return true;
        }
        
        /**
         * @brief Compute the Covariance from sigma points and noise covariance
         * 
         * @param [in] mean The mean predicted state or measurement
         * @param [in] sigmaPoints the predicted sigma state or measurement points
         * @param [in] noiseCov The system or measurement noise covariance
         * @param [out] cov The propagated state or innovation covariance
         *
         * @return True on success, false if a numerical error is encountered when updating the matrix
         */
        template<class Type>
        bool computeCovarianceFromSigmaPoints( const Type& mean, const SigmaPoints<Type>& sigmaPoints, 
                                               const Covariance<Type>& noiseCov, Covariance<Type>& cov) const
        {
            decltype(sigmaPoints) W = this->sigmaWeights_c.transpose().template replicate<Type::RowsAtCompileTime,1>();
            decltype(sigmaPoints) tmp = (sigmaPoints.colwise() - mean);
            cov = tmp.cwiseProduct(W) * tmp.transpose() + noiseCov;
            
            return true;
        }
        
        /**
         * @brief Compute the Kalman Gain from predicted measurement and sigma points and the innovation covariance.
         * 
         * @note This covers equations (11) and (12) of Algorithm 2.1 in the Paper
         *
         * @param [in] y The predicted measurement
         * @param [in] sigmaMeasurementPoints The predicted sigma measurement points
         * @param [in] P_yy The innovation covariance
         * @param [out] K The computed Kalman Gain matrix \f$ K \f$
         */
        template<class Measurement>
        bool computeKalmanGain( const Measurement& y,
                                const SigmaPoints<Measurement>& sigmaMeasurementPoints,
                                const Covariance<Measurement>& P_yy,
                                KalmanGain<Measurement>& K) const
        {
            // Note: The intermediate eval() is needed here (for now) due to a bug in Eigen that occurs
            // when Measurement::RowsAtCompileTime == 1 AND State::RowsAtCompileTime >= 8
            decltype(sigmaStatePoints) W = this->sigmaWeights_c.transpose().template replicate<State::RowsAtCompileTime,1>();
            Matrix<T, State::RowsAtCompileTime, Measurement::RowsAtCompileTime> P_xy
                    =  (sigmaStatePoints.colwise() - x).cwiseProduct( W ).eval()
                    * (sigmaMeasurementPoints.colwise() - y).transpose();
            
            K = P_xy * P_yy.inverse();
            return true;
        }
        
        /**
         * @brief Update the state covariance matrix using the Kalman Gain and the Innovation Covariance
         * 
         * @note This covers equation (14) of Algorithm 2.1 in the Paper
         *
         * @param [in] K The computed Kalman Gain matrix
         * @param [in] P_yy The innovation covariance
         * @return True on success, false if a numerical error is encountered when updating the matrix
         */
        template<class Measurement>
        bool updateStateCovariance(const KalmanGain<Measurement>& K, const Covariance<Measurement>& P_yy)
        {
            P -= K * P_yy * K.transpose();
            return true;
        }

        // template<class Measurement>
        // bool updateFadingFactor(const Covariance<Measurement>& P_yy, const Covariance<Measurement>& R) 
        // {
        //     S_k = ( rho * S_k + v_k * v_k.transpose() ) / ( rho + 1 );
        //     Covariance<Measurement> N = S_k - R;
            
        //     T lambda_k = N.trace() / P_yy.trace();

        //     // if (lambda_k < 1)
        //     // {
        //     //     alpha_f = 1;
        //     // }
        //     // // else if (lambda_k >1.1)
        //     // // {
        //     // //     alpha_f = 1.00001;
        //     // // }
        //     // else
        //     // {
        //     //     alpha_f = lambda_k;
        //     // }
        //     // std::cout<<"alpha_f: "<<alpha_f<<std::endl;
        //     return true;
        // }
        private:
        // Example implementation of fading factor adjustment
        void updateFadingFactor() {
            // Simple heuristic: adjust alpha based on the magnitude of the innovation
            double innovationMagnitude = v_k.norm();
            // std::cout << "Innovation magnitude: " << innovationMagnitude << std::endl;
            if (innovationMagnitude > threshold) {
                alpha_f = std::min(maxAlpha, alpha_f + alphaIncrement);
            } else {
                alpha_f = std::max(minAlpha, alpha_f - alphaDecrement);
            }
            // std::cout << "Fading factor: " << alpha << std::endl;
        }

        double threshold = 1.9; // Threshold for innovation magnitude
        double maxAlpha = 1.003; // Maximum allowed fading factor
        double minAlpha = 1.0; // Minimum allowed fading factor
        double alphaIncrement = 0.0001; // Increment step for increasing alpha
        double alphaDecrement = 0.00005; // Decrement step for decreasing alpha
        double alpha_f = 1.0; // Initial fading factor
    };
}

#endif

// #ifndef KALMAN_AFEXTENDEDKALMANFILTER_HPP_
// #define KALMAN_AFEXTENDEDKALMANFILTER_HPP_

// #include "KalmanFilterBase.hpp"
// #include "StandardFilterBase.hpp"
// #include "LinearizedSystemModel.hpp"
// #include "LinearizedMeasurementModel.hpp"
// #include <iostream>

// namespace Kalman {
    
//     /**
//      * @brief Extended Kalman Filter (EKF)
//      * 
//      * This implementation is based upon [An Introduction to the Kalman Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
//      * by Greg Welch and Gary Bishop.
//      *
//      * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
//      */
//     template<class StateType>
//     class AFExtendedKalmanFilter : public KalmanFilterBase<StateType>,
//                                  public StandardFilterBase<StateType>
//     {
//     public:
//         //! Kalman Filter base type
//         typedef KalmanFilterBase<StateType> KalmanBase;
//         //! Standard Filter base type
//         typedef StandardFilterBase<StateType> StandardBase;
        
//         //! Numeric Scalar Type inherited from base
//         using typename KalmanBase::T;
        
//         //! State Type inherited from base
//         using typename KalmanBase::State;
        
//         //! Linearized Measurement Model Type
//         template<class Measurement, template<class> class CovarianceBase>
//         using MeasurementModelType = LinearizedMeasurementModel<State, Measurement, CovarianceBase>;
        
//         //! Linearized System Model Type
//         template<class Control, template<class> class CovarianceBase>
//         using SystemModelType = LinearizedSystemModel<State, Control, CovarianceBase>;
        
//     protected:
//         //! Kalman Gain Matrix Type
//         template<class Measurement>
//         using KalmanGain = Kalman::KalmanGain<State, Measurement>;
        
//     protected:
//         //! State Estimate
//         using KalmanBase::x;
//         //! State Covariance Matrix
//         using StandardBase::P;

//         Vector<T, State::ColsAtCompileTime> y;
        
//     public:
//         /**
//          * @brief Constructor
//          */
//         AFExtendedKalmanFilter()
//         {
//             // Setup state and covariance
//             P.setIdentity();
//         }
        
//         /**
//          * @brief Perform filter prediction step using system model and no control input (i.e. \f$ u = 0 \f$)
//          *
//          * @param [in] s The System model
//          * @return The updated state estimate
//          */
//         template<class Control, template<class> class CovarianceBase>
//         const State& predict( SystemModelType<Control, CovarianceBase>& s )
//         {
//             // predict state (without control)
//             Control u;
//             u.setZero();
//             return predict( s, u );
//         }
        
//         /**
//          * @brief Perform filter prediction step using control input \f$u\f$ and corresponding system model
//          *
//          * @param [in] s The System model
//          * @param [in] u The Control input vector
//          * @return The updated state estimate
//          */
//         template<class Control, template<class> class CovarianceBase>
//         const State& predict( SystemModelType<Control, CovarianceBase>& s, const Control& u )
//         {
//             s.updateJacobians( x, u );
            
//             // predict state
//             x = s.f(x, u);
            
//             // predict covariance
//             P  = alpha * ( s.F * P * s.F.transpose() ) + ( s.W * s.getCovariance() * s.W.transpose() );
            
//             // return state prediction
//             return this->getState();
//         }
        
//         /**
//          * @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
//          *
//          * @param [in] m The Measurement model
//          * @param [in] z The measurement vector
//          * @return The updated state estimate
//          */
//         template<class Measurement, template<class> class CovarianceBase>
//         const State& update( MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
//         {
//             m.updateJacobians( x );
            
//             // COMPUTE KALMAN GAIN
//             // compute innovation covariance
//             Covariance<Measurement> S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );
            
//             // compute kalman gain
//             KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();
            
//             // UPDATE STATE ESTIMATE AND COVARIANCE
//             // Update state using computed kalman gain and innovation
//             y = z - m.h( x );
//             x += K * ( y );
            
//             // Update covariance
//             P -= K * m.H * P;
            
//             updateFadingFactor();
//             // return updated state estimate
//             return this->getState();
//         }
//     private:
//         // Example implementation of fading factor adjustment
//         void updateFadingFactor() {
//             // Simple heuristic: adjust alpha based on the magnitude of the innovation
//             double innovationMagnitude = y.norm();
//             // std::cout << "Innovation magnitude: " << innovationMagnitude << std::endl;
//             if (innovationMagnitude > threshold) {
//                 alpha = std::min(maxAlpha, alpha + alphaIncrement);
//             } else {
//                 alpha = std::max(minAlpha, alpha - alphaDecrement);
//             }
//             // std::cout << "Fading factor: " << alpha << std::endl;
//         }

//         double threshold = 1.0; // Threshold for innovation magnitude
//         double maxAlpha = 1.00001; // Maximum allowed fading factor
//         double minAlpha = 1.0; // Minimum allowed fading factor
//         double alphaIncrement = 0.000001; // Increment step for increasing alpha
//         double alphaDecrement = 0.0000005; // Decrement step for decreasing alpha
//         double alpha = 1.0; // Initial fading factor
//     };
// }

// #endif