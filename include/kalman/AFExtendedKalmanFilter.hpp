// // The MIT License (MIT)
// //
// // Copyright (c) 2015 Markus Herb
// //
// // Permission is hereby granted, free of charge, to any person obtaining a copy
// // of this software and associated documentation files (the "Software"), to deal
// // in the Software without restriction, including without limitation the rights
// // to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// // copies of the Software, and to permit persons to whom the Software is
// // furnished to do so, subject to the following conditions:
// //
// // The above copyright notice and this permission notice shall be included in
// // all copies or substantial portions of the Software.
// //
// // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// // AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// // OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// // THE SOFTWARE.
// #ifndef KALMAN_AFEXTENDEDKALMANFILTER_HPP_
// #define KALMAN_AFEXTENDEDKALMANFILTER_HPP_

// #include "KalmanFilterBase.hpp"
// #include "StandardFilterBase.hpp"
// #include "LinearizedSystemModel.hpp"
// #include "LinearizedMeasurementModel.hpp"


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
//         //! State Covariance Matrix Estimate
//         Covariance<StateType> Pp;

//         //! AFEKF variables
//         float lambdat,alpha;
        
//     public:
//         /**
//          * @brief Constructor
//          */
//         AFExtendedKalmanFilter()
//         {
//             // Setup state and covariance
//             P.setIdentity();

//             lambdat = 1.0;
//             alpha = 1.0;
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
//             P  = lambdat * ( s.F * P * s.F.transpose() ) + ( s.W * s.getCovariance() * s.W.transpose() );
            
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
//         template<class Measurement, template<class> class CovarianceBase, class Control>
//         const State& update( MeasurementModelType<Measurement, CovarianceBase>& m,  SystemModelType<Control, CovarianceBase>& s, const Measurement& z )
//         {
//             m.updateJacobians( x );
            
//             // COMPUTE KALMAN GAIN
//             // compute innovation covariance
//             Covariance<Measurement> S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );
            
//             // compute kalman gain
//             KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();
            
//             // UPDATE STATE ESTIMATE AND COVARIANCE
//             // Update state using computed kalman gain and innovation
//             x += K * ( z - m.h( x ) );
            
//             // Save current covariance matrix
//             Pp = P;

//             // Update covariance
//             P -= K * m.H * P;
            
//             // compute coeff for fading factor
//             Covariance<Measurement> Mt,Ct,Nt;
//             Mt = m.H * s.F * P * s.F.transpose() * m.H.transpose();
//             Ct = m.H * Pp * m.H.transpose();
//             Nt = Ct - m.H * s.W * s.getCovariance() * s.W.transpose() * m.H.transpose();

//             // compute fading factor
//             lambdat = alpha * (Nt * Mt.inverse()).trace();

//             if (lambdat < 1.0)
//                 lambdat = 1.0;
            
//             // return updated state estimate
//             return this->getState();
//         }
//     };
// }

// #endif

#ifndef KALMAN_ADAPTIVEFADINGUNSCENTEDKALMANFILTER_HPP_
#define KALMAN_ADAPTIVEFADINGUNSCENTEDKALMANFILTER_HPP_

#include "UnscentedKalmanFilterBase.hpp"
#include "StandardFilterBase.hpp"

namespace Kalman {

    /**
     * @brief Adaptive Fading Unscented Kalman Filter (AFUKF)
     *
     * This implementation is based upon the Unscented Kalman Filter with an added adaptive fading factor to improve robustness.
     * 
     * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
     */
    template<class StateType>
    class AdaptiveFadingUnscentedKalmanFilter : public UnscentedKalmanFilterBase<StateType>,
                                                public StandardFilterBase<StateType>
    {
    public:
        //! Unscented Kalman Filter base type
        typedef UnscentedKalmanFilterBase<StateType> UnscentedBase;
        
        //! Standard Filter base type
        typedef StandardFilterBase<StateType> StandardBase;
        
        //! Numeric Scalar Type inherited from base
        using typename UnscentedBase::T;
        
        //! State Type inherited from base
        using typename UnscentedBase::State;
        
        //! Measurement Model Type
        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = typename UnscentedBase::template MeasurementModelType<Measurement, CovarianceBase>;

        //! System Model Type
        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = typename UnscentedBase::template SystemModelType<Control, CovarianceBase>;
        
    protected:
        //! The number of sigma points (depending on state dimensionality)
        using UnscentedBase::SigmaPointCount;
        
        //! Matrix type containing the sigma state or measurement points
        template<class Type>
        using SigmaPoints = typename UnscentedBase::template SigmaPoints<Type>;
        
        //! Kalman Gain Matrix Type
        template<class Measurement>
        using KalmanGain = Kalman::KalmanGain<State, Measurement>;
        
    protected:
        // Member variables
        
        //! State Estimate
        using UnscentedBase::x;
        
        //! State Covariance
        using StandardBase::P;
        
        //! Sigma points (state)
        using UnscentedBase::sigmaStatePoints;
        
        //! Fading factor
        T lambda;
        
    public:
        /**
         * Constructor
         * 
         * See paper for detailed parameter explanation
         * 
         * @param alpha Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
         * @param beta Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
         * @param kappa Secondary scaling parameter (usually 0)
         * @param lambda Fading factor (usually slightly greater than 1)
         */
        AdaptiveFadingUnscentedKalmanFilter(T alpha = T(1), T beta = T(2), T kappa = T(0), T lambda = T(0.5))
            : UnscentedKalmanFilterBase<StateType>(alpha, beta, kappa), lambda(lambda)
        {
            // Init covariance to identity
            P.setIdentity();
        }
       
        /**
         * @brief Perform filter prediction step using system model and no control input (i.e. \f$ u = 0 \f$)
         *
         * @param [in] s The System model
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( const SystemModelType<Control, CovarianceBase>& s )
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
        const State& predict( const SystemModelType<Control, CovarianceBase>& s, const Control& u )
        {
            // Compute sigma points
            if(!computeSigmaPoints())
            {
                // TODO: handle numerical error
                assert(false);
            }
            
            // Compute predicted state
            x = this->template computeStatePrediction<Control, CovarianceBase>(s, u);
            
            // Compute predicted covariance with fading factor
            computeCovarianceFromSigmaPoints(x, sigmaStatePoints, s.getCovariance(), P, lambda);
            
            // Return predicted state
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
        const State& update( const MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
        {
            SigmaPoints<Measurement> sigmaMeasurementPoints;
            
            // Predict measurement (and corresponding sigma points)
            Measurement y = this->template computeMeasurementPrediction<Measurement, CovarianceBase>(m, sigmaMeasurementPoints);
            
            // Compute innovation covariance
            Covariance<Measurement> P_yy;
            computeCovarianceFromSigmaPoints(y, sigmaMeasurementPoints, m.getCovariance(), P_yy);
            
            KalmanGain<Measurement> K;
            computeKalmanGain(y, sigmaMeasurementPoints, P_yy, K);
            
            // Update state
            x += K * ( z - y );
            
            // Update state covariance
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
         * @brief Compute the Covariance from sigma points and noise covariance with fading factor
         * 
         * @param [in] mean The mean predicted state or measurement
         * @param [in] sigmaPoints the predicted sigma state or measurement points
         * @param [in] noiseCov The system or measurement noise covariance
         * @param [in] fading The fading factor
         * @param [out] cov The propagated state or innovation covariance
         *
         * @return True on success, false if a numerical error is encountered when updating the matrix
         */
        template<class Type>
        bool computeCovarianceFromSigmaPoints( const Type& mean, const SigmaPoints<Type>& sigmaPoints, 
                                               const Covariance<Type>& noiseCov, Covariance<Type>& cov, T fading = T(1.0)) const
        {
            decltype(sigmaPoints) W = this->sigmaWeights_c.transpose().template replicate<Type::RowsAtCompileTime,1>();
            decltype(sigmaPoints) tmp = (sigmaPoints.colwise() - mean);
            cov = fading * (tmp.cwiseProduct(W) * tmp.transpose() + noiseCov);
            
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
                    = (sigmaStatePoints.colwise() - x).cwiseProduct( W ).eval()
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
    };
}

#endif

