
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>


#include "SystemModelEulero.hpp"
#include "VelocityMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/AFExtendedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>


using namespace KalmanExamples2;

typedef double T;

// Some type shortcuts
typedef Estimation::State<T> State;
typedef Estimation::SystemModel<T> SystemModel;
typedef Estimation::Control<T> Control;

typedef Estimation::VelocityMeasurement<T> VelocityMeasurement;
typedef Estimation::VelocityMeasurementModel<T> VelocityModel;


int main(int argc, char** argv)
{   
    // Read the data of the simulation from the cvs file "simulation_data.cvs":
    // 1st column: desired penetration
    // 2nd column: desired velocity
    // 3rd column: actual penetration
    // 4th column: actual velocity
    std::ifstream file;
    file.open("/home/luca/Dottorato/Online Stiffness Estimation/cpp/kalman/simulation_data.csv");
    std::string line;
    std::vector<std::vector<double>> data;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::vector<double> row;
        std::string entry;
        while (std::getline(ss, entry, ','))
        {
            row.push_back(std::stod(entry));
        }
        data.push_back(row);
    }
    file.close();
    
    State x;
    x.x1() = 0.0;
    x.x2() = 0.0;
    x.x3() = 1.0;
    x.x4() = 1.0;
    
    // System
    SystemModel sys(0.002, 0.3, 2.0, 1500.0, 40.0);

    // Control input
    Control u;

    // // Measurement models
    // // Set position landmarks at (-10, -10) and (30, 75)
    VelocityModel vm;
    
    // // Random number generation (for noise simulation)
    // std::default_random_engine generator;
    // generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    // std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    Kalman::ExtendedKalmanFilter<State> ekf;
    // // Unscented Kalman Filter
    // Kalman::UnscentedKalmanFilter<State> ukf(1);
    // // Adaptive Fading Extended Kalman Filter
    // Kalman::AFExtendedKalmanFilter<State> afekf;

    // Init filters with true system state
    ekf.init(x);
    // ukf.init(x);
    // afekf.init(x);

    // Save covariance for later
    Kalman::Covariance<State> cov = ekf.getCovariance();
    // Set initial values for the covariance
    cov(0,0) = 1;
    cov(1,1) = 1;
    cov(2,2) = 1000;
    cov(3,3) = 40;

    // Set covariance
    // ukf.setCovariance(cov);
    if(ekf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    
    // cov(0,0) = 0.000001;
    // cov(1,1) = 0.1;
    // cov(2,2) = 0.1;
    // afekf.setCovariance(cov);
    // // Standard-Deviation of noise added to all state vector components during state transition
    // // T systemNoise = 0.1;
    // // // Standard-Deviation of noise added to all measurement vector components in orientation measurements
    // // T orientationNoise = 0.025;
    // // // Standard-Deviation of noise added to all measurement vector components in distance measurements
    // // T distanceNoise = 0.25;
    
    // double u, up, force;

    // Simulate for 10 seconds and a frequency of 500 Hz
    const size_t N = data.size();

    for(size_t i = 1; i < N; i++)
    {
        // Update control input
        u.u() = data[i-1][0];
        u.u_dot() = data[i-1][1];
        // Simulate system
        x = sys.f(x, u);
        
    //     // Add noise: Our robot move is affected by noise (due to actuator failures)
    //     // x.p() += systemNoise*noise(generator);
    //     // x.y() += systemNoise*noise(generator);
    //     // x.theta() += systemNoise*noise(generator);
        
        // Predict state for current time-step using the filters
        auto x_ekf = ekf.predict(sys, u);
    //     auto x_ukf = ukf.predict(sys);
    //     auto x_afekf = afekf.predict(sys);
        

        // Get the measurement of velocity
        VelocityMeasurement vel;
        vel.v() = data[i][3];
    //     // Measurement is affected by noise as well
    //     // force += noise(generator);
        
    //     // Set measurement
    //     fmes.f() = force + noise(generator)*0.8;

        // Update UKF
        x_ekf = ekf.update(vm, vel);
    //     x_ekf = ekf.update(fm, fmes);
    //     x_afekf = afekf.update(fm, sys, fmes);
    //     // Save estimated state
    //     // x = x_ukf;
        // Print to stdout as csv format
        std::cout   << data[i][0] << "," << data[i][1] << "," << data[i][2] << "," << data[i][3] 
                    << "," << x_ekf.x1() << "," << x_ekf.x2() << "," << x_ekf.x3() << "," << x_ekf.x4()
                    << std::endl;
    }
    
    return 0;
}
