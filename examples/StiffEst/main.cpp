
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>


#include "SystemModelRK4.hpp"
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
    VelocityModel vm;
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    // Adaptive Fading Extended Kalman Filter
    Kalman::AFExtendedKalmanFilter<State> afekf;

    // Init filters with true system state
    ekf.init(x);
    ukf.init(x);
    afekf.init(x);

    // Save covariance for later
    Kalman::Covariance<State> cov = ekf.getCovariance();
    // Set initial values for the covariance
    cov(0,0) = 1;
    cov(1,1) = 1;
    cov(2,2) = 5000;
    cov(3,3) = 40;

    // Set covariance of the filters
    if(ekf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    if(ukf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    if(afekf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    // Set covariance of the process noise
    cov(0,0) = pow(1.080336677273649e-07,2);
    cov(1,1) = pow(8.125605216973451e-05,2);
    cov(2,2) = 0.0;
    cov(3,3) = 0.0;
    if(sys.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    // Set covariance of the measurement noise
    Kalman::Covariance<VelocityMeasurement> cov2 = vm.getCovariance();
    cov2(0,0) = pow(5e-2,2);
    if(vm.setCovariance(cov2)!= true)
        std::cout << "Error in setting covariance" << std::endl;

    // Simulate for 10 seconds and a frequency of 500 Hz
    const size_t N = data.size();

    for(size_t i = 1; i < N; i++)
    {
        // Update control input
        u.u() = data[i-1][0];
        u.u_dot() = data[i-1][1];
        // Simulate system
        x = sys.f(x, u);
        
        // Predict state for current time-step using the filters
        auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);
        auto x_afekf = afekf.predict(sys, u);    

        // Get the measurement of velocity
        VelocityMeasurement vel;
        vel.v() = data[i][3];
    //     // Measurement is affected by noise as well
    //     // force += noise(generator);
        
        // Set measurement
        vel.v() += noise(generator)*5e-2;

        // Update UKF
        x_ekf = ekf.update(vm, vel);
        x_ukf = ukf.update(vm, vel);
        x_afekf = afekf.update(vm, sys, vel);
    //     // Save estimated state
    //     // x = x_ukf;
        // Print to stdout as csv format
        std::cout   << data[i][0] << "," << data[i][1] << "," << data[i][2] << "," << vel.v() 
                    << "," << x_ekf.x1() << "," << x_ekf.x2() << "," << x_ekf.x3() << "," << x_ekf.x4()
                    << "," << x_ukf.x1() << "," << x_ukf.x2() << "," << x_ukf.x3() << "," << x_ukf.x4()
                    << "," << x_afekf.x1() << "," << x_afekf.x2() << "," << x_afekf.x3() << "," << x_afekf.x4()
                    << std::endl;
    }
    
    return 0;
}
