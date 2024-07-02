
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>


#include "SystemModelEuleroNoFTNL.hpp"
#include "VelocityMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/AFExtendedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>


using namespace KalmanExperimentsNoFTNL;

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
    file.open("/home/luca/Dottorato/Online Stiffness Estimation/cpp/kalman/simulation_data_soft_cancer.csv");
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

    x.x1() = 1;    
    x.x2() = data[0][3];
    x.x3() = 0.1;//1e4;
    x.x4() = 0.1;//1e3;
    
    // x.x1() = data[0][2];    
    // x.x2() = data[0][3];
    // x.x3() = 23348;
    // x.x4() = 1166;

    // System
    SystemModel sys(0.002, 0.14e-3, 1000, 2*sqrt(1000));

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
    cov(0,0) = 5;
    cov(1,1) = 1;
    cov(2,2) = 1;
    cov(3,3) = 1;

    // Set covariance of the filters
    if(ekf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    if(ukf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    if(afekf.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    // Set covariance of the process noise
    cov(0,0) = 1e-8;
    cov(1,1) = 1e-5;
    cov(2,2) = 0.0;
    cov(3,3) = 0.0;
    if(sys.setCovariance(cov)!= true)
        std::cout << "Error in setting covariance" << std::endl;
    // Set covariance of the measurement noise
    Kalman::Covariance<VelocityMeasurement> cov2 = vm.getCovariance();
    cov2(0,0) = 1e-2;
    if(vm.setCovariance(cov2)!= true)
        std::cout << "Error in setting covariance" << std::endl;

    // Simulate for 10 seconds and a frequency of 500 Hz
    const size_t N = data.size();

    // Initialize the state with true values
    x.x1() = data[0][2];    
    x.x2() = data[0][3];
    x.x3() = data[0][7];
    x.x4() = data[0][8];

    for(size_t i = 1; i < N; i++)
    {
        // Update control input
        u.u() = data[i-1][4];
        // Simulate system
        x = sys.f(x, u);
        
        // Predict state for current time-step using the filters
        auto x_ekf_tmp = ekf.predict(sys, u);
        auto x_ukf_tmp = ukf.predict(sys, u);
        auto x_afekf_tmp = afekf.predict(sys, u);    

        // Get the measurement of velocity
        VelocityMeasurement vel;
        vel.v() = data[i][3];
    //     // Measurement is affected by noise as well
    //     // force += noise(generator);

        // Update UKF
        auto x_ekf = ekf.update(vm, vel);
        auto x_ukf = ukf.update(vm, vel);
        auto x_afekf = afekf.update(vm, sys, vel);

        // Print to stdout as csv format
        std::cout   << data[i][0] << "," << data[i][1] << "," << data[i][2] << "," << vel.v() 
                    << "," << x_ekf.x1() << "," << x_ekf.x2() << "," << x_ekf.x3() << "," << x_ekf.x4()
                    << "," << x_ukf.x1() << "," << x_ukf.x2() << "," << x_ukf.x3() << "," << x_ukf.x4()
                    << "," << x_afekf.x1() << "," << x_afekf.x2() << "," << x_afekf.x3() << "," << x_afekf.x4()
                    << "," << x.x1() << "," << x.x2() << "," << x.x3() << "," << x.x4()
                    // Print the covariance of ekf and ukf
                    << "," << ekf.getCovariance()(0,0) << "," << ekf.getCovariance()(1,1) << "," << ekf.getCovariance()(2,2) << "," << ekf.getCovariance()(3,3)
                    << "," << ukf.getCovariance()(0,0) << "," << ukf.getCovariance()(1,1) << "," << ukf.getCovariance()(2,2) << "," << ukf.getCovariance()(3,3)
                    // Print the value of ekf and ukf after the predict step
                    << "," << x_ekf_tmp.x1() << "," << x_ekf_tmp.x2() << "," << x_ekf_tmp.x3() << "," << x_ekf_tmp.x4()
                    << "," << x_ukf_tmp.x1() << "," << x_ukf_tmp.x2() << "," << x_ukf_tmp.x3() << "," << x_ukf_tmp.x4()
                    << "," << data[i-1][4]
                    << std::endl;
    }
    
    return 0;
}
